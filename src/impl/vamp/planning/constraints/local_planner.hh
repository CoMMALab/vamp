#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/validate.hh>
#include <vamp/space.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Perturbation multipliers 0, -0.25, +0.25, -0.5, +0.5, ...: index 0 is unperturbed so
    // one lane always aims exactly at the steer target.
    template <std::size_t n>
    inline constexpr auto generate_stddev_samples() -> std::array<float, n>
    {
        std::array<float, n> samples{};
        for (std::size_t i = 1; i < n; ++i)
        {
            const auto magnitude = static_cast<float>((i + 1) / 2) * 0.25F;
            samples[i] = (i % 2 == 1) ? -magnitude : magnitude;
        }

        return samples;
    }

    template <std::size_t n>
    struct StdDevSamples
    {
        inline static constexpr auto samples = generate_stddev_samples<n>();
    };

    // Local planner over a constraint manifold: local paths are geodesic approximations
    // traced by projecting the straight-line discretization lane-by-lane onto the manifold,
    // as in CBiRRT2 (Berenson et al.). Paths are direction-symmetric, so `forward` is
    // ignored. Holds a mutable per-constraint evaluation cache via ConstraintSet: not
    // thread-safe, use one instance (with unshared constraints) per thread.
    //
    // Planning in a Space other than the robot's own (SplineSpace): states carry coordinates beyond the joints, which
    // interpolate along with them and are handed to the constraints per lane (ConstraintSet::set_parameter) before
    // every projection, so the projections solve for the joints at the parameter the lane has. Everything geometric
    // (continuity, collision, tolerances, drift) is measured on the ambient configuration.
    template <typename Robot, std::size_t rake, std::size_t resolution, typename Space = Robot>
    struct ConstrainedLocalPlanner
    {
        // Chord discretization is linear in configuration coordinates; quaternion segments
        // are renormalized after every linear construction (nlerp), which covers robots
        // whose only non-Euclidean joints are quaternion-parameterized (SO(3)/free-flyer).
        static_assert(
            Robot::euclidean or Robot::so3_offsets.size() > 0,
            "ConstrainedLocalPlanner requires a Euclidean robot or one whose non-Euclidean "
            "joints are all quaternion-parameterized (so3_offsets)");

        using Traits = SpaceTraits<Robot, Space>;

        // The planner's states (a Configuration when the robot is its own Space), the robot's configurations they sit
        // over, and the blocks of each: the constraints work on ambient blocks.
        using Configuration = typename Traits::State;
        using Ambient = typename Robot::Configuration;
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using StateBlock = typename Traits::template StateBlock<rake>;
        using Environment = collision::Environment<FloatVector<rake>>;
        using Waypoints = std::array<Configuration, rake>;

        static constexpr std::size_t state_dimension = Traits::dimension;
        static constexpr bool has_parameter = Traits::extra > 0;
        static_assert(Traits::extra <= 1, "ConstrainedLocalPlanner supports one extra state coordinate");

        static constexpr bool projecting = true;

        explicit ConstrainedLocalPlanner(ConstraintSet<Robot, rake> constraint_set) noexcept
          : constraints(std::move(constraint_set)), connect_slack(constraints.settings().connect_slack)
        {
        }

        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool = true) const noexcept -> bool
        {
            Waypoints waypoints;
            return trace(a, b, e, waypoints) and
                   (Traits::ambient(waypoints.back()) - Traits::ambient(b)).squared_l2_norm() <
                       constraints.settings().endpoint_tolerance2;
        }

        // The local path a -> b is defined by its projected chain: on success the extension
        // holds its interior waypoints (the last lane only approximates b and is dropped),
        // which planners must keep for reconstructed paths to stay on the manifold. Admits
        // the path only if the interior count is below `max_states` and the chain's cost
        // through those waypoints is within `budget()`.
        template <typename Budget>
        inline auto connect_within(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Budget &&budget,
            std::size_t max_states) const noexcept -> Extension<Robot, Space>
        {
            chain_.clear();

            Waypoints waypoints;
            if (rake - 1 >= max_states or not trace(a, b, e, waypoints) or
                (Traits::ambient(waypoints.back()) - Traits::ambient(b)).squared_l2_norm() >=
                    constraints.settings().endpoint_tolerance2)
            {
                return {SteerStatus::Trapped, chain_};
            }

            float chain_cost = cost<Robot>(Traits::ambient(a), Traits::ambient(waypoints.front()));
            for (auto lane = 0U; lane + 2 < rake; ++lane)
            {
                chain_cost += cost<Robot>(Traits::ambient(waypoints[lane]), Traits::ambient(waypoints[lane + 1]));
            }

            chain_cost += cost<Robot>(Traits::ambient(waypoints[rake - 2]), Traits::ambient(b));
            if (chain_cost > budget())
            {
                return {SteerStatus::Trapped, chain_};
            }

            chain_.assign(waypoints.begin(), waypoints.end() - 1);
            return {SteerStatus::Reached, chain_};
        }

        template <typename Accept = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float distance,
            float range,
            bool,
            const Environment &e,
            Accept &&accept = Accept()) const noexcept -> Extension<Robot, Space>
        {
            chain_.clear();

            const bool reach = distance < range;
            const float step_length = (reach) ? distance : range;
            const auto vector = (target - from) * ((reach) ? 1.F : range / distance);

            // Perturb the step length per lane so projection has fallbacks when the exact
            // candidate projects poorly; lane 0 aims exactly at the candidate.
            const auto perturb = FloatVector<rake>::fill(1.F) +
                                 FloatVector<rake>(StdDevSamples<rake>::samples) *
                                     constraints.settings().perturbation_scale;

            StateBlock block;
            for (auto i = 0U; i < state_dimension; ++i)
            {
                block[i] = from.broadcast(i) + vector.broadcast(i) * perturb;
            }

            renormalize(block);
            const auto winner = on_manifold(
                block, [&](Block &q) { return constraints.project_any(q, step_length); });
            if (winner < 0)
            {
                return {SteerStatus::Trapped, chain_};
            }

            const auto lane = static_cast<std::size_t>(winner);
            const auto from_arr = from.to_array();
            std::array<float, state_dimension> next_arr;
            float drift2 = 0.F;
            for (auto j = 0U; j < state_dimension; ++j)
            {
                next_arr[j] = block[{j, lane}];
                if (j < Robot::dimension)
                {
                    const float diff = next_arr[j] - from_arr[j];
                    drift2 += diff * diff;
                }
            }

            if (drift2 > 4.F * step_length * step_length)
            {
                return {SteerStatus::Trapped, chain_};
            }

            Configuration next(next_arr);
            if (not accept(next))
            {
                return {SteerStatus::Rejected, chain_};
            }

            // Check the candidate endpoint for collision before paying for the full trace.
            Block next_block;
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                next_block[j] = next.broadcast(j);  // the joints of the state
            }

            if (not fkcc_block<Robot, rake>(e, next_block))
            {
                return {SteerStatus::Trapped, chain_};
            }

            Waypoints waypoints;
            if (not trace(from, next, e, waypoints))
            {
                return {SteerStatus::Trapped, chain_};
            }

            next = waypoints.back();
            if (constraints.settings().emit_all_waypoints)
            {
                chain_.assign(waypoints.begin(), waypoints.end());
            }
            else
            {
                chain_.emplace_back(next);
            }

            const bool reached =
                reach and (Traits::ambient(next) - Traits::ambient(target)).squared_l2_norm() <
                              constraints.settings().reached_radius2;
            return {(reached) ? SteerStatus::Reached : SteerStatus::Advanced, chain_};
        }

        // Project the state's configuration onto the manifold at the state's own parameter (if it has one)
        inline auto project(Configuration &q) const noexcept -> bool
        {
            if constexpr (not has_parameter)
            {
                return constraints.project(q);
            }
            else
            {
                Ambient projected = Traits::ambient(q);
                constraints.set_parameter(parameter_row(q));
                if (not constraints.project(projected))
                {
                    return false;
                }

                q = Traits::with_ambient(q, projected);
                return true;
            }
        }

        inline auto satisfied(const Configuration &q) const noexcept -> bool
        {
            if constexpr (not has_parameter)
            {
                return constraints.satisfied(q);
            }
            else
            {
                constraints.set_parameter(parameter_row(q));
                return constraints.satisfied(Traits::ambient(q));
            }
        }

        ConstraintSet<Robot, rake> constraints;

        // Read by planners as lp.connect_slack, uniformly with the static constant on the
        // non-projecting local planners.
        float connect_slack;

    private:
        mutable std::vector<Configuration> chain_;

        using Row = FloatVector<rake, 1>;

        // The states' extra coordinate (the last scalar) of a state, in every lane
        static inline auto parameter_row(const Configuration &q) noexcept -> Row
        {
            return Row::fill(q.to_array()[Robot::dimension]);
        }

        // The ambient block of a block of states
        static inline auto ambient_block(const StateBlock &states) noexcept
            -> std::conditional_t<has_parameter, Block, const Block &>
        {
            if constexpr (not has_parameter)
            {
                return states;
            }
            else
            {
                Block q;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    q[j] = states[j];
                }

                return q;
            }
        }

        static inline void store_ambient(StateBlock &states, const Block &q) noexcept
        {
            if constexpr (not has_parameter)
            {
                states = q;
            }
            else
            {
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    states[j] = q[j];
                }
            }
        }

        static inline void renormalize(StateBlock &states) noexcept
        {
            if constexpr (Robot::so3_offsets.size() > 0)
            {
                auto q = ambient_block(states);
                renormalize_so3<Robot, rake>(q);
                store_ambient(states, q);
            }
        }

        // Run a projection of the constraint set on the ambient blocks of `states`, fixing the extra coordinate
        // (if any) of each lane at the lane's own value, and write the projected joints back.
        template <typename Projection>
        inline auto on_manifold(StateBlock &states, Projection &&projection) const noexcept
        {
            if constexpr (not has_parameter)
            {
                return projection(states);
            }
            else
            {
                auto q = ambient_block(states);
                constraints.set_parameter(states[Robot::dimension]);
                const auto result = projection(q);
                store_ambient(states, q);
                return result;
            }
        }

        // Trace the constrained local path a -> b: project the rake-lane discretization of
        // the chord onto the manifold, require continuity (adjacent lanes within twice the
        // unconstrained lane spacing), collision-check the chain, then sweep the inter-lane
        // gaps at `resolution`. On success `waypoints` holds the projected chain in a -> b
        // order; its last entry approximates b whenever b is on the manifold.
        auto trace(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Waypoints &waypoints) const noexcept -> bool
        {
            const auto vector = b - a;
            const float distance = (Traits::ambient(b) - Traits::ambient(a)).l2_norm();
            const auto percents = FloatVector<rake>(Percents<rake>::percents);

            StateBlock block;
            for (auto i = 0U; i < state_dimension; ++i)
            {
                block[i] = a.broadcast(i) + vector.broadcast(i) * percents;
            }

            renormalize(block);
            if (not on_manifold(block, [&](Block &q) { return constraints.project_all(q, distance); }))
            {
                return false;
            }

            // Projection must not tear the path apart; record the inter-lane differences
            // for the sweep below.
            const float lane_spacing = distance / static_cast<float>(rake);
            const float max_gap2_allowed = 4.F * lane_spacing * lane_spacing;
            const auto a_arr = a.to_array();

            std::array<float, state_dimension * rake> diffs;
            float max_gap2 = 0.F;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                float gap2 = 0.F;
                for (auto j = 0U; j < state_dimension; ++j)
                {
                    const float prev = (lane == 0) ? a_arr[j] : block[{j, lane - 1}];
                    const float diff = block[{j, lane}] - prev;
                    diffs[lane + j * rake] = diff;
                    if (j < Robot::dimension)
                    {
                        gap2 += diff * diff;
                    }
                }

                if (gap2 > max_gap2_allowed)
                {
                    return false;
                }

                max_gap2 = std::max(max_gap2, gap2);
            }

            if (not fkcc_block<Robot, rake>(e, ambient_block(block)))
            {
                return false;
            }

            // Extract the chain before the sweep mutates the block.
            std::array<float, state_dimension> lane_arr;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                for (auto j = 0U; j < state_dimension; ++j)
                {
                    lane_arr[j] = block[{j, lane}];
                }

                waypoints[lane] = Configuration(lane_arr);
            }

            const float max_gap = std::sqrt(max_gap2);
            const auto n_steps =
                static_cast<std::size_t>(std::max(std::ceil(max_gap * resolution), 1.F));
            if (n_steps == 1)
            {
                return true;
            }

            // Sweep the inter-lane gaps: slide each lane linearly back toward its
            // predecessor, projecting a copy at each substep and requiring it to stay
            // collision-free and near the interpolant.
            const auto shift = StateBlock(diffs) / static_cast<float>(n_steps);
            const float substep = max_gap / static_cast<float>(n_steps);
            const auto max_drift2 = FloatVector<rake, 1>::fill(4.F * substep * substep);

            auto linear = block;
            for (auto i = 1U; i < n_steps; ++i)
            {
                for (auto j = 0U; j < state_dimension; ++j)
                {
                    linear[j] = linear[j] - shift[j];
                }

                auto projected = linear;
                renormalize(projected);
                if (not on_manifold(
                        projected, [&](Block &q) { return constraints.project_all(q, max_gap * rake); }))
                {
                    return false;
                }

                if (ConstraintSet<Robot, rake>::squared_distance(ambient_block(projected), ambient_block(linear))
                        .test_any_greater_equal(max_drift2))
                {
                    return false;
                }

                if (not fkcc_block<Robot, rake>(e, ambient_block(projected)))
                {
                    return false;
                }
            }

            return true;
        }
    };
}  // namespace vamp::planning::constraint
