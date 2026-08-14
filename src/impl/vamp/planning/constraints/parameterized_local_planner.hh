#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/validate.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    namespace detail
    {
        // Detects whether Space exposes a compute_com(Ambient::ConfigurationBlock<rake>) static
        // utility (currently only RBY1::ParameterizedSpace does); guards the static-stability
        // check below so ParameterizedLocalPlanner stays usable for Spaces without one.
        template <typename Space, std::size_t rake, typename = void>
        struct has_compute_com : std::false_type
        {
        };

        template <typename Space, std::size_t rake>
        struct has_compute_com<
            Space,
            rake,
            std::void_t<decltype(Space::template compute_com<rake>(
                std::declval<const typename Space::Ambient::template ConfigurationBlock<rake> &>()))>>
            : std::true_type
        {
        };
    }  // namespace detail

    // Local planner over a robot's ParameterizedSpace (e.g. Robot::ParameterizedSpace): steers
    // and interpolates entirely in the task space `Space`, but validates by resolving each
    // interpolated block through Space::resolve_block into Space::Ambient's (the real
    // joint-space robot's) Configuration and running FK/collision-checking there. This is the
    // IK-resolution bridge a Space that differs from Robot needs -- Robot itself stays the
    // sole owner of Configuration/fkcc, exactly as with UnconstrainedLocalPlanner.
    //
    // Local paths have no interior waypoints (same as UnconstrainedLocalPlanner), so planners
    // using this perform the same arithmetic as calling validate_resolved directly.
    template <
        typename Robot,
        std::size_t rake,
        std::size_t resolution,
        typename Space = typename Robot::ParameterizedSpace>
    struct ParameterizedLocalPlanner
    {
        using Ambient = typename Space::Ambient;
        using Configuration = typename Space::State;
        using Environment = collision::Environment<FloatVector<rake>>;

        ParameterizedLocalPlanner() = default;

        // Task-space samples are resolved (via IK) before use, not projected in place.
        static constexpr bool projecting = false;

        // Multiplier on the number of steps a connect loop may take: IK-resolve drift can
        // require more steps than ceil(distance / range), same rationale as constraint-manifold
        // local planners.
        static constexpr float connect_slack = 1.F;

        // Validity of the local path a -> b in execution order; `forward` is false when the
        // caller's a precedes b in tree order but follows it in execution order (goal trees).
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward = true) const noexcept -> bool
        {
            const Configuration &start = (forward) ? a : b;
            const Configuration &goal = (forward) ? b : a;
            return validate_resolved(start, goal, e);
        }

        // Admit the local path a -> b only if it is valid and has fewer interior waypoints
        // than `max_states` (always true here: no interior waypoints). Reached on success,
        // Trapped otherwise.
        template <typename Budget>
        inline auto connect_within(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Budget &&,
            std::size_t max_states) const noexcept -> Extension<Robot, Space>
        {
            chain_.clear();
            const bool valid = 0 < max_states and validate(a, b, e);
            return {(valid) ? SteerStatus::Reached : SteerStatus::Trapped, chain_};
        }

        // Steer from `from` toward `target` by at most `range`, interpolating in the task
        // space. `distance` is Space::distance(from, target), precomputed by callers.
        template <typename Accept = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float distance,
            float range,
            bool forward,
            const Environment &e,
            Accept &&accept = Accept()) const noexcept -> Extension<Robot, Space>
        {
            chain_.clear();

            const bool reach = distance < range;
            const float step = range / distance;
            const auto next = (reach)   ? target :
                              (forward) ? Space::interpolate(from, target, step) :
                                          Space::interpolate(target, from, 1.F - step);

            if (not accept(next))
            {
                return {SteerStatus::Rejected, chain_};
            }

            if (not validate(from, next, e, forward))
            {
                return {SteerStatus::Trapped, chain_};
            }

            chain_.emplace_back(next);
            return {(reach) ? SteerStatus::Reached : SteerStatus::Advanced, chain_};
        }

        // Task-space samples are resolved fresh via IK on every validate() call, so there is
        // no persistent constraint to project onto ahead of time.
        inline auto project(Configuration &) const noexcept -> bool
        {
            return true;
        }

        // Whether a configuration satisfies the constraint: resolvability is checked lazily
        // by validate(), so this is trivially true.
        inline auto satisfied(const Configuration &) const noexcept -> bool
        {
            return true;
        }

        // Runs the same per-block checks validate_resolved() applies to every interpolated
        // sample -- eef-collision prefilter, IK resolve_block, support-polygon stability, then
        // fkcc/fkcc_attach -- against a single caller-supplied block. Exposed publicly so
        // callers can sanity-check a start/goal state against every condition RRTC will
        // eventually enforce along the path, not just resolvability and fkcc individually.
        inline auto resolve_and_check(
            const typename Space::template StateBlock<rake> &interp_block,
            const Environment &environment) const noexcept -> bool
        {
            return resolve_and_check_impl(interp_block, environment);
        }

    private:
        mutable std::vector<Configuration> chain_;

        // Static-stability support polygon, in the mobile base's local xy frame (matches
        // Space::compute_com's output frame). Vertices trace the ground-contact points in
        // order (right wheel -> left wheel -> left caster -> right caster).
        static constexpr std::array<std::array<float, 2>, 4> support_polygon_xy = {{
            {0.228000F, -0.265000F},   // right wheel
            {0.228000F, 0.265000F},    // left wheel
            {-0.248686F, 0.066310F},   // left caster
            {-0.248686F, -0.066310F},  // right caster
        }};

        static inline auto point_in_support_polygon(FloatVector<rake> x, FloatVector<rake> y) noexcept -> bool
        {
            using V = FloatVector<rake>;

            V inside = V::zero_vector();
            for (std::size_t i = 0, j = support_polygon_xy.size() - 1; i < support_polygon_xy.size(); j = i++)
            {
                const auto &pi = support_polygon_xy[i];
                const auto &pj = support_polygon_xy[j];

                const V pi_y(pi[1]);
                const V pj_y(pj[1]);
                const V crosses_y = (pi_y > y) ^ (pj_y > y);
                const V slope((pj[0] - pi[0]) / (pj[1] - pi[1]));
                const V x_at_y = V(pi[0]) + slope * (y - pi_y);

                inside = inside ^ (crosses_y & (x < x_at_y));
            }

            return inside.all();
        }

        // Checks that every lane's center of mass (in the base-local frame Space::compute_com
        // returns) lies within the static support polygon. compute_com is batched over the
        // whole block directly, and the polygon test itself is vectorized over lanes, so there
        // is no per-lane splitting anywhere in this check. No-op (always true) for Spaces
        // without a compute_com utility.
        template <typename S = Space>
        static inline auto com_within_support_polygon(
            const typename Ambient::template ConfigurationBlock<rake> &block) noexcept
            -> std::enable_if_t<detail::has_compute_com<S, rake>::value, bool>
        {
            const auto com = S::template compute_com<rake>(block);
            return point_in_support_polygon(com[0], com[1]);
        }

        template <typename S = Space>
        static inline auto com_within_support_polygon(
            const typename Ambient::template ConfigurationBlock<rake> &) noexcept
            -> std::enable_if_t<not detail::has_compute_com<S, rake>::value, bool>
        {
            return true;
        }

        inline auto print_configuration(const Configuration &c, const std::string &label) const noexcept -> void
        {
            std::cout << label << " = ";
            // convert c to array for printing, since Configuration doesn't have an ostream operator<<
            auto c_array = c.to_array();
            for (std::size_t i = 0; i < Space::dimension; ++i)
            {
                std::cout << c_array[i] << (i + 1 < Space::dimension ? ", " : "\n");
            }
        }

        // Interpolates task-space samples along start -> goal, resolving each through
        // Space::resolve_block and collision-checking the resulting ambient block; mirrors
        // validate_motion's non-Euclidean branch but with an IK-resolve step interposed
        // between interpolation and FK/CC. Returns false as soon as either the IK resolve or
        // the collision check fails for any sample block.
        inline auto validate_resolved(
            const Configuration &start,
            const Configuration &goal,
            const Environment &environment) const noexcept -> bool
        {
            // std::cout << "Attempting to connect start ";
            // print_configuration(start, "start");
            // std::cout << " to goal ";
            // print_configuration(goal, "goal");
            const float distance = Space::distance(start, goal);
            // const float distance = 1.695;
            const std::size_t n =
                std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
            const auto percents = FloatVector<rake>(Percents<rake>::percents);
            const auto t_step = FloatVector<rake>::fill(1.F / static_cast<float>(rake * n));

            typename Space::template StateBlock<rake> interp_block;
            auto t_block = percents;
            Space::template interpolate_block<rake>(start, goal, t_block, interp_block);

            if (not resolve_and_check_impl(interp_block, environment))
            {
                return false;
            }

            if (n == 1)
            {
                return true;
            }

            for (auto i = 1U; i < n; ++i)
            {
                t_block = t_block - t_step;
                Space::template interpolate_block<rake>(start, goal, t_block, interp_block);

                if (not resolve_and_check_impl(interp_block, environment))
                {
                    return false;
                }
            }

            return true;
        }

        inline auto resolve_and_check_impl(
            const typename Space::template StateBlock<rake> &interp_block,
            const Environment &environment) const noexcept -> bool
        {
            // just reject if the sampled eefs are in collision
            auto eef_coll_res = Space::template eefs_collision_free<rake>(environment, interp_block);
            if (not eef_coll_res)
            {
                return false;
            }

            auto [param_valid, ambient_block] = Space::template resolve_block<rake>(interp_block);
            if (not param_valid)
            {
                return false;
            }

            if (not com_within_support_polygon(ambient_block))
            {
                return false;
            }

            return (environment.attachments.empty()) ?
                       Ambient::template fkcc<rake>(environment, ambient_block) :
                       Ambient::template fkcc_attach<rake>(environment, ambient_block);
        }
    };
}  // namespace vamp::planning::constraint
