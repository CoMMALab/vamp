#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    enum struct SteerStatus : std::uint8_t
    {
        Rejected,  // the caller's cost bound pruned the candidate before validation
        Trapped,   // no progress: the first step of the local path was invalid
        Advanced,  // moved toward the target but did not reach it
        Reached,   // the target configuration was attained
    };

    // Result of a local-planner extension: a status plus the accepted waypoints of the local
    // path, in execution order. The waypoints view a buffer owned by the local planner and
    // are valid only until its next steer/connect_within call; they are empty on
    // Rejected/Trapped, and empty on success when the local path has no interior waypoints
    // (the unconstrained connect_within).
    template <typename Robot>
    struct [[nodiscard]] Extension
    {
        using Configuration = typename Robot::Configuration;

        SteerStatus status;
        const std::vector<Configuration> &waypoints;

        // The last accepted waypoint. Only meaningful on Advanced/Reached with a nonempty
        // chain (always true for steer).
        [[nodiscard]] inline auto endpoint() const noexcept -> const Configuration &
        {
            return waypoints.back();
        }
    };

    // Insert an extension's waypoints as a chain of tree nodes: add_node(configuration,
    // parent_index) inserts one node and returns its index, or std::nullopt if the tree is
    // full. Returns the index of the last inserted node (or `parent` if none were) and
    // whether insertion was truncated by a full tree.
    template <typename Iterator, typename AddNode>
    inline auto insert_chain(Iterator begin, Iterator end, std::size_t parent, AddNode &&add_node)
        -> std::pair<std::size_t, bool>
    {
        for (; begin != end; ++begin)
        {
            const auto index = add_node(*begin, parent);
            if (not index)
            {
                return {parent, true};
            }

            parent = *index;
        }

        return {parent, false};
    }

    template <typename Robot, typename AddNode>
    inline auto insert_chain(
        const std::vector<typename Robot::Configuration> &waypoints,
        std::size_t parent,
        AddNode &&add_node) -> std::pair<std::size_t, bool>
    {
        return insert_chain(waypoints.begin(), waypoints.end(), parent, std::forward<AddNode>(add_node));
    }

    // Local planners generate and validate the local paths that connect configurations. All
    // operations are directed: a local path from a to b is traversed in execution order
    // (start to goal), which matters for planners whose goal-tree edges run child to parent.
    //
    // The default local planner steers by Robot::interpolate and validates with
    // validate_motion; its local paths have no interior waypoints, so planners using it
    // perform the same arithmetic as calling those primitives directly (bit-identical
    // under -ffp-contract=off; under the release flags' -ffp-contract=fast, inlining
    // context may contract FMAs differently, which can shift results by 1 ULP).
    // Constrained local planners instead project candidates onto a constraint manifold
    // and return the whole projected waypoint chain, which planners insert as a chain of
    // tree nodes.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct UnconstrainedLocalPlanner
    {
        using Configuration = typename Robot::Configuration;
        using Environment = collision::Environment<FloatVector<rake>>;

        UnconstrainedLocalPlanner() = default;

        // True if configurations must be projected to remain valid (e.g. onto a constraint
        // manifold): planners must lp.project() any configuration they synthesize
        // (interpolated midpoints, perturbations) before use.
        static constexpr bool projecting = false;

        // Multiplier on the number of steps a connect loop may take: projection drift can
        // require more steps than ceil(distance / range).
        static constexpr float connect_slack = 1.F;

        // Validity of the local path a -> b. `a` is the configuration already known to be valid and
        // `b` the new one: validate_motion does not check its start, so the path is always checked
        // in this order. The straight-line path is the same in either direction, so `forward` does
        // not matter here (swapping the endpoints for goal trees would leave the new node, and so
        // every goal-tree node, unchecked).
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool /* forward */ = true) const noexcept -> bool
        {
            return validate_motion<Robot, rake, resolution>(a, b, e);
        }

        // Admit the local path a -> b only if it is valid, has fewer interior waypoints than
        // `max_states`, and costs no more than `budget()`. The budget is lazy so that this
        // planner, whose local paths have no interior waypoints and never re-cost, does not
        // evaluate it at all. Reached on success (with the interior waypoints, here none),
        // Trapped otherwise.
        template <typename Budget>
        inline auto connect_within(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Budget &&,
            std::size_t max_states) const noexcept -> Extension<Robot>
        {
            chain_.clear();
            const bool valid = 0 < max_states and validate(a, b, e);
            return {(valid) ? SteerStatus::Reached : SteerStatus::Trapped, chain_};
        }

        // Steer from `from` toward `target` by at most `range`. `forward` is true iff `from`
        // precedes `target` in execution order. `distance` is Robot::distance(from, target),
        // precomputed by callers (usually from a nearest-neighbor query). `accept` is invoked
        // on the candidate endpoint after interpolation (and projection) but before
        // validation; returning false aborts with Rejected, letting optimizing planners
        // prune an extension on cost before paying for the validity check.
        template <typename Accept = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float distance,
            float range,
            bool forward,
            const Environment &e,
            Accept &&accept = Accept()) const noexcept -> Extension<Robot>
        {
            chain_.clear();

            const bool reach = distance < range;
            const float step = range / distance;
            const auto next = (reach)    ? target :
                              (forward)  ? Robot::interpolate(from, target, step) :
                                           Robot::interpolate(target, from, 1.F - step);

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

        // Project a configuration onto the constraint manifold (identity here). Returns false
        // if projection fails; the configuration is then unusable.
        inline auto project(Configuration &) const noexcept -> bool
        {
            return true;
        }

        // Whether a configuration satisfies the constraint: no manifold here, so always true.
        inline auto satisfied(const Configuration &) const noexcept -> bool
        {
            return true;
        }

    private:
        mutable std::vector<Configuration> chain_;
    };
}  // namespace vamp::planning
