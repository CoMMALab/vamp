#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/nn/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planners/task_space_rrtc_settings.hh>
#include <vamp/planning/planners/task_space_trace.hh>
#include <vamp/random/rng.hh>
#include <vamp/space.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    // RRT-Connect toward a task-space goal region instead of a goal configuration. No goal
    // configuration is ever computed (no IK): the region is a ConstraintSet (e.g. a TSR) and
    // a single tree is grown from the start.
    //
    // Each iteration extends the nearest node toward a random sample with the local planner
    // (as RRTC does), then "connects": from the node just added, a greedy walk takes
    // Levenberg-Marquardt steps on the constraint error, each clamped to `range`, adding a
    // node per step, until a step fails to lower the error, hits a joint limit or a
    // collision, or the region is reached. This plays the role of RRTC's connect step toward
    // the goal tree, with the region's error gradient in place of the straight line to a goal
    // configuration.
    //
    // The planner succeeds when a node satisfies the region (error within the set's
    // tolerance). Edges are straight lines checked by the local planner; only the
    // goal-directed steps look at the constraint.
    //
    // The tree's nodes are states of `Space` (the robot's configurations when it is its own Space). A Space with
    // extra coordinates (SplineSpace's curve parameter t) is planned in as a whole: samples, extensions and nodes
    // carry them, while the region, the nearest-neighbor tree and collision checking see the ambient configuration
    // (the first Robot::dimension coordinates). A goal-directed step then also advances the extra coordinate
    // (Space::advance_parameter, by settings.parameter_step), which the local planner holds fixed while projecting
    // the joints.
    template <typename Robot, std::size_t rake, std::size_t resolution, typename Space = Robot>
    struct TaskSpaceRRTC
    {
        using Traits = vamp::SpaceTraits<Robot, Space>;
        using Configuration = typename Traits::State;
        static constexpr auto dimension = Traits::dimension;
        static constexpr auto ambient_dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot, Space>;
        using Region = constraint::ConstraintSet<Robot, rake>;
        using Block = typename Robot::template ConfigurationBlock<rake>;

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const Region &region,
            const collision::Environment<FloatVector<rake>> &environment,
            const TaskSpaceRRTCSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner(),
            TaskSpaceTrace<Robot, Space> *trace = nullptr) noexcept -> PlanningResult<Robot, Space>
        {
            size_t extend_counter_failed = 0;
            size_t extend_counter_succeeded = 0;

            PlanningResult<Robot, Space> result;
            const auto start_time = std::chrono::steady_clock::now();

            NN<Robot> tree;
            tree.reserve(settings.max_samples);
            tree.set_epsilon(settings.nn_epsilon);

            auto buffer = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                settings.max_samples * Configuration::num_scalars_rounded);
            auto parents =
                vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(settings.max_samples);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            const float tolerance = region.settings().tolerance;
            std::size_t iter = 0;

            std::size_t free_index = 0;
            const auto add_node = [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (free_index >= settings.max_samples)
                {
                    return std::nullopt;
                }

                float *ptr = buffer_index(free_index);
                c.to_array(ptr);
                tree.insert(free_index, ptr);
                parents[free_index] = (free_index == 0) ? 0 : parent;
                return free_index++;
            };

            using Lane = std::array<float, dimension>;
            const auto load = [&buffer_index](std::size_t index) -> Lane
            {
                Lane lane;
                const float *ptr = buffer_index(index);
                for (auto j = 0U; j < dimension; ++j)
                {
                    lane[j] = ptr[j];
                }

                return lane;
            };

            const auto lane_of = [](const Configuration &c) -> Lane
            {
                Lane lane;
                const auto array = c.to_array();
                for (auto j = 0U; j < dimension; ++j)
                {
                    lane[j] = array[j];
                }

                return lane;
            };

            constexpr float nan = std::numeric_limits<float>::quiet_NaN();
            const auto record = [&](TraceEvent event,
                                    TraceOutcome outcome,
                                    std::int32_t node,
                                    std::int32_t parent,
                                    float error,
                                    float reference_error,
                                    const Lane &configuration)
            {
                if (trace)
                {
                    trace->entries.push_back(
                        {event,
                         outcome,
                         static_cast<std::uint32_t>(iter),
                         node,
                         parent,
                         error,
                         reference_error,
                         configuration});
                }
            };

            // The constraint kernels evaluate `rake` lanes at once; the walk has a single
            // configuration, so every lane holds a copy of its joints.
            const auto broadcast = [](const Lane &q) -> Block
            {
                Block block;
                for (auto j = 0U; j < ambient_dimension; ++j)
                {
                    block[j] = FloatVector<rake>::fill(q[j]);
                }

                return block;
            };

            std::optional<std::size_t> solution;

            // Greedy walk from `node` toward the region: one clamped LM step per added node.
            const auto walk = [&](std::size_t node)
            {
                std::size_t current = node;
                Lane q = load(current);
                Block block = broadcast(q);
                float error = region.squared_error(block)[{0, 0}];
                record(TraceEvent::WalkStart, TraceOutcome::None, node, -1, error, nan, q);

                for (std::size_t step = 0; step < settings.max_greedy_steps; ++step)
                {
                    if (error <= tolerance)
                    {
                        break;
                    }

                    // The kernel cache holds the Jacobian at `block`: take the LM step from it.
                    Block proposed = block;
                    region.step_in_place(proposed);
                    // std::cout << "Step " << step << ": error = " << error << ", proposed = " << proposed << std::endl;

                    Lane next = q;
                    float norm2 = 0.F;
                    for (auto j = 0U; j < ambient_dimension; ++j)
                    {
                        next[j] = proposed[{j, 0}] - q[j];
                        norm2 += next[j] * next[j];
                    }

                    if (trace)
                    {
                        Lane lm = q;
                        for (auto j = 0U; j < ambient_dimension; ++j)
                        {
                            lm[j] = proposed[{j, 0}];
                        }

                        record(TraceEvent::WalkLm, TraceOutcome::None, -1, current, nan, error, lm);
                    }

                    const float norm = std::sqrt(norm2);
                    if (not(norm > 1e-9F))
                    {
                        record(TraceEvent::WalkProposal, TraceOutcome::ZeroStep, -1, current, nan, error, q);
                        return;
                    }

                    const float clamp = std::min(1.F, settings.range / norm);
                    for (auto j = 0U; j < ambient_dimension; ++j)
                    {
                        next[j] = q[j] + next[j] * clamp;
                    }

                    if constexpr (Traits::extra > 0)
                    {
                        // the extra coordinate (the curve parameter) moves on toward the end of its range
                        next[ambient_dimension] =
                            Space::advance_parameter(q[ambient_dimension], settings.parameter_step);
                    }

                    if constexpr (LocalPlanner::projecting)
                    {
                        // On a constraint manifold the LM point is only a target: it heads straight for the region and
                        // leaves the manifold. lp.steer takes the step toward it and projects onto the manifold, as
                        // every extension of a constrained RRT does (with the continuity and collision checks), and
                        // returns the chain of on-manifold waypoints. The walk then judges the chain's endpoint.
                        const Configuration target(next);
                        const float step_length = std::min(norm, settings.range);
                        const auto extension = lp.steer(
                            Configuration(buffer_index(current)), target, step_length, settings.range, true, environment);

                        if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                        {
                            record(TraceEvent::WalkProposal, TraceOutcome::Collision, -1, current, nan, error, next);
                            return;
                        }

                        const Lane endpoint = lane_of(extension.endpoint());
                        const Block endpoint_block = broadcast(endpoint);
                        const float endpoint_error = region.squared_error(endpoint_block)[{0, 0}];
                        if (not(endpoint_error < error))
                        {
                            record(
                                TraceEvent::WalkProposal, TraceOutcome::NoImprovement, -1, current, endpoint_error, error, endpoint);
                            return;
                        }

                        const auto [last, truncated] = insert_chain<Robot, Space>(
                            extension.waypoints,
                            current,
                            [&](const Configuration &c, std::size_t parent) { return add_node(c, parent); });
                        if (truncated or last == current)
                        {
                            record(TraceEvent::WalkProposal, TraceOutcome::TreeFull, -1, current, endpoint_error, error, endpoint);
                            return;
                        }

                        record(
                            TraceEvent::WalkProposal,
                            TraceOutcome::Accepted,
                            static_cast<std::int32_t>(last),
                            current,
                            endpoint_error,
                            error,
                            endpoint);
                        current = last;
                        q = endpoint;
                        block = endpoint_block;  // the region's Jacobian cache now holds this block, for the next step
                        error = endpoint_error;
                        continue;
                    }

                    Block candidate = broadcast(next);
                    constraint::renormalize_so3<Robot, rake>(candidate);
                    const float next_error = region.squared_error(candidate)[{0, 0}];
                    const auto propose = [&](TraceOutcome outcome, std::int32_t node)
                    { record(TraceEvent::WalkProposal, outcome, node, current, next_error, error, next); };

                    if (not(next_error < error))
                    {
                        propose(TraceOutcome::NoImprovement, -1);
                        return;
                    }

                    const Configuration c(next);
                    if (not Traits::in_bounds(c))
                    {
                        propose(TraceOutcome::OutOfBounds, -1);
                        return;
                    }

                    if (not lp.validate(Configuration(buffer_index(current)), c, environment))
                    {
                        propose(TraceOutcome::Collision, -1);
                        return;
                    }

                    const auto added = add_node(c, current);
                    if (not added)
                    {
                        propose(TraceOutcome::TreeFull, -1);
                        return;
                    }

                    propose(TraceOutcome::Accepted, static_cast<std::int32_t>(*added));
                    current = *added;
                    q = next;
                    block = candidate;
                    error = next_error;
                }

                if (error <= tolerance)
                {
                    solution = current;
                }
            };

            // Straight-line extension of the nearest node toward a random sample; returns the
            // node added, if any.
            const auto explore = [&]() -> std::optional<std::size_t>
            {
                auto sample = rng->next();
                record(TraceEvent::Sample, TraceOutcome::None, -1, -1, nan, nan, lane_of(sample));

                if constexpr (LocalPlanner::projecting)
                {
                    // A sample on the manifold (at its own parameter), so the nearest node is the nearest on it
                    if (settings.project_samples and not lp.project(sample))
                    {
                        record(
                            TraceEvent::ExtendFailed, TraceOutcome::NoImprovement, -1, -1, nan, nan, lane_of(sample));
                        return std::nullopt;
                    }
                }

                typename Traits::StateBuffer sample_array;
                sample.to_array(sample_array.data());

                const auto nearest = tree.nearest(sample_array.data());
                if (not nearest)
                {
                    return std::nullopt;
                }

                const auto [nearest_index, nearest_distance] = *nearest;
                const auto extension = lp.steer(
                    Configuration(buffer_index(nearest_index)),
                    sample,
                    nearest_distance,
                    settings.range,
                    true,
                    environment);

                if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                {
                    record(
                        TraceEvent::ExtendFailed,
                        TraceOutcome::Collision,
                        -1,
                        static_cast<std::int32_t>(nearest_index),
                        nan,
                        nan,
                        lane_of(sample));
                    extend_counter_failed++;
                    // std::cout << extend_counter_failed << ": Steer failed: " << static_cast<int>(extension.status) << " for config "
                            //   << Configuration(buffer_index(nearest_index)) << " toward " << sample
                            //   << std::endl;
                    return std::nullopt;
                }
                extend_counter_succeeded++;
                // std::cout << extend_counter_succeeded << ": Steer succeeded: " << static_cast<int>(extension.status) << " for config "
                        //   << Configuration(buffer_index(nearest_index)) << " toward " << sample
                        //   << std::endl;

                const auto [last, truncated] = insert_chain<Robot, Space>(
                    extension.waypoints,
                    nearest_index,
                    [&](const Configuration &c, std::size_t parent)
                    {
                        const auto added = add_node(c, parent);
                        if (added)
                        {
                            record(
                                TraceEvent::Extend,
                                TraceOutcome::Accepted,
                                static_cast<std::int32_t>(*added),
                                static_cast<std::int32_t>(parent),
                                nan,
                                nan,
                                lane_of(c));
                        }

                        return added;
                    });
                if (truncated or last == nearest_index)
                {
                    return std::nullopt;
                }

                return last;
            };

            add_node(start, 0);
            record(TraceEvent::Root, TraceOutcome::None, 0, -1, nan, nan, lane_of(start));
            walk(0);

            while (not solution and iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                if (const auto added = explore())
                {
                    walk(*added);
                }
            }

            if (solution)
            {
                record(
                    TraceEvent::Solved,
                    TraceOutcome::None,
                    static_cast<std::int32_t>(*solution),
                    -1,
                    nan,
                    nan,
                    load(*solution));

                auto current = *solution;
                result.path.emplace_back(buffer_index(current));
                while (parents[current] != current)
                {
                    const auto parent = parents[current];
                    result.path.emplace_back(buffer_index(parent));
                    result.cost += Traits::distance(
                        result.path[result.path.size() - 1], result.path[result.path.size() - 2]);
                    current = parent;
                }

                std::reverse(result.path.begin(), result.path.end());
                result.solved = true;
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(tree.size());
            return result;
        }
    };
}  // namespace vamp::planning
