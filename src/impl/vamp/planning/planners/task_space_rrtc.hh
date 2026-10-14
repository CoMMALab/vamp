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
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct TaskSpaceRRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
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
            TaskSpaceTrace<Robot> *trace = nullptr) noexcept -> PlanningResult<Robot>
        {
            size_t extend_counter_failed = 0;
            size_t extend_counter_succeeded = 0;

            PlanningResult<Robot> result;
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
            // configuration, so every lane holds a copy of it.
            const auto broadcast = [](const Lane &q) -> Block
            {
                Block block;
                for (auto j = 0U; j < dimension; ++j)
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

                    Lane next;
                    float norm2 = 0.F;
                    for (auto j = 0U; j < dimension; ++j)
                    {
                        next[j] = proposed[{j, 0}] - q[j];
                        norm2 += next[j] * next[j];
                    }

                    if (trace)
                    {
                        Lane lm;
                        for (auto j = 0U; j < dimension; ++j)
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
                    for (auto j = 0U; j < dimension; ++j)
                    {
                        next[j] = q[j] + next[j] * clamp;
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
                    if (not Robot::in_bounds(c))
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
                const auto sample = rng->next();
                record(TraceEvent::Sample, TraceOutcome::None, -1, -1, nan, nan, lane_of(sample));

                typename Robot::ConfigurationBuffer sample_array;
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

                const auto [last, truncated] = insert_chain<Robot>(
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
                    result.cost += Robot::distance(
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
