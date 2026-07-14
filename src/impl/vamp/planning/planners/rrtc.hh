#pragma once

#include <memory>
#include <optional>

#include <vamp/collision/environment.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct RRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng, lp);
        }

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<Robot> start_tree;
            NN<Robot> goal_tree;

            constexpr const std::size_t start_index = 0;

            auto buffer = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                settings.max_samples * Configuration::num_scalars_rounded);
            auto parents =
                vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(settings.max_samples);
            auto radii = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(settings.max_samples);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            auto start_time = std::chrono::steady_clock::now();

            constexpr auto unbounded = [] { return std::numeric_limits<float>::infinity(); };

            for (const auto &goal : goals)
            {
                const auto direct = lp.connect_within(
                    start, goal, environment, unbounded, std::numeric_limits<std::size_t>::max());
                if (direct.status == SteerStatus::Reached)
                {
                    result.path.emplace_back(start);
                    for (const auto &c : direct.waypoints)
                    {
                        result.path.emplace_back(c);
                    }
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    result.size.emplace_back(1);

                    return result;
                }
            }

            // trees
            bool tree_a_is_start = not settings.start_tree_first;
            auto *tree_a = (settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (settings.start_tree_first) ? &start_tree : &goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNode<Robot>{start_index, Robot::nn_key(buffer_index(start_index))});
            parents[start_index] = start_index;
            radii[start_index] = std::numeric_limits<float>::max();

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNode<Robot>{free_index, Robot::nn_key(buffer_index(free_index))});
                parents[free_index] = free_index;
                radii[free_index] = std::numeric_limits<float>::max();
                free_index++;
            }

            // Insert one waypoint into tree_a; nullopt when the sample limit is hit.
            const auto add_node =
                [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (free_index >= settings.max_samples)
                {
                    return std::nullopt;
                }

                float *ptr = buffer_index(free_index);
                c.to_array(ptr);
                tree_a->insert(NNNode<Robot>{free_index, Robot::nn_key(ptr)});
                parents[free_index] = parent;
                radii[free_index] = std::numeric_limits<float>::max();
                return free_index++;
            };

            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                if ((not settings.balance) or ratio < settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                const auto nearest = tree_a->nearest(Robot::nn_key(temp_array.data()));
                if (not nearest)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                const auto nearest_radius = radii[nearest_node.index];

                if (settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                const auto nearest_configuration = Configuration(buffer_index(nearest_node.index));

                // Edges are executed from start to goal: goal-tree edges run child -> parent,
                // so the local planner steers and validates along the local path in that
                // direction (identical for symmetric interpolation).
                const auto extension = lp.steer(
                    nearest_configuration,
                    temp,
                    nearest_distance,
                    settings.range,
                    tree_a_is_start,
                    environment);

                if (extension.status != SteerStatus::Trapped)
                {
                    const auto [new_index, extend_truncated] =
                        insert_chain<Robot>(extension.waypoints, nearest_node.index, add_node);

                    if (settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + settings.alpha);
                    }

                    if (extend_truncated)
                    {
                        continue;
                    }

                    // Extend to goal tree
                    const auto other_nearest =
                        tree_b->nearest(Robot::nn_key(buffer_index(new_index)));
                    if (not other_nearest)
                    {
                        continue;
                    }

                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const auto other_nearest_configuration = Configuration(buffer_index(other_nearest_node.index));

                    const std::size_t n_extensions =
                        std::ceil(LocalPlanner::connect_slack * other_nearest_distance / settings.range);

                    std::size_t i_extension = 0;
                    bool connected = false;
                    Configuration prior = extension.endpoint();
                    std::size_t prior_index = new_index;
                    while (not connected and i_extension < n_extensions and
                           free_index < settings.max_samples)
                    {
                        const float remaining =
                            Robot::distance(prior, other_nearest_configuration);
                        const bool final_step =
                            (i_extension == n_extensions - 1) or (remaining <= settings.range);

                        const auto step = lp.steer(
                            prior,
                            other_nearest_configuration,
                            remaining,
                            (final_step) ? std::numeric_limits<float>::max() : settings.range,
                            tree_a_is_start,
                            environment);

                        if (step.status == SteerStatus::Trapped)
                        {
                            break;
                        }

                        const auto [step_index, step_truncated] =
                            insert_chain<Robot>(step.waypoints, prior_index, add_node);
                        if (step_truncated)
                        {
                            break;
                        }

                        connected = (step.status == SteerStatus::Reached);
                        i_extension++;
                        prior = step.endpoint();
                        prior_index = step_index;
                    }

                    if (connected)
                    {
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.cost += Robot::distance(
                                result.path[result.path.size() - 1],
                                result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        current = other_nearest_node.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.cost += Robot::distance(
                                result.path[result.path.size() - 1],
                                result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        if (not tree_a_is_start)
                        {
                            std::reverse(result.path.begin(), result.path.end());
                        }

                        break;
                    }
                }
                else if (settings.dynamic_domain)
                {
                    if (nearest_radius == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] =
                            std::max(radii[nearest_node.index] * (1.F - settings.alpha), settings.min_radius);
                    }
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
            return result;
        }
    };
}  // namespace vamp::planning
