#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/nn/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    struct RRTSettings
    {
        float range = 1.;

        // Fraction of iterations that steer toward a fresh goal draw instead of a random sample,
        // scheduled deterministically (an accumulator, not a coin flip).
        float goal_bias = 0.1;

        // A goal steer starts from one of the goal draw's goal_candidates nearest nodes
        // (Euclidean), tried in order of local-path cost (Robot::cost) until one makes progress.
        std::size_t goal_candidates = 4;

        bool dynamic_domain = true;
        float radius = 4.;
        float alpha = 0.0001;
        float min_radius = 1.;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;

        // (1 + epsilon)-approximate nearest-neighbor queries; 0 is exact.
        float nn_epsilon = 0.;
    };

    // Forward-only RRT: one tree grown from the start, every edge steered in execution
    // direction. A goal_bias fraction of iterations draws a goal from `goals` -- a callable
    // returning std::optional<Configuration>, e.g. a goal region's sampler, so goal sets are
    // sampled throughout the search instead of fixed up front -- and steers toward it; reaching
    // a goal member solves the query. There is no second tree, so no edge ever has to join two
    // trees' states.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct RRT
    {
        using Configuration = typename Robot::Configuration;
        using RNG = typename vamp::rng::RNG<Robot>;

        template <typename GoalSampler, typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            GoalSampler &&goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;
            lp.reset_stats();
            std::size_t samples_drawn = 0;

            NN<Robot> tree;
            tree.reserve(settings.max_samples);
            tree.set_epsilon(settings.nn_epsilon);

            auto buffer = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                settings.max_samples * Configuration::num_scalars_rounded);
            auto parents = vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(settings.max_samples);
            auto radii = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(settings.max_samples);
            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            auto start_time = std::chrono::steady_clock::now();

            start.to_array(buffer_index(0));
            tree.insert(0, buffer_index(0));
            parents[0] = 0;
            radii[0] = std::numeric_limits<float>::max();
            std::size_t free_index = 1;

            const auto add_node = [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (free_index >= settings.max_samples)
                {
                    return std::nullopt;
                }

                float *ptr = buffer_index(free_index);
                c.to_array(ptr);
                tree.insert(free_index, ptr);
                parents[free_index] = parent;
                radii[free_index] = std::numeric_limits<float>::max();
                return free_index++;
            };

            std::vector<std::pair<std::size_t, float>> candidates;
            std::vector<std::pair<float, std::size_t>> ranked;
            float goal_accumulator = 0.F;
            std::optional<std::size_t> solution;

            std::size_t iter = 0;
            while (not solution and iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                goal_accumulator += settings.goal_bias;
                if (goal_accumulator >= 1.F)
                {
                    goal_accumulator -= 1.F;
                    const std::optional<Configuration> goal = goals();
                    if (not goal)
                    {
                        continue;
                    }

                    typename Robot::ConfigurationBuffer goal_array;
                    goal->to_array(goal_array.data());
                    tree.nearest(candidates, goal_array.data(), std::max<std::size_t>(settings.goal_candidates, 1));

                    ranked.clear();
                    for (const auto &[index, distance] : candidates)
                    {
                        ranked.emplace_back(cost<Robot>(Configuration(buffer_index(index)), *goal), index);
                    }

                    std::sort(ranked.begin(), ranked.end());
                    for (const auto &[c, index] : ranked)
                    {
                        const Configuration from(buffer_index(index));
                        const auto extension =
                            lp.steer(from, *goal, Robot::distance(from, *goal), settings.range, true, environment);
                        if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                        {
                            continue;
                        }

                        const auto [new_index, truncated] = insert_chain<Robot>(extension.waypoints, index, add_node);
                        if (not truncated and extension.status == SteerStatus::Reached)
                        {
                            solution = new_index;
                        }

                        break;
                    }

                    continue;
                }

                const auto sample = rng->next();
                ++samples_drawn;
                typename Robot::ConfigurationBuffer sample_array;
                sample.to_array(sample_array.data());

                const auto nearest = tree.nearest(sample_array.data());
                if (not nearest)
                {
                    continue;
                }

                const auto [nearest_index, nearest_distance] = *nearest;
                const auto nearest_radius = radii[nearest_index];
                if (settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                const auto extension = lp.steer(
                    Configuration(buffer_index(nearest_index)), sample, nearest_distance, settings.range, true, environment);

                if (extension.status != SteerStatus::Trapped)
                {
                    insert_chain<Robot>(extension.waypoints, nearest_index, add_node);
                    if (settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_index] *= (1 + settings.alpha);
                    }
                }
                else if (settings.dynamic_domain)
                {
                    radii[nearest_index] = (nearest_radius == std::numeric_limits<float>::max()) ?
                                               settings.radius :
                                               std::max(nearest_radius * (1.F - settings.alpha), settings.min_radius);
                }
            }

            if (solution)
            {
                auto current = *solution;
                result.path.emplace_back(buffer_index(current));
                while (parents[current] != current)
                {
                    current = parents[current];
                    result.path.emplace_back(buffer_index(current));
                }

                std::reverse(result.path.begin(), result.path.end());
                result.cost = result.path.cost();
                result.solved = true;
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(tree.size());
            result.stats = lp.stats();
            result.stats.samples_drawn = samples_drawn;
            result.stats.nn_queries = tree.n_queries();
            return result;
        }
    };
}  // namespace vamp::planning
