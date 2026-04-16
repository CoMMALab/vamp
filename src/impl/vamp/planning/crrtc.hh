#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/planning/constraints/constraint_settings.hh>
#include <vamp/planning/constraints/crrtc_settings.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    struct CRRTC
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const CRRTCSettings &crrtc_settings,
            vamp::planning::constraint::ComposableConstraints<Robot, rake, Constraints...> &constraint,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(
                start, std::vector<Configuration>{goal}, environment, crrtc_settings, constraint, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const CRRTCSettings &crrtc_settings,
            vamp::planning::constraint::ComposableConstraints<Robot, rake, Constraints...> &constraint,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<dimension> start_tree;
            NN<dimension> goal_tree;
            std::vector<Configuration> projected_vector;

            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    crrtc_settings.rrtc_settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parents(crrtc_settings.rrtc_settings.max_samples);
            std::vector<float> radii(crrtc_settings.rrtc_settings.max_samples);

            auto start_time = std::chrono::steady_clock::now();

            for (const auto &goal : goals)
            {
                if (vamp::planning::constraint::project_constraint_motion<Robot, rake, resolution>(
                        start,
                        goal,
                        projected_vector,
                        constraint,
                        environment,
                        static_cast<vamp::planning::constraint::ProjMethod>(
                            crrtc_settings.constraint_settings.projection_method),
                        crrtc_settings.constraint_settings.descend_rate,
                        crrtc_settings.constraint_settings.num_projection_iterations,
                        crrtc_settings.constraint_settings.insert_all_to_tree))
                {
                    if ((projected_vector.back() - goal).l2_norm() < 0.001F)
                    {
                        result.path.emplace_back(start);
                        result.path.emplace_back(goal);
                        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                        result.iterations = 0;
                        result.size.emplace_back(1);
                        result.size.emplace_back(1);

                        return result;
                    }
                }
            }

            // trees
            bool tree_a_is_start = not crrtc_settings.rrtc_settings.start_tree_first;
            auto *tree_a = (crrtc_settings.rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (crrtc_settings.rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNode<dimension>{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            radii[start_index] = std::numeric_limits<float>::max();

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNode<dimension>{free_index, {buffer_index(free_index)}});
                parents[free_index] = free_index;
                radii[free_index] = std::numeric_limits<float>::max();
                free_index++;
            }

            bool connected = false;
            while (iter++ < crrtc_settings.rrtc_settings.max_iterations and
                   free_index < crrtc_settings.rrtc_settings.max_samples and not connected)
            {
                // if (iter % 1 == 0)
                //     std::cout << "Starting iteration : " << iter << ", " << free_index << ", " <<connected
                //     << " , " << settings.max_samples << std::endl;
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                if ((not crrtc_settings.rrtc_settings.balance) or
                    ratio < crrtc_settings.rrtc_settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                const auto nearest = tree_a->nearest(NNFloatArray<dimension>{temp_array.data()});
                if (not nearest)
                {
                    std::cout << "No nearest " << std::endl;
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                const auto nearest_radius = radii[nearest_node.index];

                // std::cout << nearest_radius << ", " << nearest_distance << ", " <<  nearest_node.index <<
                // std::endl;
                if (crrtc_settings.rrtc_settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                const auto nearest_configuration = nearest_node.as_vector();

                auto nearest_vector = temp - nearest_configuration;

                bool reach = nearest_distance < crrtc_settings.rrtc_settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector :
                              nearest_vector * (crrtc_settings.rrtc_settings.range / nearest_distance);
                if (vamp::planning::constraint::project_constraint_vector<Robot, rake, resolution>(
                        nearest_configuration,
                        extension_vector,
                        (reach) ? nearest_distance : crrtc_settings.rrtc_settings.range,
                        projected_vector,
                        constraint,
                        environment,
                        static_cast<vamp::planning::constraint::ProjMethod>(
                            crrtc_settings.constraint_settings.projection_method),
                        crrtc_settings.constraint_settings.descend_rate,
                        crrtc_settings.constraint_settings.num_projection_iterations,
                        crrtc_settings.constraint_settings.std_dev_scaling_factor,
                        crrtc_settings.constraint_settings.insert_all_to_tree))

                {
                    float *new_configuration_index =
                        nullptr;  // Initialize to prevent use of uninitialized variable
                    Configuration new_configuration;
                    auto parent_index = nearest_node.index;

                    for (auto proj_vector : projected_vector)
                    {
                        if (free_index >= crrtc_settings.rrtc_settings.max_samples)
                        {
                            break;  // Prevent buffer overflow
                        }

                        new_configuration_index = buffer_index(free_index);
                        new_configuration = proj_vector;
                        new_configuration.to_array(new_configuration_index);
                        tree_a->insert(NNNode<dimension>{free_index, {new_configuration_index}});

                        parents[free_index] = parent_index;
                        radii[free_index] = std::numeric_limits<float>::max();

                        if (crrtc_settings.rrtc_settings.dynamic_domain and
                            nearest_radius != std::numeric_limits<float>::max())
                        {
                            radii[parent_index] *= (1 + crrtc_settings.rrtc_settings.alpha);
                        }

                        parent_index = free_index;
                        free_index++;
                    }

                    if (new_configuration_index == nullptr)
                    {
                        continue;  // Skip if no configurations were added
                    }

                    auto prior = new_configuration;
                    auto prior_index = new_configuration_index;
                    const auto other_nearest =
                        tree_b->nearest(NNFloatArray<dimension>{new_configuration_index});
                    if (not other_nearest)
                    {
                        continue;
                    }
                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const std::size_t n_extensions = static_cast<std::size_t>(
                        std::ceil(other_nearest_distance / crrtc_settings.rrtc_settings.range));
                    auto max_connect_attempts = 2 * n_extensions;

                    auto counter = 0U;

                    // try to connect to goal directly
                    while (not connected)
                    {
                        counter++;
                        if (counter > max_connect_attempts)
                        {
                            break;
                        }
                        // Extend to goal tree
                        const auto other_nearest = tree_b->nearest(NNFloatArray<dimension>{prior_index});
                        if (not other_nearest)
                        {
                            break;
                        }

                        const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                        const auto other_nearest_configuration = other_nearest_node.as_vector();

                        auto other_nearest_vector = other_nearest_configuration - prior;
                        bool other_reach = other_nearest_distance < crrtc_settings.rrtc_settings.range;

                        auto other_extension_vector =
                            (other_reach) ? other_nearest_vector :
                                            other_nearest_vector *
                                                (crrtc_settings.rrtc_settings.range / other_nearest_distance);

                        if (not vamp::planning::constraint::
                                project_constraint_vector<Robot, rake, resolution>(
                                    prior,
                                    other_extension_vector,
                                    (other_reach) ? other_nearest_distance :
                                                    crrtc_settings.rrtc_settings.range,
                                    projected_vector,
                                    constraint,
                                    environment,
                                    static_cast<vamp::planning::constraint::ProjMethod>(
                                        crrtc_settings.constraint_settings.projection_method),
                                    crrtc_settings.constraint_settings.descend_rate,
                                    crrtc_settings.constraint_settings.num_projection_iterations,
                                    crrtc_settings.constraint_settings.std_dev_scaling_factor,
                                    crrtc_settings.constraint_settings.insert_all_to_tree))
                        {
                            break;
                        }

                        float *next_index = nullptr;  // Initialize to prevent use of uninitialized variable
                        Configuration next;

                        for (auto proj_vector : projected_vector)
                        {
                            if (free_index >= crrtc_settings.rrtc_settings.max_samples)
                            {
                                break;  // Prevent buffer overflow
                            }

                            next_index = buffer_index(free_index);
                            next = proj_vector;
                            next.to_array(next_index);
                            tree_a->insert(NNNode<dimension>{free_index, {next_index}});
                            parents[free_index] = free_index - 1;
                            radii[free_index] = std::numeric_limits<float>::max();
                            free_index++;
                        }

                        if (next_index == nullptr)
                        {
                            continue;  // Skip if no configurations were added
                        }

                        bool other_reached = (other_nearest_configuration - next).squared_l2_norm() < 0.01;
                        if (other_reached)  // connected
                        {
                            auto current = free_index - 1;
                            result.path.emplace_back(buffer_index(current));
                            while (parents[current] != current)
                            {
                                auto parent = parents[current];
                                result.path.emplace_back(buffer_index(parent));
                                result.cost += result.path[result.path.size() - 1].distance(
                                    result.path[result.path.size() - 2]);
                                current = parent;
                            }

                            std::reverse(result.path.begin(), result.path.end());
                            current = other_nearest_node.index;

                            while (parents[current] != current)
                            {
                                auto parent = parents[current];
                                result.path.emplace_back(buffer_index(parent));
                                result.cost += result.path[result.path.size() - 1].distance(
                                    result.path[result.path.size() - 2]);
                                current = parent;
                            }

                            if (not tree_a_is_start)
                            {
                                std::reverse(result.path.begin(), result.path.end());
                            }
                            connected = true;

                            break;
                        }

                        prior = next;
                        prior_index = next_index;
                    }
                }
                else if (crrtc_settings.rrtc_settings.dynamic_domain)
                {
                    if (nearest_radius == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = crrtc_settings.rrtc_settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] = std::max(
                            radii[nearest_node.index] * (1.F - crrtc_settings.rrtc_settings.alpha),
                            crrtc_settings.rrtc_settings.min_radius);
                    }
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
            // settings.display();
            return result;
        }
    };
}  // namespace vamp::planning
