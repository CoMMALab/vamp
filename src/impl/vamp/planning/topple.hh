#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate_bezier.hh>
#include <vamp/planning/topple_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <map>
namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct TOPPLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<dimension> start_tree;
            NN<dimension> goal_tree;

            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parents(settings.max_samples);
            std::map<std::pair<std::size_t, std::size_t>, Bezier> bezier_map;
            std::vector<float> extensions(settings.max_samples);

            std::vector<float> radii(settings.max_samples);

            auto [weights, bias] = Robot::load_matrices();

            auto start_time = std::chrono::steady_clock::now();

            for (const auto &goal : goals)
            {
                auto [valid_extension, sub_bez] = validate_bez_motion<Robot, rake, resolution>(
                    start,
                    goal,
                    environment,
                    1.0,
                    weights,
                    bias);

                if (valid_extension)
                {
                    result.path.emplace_back(start);
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    result.size.emplace_back(1);
                    result.beziers.push_back(sub_bez);

                    return result;
                }
            }

            // trees
            bool tree_a_is_start = not settings.rrtc.start_tree_first;
            auto *tree_a = (settings.rrtc.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (settings.rrtc.start_tree_first) ? &start_tree : &goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNode<dimension>{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            extensions[start_index] = settings.bez_range;
            radii[start_index] = std::numeric_limits<float>::max();

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNode<dimension>{free_index, {buffer_index(free_index)}});
                parents[free_index] = free_index;
                extensions[free_index] = settings.bez_range;
                radii[free_index] = std::numeric_limits<float>::max();
                free_index++;
            }

            float scale = settings.sampling_bias;
            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                if ((not settings.rrtc.balance) or ratio < settings.rrtc.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                // Scale down the vels and accels
                for (auto i = Robot::dimension / 3; i < Robot::dimension; i++)
                {
                    temp_array[i] *= scale;
                }

                // std::cout << "Sampling random configuration: " << temp << std::endl;

                const auto nearest = tree_a->nearest(NNFloatArray<dimension>{temp_array.data()});
                if (not nearest)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                // std::cout << "Nearest node index is : " << nearest_node.index << " for tree " << (tree_a_is_start ? "goal" : "start") << " of size " << tree_a->size() << std::endl;
                const auto nearest_radius = radii[nearest_node.index];

                const auto nearest_configuration = nearest_node.as_vector();

                // std::cout << " --> " << temp << std::endl;

                auto nearest_vector = temp - nearest_configuration;


                // just be super dumb and convert to arrays and work
                std::array<float, Robot::dimension> nearest_vector_array, nearest_configuration_array, extension_vector_array, to_extend_array;
                nearest_vector.to_array(nearest_vector_array.data());
                nearest_configuration.to_array(nearest_configuration_array.data());

                float position_distance = 0.0f;
                for (auto i = 0U; i < Robot::dimension / 3; i++)
                {
                    position_distance += nearest_vector_array[i] * nearest_vector_array[i];
                }
                position_distance = std::sqrt(position_distance);

                if (settings.rrtc.dynamic_domain and nearest_radius < position_distance)
                {
                    continue;
                }

                // std::cout << " --> " << nearest_configuration << " to " << temp << "via " << nearest_vector << std::endl;


                auto extensions_start_time = std::chrono::steady_clock::now();
                bool reach = position_distance < settings.rrtc.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (settings.rrtc.range / position_distance);
                
                extension_vector.to_array(extension_vector_array.data());

                // // only replace the position part of configuration with a fixed range,
                // // and retain the sampled velocity and acceleration
                // typename Robot::ConfigurationBuffer to_extend_array;
                for (auto i = 0U; i < Robot::dimension / 3; i++)
                {
                    // std::cout << "[" << i << "] Adding extension vector: " << extension_vector_array[i] << " to nearest vector: " << nearest_configuration_array[i] << std::endl;
                    to_extend_array[i] = nearest_configuration_array[i] + extension_vector_array[i];
                }
                for (auto i = Robot::dimension / 3; i < Robot::dimension; i++)
                {
                    to_extend_array[i] = temp_array[i];
                }


                auto to_extend = typename Robot::Configuration(to_extend_array.data());
                // std::cout << " <-- " << to_extend << std::endl;
                // std::cout << "Extending for " << extensions[nearest_node.index] << std::endl;
                auto [valid_extension, sub_bez] = validate_bez_motion<Robot, rake, resolution>(
                        nearest_configuration,
                        to_extend,
                        environment,
                        extensions[nearest_node.index],
                        weights,
                        bias);

                if (valid_extension)
                {
                    scale = std::min(1.0f, scale * (1.0f + settings.sampling_alpha));
                    // create new config ending at sub bez
                    Bezier dsub_bez = sub_bez.derivative();
                    Bezier ddsub_bez = dsub_bez.derivative();

                    // check this when reversed
                    auto new_q = sub_bez.anchors.row(sub_bez.anchors.rows() - 1);
                    auto new_dq = dsub_bez.anchors.row(dsub_bez.anchors.rows() - 1);
                    auto new_ddq = ddsub_bez.anchors.row(ddsub_bez.anchors.rows() - 1);
                    
                    // convert to Robot configuration in phase space
                    std::array<float, Robot::dimension> new_configuration_array;
                    
                    for (auto i = 0U; i < Robot::dimension; i++)
                    {
                        if (i < Robot::dimension / 3)
                        {
                            new_configuration_array[i] = new_q(i);
                        }
                        else if (i < 2 * Robot::dimension / 3)
                        {
                            new_configuration_array[i] = new_dq(i - Robot::dimension / 3);
                        }
                        else
                        {
                            new_configuration_array[i] = new_ddq(i - 2 * Robot::dimension / 3);
                        }
                    }
                    Configuration new_configuration_bez(new_configuration_array);

                    float *new_configuration_index = buffer_index(free_index);
                    new_configuration_bez.to_array(new_configuration_index);
                    tree_a->insert(NNNode<dimension>{free_index, {new_configuration_index}});
                    parents[free_index] = nearest_node.index;
                    
                    if (not tree_a_is_start) {
                        sub_bez.reverse();
                    }
                    
                    bezier_map[{nearest_node.index, free_index}] = sub_bez;
                    extensions[free_index] = settings.bez_range;
                    radii[free_index] = std::numeric_limits<float>::max();

                    free_index++;

                    if (settings.dynamic_extension)
                    {
                        extensions[nearest_node.index] *= (1 + settings.alpha);
                        extensions[nearest_node.index] = std::min(extensions[nearest_node.index], 1.0f);
                    }

                    if (settings.rrtc.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + settings.rrtc.alpha);
                    }

                    // Extend to goal tree
                    const auto other_nearest = tree_b->nearest(NNFloatArray<dimension>{new_configuration_index});
                    if (not other_nearest)
                    {
                        continue;
                    }

                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const auto other_nearest_configuration = other_nearest_node.as_vector();

                    // flip v and a of other_nearest_configuration
                    std::array<float, Robot::dimension> other_nearest_array;
                    auto other_nearest_configuration_arr = other_nearest_configuration.to_array();
                    for (auto i = 0; i < Robot::dimension; i++) {
                        if (i < Robot::dimension / 3) {
                            other_nearest_array[i] = other_nearest_configuration_arr[i];
                        }
                        else {
                            other_nearest_array[i] = other_nearest_configuration_arr[i] * -1;
                        }
                    }

                    Configuration other_nearest_corrected(other_nearest_array);

                    const std::size_t n_extensions = 1.0f / extensions[free_index - 1] + 0.5f;

                    auto connection_bez = compute_bez<Robot, rake>(
                        new_configuration_bez,
                        other_nearest_corrected,
                        weights,
                        bias);

                    auto temp_bez = connection_bez;

                    std::size_t i_extension = 0;
                    float alpha_i = 1.0 / n_extensions;

                    // std::cout << n_extensions << std::endl;
                    for (; i_extension < n_extensions and free_index < settings.max_samples; ++i_extension)
                    {
                        // std::cout << "Reached" << std::endl;
                        std::pair<Bezier, Bezier> sub_bez_pair;
                        if (i_extension == n_extensions - 1) {
                            sub_bez_pair = {connection_bez, connection_bez};
                        }
                        else {
                            sub_bez_pair = connection_bez.subdivide(alpha_i);
                        }
                        
                        Bezier sub_bez_l = sub_bez_pair.first;
                        connection_bez = sub_bez_pair.second;

                        if (validate_bez<Robot, rake, resolution>(sub_bez_l, environment) && 
                            validate_dbez<Robot, rake, resolution>(sub_bez_l, sub_bez_l.time) && 
                            validate_ddbez<Robot, rake, resolution>(sub_bez_l, sub_bez_l.time))
                        {
                            Bezier dsub_bez_l = sub_bez_l.derivative();
                            Bezier ddsub_bez_l = dsub_bez_l.derivative();
                            auto sub_end = sub_bez_l.anchors.row(sub_bez_l.anchors.rows() - 1);
                            auto dsub_end = dsub_bez_l.anchors.row(dsub_bez_l.anchors.rows() - 1);
                            auto ddsub_end = ddsub_bez_l.anchors.row(ddsub_bez_l.anchors.rows() - 1);

                            std::array<float, Robot::dimension> sub_connection_end_array;
                            for (auto i = 0U; i < Robot::dimension; i++)
                            {
                                if (i < Robot::dimension / 3)
                                {
                                    sub_connection_end_array[i] = sub_end(i);
                                }
                                else if (i < 2 * Robot::dimension / 3)
                                {
                                    sub_connection_end_array[i] = dsub_end(i - Robot::dimension / 3);
                                }
                                else
                                {
                                    sub_connection_end_array[i] = ddsub_end(i - 2 * Robot::dimension / 3);
                                }
                            }
                            Configuration sub_connection_end_config(sub_connection_end_array);

                            float *next_index = buffer_index(free_index);
                            sub_connection_end_config.to_array(next_index);
                            tree_a->insert(NNNode<dimension>{free_index, {next_index}});
                            parents[free_index] = free_index - 1;

                            if (not tree_a_is_start) {
                                sub_bez_l.reverse();
                            }

                            bezier_map[{free_index - 1, free_index}] = sub_bez_l;
                            extensions[free_index] = settings.bez_range;
                            radii[free_index] = std::numeric_limits<float>::max();
                            free_index++;
                            alpha_i /= (1 - alpha_i);
                        }
                        else
                        {
                            break;
                        }
                    }

                    // std::cout << "Reached connection" << std::endl;
                    if (i_extension == n_extensions)  // connected
                    {
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.push_back(bezier_map[{parent, current}]);
                            result.cost += result.path[result.path.size() - 1].distance(result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        std::reverse(result.beziers.begin(), result.beziers.end());
                        current = other_nearest_node.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.push_back(bezier_map[{parent, current}]);
                            result.cost += result.path[result.path.size() - 1].distance(result.path[result.path.size() - 2]);
                            current = parent;
                        }

                        if (not tree_a_is_start)
                        {
                            std::reverse(result.path.begin(), result.path.end());
                            std::reverse(result.beziers.begin(), result.beziers.end());
                        }

                        break;
                    }
                }
                else
                {
                    // std::cout << "Invalid extension from node index : " << nearest_node.index << " for tree " << (tree_a_is_start ? "goal" : "start") << " from " << nearest_configuration << " towards " << temp << std::endl ;
                    if (settings.dynamic_extension)
                    {
                        // std::cout << "INVALID EXTENSION" << std::endl;
                        extensions[nearest_node.index] *= (1 - settings.alpha);
                        extensions[nearest_node.index] = std::max(extensions[nearest_node.index], 0.01f);
                        // std::cout << "EXTENSION UPDATED: " << extension[nearest_node.index] << std::endl;
                    }

                    if (settings.rrtc.dynamic_domain)
                    {
                        if (nearest_radius == std::numeric_limits<float>::max())
                        {
                            radii[nearest_node.index] = settings.rrtc.radius;
                        }
                        else
                        {
                            radii[nearest_node.index] =
                                std::max(radii[nearest_node.index] * (1.F - settings.rrtc.alpha), settings.rrtc.min_radius);
                        }
                    }
                    scale = std::max(0.0f, scale * (1.0f - settings.sampling_alpha));
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
