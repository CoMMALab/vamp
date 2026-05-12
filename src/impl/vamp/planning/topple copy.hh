#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
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
        using Phase = typename Robot::Phase;
        static constexpr auto topple_dimension = Robot::topple_dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Phase &start,
            const Phase &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Phase>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Phase &start,
            const std::vector<Phase> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<topple_dimension> start_tree;
            NN<topple_dimension> goal_tree;

            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Phase::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Phase::num_scalars_rounded; };

            std::vector<std::size_t> parents(settings.max_samples);
            std::map<std::pair<std::size_t, std::size_t>, Bezier> bezier_map;
            std::vector<float> extensions(settings.max_samples);

            std::vector<float> radii(settings.max_samples);

            auto start_time = std::chrono::steady_clock::now();

            for (const auto &goal : goals)
            {
                auto [valid_extension, sub_bez] = validate_sub_bez_motion<Robot, rake, resolution>(
                    start,
                    goal,
                    environment,
                    1.0);

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
            start_tree.insert(NNNode<topple_dimension>{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            extensions[start_index] = 1.0;
            radii[start_index] = std::numeric_limits<float>::max();

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNode<topple_dimension>{free_index, {buffer_index(free_index)}});
                parents[free_index] = free_index;
                extensions[free_index] = 1.0;
                radii[free_index] = std::numeric_limits<float>::max();
                free_index++;
            }

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

                auto temp = rng->next_s();
                typename Robot::PhaseBuffer temp_array;
                temp.to_array(temp_array.data());

                // std::cout << "Sampling random configuration: " << temp << std::endl;

                const auto nearest = tree_a->nearest(NNFloatArray<topple_dimension>{temp_array.data()});
                if (not nearest)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                // std::cout << "Nearest node index is : " << nearest_node.index << " for tree " << (tree_a_is_start ? "goal" : "start") << " of size " << tree_a->size() << std::endl;                // const auto nearest_radius = radii[nearest_node.index];

                // if (settings.dynamic_domain and nearest_radius < nearest_distance)
                // {
                //     continue;
                // }

                const auto nearest_configuration = nearest_node.as_vector();

                // auto nearest_vector = temp - nearest_configuration;

                // bool reach = nearest_distance < settings.range;
                // auto extension_vector =
                //     (reach) ? nearest_vector : nearest_vector * (settings.range / nearest_distance);

                auto [valid_extension, sub_bez] = validate_sub_bez_motion<Robot, rake, resolution>(
                        nearest_configuration,
                        temp,
                        environment,
                        extensions[nearest_node.index]);

                // create new config ending at sub bez
                Bezier dsub_bez = sub_bez.derivative();
                Bezier ddsub_bez = dsub_bez.derivative();

                // check this when reversed
                auto new_q = sub_bez.anchors.row(sub_bez.anchors.rows() - 1);
                auto new_dq = dsub_bez.anchors.row(dsub_bez.anchors.rows() - 1);
                auto new_ddq = ddsub_bez.anchors.row(ddsub_bez.anchors.rows() - 1);
                
                // convert to Robot configuration in phase space
                std::array<float, Robot::topple_dimension> new_configuration_array;
                
                for (auto i = 0U; i < Robot::topple_dimension; i++)
                {
                    if (i < Robot::dimension)
                    {
                        new_configuration_array[i] = new_q(i);
                    }
                    else if (i < 2 * Robot::dimension)
                    {
                        new_configuration_array[i] = new_dq(i - Robot::dimension);
                    }
                    else
                    {
                        new_configuration_array[i] = new_ddq(i - 2 * Robot::dimension);
                    }
                }
                // std::cout << std::endl;
                Phase new_configuration_bez(new_configuration_array);

                if (valid_extension)
                {
                    
                    float *new_configuration_index = buffer_index(free_index);
                    new_configuration_bez.to_array(new_configuration_index);
                    tree_a->insert(NNNode<topple_dimension>{free_index, {new_configuration_index}});
                    // std::cout << "Added new node index : " << free_index << " for tree " << (tree_a_is_start ? "goal" : "start") << " with configuration " << new_configuration_bez << std::endl;

                    if (not tree_a_is_start) {
                        sub_bez.reverse();
                    }

                    parents[free_index] = nearest_node.index;
                    bezier_map[{nearest_node.index, free_index}] = sub_bez;
                    // radii[free_index] = std::numeric_limits<float>::max();

                    free_index++;

                    if (settings.dynamic_extension)
                    {
                        extensions[nearest_node.index] *= (1 + settings.alpha);
                        extensions[nearest_node.index] = std::min(extensions[nearest_node.index], 1.0f);
                    }

                    // Extend to goal tree
                    const auto other_nearest = tree_b->nearest(NNFloatArray<topple_dimension>{new_configuration_index});
                    if (not other_nearest)
                    {
                        continue;
                    }

                    const std::size_t n_extensions = 1 / settings.bez_range + 0.5f;
                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const auto other_nearest_configuration = other_nearest_node.as_vector();

                    auto connection_bez = compute_bez<Robot, rake>(
                        new_configuration_bez,
                        other_nearest_configuration);
                    // std::cout << "Tried to validate connection between " << new_configuration_bez << " and " << other_nearest_configuration << " with bezier " << connection_bez.anchors << std::endl;

                    std::size_t i_extension = 0;
                    // std::size_t new_index = free_index - 1;
                    float alpha_i = settings.bez_range;
                    for (; i_extension < n_extensions and free_index < settings.max_samples; ++i_extension)
                    {
                        std::pair<Bezier, Bezier> sub_bez_pair;
                        if (i_extension == n_extensions - 1) {
                            sub_bez_pair = {connection_bez, connection_bez};
                        }
                        else {
                            sub_bez_pair = connection_bez.subdivide(alpha_i);
                        }
                        
                        Bezier sub_bez_l = sub_bez_pair.first;
                        connection_bez = sub_bez_pair.second;

                        if (validate_bez<Robot, rake, resolution>(sub_bez_l, environment))
                        {
                            Bezier dsub_bez_l = sub_bez_l.derivative();
                            Bezier ddsub_bez_l = dsub_bez_l.derivative();
                            auto sub_end = sub_bez_l.anchors.row(sub_bez_l.anchors.rows() - 1);
                            auto dsub_end = dsub_bez_l.anchors.row(dsub_bez_l.anchors.rows() - 1);
                            auto ddsub_end = ddsub_bez_l.anchors.row(ddsub_bez_l.anchors.rows() - 1);

                            // std::cout << sub_bez_l.anchors <<", " << dsub_bez_l.anchors << " " << ddsub_bez_l.anchors << std::endl;

                            std::array<float, Robot::topple_dimension> sub_connection_end_array;
                            for (auto i = 0U; i < Robot::topple_dimension; i++)
                            {
                                if (i < Robot::dimension)
                                {
                                    sub_connection_end_array[i] = sub_end(i);
                                }
                                else if (i < 2 * Robot::dimension)
                                {
                                    sub_connection_end_array[i] = dsub_end(i - Robot::dimension);
                                }
                                else
                                {
                                    sub_connection_end_array[i] = ddsub_end(i - 2 * Robot::dimension);
                                }
                            }
                            Phase sub_connection_end_config(sub_connection_end_array);
                            float *next_index = buffer_index(free_index);
                            sub_connection_end_config.to_array(next_index);
                            tree_a->insert(NNNode<topple_dimension>{free_index, {next_index}});
                            parents[free_index] = free_index - 1;
                            if (not tree_a_is_start) {
                                sub_bez_l.reverse();
                            }
                            bezier_map[{free_index - 1, free_index}] = sub_bez_l;
                            free_index++;
                            alpha_i /= (1 - alpha_i);
                        }
                        else
                        {
                            break;
                        }
                    }

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

                    // KNN
                    if (settings.rand_connect) {
                        bool connected = false;
                        int tree_size = static_cast<int>(tree_a->size());
                        for (int i = 0; i < static_cast<int>(settings.rand_ratio * tree_size); i++) {
                            int rand = rng->dist.uniform_integer(0, static_cast<int>(tree_a->size()) - 1);
                            auto rand_node_a = tree_a->list()[rand];

                            // std::cout << rand << std::endl;
                            
                            rand = rng->dist.uniform_integer(0, static_cast<int>(tree_b->size()) - 1);
                            auto rand_node_b = tree_b->list()[rand];

                            // std::cout << rand << std::endl;

                            auto connection = validate_sub_bez_motion<Robot, rake, resolution>(
                                rand_node_a.as_vector(),
                                rand_node_b.as_vector(),
                                environment,
                                1);
                            if (connection.first) {
                                connected = true;
                                if (not tree_a_is_start) {
                                    connection.second.reverse();
                                }
                                bezier_map[{rand_node_a.index, rand_node_b.index}] = connection.second;

                                auto current = rand_node_a.index;
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

                                current = rand_node_b.index;
                                result.path.emplace_back(buffer_index(current));
                                result.beziers.push_back(bezier_map[{rand_node_a.index, rand_node_b.index}]);
                                result.cost += result.path[result.path.size() - 1].distance(
                                        result.path[result.path.size() - 2]);
                                
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
                        if (connected) {
                            break;
                        }
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
