#pragma once

#include <limits>
#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/aotopple_nn.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate_bezier.hh>
#include <vamp/planning/aotopple_settings.hh>
#include <vamp/planning/topple.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOX_TOPPLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        using NNNode = TNATNode<dimension>;
        using NN = NearestNeighborsTNAT<NNNode>;

        std::unique_ptr<float, decltype(&free)> buffer;
        std::vector<std::size_t> parents;
        std::map<std::pair<std::size_t, std::size_t>, Bezier> bezier_map;
        std::vector<float> radii;
        std::vector<float> costs;

        inline auto buffer_index(std::size_t index) -> float *
        {
            return buffer.get() + index * Configuration::num_scalars_rounded;
        };

        inline auto
        add_to_tree(NN *nn, const Configuration &c, std::size_t index, std::size_t parent_index, float cost)
            -> NNNode
        {
            c.to_array(buffer_index(index));

            radii[index] = std::numeric_limits<float>::max();
            parents[index] = parent_index;
            costs[index] = cost;

            auto node = NNNode{index, cost, c};
            nn->add(node);

            return node;
        };

        // Get r-disc neighbours, then iterate through list until a valid connection is found
        // Necessary workaround given asymmetric cost function
        //* ------------------ ------ -------------------
        // Only need to check nodes that are closer than the root of the tree, since connecting to the
        // root will always be valid
        inline auto find_nearest(NN *nn, const NNNode &root, const Configuration &c, float cost)
            -> std::pair<NNNode, float>
        {
            std::vector<NNNode> near_list;

            // Almost always just pulls in the entire graph, but good to be principled.
            near_list.reserve(nn->size());

            auto temp_node = NNNode{0, cost, c};
            nn->nearestR(temp_node, NNNode::distance(temp_node, root), near_list);

            const auto *new_nearest_node = &near_list[0];
            float new_nearest_distance = c.distance(new_nearest_node->array);

            for (auto idx = 1U; new_nearest_node->cost > 0                                //
                                and cost < new_nearest_node->cost + new_nearest_distance  //
                                and idx < near_list.size();
                 ++idx)
            {
                new_nearest_node = &near_list[idx];
                new_nearest_distance = c.distance(new_nearest_node->array);
            }

            return {*new_nearest_node, new_nearest_distance};
        }

        AOX_TOPPLE(std::size_t max_samples)
          : buffer(
                std::unique_ptr<float, decltype(&free)>(
                    vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                        max_samples * Configuration::num_scalars_rounded),
                    &free))
        {
            parents.resize(max_samples);
            radii.resize(max_samples);
            costs.resize(max_samples);
        }

        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AOTOPPLESettings &settings,
            const float max_cost,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            static constexpr std::size_t start_index = 0;
            // const AOTOPPLESettings &rrtc_settings = settings.rrtc;
            PlanningResult<Robot> result;

            NN start_tree;
            NN goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            auto start_vert = add_to_tree(&start_tree, start, start_index, start_index, 0);

            // Add goals to tree
            std::vector<NNNode> goal_verts;
            goal_verts.reserve(goals.size());

            for (const auto &goal : goals)
            {
                goal_verts.emplace_back(add_to_tree(&goal_tree, goal, free_index, free_index, 0));
                free_index++;
            }

            // trees
            bool tree_a_is_start = not settings.rrtc.start_tree_first;
            auto *tree_a = (settings.rrtc.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (settings.rrtc.start_tree_first) ? &start_tree : &goal_tree;

            std::cout << "Reached" << std::endl;
            auto [weights, bias] = Robot::load_matrices();
            fflush(stdout);

            // Search loop
            while (iter++ < settings.rrtc.max_iterations and free_index < settings.rrtc.max_samples)
            {
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                // Balanced RRTC
                if ((not settings.rrtc.balance) or ratio < settings.rrtc.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                const auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());
                // std::cout << "sample" << std::endl;

                NNNode goal_vert = *std::min_element(
                    goal_verts.begin(),
                    goal_verts.end(),
                    [&temp](const auto &a, const auto &b)
                    { return temp.distance(a.array) < temp.distance(b.array); });

                const auto &root_vert = tree_a_is_start ? start_vert : goal_vert;
                const auto &target_vert = tree_a_is_start ? goal_vert : start_vert;


                auto g_hat_bez = compute_bez<Robot, rake>(
                        root_vert.array,
                        temp);

                auto h_hat_bez = compute_bez<Robot, rake>(
                        temp,
                        target_vert.array);

                // const float g_hat = temp.distance(root_vert.array);
                // const float h_hat = temp.distance(target_vert.array);
                const float g_hat = g_hat_bez.time;
                const float h_hat = h_hat_bez.time;
                const float f_hat = g_hat + h_hat;

                // The range between the minimum possible cost and maximum allowable cost
                // - Floating point error can result in a (barely) negative range
                // - If c_range is 0, only valid connection is to root of tree
                //   (sampled upper cost bound == g^)
                const float c_range = std::max(max_cost - f_hat, 0.0F);

                // Sampled upper cost bound
                const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                // Find nearest with asymmetric cost function
                auto [nearest_node, nearest_distance] = find_nearest(tree_a, root_vert, temp, c_rand);

                // Do dynamic domain in q space only
                const auto nearest_radius = radii[nearest_node.index];
                const auto nearest_configuration = nearest_node.array;
                const auto nearest_vector = temp - nearest_configuration;

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

                if (settings.rrtc.dynamic_domain and radii[nearest_node.index] < position_distance)
                {
                    // std::cout << "rejected" << std::endl;
                    continue;
                }

                // do fixed position extend
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

                // const auto nearest_vector = temp - nearest_node.array;
                // bool reach = nearest_distance < rrtc_settings.range;
                // const auto extension_vector =
                //     (reach) ? nearest_vector : nearest_vector * (rrtc_settings.range / nearest_distance);

                auto [valid_extension, sub_bez] = validate_sub_bez_motion<Robot, rake, resolution>(
                        nearest_configuration,
                        to_extend,
                        environment,
                        settings.bez_range);

                // Evaluate edge reaching towards sample
                if (valid_extension)
                {
                    const auto new_configuration = to_extend;

                    // Calculate and store actual node cost
                    // auto new_cost = nearest_node.cost + new_configuration.distance(nearest_node.array);
                    auto new_cost = nearest_node.cost + sub_bez.time;
                    // std::cout << "Cost: " << new_cost << std::endl;

                    // If resampling costs to try and find a better parent...
                    if (settings.cost_bound_resample)
                    {
                        const float g_hat = new_configuration.distance(root_vert.array);

                        // Continuously resample cost until an invalid connection is found
                        for (auto i = 0U; i < settings.max_cost_bound_resamples; ++i)
                        {
                            const float c_range = std::max(new_cost - g_hat, 0.0F);
                            const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                            auto [new_nearest_node, new_nearest_distance] =
                                find_nearest(tree_a, root_vert, new_configuration, c_rand);

                            // If we have connected:
                            //      to the same parent
                            //      with a worse cost
                            //      to the best possible parent before this (crange == 0 \equiv cost == g^)
                            // ...then stop spending effort resampling costs
                            if (new_nearest_node.index == nearest_node.index or
                                new_nearest_node.cost + new_nearest_distance >= new_cost or c_range == 0)
                            {
                                break;
                            }
                            // Validate edge to newly found parent
                            else if (validate_vector<Robot, rake, resolution>(
                                         new_nearest_node.array,
                                         new_configuration - new_nearest_node.array,
                                         new_nearest_distance,
                                         environment))
                            {
                                // Congratulations to the new parent
                                nearest_node = new_nearest_node;
                                new_cost = new_nearest_node.cost + new_nearest_distance;
                            }
                            // The edge is invalid, we have failed a connection. Stop resampling!
                            else
                            {
                                break;
                            }
                        }
                    }

                    // Create new config
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
                    // std::cout << std::endl;
                    Configuration new_configuration_bez(new_configuration_array);

                    add_to_tree(tree_a, new_configuration_bez, free_index, nearest_node.index, new_cost);
                    if (not tree_a_is_start) {
                        sub_bez.reverse();
                    }
                    bezier_map[{nearest_node.index, free_index}] = sub_bez;

                    free_index++;

                    if (settings.rrtc.dynamic_domain and
                        radii[nearest_node.index] != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + settings.rrtc.alpha);
                    }

                    // Extend to goal tree

                    // Because we are extending to the other tree, we need to change our upper cost bound
                    // We need to find a connection that improves upon our current best solution cost
                    // The cost from the root of the other tree to our new vertex, + our new vertex's cost
                    // through the current tree, must be lesser than our maximum path cost
                    // Therefore, our maximum allowable cost for a connection through the other tree is
                    // max_cost - vertex_cost
                    const auto [other_nearest_node, other_nearest_distance] =
                        find_nearest(tree_b, target_vert, new_configuration_bez, max_cost - new_cost);
                    const auto other_nearest_configuration = other_nearest_node.array;
                    // const auto other_nearest_vector = other_nearest_node.array - new_configuration_bez;
                    // if (not other_nearest_node)
                    // {
                    //     continue;
                    // }
                    // Just to be safe, make sure we've improved upon our best solution
                    auto connection_bez = compute_bez<Robot, rake>(
                        new_configuration_bez,
                        other_nearest_configuration);

                    // if (new_cost + other_nearest_distance + other_nearest_node.cost >= max_cost)
                    // {
                    //     continue;
                    // }
                    if (new_cost + connection_bez.time + other_nearest_node.cost >= max_cost)
                    {
                        continue;
                    }

                    const std::size_t n_extensions = 1 / settings.bez_range + 0.5f;
                    // Extend incrementally towards other tree
                    // const std::size_t n_extensions = std::ceil(other_nearest_distance / rrtc_settings.range);
                    // const float increment_length = other_nearest_distance / static_cast<float>(n_extensions);
                    // const auto increment = other_nearest_vector * (1.0F / static_cast<float>(n_extensions));

                    std::size_t i_extension = 0;
                    float alpha_i = settings.bez_range;
                    auto prior = new_configuration;
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
                            float increment_cost = sub_bez_l.time;
                            add_to_tree(
                                tree_a,
                                sub_connection_end_config,
                                free_index,
                                free_index - 1,
                                increment_cost + costs[free_index - 1]);
                            if (not tree_a_is_start) {
                                sub_bez_l.reverse();
                            }
                            bezier_map[{free_index - 1, free_index}] = sub_bez_l;
                            radii[free_index] = std::numeric_limits<float>::max();
                            free_index++;
                            alpha_i /= (1 - alpha_i);
                            prior = sub_connection_end_config;
                        }
                        else {
                            break;
                        }

                    }

                    if (i_extension == n_extensions)  // connected
                    {
                        // std::cout << "Connected" << std::endl;
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.push_back(bezier_map[{parent, current}]);
                            result.cost += result.path[result.path.size() - 1].distance(
                                result.path[result.path.size() - 2]);
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
                            result.cost += result.path[result.path.size() - 1].distance(
                                result.path[result.path.size() - 2]);
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
                else if (settings.rrtc.dynamic_domain)
                {
                    if (nearest_radius == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = settings.rrtc.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] = std::max(
                            radii[nearest_node.index] * (1.F - settings.rrtc.alpha),
                            settings.rrtc.min_radius);
                    }
                }
            }

            result.iterations = iter;
            return result;
        }
    };

    // --------------------------------------------- AOX RRTC Algorithm
    // ---------------------------------------------
    // ==============================================================================================================
    // --------------------------------------------- AOX Meta Algorithm
    // ---------------------------------------------

    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOTOPPLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using AOX_TOPPLE = typename vamp::planning::AOX_TOPPLE<Robot, rake, resolution>;
        using TOPPLE = typename vamp::planning::TOPPLE<Robot, rake, resolution>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const AOTOPPLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AOTOPPLESettings &settings_in,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();
            
            // Update the settings for internal searches
            AOTOPPLESettings settings = settings_in;  // make a mutable copy
            const std::size_t &max_samples = settings.max_samples;
            const std::size_t &max_iterations = settings.max_iterations;

            // Configure internal RRTC settings
            // RRTCSettings &rrtc_settings = settings.rrtc;
            TOPPLESettings topple_settings;
            topple_settings.max_iterations = max_iterations;
            topple_settings.max_samples = max_samples;
            topple_settings.bez_range = settings.bez_range;
            topple_settings.rrtc = settings.rrtc;
            topple_settings.alpha = settings.alpha;
            topple_settings.dynamic_extension = settings.dynamic_extension;
            topple_settings.rand_connect = settings.rand_connect;


            PlanningResult<Robot> result;
            float best_path_cost = std::numeric_limits<float>::max();
            std::size_t iters = 0;

            std::cout << "Solving" << std::endl;
            do
            {
                // Find an initial solution
                result = TOPPLE::solve(start, goals, environment, topple_settings, rng);
                iters += result.iterations;
            } while (result.path.empty() and iters < settings.max_iterations);

            // Simplify solution if enabled
            // if (settings.simplify_intermediate and not result.path.empty())
            // {
            //     result = simplify<Robot, rake, resolution>(result.path, environment, settings.simplify, rng);
            // }

            // Exit early if trivial, unsolved, or not optimizing
            if (not settings.optimize or result.path.empty())
            {
                return result;
            }

            // We have a new best solution
            PlanningResult<Robot> final_result;
            final_result.path = result.path;
            final_result.beziers = result.beziers;
            // compute cost w/ beziers
            float cost = 0;
            for (auto i = 0; i < result.beziers.size(); i++) {
                cost += result.beziers[i].time;
            }
            // best_path_cost = result.path.cost();
            best_path_cost = cost;
            // std::cout << "Initial cost: " << cost << std::endl;

            float best_possible_cost = std::numeric_limits<float>::max();
            for (const auto &goal : goals)
            {
                best_possible_cost = std::min(best_possible_cost, start.distance(goal));
            }

            ProlateHyperspheroid<Robot> phs(start, goals[0]);
            phs.set_transverse_diameter(best_path_cost);

            auto phs_rng = std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, rng);

            AOX_TOPPLE instance(max_samples);

            // If we get close to straight line, just call it.
            // Also handles numerical issues with PHS when too close to straight line...
            // std::cout << "Optimizing" << std::endl;
            while (iters < max_iterations and (best_path_cost - 0) > 1e-8)
            {
                // Update internal maximum iterations
                // std::cout << "Running AOX" << std::endl;
                settings.rrtc.max_iterations =
                    std::min(settings.max_iterations - iters, settings.max_internal_iterations);

                // By default, use AORRTC
                // if (not settings.anytime)
                // {
                    // If there is a single goal, sample with PHS
                if (settings.use_phs and goals.size() == 1)
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, phs_rng);
                }
                else
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, rng);
                }
                // }
                // If anytime, use Anytime RRTC
                // else
                // {
                //     if (settings.use_phs and goals.size() == 1)
                //     {
                //         result = RRTC::solve(start, goals, environment, rrtc_settings, phs_rng);
                //     }
                //     else
                //     {
                //         result = RRTC::solve(start, goals, environment, rrtc_settings, rng);
                //     }
                // }

                iters += result.iterations;

                // If last search found a solution
                if (not result.path.empty())
                {
                    // std::cout << "Found new path" << std::endl;
                    // Simplify
                    // if (settings.simplify_intermediate)
                    // {
                    //     result = simplify<Robot, rake, resolution>(
                    //         result.path, environment, settings.simplify, rng);
                    // }

                    // To be safe, ensure new path is actually a better solution
                    // compute cost w/ beziers
                    cost = 0;
                    for (auto i = 0; i < result.beziers.size(); i++) {
                        cost += result.beziers[i].time;
                    }
                    if (cost < best_path_cost)
                    {
                        // Update best solution
                        final_result.path = result.path;
                        final_result.beziers = result.beziers;
                        // best_path_cost = result.path.cost();
                        best_path_cost = cost;
                        

                        phs_rng->phs.set_transverse_diameter(best_path_cost);
                    }
                }
            }

            final_result.iterations = iters;
            final_result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            return final_result;
        }
    };
}  // namespace vamp::planning
