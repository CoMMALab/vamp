#pragma once

#include <limits>
#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/tortle_nn.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/tortle_settings.hh>
#include <vamp/planning/rrtctopp.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/planning/bezier.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOX_TORTLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        using NNNode = TDNATNode<Robot, dimension>;
        using NN = NearestNeighborsTDNAT<Robot, NNNode>;

        std::unique_ptr<float, decltype(&free)> buffer;
        std::vector<std::size_t> parents;
        // add buffer to store times and dists
        std::vector<float> radii;
        // std::vector<float> times;
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
            // times[index] = std::numeric_limits<float>::max();
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
        inline auto find_nearest(NN *nn, const NNNode &root, const Configuration &c, float cost, bool reverse)
            -> std::pair<NNNode, float>
        {
            std::vector<NNNode> near_list;

            // Almost always just pulls in the entire graph, but good to be principled.
            near_list.reserve(nn->size());

            auto temp_node = NNNode{0, cost, c};
            nn->nearestR(temp_node, NNNode::distance(temp_node, root), near_list);

            const auto *new_nearest_node = &near_list[0];
            float new_nearest_distance = reverse ? NNNode::distance(temp_node, *new_nearest_node) : NNNode::distance(*new_nearest_node, temp_node); // change back to nn_time if distance is not nntime already

            for (auto idx = 1U; new_nearest_node->cost > 0                                //
                                and cost < new_nearest_node->cost + new_nearest_distance  //
                                and idx < near_list.size();
                 ++idx)
            {
                new_nearest_node = &near_list[idx];
                new_nearest_distance = reverse ? NNNode::distance(temp_node, *new_nearest_node) : NNNode::distance(*new_nearest_node, temp_node);
            }

            return {*new_nearest_node, new_nearest_distance};
        }

        inline auto find_nearest_k(NN *nn, const NNNode &root, const Configuration &c, float cost, bool reverse, int k)
            -> std::vector<std::pair<NNNode, float>>
        {
            std::vector<NNNode> near_list;
            std::vector<std::pair<NNNode, float>> near_list_with_costs;

            // Almost always just pulls in the entire graph, but good to be principled.
            near_list.reserve(nn->size());

            auto temp_node = NNNode{0, cost, c};
            nn->nearestR(temp_node, NNNode::distance(temp_node, root), near_list);

            for (auto idx = 1U; idx < std::min(k, static_cast<int>(near_list.size())); ++idx)
            {
                const auto *temp_near_node = &near_list[idx];
                float distance = reverse ? NNNode::distance(temp_node, *temp_near_node) : NNNode::distance(*temp_near_node, temp_node);
                near_list_with_costs.push_back({*temp_near_node, distance});
            }

            return near_list_with_costs;
        }

        AOX_TORTLE(std::size_t max_samples)
          : buffer(
                std::unique_ptr<float, decltype(&free)>(
                    vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                        max_samples * Configuration::num_scalars_rounded),
                    &free))
        {
            parents.resize(max_samples);
            radii.resize(max_samples);
            // times.resize(max_samples);
            costs.resize(max_samples);
        }

        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TORTLESettings &settings,
            const float max_cost,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            static constexpr std::size_t start_index = 0;
            const RRTCSettings &rrtc_settings = settings.rrtc;
            PlanningResult<Robot> result;

            NN start_tree;
            NN goal_tree;

            // map indices to beziers
            std::map<std::size_t, Bezier> bezier_map;

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
            bool tree_a_is_start = not rrtc_settings.start_tree_first;
            auto *tree_a = (rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            // Search loop
            while (iter++ < rrtc_settings.max_iterations and free_index < rrtc_settings.max_samples)
            {
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                // Balanced RRTC
                if ((not rrtc_settings.balance) or ratio < rrtc_settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start; // BE CAREFUL
                }

                const auto temp = rng->next();

                NNNode goal_vert = *std::min_element(
                    goal_verts.begin(),
                    goal_verts.end(),
                    [&temp](const auto &a, const auto &b)
                    { return Robot::template get_nn_time(temp, a.array) < Robot::template get_nn_time(temp, b.array); });

                const auto &root_vert = tree_a_is_start ? start_vert : goal_vert;
                const auto &target_vert = tree_a_is_start ? goal_vert : start_vert;

                // computue cost sample bounds
                const float g_hat = not tree_a_is_start ? Robot::template get_nn_time(temp, root_vert.array) : Robot::template get_nn_time(root_vert.array, temp);
                const float h_hat = not tree_a_is_start ? Robot::template get_nn_time(temp, target_vert.array) : Robot::template get_nn_time(target_vert.array, temp);
                const float f_hat = g_hat + h_hat;

                // The range between the minimum possible cost and maximum allowable cost
                // - Floating point error can result in a (barely) negative range
                // - If c_range is 0, only valid connection is to root of tree
                //   (sampled upper cost bound == g^)
                const float c_range = std::max(max_cost - f_hat, 0.0F);

                // Sampled upper cost bound
                const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                // Find nearest with asymmetric cost function
                // MODIFY FIND NEAREST
                auto [nearest_node, nearest_distance] = find_nearest(tree_a, root_vert, temp, c_rand, not tree_a_is_start);

                const auto nearest_vector = temp - nearest_node.array;
                auto new_node = temp;
                                
                // bool, Bezier
                auto [valid_extension, sub_bez] = tree_a_is_start ? 
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                                                    nearest_node.array,
                                                                    new_node,
                                                                    environment,
                                                                    settings.bez_range) :
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                                                    new_node,
                                                                    nearest_node.array,
                                                                    environment,
                                                                    settings.bez_range);

                if (valid_extension)
                {
                    // store end of sub bez
                    const auto new_configuration = new_node;

                    // Calculate and store actual node cost
                    // REPLACE WITH NN INFERENCE
                    auto new_cost = tree_a_is_start ? nearest_node.cost + Robot::template get_nn_time(nearest_node.array, new_configuration) :
                                                     nearest_node.cost + Robot::template get_nn_time(new_configuration, nearest_node.array);
                    // float new_cost = nearest_node.cost + sub_bez.time;

                    // If resampling costs to try and find a better parent...
                    if (settings.cost_bound_resample)
                    {
                        const float g_hat = tree_a_is_start ? Robot::template get_nn_time(root_vert.array, new_configuration) :
                                                             Robot::template get_nn_time(new_configuration, root_vert.array);

                        // Continuously resample cost until an invalid connection is found
                        for (auto i = 0U; i < settings.max_cost_bound_resamples; ++i)
                        {
                            const float c_range = std::max(new_cost - g_hat, 0.0F);
                            const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                            auto [new_nearest_node, new_nearest_distance] =
                                find_nearest(tree_a, root_vert, new_configuration, c_rand, not tree_a_is_start);


                            // Validate edge to newly found parent
                            auto [valid_resample, sub_bez_resample] = tree_a_is_start ? 
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                            new_nearest_node.array,
                                            new_configuration,
                                            environment,
                                            settings.bez_range) : 
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                            new_configuration,
                                            new_nearest_node.array,
                                            environment,
                                            settings.bez_range);
                            
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
                            else if (valid_resample)
                            {
                                // Congratulations to the new parent
                                nearest_node = new_nearest_node;
                                new_cost = new_nearest_node.cost + sub_bez_resample.time;
                                // Store the new bezier
                                sub_bez = sub_bez_resample;
                            }
                            // The edge is invalid, we have failed a connection. Stop resampling!
                            else
                            {
                                break;
                            }
                        }
                    }

                    // Need to add the end of the sub bezier to the tree, not the original new node
                    // compute higher order terms of new node
                    Bezier dsub_bez = sub_bez.derivative();
                    Bezier ddsub_bez = dsub_bez.derivative();

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
                    add_to_tree(tree_a, new_configuration_bez, free_index, nearest_node.index, new_cost);
                    bezier_map[free_index] = sub_bez;
                    free_index++;

                    // Connect trees:
                    // Attempt direct connections from new node to other tree, similar to rrt+
                    std::vector<std::pair<NNNode, float>> near_nodes = find_nearest_k(tree_b, target_vert, new_configuration_bez, max_cost - new_cost, tree_a_is_start, settings.k_nearest);
                    if (near_nodes.size() == 0) {
                        continue;
                    }
                    // find the minimum feasible connection
                    bool valid_found = false;
                    auto best_connection = near_nodes[0];

                    Bezier best_bez;

                    for (int i = 0; i < near_nodes.size(); i++) {
                        auto [other_nearest_node, other_nearest_distance] = near_nodes[i];
                        if (new_cost + other_nearest_distance + other_nearest_node.cost >= max_cost)
                        {
                            continue;
                        }

                        auto [valid_connection, sub_bez_connection] = tree_a_is_start ? 
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                            new_configuration_bez,
                                            other_nearest_node.array,
                                            environment,
                                            1) : 
                                        validate_sub_bez_motion<Robot, rake, resolution>(
                                            other_nearest_node.array,
                                            new_configuration_bez,
                                            environment,
                                            1);
                        if (valid_connection)
                        {
                            valid_found = true;
                            best_connection = {other_nearest_node, other_nearest_distance};
                            best_bez = sub_bez_connection;
                        }
                    }
                    // solution found, construct path
                    if (valid_found)
                    {
                        Bezier dbest_bez = best_bez.derivative();
                        Bezier ddbest_bez = dbest_bez.derivative();

                        auto new_q = best_bez.anchors.row(best_bez.anchors.rows() - 1);
                        auto new_dq = dbest_bez.anchors.row(dbest_bez.anchors.rows() - 1);
                        auto new_ddq = ddbest_bez.anchors.row(ddbest_bez.anchors.rows() - 1);
                        
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
                        Configuration new_configuration_connect(new_configuration_array);
                        add_to_tree(tree_a, new_configuration_connect, free_index, free_index - 1, new_cost + best_bez.time);
                        bezier_map[free_index] = best_bez;
                        free_index++;

                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        result.beziers.emplace_back(bezier_map[current]);
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.emplace_back(bezier_map[parent]);
                            result.cost += Robot::template get_nn_time(result.path[result.path.size() - 2], result.path[result.path.size() - 1]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        std::reverse(result.beziers.begin(), result.beziers.end());
                        current = best_connection.first.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.emplace_back(bezier_map[parent]);
                            result.cost += Robot::template get_nn_time(result.path[result.path.size() - 1], result.path[result.path.size() - 2]);
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
    struct TORTLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using AOX_TORTLE = typename vamp::planning::AOX_TORTLE<Robot, rake, resolution>;
        using RRTCTOPP = typename vamp::planning::RRTCTOPP<Robot, rake, resolution>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const TORTLESettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TORTLESettings &settings_in,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            // Update the settings for internal searches
            TORTLESettings settings = settings_in;  // make a mutable copy
            const std::size_t &max_samples = settings.max_samples;
            const std::size_t &max_iterations = settings.max_iterations;

            // Configure internal RRTC settings
            RRTCSettings &rrtc_settings = settings.rrtc;
            // rrtc_settings.max_iterations = max_iterations;
            rrtc_settings.max_samples = max_samples;

            PlanningResult<Robot> result;
            float best_path_cost = std::numeric_limits<float>::max();
            std::size_t iters = 0;
            std::size_t runs = 0;

            AOX_TORTLE instance(max_samples);

            do
            {
                // Find an initial solution
                // result = RRTCTOPP::solve(start, goals, environment, rrtc_settings, rng);
                result = instance.solve(start, goals, environment, settings, best_path_cost, rng);
                iters += result.iterations;
            } while (result.path.empty() and iters < settings.max_iterations);
            std::cout << "Initial solution found." << std::endl;
            fflush(stdout);

            // Simplify solution if enabled
            if (settings.simplify_intermediate and not result.path.empty())
            {
                result = simplify<Robot, rake, resolution>(result.path, environment, settings.simplify, rng);
            }

            // Exit early if trivial, unsolved, or not optimizing
            if (not settings.optimize or result.path.empty() or result.path.size() == 2)
            {
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                return result;
            }

            // We have a new best solution
            PlanningResult<Robot> final_result;
            final_result.path = result.path;
            best_path_cost = result.path.time(); // path no longer stores the time, use beziers

            float best_possible_cost = std::numeric_limits<float>::max();
            for (const auto &goal : goals)
            {
                best_possible_cost = std::min(best_possible_cost, 0.1f);
            }

            ProlateHyperspheroid<Robot> phs(start, goals[0]);
            phs.set_transverse_diameter(best_path_cost);

            auto phs_rng = std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, rng);

            // AOX_RRTCTOPP instance(max_samples);

            // If we get close to straight line, just call it.
            // Also handles numerical issues with PHS when too close to straight line...
            // std::cout << "Optimizing" << std::endl;
            while (iters < max_iterations and (best_path_cost - best_possible_cost) > 1e-8 and runs++ < settings.max_runs)
            {
                std::cout << "Refining solution" << std::endl;
                fflush(stdout);
                // Update internal maximum iterations
                rrtc_settings.max_iterations =
                    std::min(settings.max_iterations - iters, settings.max_internal_iterations);
                // std::cout << iters << std::endl;
                // By default, use AORRTC
                if (not settings.anytime)
                {
                    // If there is a single goal, sample with PHS
                    if (settings.use_phs and goals.size() == 1)
                    {
                        result = instance.solve(start, goals, environment, settings, best_path_cost, phs_rng);
                    }
                    else
                    {
                        result = instance.solve(start, goals, environment, settings, best_path_cost, rng);
                    }
                }
                // If anytime, use Anytime RRTC
                else
                {
                    if (settings.use_phs and goals.size() == 1)
                    {
                        result = RRTCTOPP::solve(start, goals, environment, rrtc_settings, phs_rng);
                    }
                    else
                    {
                        result = RRTCTOPP::solve(start, goals, environment, rrtc_settings, rng);
                    }
                }

                iters += result.iterations;

                // If last search found a solution
                if (not result.path.empty())
                {
                    // Simplify
                    if (settings.simplify_intermediate)
                    {
                        result = simplify<Robot, rake, resolution>(
                            result.path, environment, settings.simplify, rng);
                    }

                    // To be safe, ensure new path is actually a better solution
                    if (result.path.time() < best_path_cost)
                    {
                        // Update best solution
                        final_result.path = result.path;
                        best_path_cost = result.path.time();
                        auto best_path_length = result.path.cost();
                        phs_rng->phs.set_transverse_diameter(best_path_length); // this creates optimality problems
                    }
                }
            }

            final_result.iterations = iters;
            final_result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            return final_result;
        }
    };
}  // namespace vamp::planning
