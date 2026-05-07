#pragma once

#include <limits>
#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/aox_nn.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/topple_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/planning/bezier.hh>

namespace vamp::planning
{

    struct TreeGrowthData
    {
        enum class TreeSide
        {
            Start,
            Goal
        };

        struct Node
        {
            size_t planner_index;          // node index in planner tree/buffer
            std::vector<float> config;      // configuration space position
            size_t parent_index;            // parent planner index
            size_t iteration;               // planner iteration when this node was added
            std::string loop_type;          // seed | outer_extend | inner_connect
            TreeSide tree_side;             // Start | Goal
        };

        struct FailedExtension
        {
            std::vector<float> config;      // candidate config that failed collision validation
            size_t iteration;               // planner iteration when this failure happened
            std::string loop_type;          // outer_extend | inner_connect
            TreeSide tree_side;             // Start | Goal
        };

        struct BridgeEdge
        {
            size_t from_index;
            size_t to_index;
            size_t iteration;
            std::string loop_type;
            TreeSide tree_side;
        };

        std::vector<Node> nodes;            // all nodes in chronological order
        std::vector<size_t> iteration_starts;  // iteration_starts[i] = index of first node in iteration i
        std::vector<FailedExtension> failed_extensions;  // failed candidate extensions by iteration
        std::vector<BridgeEdge> bridge_edges;  // explicit bridge edges (e.g. final goal connect)
    };    

    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOX_TOPPLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        using NNNode = GNATNode<dimension>;
        using NN = NearestNeighborsGNAT<NNNode>;

        std::unique_ptr<float, decltype(&free)> buffer;
        std::vector<std::size_t> parents;
        // map indices to beziers
        std::map<std::pair<std::size_t, std::size_t>, Bezier> bezier_map;
        // add buffer to store times and dists
        std::vector<float> radii;
        std::vector<float> extension;
        std::vector<float> costs;

        inline auto buffer_index(std::size_t index) -> float *
        {
            return buffer.get() + index * Configuration::num_scalars_rounded;
        };

        inline auto
        add_to_tree(NN *nn, const Configuration &c, std::size_t index, std::size_t parent_index, float cost, float bez_range)
            -> NNNode
        {
            c.to_array(buffer_index(index));

            radii[index] = std::numeric_limits<float>::max();
            extension[index] = bez_range;
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
            float new_nearest_distance = c.distance(new_nearest_node->array); // change back to nn_time if distance is not nntime already

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

        inline auto find_nearest_k(NN *nn, const NNNode &root, const Configuration &c, float cost, int k)
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
                float distance = c.distance(temp_near_node->array);
                near_list_with_costs.push_back({*temp_near_node, distance});
            }

            return near_list_with_costs;
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
            extension.resize(max_samples);
            costs.resize(max_samples);
        }

        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            const float max_cost,
            typename RNG::Ptr rng,
            TreeGrowthData* tree_growth = nullptr) noexcept -> PlanningResult<Robot>
        {
            // profiling variables
            int total_samples = 0;
            int valid_extensions = 0;

            if (tree_growth) {
                tree_growth->nodes.clear();
                tree_growth->iteration_starts.clear();
                tree_growth->failed_extensions.clear();
                tree_growth->bridge_edges.clear();
                tree_growth->iteration_starts.push_back(0);
            }
            
            
            static constexpr std::size_t start_index = 0;
            const RRTCSettings &rrtc_settings = settings.rrtc;
            PlanningResult<Robot> result;

            NN start_tree;
            NN goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            auto start_vert = add_to_tree(&start_tree, start, start_index, start_index, 0, settings.bez_range);

            // Add goals to tree
            std::vector<NNNode> goal_verts;
            goal_verts.reserve(goals.size());

            for (const auto &goal : goals)
            {
                goal_verts.emplace_back(add_to_tree(&goal_tree, goal, free_index, free_index, 0, settings.bez_range));
                free_index++;
            }

            // trees
            bool tree_a_is_start = not rrtc_settings.start_tree_first;
            auto *tree_a = (rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            if (tree_growth) {
                typename Robot::ConfigurationBuffer start_buffer;
                start.to_array(start_buffer.data());
                TreeGrowthData::Node start_node;
                start_node.planner_index = 0;
                start_node.config.assign(start_buffer.begin(), start_buffer.end());
                start_node.parent_index = 0;
                start_node.iteration = 0;
                start_node.loop_type = "seed";
                start_node.tree_side = TreeGrowthData::TreeSide::Start;
                tree_growth->nodes.push_back(start_node);
            }


            // Search loop
            while (iter++ < rrtc_settings.max_iterations and free_index < rrtc_settings.max_samples)
            {
                if (tree_growth && tree_growth->iteration_starts.size() == iter) {
                    tree_growth->iteration_starts.push_back(tree_growth->nodes.size());
                }

                // std::cout << "ITERATION: " << iter << std::endl;
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                // Balanced RRTC
                if ((not rrtc_settings.balance) or ratio < rrtc_settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                // std::cout << "SAMPLE" << std::endl;
                const auto temp = rng->next();
                total_samples++;

                NNNode goal_vert = *std::min_element(
                    goal_verts.begin(),
                    goal_verts.end(),
                    [&temp](const auto &a, const auto &b)
                    { return temp.distance(a.array) < temp.distance(b.array); });

                const auto &root_vert = tree_a_is_start ? start_vert : goal_vert;
                const auto &target_vert = tree_a_is_start ? goal_vert : start_vert;

                // computue cost sample bounds
                const float g_hat = temp.distance(root_vert.array);
                const float h_hat = temp.distance(target_vert.array);
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
                auto [nearest_node, nearest_distance] = find_nearest(tree_a, root_vert, temp, c_rand);

                const auto nearest_vector = temp - nearest_node.array;
                auto new_node = temp;

                auto bez = compute_bez<Robot, rake, resolution>(nearest_node.array, new_node);
                auto sub_bez = compute_sub_bez<Robot, rake, resolution>(bez, extension[nearest_node.index]);
                auto valid_extension = validate_bez<Robot, rake, resolution>(sub_bez, environment);
                                
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

                if (valid_extension)
                {
                    valid_extensions++;
                    // store end of sub bez
                    // const auto new_configuration = new_node;

                    // Calculate and store actual node cost
                    // REPLACE WITH NN INFERENCE
                    auto new_cost = nearest_node.cost + new_configuration_bez.distance(nearest_node.array);

                    add_to_tree(tree_a, new_configuration_bez, free_index, nearest_node.index, new_cost, settings.bez_range);
                    if (not tree_a_is_start) {
                        sub_bez.reverse();
                    }

                    if (tree_growth) {
                        typename Robot::ConfigurationBuffer new_buffer;
                        new_configuration_bez.to_array(new_buffer.data());
                        TreeGrowthData::Node node;
                        node.planner_index = free_index;
                        node.config.assign(new_buffer.begin(), new_buffer.end());
                        node.parent_index = nearest_node.index;
                        node.iteration = iter;
                        node.loop_type = "outer_extend";
                        node.tree_side = tree_a_is_start ? TreeGrowthData::TreeSide::Start : TreeGrowthData::TreeSide::Goal;
                        tree_growth->nodes.push_back(node);
                    }

                    bezier_map[{nearest_node.index, free_index}] = sub_bez;
                    free_index++;

                    // increase extension of valid connection
                    if (settings.dynamic_extension)
                    {
                        extension[nearest_node.index] *= (1 + settings.alpha);
                        extension[nearest_node.index] = std::min(extension[nearest_node.index], 1.0f);
                    }

                    // connect the trees
                    const std::size_t n_extensions = 1 / extension[free_index - 1];
                    std::pair<NNNode, float> near_node = find_nearest(tree_b, target_vert, new_configuration_bez, max_cost - new_cost);
                    
                    // get full bezier connection
                    std::pair<bool, Bezier> connection = validate_sub_bez_motion<Robot, rake, resolution>(
                        new_configuration_bez,
                        near_node.first.array,
                        environment,
                        1);
                    Bezier connection_bez = connection.second;

                    // validate bezier incrementally
                    std::size_t i_extension = 0;
                    std::size_t new_index = free_index - 1;
                    Configuration sub_connection_end_config;
                    float alpha_i = extension[new_index];
                    for (; i_extension < n_extensions and free_index < rrtc_settings.max_samples; ++i_extension)
                    {
                        std::pair<Bezier, Bezier> sub_bez_pair = connection_bez.subdivide(alpha_i);
                        Bezier sub_bez_l = sub_bez_pair.first;
                        connection_bez = sub_bez_pair.second;
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
                        sub_connection_end_config = Configuration(sub_connection_end_array);

                        if (validate_bez<Robot, rake, resolution>(sub_bez_l, environment))
                        {
                            add_to_tree(tree_a, sub_connection_end_config, free_index, free_index - 1, new_cost + sub_bez_l.time, extension[new_index]);
                            if (not tree_a_is_start) {
                                sub_bez_l.reverse();
                            }

                            if (tree_growth) {
                                typename Robot::ConfigurationBuffer next_buffer;
                                sub_connection_end_config.to_array(next_buffer.data());
                                TreeGrowthData::Node node;
                                node.planner_index = free_index;
                                node.config.assign(next_buffer.begin(), next_buffer.end());
                                node.parent_index = free_index - 1;
                                node.iteration = iter;
                                node.loop_type = "inner_connect";
                                node.tree_side = tree_a_is_start ? TreeGrowthData::TreeSide::Start : TreeGrowthData::TreeSide::Goal;
                                tree_growth->nodes.push_back(node);
                            }

                            bezier_map[{free_index - 1, free_index}] = sub_bez_l;
                            free_index++;
                            alpha_i /= (1 - alpha_i);



                        }
                        else
                        {

                            if (tree_growth)
                            {
                                typename Robot::ConfigurationBuffer failed_buffer;
                                sub_connection_end_config.to_array(failed_buffer.data());
                                TreeGrowthData::FailedExtension failed;
                                failed.config.assign(failed_buffer.begin(), failed_buffer.end());
                                failed.iteration = iter;
                                failed.loop_type = "inner_connect";
                                tree_growth->failed_extensions.push_back(std::move(failed));
                            }

                            break;
                        }
                    }
                    if (i_extension == n_extensions)  // connected
                    {
                        if (tree_growth)
                        {
                            typename Robot::ConfigurationBuffer reached_buffer;
                            sub_connection_end_config.to_array(reached_buffer.data());
                            TreeGrowthData::Node node;
                            node.planner_index = free_index;
                            node.config.assign(reached_buffer.begin(), reached_buffer.end());
                            node.parent_index = free_index - 1;
                            node.iteration = iter;
                            node.loop_type = "goal_connect";
                            node.tree_side = tree_a_is_start ? TreeGrowthData::TreeSide::Start : TreeGrowthData::TreeSide::Goal;
                            tree_growth->nodes.push_back(std::move(node));

                            TreeGrowthData::BridgeEdge bridge;
                            bridge.from_index = free_index - 1;
                            bridge.to_index = free_index;
                            bridge.iteration = iter;
                            bridge.loop_type = "goal_connect";
                            bridge.tree_side = tree_a_is_start ? TreeGrowthData::TreeSide::Start : TreeGrowthData::TreeSide::Goal;
                            tree_growth->bridge_edges.push_back(std::move(bridge));
                        }

                        // std::cout << "SOLUTION FOUND" << std::endl;
                        // auto connect_node = best_connection.first.array;
                        // add_to_tree(tree_a, connect_node, free_index, free_index - 1, new_cost + best_bez.time, settings.bez_range);
                        // bezier_map[{free_index - 1, free_index}] = best_bez;
                        auto current = free_index - 1;
                        result.path.emplace_back(buffer_index(current));
                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.push_back(bezier_map[{parent, current}]);
                            result.cost += Robot::template get_nn_time(result.path[result.path.size() - 2], result.path[result.path.size() - 1]);
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        std::reverse(result.beziers.begin(), result.beziers.end());
                        current = near_node.first.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            result.beziers.push_back(bezier_map[{parent, current}]);
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
                else {

                    if (tree_growth)
                    {
                        auto failed_configuration = temp;
                        typename Robot::ConfigurationBuffer failed_buffer;
                        failed_configuration.to_array(failed_buffer.data());
                        TreeGrowthData::FailedExtension failed;
                        failed.config.assign(failed_buffer.begin(), failed_buffer.end());
                        failed.iteration = iter;
                        failed.loop_type = "outer_extend";
                        failed.tree_side = tree_a_is_start ? TreeGrowthData::TreeSide::Start : TreeGrowthData::TreeSide::Goal;
                        tree_growth->failed_extensions.push_back(std::move(failed));
                    }


                    // decrease extension
                    if (settings.dynamic_extension)
                    {
                        // std::cout << "INVALID EXTENSION" << std::endl;
                        extension[nearest_node.index] *= (1 - settings.alpha);
                        extension[nearest_node.index] = std::max(extension[nearest_node.index], 0.01f);
                        // std::cout << "EXTENSION UPDATED: " << extension[nearest_node.index] << std::endl;
                    }
                }
            }
            std::cout << "DONE" << std::endl;
            std::cout << "TOTAL SAMPLES: " << total_samples << std::endl;
            std::cout << "VALID EXTENSIONS: " << valid_extensions << std::endl;
            std::cout << "VALID RATIO: " << 1.0 * valid_extensions / total_samples << std::endl;
            std::cout << "Tree sizes are : " << start_tree.size() << " and " << goal_tree.size() << std::endl;
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
    struct TOPPLE
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using AOX_TOPPLE = typename vamp::planning::AOX_TOPPLE<Robot, rake, resolution>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings,
            typename RNG::Ptr rng,
            TreeGrowthData *tree_growth = nullptr) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng, tree_growth);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const TOPPLESettings &settings_in,
            typename RNG::Ptr rng,
            TreeGrowthData *tree_growth = nullptr) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            // Update the settings for internal searches
            TOPPLESettings settings = settings_in;  // make a mutable copy
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

            AOX_TOPPLE instance(max_samples);

            do
            {
                // Find an initial solution
                result = instance.solve(start, goals, environment, settings, best_path_cost, rng, tree_growth);
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

                // If there is a single goal, sample with PHS
                if (settings.use_phs and goals.size() == 1)
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, phs_rng, tree_growth);
                }
                else
                {
                    result = instance.solve(start, goals, environment, settings, best_path_cost, rng, tree_growth);
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
