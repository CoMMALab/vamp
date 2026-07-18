#pragma once

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <optional>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn/gnat.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/aorrtc_settings.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/random/pinned.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct AOX_RRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        using NNNode = GNATNode<dimension>;
        using NN = NearestNeighborsGNAT<NNNode>;

        vamp::utils::buffer_ptr<float> buffer;
        vamp::utils::buffer_ptr<std::size_t> parents;
        vamp::utils::buffer_ptr<float> radii;
        vamp::utils::buffer_ptr<float> costs;

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

        struct NearestResult
        {
            NNNode node;
            float distance;   // configuration-space distance (steering / dynamic domain / extension)
            float edge_cost;  // directed edge cost between the tree node and c (execution order)
        };

        // Get r-disc neighbours, then iterate through list until a valid connection is found
        // Necessary workaround given asymmetric cost function
        //* ------------------ ------ -------------------
        // Only need to check nodes that are closer than the root of the tree, since connecting to the
        // root will always be valid
        inline auto find_nearest(
            NN *nn,
            const NNNode &root,
            const Configuration &c,
            float cost_bound,
            bool tree_is_start) -> NearestResult
        {
            std::vector<NNNode> near_list;

            // Almost always just pulls in the entire graph, but good to be principled.
            near_list.reserve(nn->size());

            auto temp_node = NNNode{0, cost_bound, c};
            nn->nearestR(temp_node, NNNode::distance(temp_node, root), near_list);

            // Start-tree edges execute node -> c; goal-tree edges execute c -> node
            const auto edge_cost = [&c, tree_is_start](const NNNode &node) -> float
            {
                return (tree_is_start) ? planning::cost<Robot>(node.array, c) :
                                         planning::cost<Robot>(c, node.array);
            };

            // Explicitly handle case where no neighbors are within r distance.
            if (near_list.empty())
            {
                return {root, Robot::distance(c, root.array), edge_cost(root)};
            }

            const auto *new_nearest_node = &near_list[0];
            float new_nearest_cost = edge_cost(*new_nearest_node);

            for (auto idx = 1U; new_nearest_node->cost > 0                            //
                                and cost_bound < new_nearest_node->cost + new_nearest_cost  //
                                and idx < near_list.size();
                 ++idx)
            {
                new_nearest_node = &near_list[idx];
                new_nearest_cost = edge_cost(*new_nearest_node);
            }

            return {*new_nearest_node, Robot::distance(c, new_nearest_node->array), new_nearest_cost};
        }

        AOX_RRTC(std::size_t max_samples)
          : buffer(vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                max_samples * Configuration::num_scalars_rounded))
          , parents(vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(max_samples))
          , radii(vamp::utils::buffer_alloc<float, FloatVectorAlignment>(max_samples))
          , costs(vamp::utils::buffer_alloc<float, FloatVectorAlignment>(max_samples))
        {
        }

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
            const float max_cost,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            static constexpr std::size_t start_index = 0;
            const RRTCSettings &rrtc_settings = settings.rrtc;
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
            bool tree_a_is_start = not rrtc_settings.start_tree_first;
            auto *tree_a = (rrtc_settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (rrtc_settings.start_tree_first) ? &start_tree : &goal_tree;

            // Directed cost of a tree edge from parent to child: start-tree edges execute
            // parent -> child, goal-tree edges child -> parent.
            const auto chain_edge_cost =
                [&tree_a_is_start](const Configuration &parent, const Configuration &child) -> float
            {
                return (tree_a_is_start) ? planning::cost<Robot>(parent, child) :
                                           planning::cost<Robot>(child, parent);
            };

            // Insert one waypoint into tree_a with its accumulated directed cost; nullopt
            // when the sample limit is hit.
            const auto add_node =
                [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (free_index >= rrtc_settings.max_samples)
                {
                    return std::nullopt;
                }

                const float c_cost =
                    costs[parent] + chain_edge_cost(Configuration(buffer_index(parent)), c);
                add_to_tree(tree_a, c, free_index, parent, c_cost);
                return free_index++;
            };

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
                    tree_a_is_start = not tree_a_is_start;
                }

                const auto temp = rng->next();

                NNNode goal_vert = *std::min_element(
                    goal_verts.begin(),
                    goal_verts.end(),
                    [&temp](const auto &a, const auto &b) {
                        return planning::cost<Robot>(temp, a.array) <
                               planning::cost<Robot>(temp, b.array);
                    });

                const auto &root_vert = tree_a_is_start ? start_vert : goal_vert;
                const auto &target_vert = tree_a_is_start ? goal_vert : start_vert;

                // Admissible cost-to-come/cost-to-go through temp, in execution direction: the
                // start-tree root precedes temp, the goal-tree root follows it
                const float g_hat = (tree_a_is_start) ? planning::cost<Robot>(root_vert.array, temp) :
                                                        planning::cost<Robot>(temp, root_vert.array);
                const float h_hat = (tree_a_is_start) ? planning::cost<Robot>(temp, target_vert.array) :
                                                        planning::cost<Robot>(target_vert.array, temp);
                const float f_hat = g_hat + h_hat;

                // The range between the minimum possible cost and maximum allowable cost
                // - Floating point error can result in a (barely) negative range
                // - If c_range is 0, only valid connection is to root of tree
                //   (sampled upper cost bound == g^)
                const float c_range = std::max(max_cost - f_hat, 0.0F);

                // Sampled upper cost bound
                const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                // Find nearest with asymmetric cost function
                auto [nearest_node, nearest_distance, nearest_edge_cost] =
                    find_nearest(tree_a, root_vert, temp, c_rand, tree_a_is_start);
                if (rrtc_settings.dynamic_domain and radii[nearest_node.index] < nearest_distance)
                {
                    continue;
                }

                // Edges are executed from start to goal: goal-tree edges run child -> parent, so
                // steer and validate along the local path in that direction (identical for
                // symmetric interpolation).
                const auto extension = lp.steer(
                    nearest_node.array,
                    temp,
                    nearest_distance,
                    rrtc_settings.range,
                    tree_a_is_start,
                    environment);

                // Evaluate edge reaching towards sample
                if (extension.status != SteerStatus::Trapped)
                {
                    const bool reach = (extension.status == SteerStatus::Reached);
                    const auto &waypoints = extension.waypoints;

                    // Calculate and store actual node cost (directed, mirroring the steer);
                    // reuse the precomputed edge cost when the sample itself was reached
                    auto new_cost = nearest_node.cost + ((reach and waypoints.size() == 1) ?
                                                             nearest_edge_cost :
                                                             chain_edge_cost(
                                                                 nearest_node.array, waypoints.front()));

                    // If resampling costs to try and find a better parent... (re-parenting is
                    // only sound when tree edges are straight lines, not projected chains)
                    if constexpr (not LocalPlanner::projecting)
                    {
                        if (settings.cost_bound_resample)
                        {
                            const float g_hat =
                                (tree_a_is_start) ?
                                    planning::cost<Robot>(root_vert.array, waypoints.front()) :
                                    planning::cost<Robot>(waypoints.front(), root_vert.array);

                            // Continuously resample cost until an invalid connection is found
                            for (auto i = 0U; i < settings.max_cost_bound_resamples; ++i)
                            {
                                const float c_range = std::max(new_cost - g_hat, 0.0F);
                                const float c_rand = (rng->dist.uniform_01() * c_range) + g_hat;

                                auto [new_nearest_node, new_nearest_distance, new_nearest_edge_cost] =
                                    find_nearest(
                                        tree_a, root_vert, waypoints.front(), c_rand, tree_a_is_start);

                                // If we have connected:
                                //      to the same parent
                                //      with a worse cost
                                //      to the best possible parent before this
                                //      (crange == 0 \equiv cost == g^)
                                // ...then stop spending effort resampling costs
                                if (new_nearest_node.index == nearest_node.index or
                                    new_nearest_node.cost + new_nearest_edge_cost >= new_cost or
                                    c_range == 0)
                                {
                                    break;
                                }
                                // Validate edge to newly found parent
                                else if (lp.validate(
                                             new_nearest_node.array,
                                             waypoints.front(),
                                             environment,
                                             tree_a_is_start))
                                {
                                    // Congratulations to the new parent
                                    nearest_node = new_nearest_node;
                                    new_cost = new_nearest_node.cost + new_nearest_edge_cost;
                                }
                                // The edge is invalid, we have failed a connection. Stop resampling!
                                else
                                {
                                    break;
                                }
                            }
                        }
                    }

                    // The first waypoint's edge cost is precomputed above (and possibly
                    // re-parented); the remainder of the chain accumulates through add_node.
                    add_to_tree(tree_a, waypoints.front(), free_index, nearest_node.index, new_cost);
                    free_index++;

                    if (rrtc_settings.dynamic_domain and
                        radii[nearest_node.index] != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + rrtc_settings.alpha);
                    }

                    const auto [new_index, extend_truncated] =
                        insert_chain(waypoints.begin() + 1, waypoints.end(), free_index - 1, add_node);

                    if (extend_truncated)
                    {
                        continue;
                    }

                    new_cost = costs[new_index];

                    // Extend to goal tree

                    // Because we are extending to the other tree, we need to change our upper cost bound
                    // We need to find a connection that improves upon our current best solution cost
                    // The cost from the root of the other tree to our new vertex, + our new vertex's cost
                    // through the current tree, must be lesser than our maximum path cost
                    // Therefore, our maximum allowable cost for a connection through the other tree is
                    // max_cost - vertex_cost
                    const auto [other_nearest_node, other_nearest_distance, other_nearest_edge_cost] =
                        find_nearest(
                            tree_b,
                            target_vert,
                            extension.endpoint(),
                            max_cost - new_cost,
                            not tree_a_is_start);

                    // Just to be safe, make sure we've improved upon our best solution
                    if (new_cost + other_nearest_edge_cost + other_nearest_node.cost >= max_cost)
                    {
                        continue;
                    }

                    // Extend incrementally towards other tree
                    const std::size_t n_extensions = std::ceil(
                        lp.connect_slack * other_nearest_distance / rrtc_settings.range);

                    std::size_t i_extension = 0;
                    bool connected = false;
                    Configuration prior = extension.endpoint();
                    std::size_t prior_index = free_index - 1;
                    while (not connected and i_extension < n_extensions and
                           free_index < rrtc_settings.max_samples)
                    {
                        const float remaining = Robot::distance(prior, other_nearest_node.array);
                        const bool final_step =
                            (i_extension == n_extensions - 1) or (remaining <= rrtc_settings.range);

                        const auto step = lp.steer(
                            prior,
                            other_nearest_node.array,
                            remaining,
                            (final_step) ? std::numeric_limits<float>::max() : rrtc_settings.range,
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
                            current = parent;
                        }

                        std::reverse(result.path.begin(), result.path.end());
                        current = other_nearest_node.index;

                        while (parents[current] != current)
                        {
                            auto parent = parents[current];
                            result.path.emplace_back(buffer_index(parent));
                            current = parent;
                        }

                        if (not tree_a_is_start)
                        {
                            std::reverse(result.path.begin(), result.path.end());
                        }

                        // Path is now in execution order; cost is directed-edge sum
                        result.cost = result.path.cost();
                        result.solved = true;
                        break;
                    }
                }
                else if (rrtc_settings.dynamic_domain)
                {
                    if (radii[nearest_node.index] == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = rrtc_settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] = std::max(
                            radii[nearest_node.index] * (1.F - rrtc_settings.alpha),
                            rrtc_settings.min_radius);
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
    struct AORRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;
        using AOX_RRTC = typename vamp::planning::AOX_RRTC<Robot, rake, resolution>;
        using RRTC = typename vamp::planning::RRTC<Robot, rake, resolution>;

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const AORRTCSettings &settings,
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
            const AORRTCSettings &settings_in,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            auto start_time = std::chrono::steady_clock::now();

            // Update the settings for internal searches
            AORRTCSettings settings = settings_in;  // make a mutable copy

            // Cap the internal RRTC limits by the total budget without discarding
            // user-set nested values. max_samples in particular must not exceed the
            // top-level value: it bounds the AOX_RRTC buffer allocation below.
            RRTCSettings &rrtc_settings = settings.rrtc;
            rrtc_settings.max_iterations = std::min(rrtc_settings.max_iterations, settings.max_iterations);
            rrtc_settings.max_samples = std::min(rrtc_settings.max_samples, settings.max_samples);

            PlanningResult<Robot> result;
            float best_path_cost = std::numeric_limits<float>::max();
            std::size_t iters = 0;

            do
            {
                // Find an initial solution
                result = RRTC::solve(start, goals, environment, rrtc_settings, rng, lp);
                iters += result.iterations;
            } while (result.path.empty() and iters < settings.max_iterations);

            // Simplify solution if enabled
            if (settings.simplify_intermediate and not result.path.empty())
            {
                result = simplify<Robot, rake, resolution>(
                    result.path, environment, settings.simplify, rng, lp);
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iters;

            // Exit early if trivial, unsolved, or not optimizing
            if (not settings.optimize or result.path.empty() or result.path.size() == 2)
            {
                return result;
            }

            // We have a new best solution
            PlanningResult<Robot> final_result;
            final_result.path = result.path;
            final_result.solved = true;
            best_path_cost = result.path.cost();

            float best_possible_cost = std::numeric_limits<float>::max();
            for (const auto &goal : goals)
            {
                best_possible_cost = std::min(best_possible_cost, planning::cost<Robot>(start, goal));
            }

            // Informed sampler: PHS keyed by the single-goal best_path_cost. Disabled for
            // cost robots (flask): the L2 spheroid is not admissible for C_loc.
            typename rng::RNG<Robot>::Ptr informed_rng;
            std::function<void(float)> update_informed_bound = [](float) {};
            if constexpr (not has_cost_v<Robot>)
            {
                if (settings.use_phs and goals.size() == 1)
                {
                    ProlateHyperspheroid<Robot> phs(start, goals[0]);
                    phs.set_transverse_diameter(best_path_cost);

                    // A pinned RNG must stay outermost: the PHS transform scatters into
                    // all dimensions, so wrap PHS around the unpinned inner sampler and
                    // re-apply the pins on top (projection onto the pinned slice).
                    if (auto pinned = std::dynamic_pointer_cast<rng::PinnedRNG<Robot>>(rng))
                    {
                        auto phs_rng =
                            std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, pinned->inner);
                        informed_rng = std::make_shared<rng::PinnedRNG<Robot>>(
                            phs_rng, pinned->mask, pinned->values);
                        update_informed_bound = [phs_rng](float c)
                        { phs_rng->phs.set_transverse_diameter(c); };
                    }
                    else
                    {
                        auto phs_rng = std::make_shared<ProlateHyperspheroidRNG<Robot>>(phs, rng);
                        informed_rng = phs_rng;
                        update_informed_bound = [phs_rng](float c)
                        { phs_rng->phs.set_transverse_diameter(c); };
                    }
                }
            }

            AOX_RRTC instance(settings.max_samples);

            // If we get close to straight line, just call it.
            // Also handles numerical issues with PHS when too close to straight line...
            while (iters < settings.max_iterations and (best_path_cost - best_possible_cost) > 1e-8)
            {
                // Update internal maximum iterations: remaining total budget, the per-search
                // cap, and the user's nested RRTC limit
                rrtc_settings.max_iterations = std::min(
                    {settings_in.rrtc.max_iterations,
                     settings.max_iterations - iters,
                     settings.max_internal_iterations});

                auto &sample_rng = informed_rng ? informed_rng : rng;

                if (not settings.anytime)
                {
                    result = instance.solve(
                        start, goals, environment, settings, best_path_cost, sample_rng, lp);
                }
                else
                {
                    result = RRTC::solve(start, goals, environment, rrtc_settings, sample_rng, lp);
                }

                iters += result.iterations;

                // If last search found a solution
                if (not result.path.empty())
                {
                    // Simplify
                    if (settings.simplify_intermediate)
                    {
                        result = simplify<Robot, rake, resolution>(
                            result.path, environment, settings.simplify, rng, lp);
                    }

                    // To be safe, ensure new path is actually a better solution
                    if (result.path.cost() < best_path_cost)
                    {
                        // Update best solution
                        final_result.path = result.path;
                        best_path_cost = result.path.cost();

                        update_informed_bound(best_path_cost);
                    }
                }
            }

            final_result.iterations = iters;
            final_result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

            return final_result;
        }
    };
}  // namespace vamp::planning
