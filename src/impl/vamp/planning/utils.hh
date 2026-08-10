#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

#include <pdqsort.h>

#include <vamp/planning/nn/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planners/roadmap.hh>
#include <vamp/vector.hh>

namespace vamp::planning::utils
{
    struct ConnectedComponent
    {
        unsigned int parent;
        unsigned int size;
    };

    // Basic union-find
    // TODO: We may be able to improve these implementations
    inline auto
    find_root(std::vector<ConnectedComponent> &components, unsigned int c_idx) noexcept -> unsigned int
    {
        while (c_idx != components[c_idx].parent)
        {
            const auto old_parent = components[c_idx].parent;
            components[c_idx].parent = components[components[c_idx].parent].parent;
            c_idx = old_parent;
        }

        return c_idx;
    }

    inline void merge_components(
        std::vector<ConnectedComponent> &components,
        unsigned int idx_a,
        unsigned int idx_b) noexcept
    {
        idx_a = find_root(components, idx_a);
        idx_b = find_root(components, idx_b);
        if (idx_a == idx_b)
        {
            return;
        }

        auto &c_a = components[idx_a];
        auto &c_b = components[idx_b];
        if (c_a.size < c_b.size)
        {
            c_a.parent = idx_b;
            c_b.size += c_a.size;
        }
        else
        {
            c_b.parent = idx_a;
            c_a.size += c_b.size;
        }
    }

    struct QueueNode
    {
        unsigned int index;
        float cost;

        inline constexpr auto operator==(const QueueNode &o) const noexcept -> bool
        {
            return index == o.index;
        }
    };

    // Parent-array convention shared by astar and recover_path: unreached nodes hold
    // max(), the root (the start, index 0) is its own parent.
    template <typename Robot, typename Space = Robot, typename Configuration, typename StateLookupFn>
    inline auto astar(
        std::vector<RoadmapNode> &graph,
        const Configuration &start,
        const Configuration &goal,
        const StateLookupFn &state_index,
        unsigned int goal_index = 1) noexcept -> std::unique_ptr<unsigned int[]>
    {
        // TODO: With C++20, use make_unique_for_overwrite here
        auto parents = std::make_unique<unsigned int[]>(graph.size());
        auto expanded = std::vector<bool>(graph.size());
        for (auto i = 0U; i < graph.size(); ++i)
        {
            parents[i] = std::numeric_limits<unsigned int>::max();
            expanded[i] = false;
        }

        // NOTE: Assumes that the start is the first element of the graph
        constexpr const unsigned int start_index = 0;
        parents[start_index] = start_index;
        std::vector<utils::QueueNode> open_set;
        open_set.emplace_back(utils::QueueNode{start_index, Space::distance(goal, start)});
        Configuration temp;
        while (not open_set.empty())
        {
            const auto current = open_set.back();
            open_set.pop_back();
            if (current.index == goal_index)
            {
                break;
            }

            if (expanded[current.index])
            {
                continue;
            }
            expanded[current.index] = true;

            const auto &current_node = graph[current.index];
            const auto current_g = current_node.g;
            for (const auto &[neighbor_index, neighbor_distance] : current_node.neighbors)
            {
                auto &neighbor_node = graph[neighbor_index];
                if (current_g + neighbor_distance >= neighbor_node.g)
                {
                    continue;
                }

                neighbor_node.g = current_g + neighbor_distance;
                parents[neighbor_index] = current_node.index;
                temp = state_index(neighbor_index);
                open_set.emplace_back(
                    utils::QueueNode{neighbor_index, neighbor_node.g + Space::distance(goal, temp)});
            }

            pdqsort(
                open_set.begin(),
                open_set.end(),
                [](const auto &a, const auto &b) { return a.cost > b.cost; });
        }

        if (parents[goal_index] == std::numeric_limits<unsigned int>::max())
        {
            return nullptr;
        }

        return parents;
    }

    // Walk the parent array from goal_index back to the root (the root is its own
    // parent), emitting the path in start-to-goal order. The goal must have been
    // reached: unreached entries hold max() and are not valid to walk from.
    template <typename Robot, typename Space = Robot, typename StateLookupFn>
    inline void recover_path(
        const unsigned int *parents,
        const StateLookupFn &state_index,
        Path<Robot, Space> &path,
        unsigned int goal_index = 1) noexcept
    {
        auto current_index = goal_index;
        while (parents[current_index] != current_index)
        {
            path.emplace_back(state_index(current_index));
            current_index = parents[current_index];
        }

        path.emplace_back(state_index(current_index));
        std::reverse(path.begin(), path.end());
    }
}  // namespace vamp::planning::utils
