#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include <vamp/collision/environment.hh>

namespace vamp::collision
{
    // Which link hit which obstacle, counted while collision-checking: a heuristic for what a planner kept running
    // into, not an exact number. Only the first collision of a rejected block of samples is counted (the collision
    // kernels stop at the first one), and a block is one rejected edge for the validators and the local planners.
    //
    // The matrix has one row per robot link with geometry and a last row for what the robot holds (attachments),
    // and one column per obstacle of the environment and a last column for the robot hitting itself.
    //
    // Nothing is counted until an instance is made active on the current thread (see `Scope`).
    struct CollisionStats
    {
        static constexpr std::size_t unknown = std::numeric_limits<std::size_t>::max();

        std::vector<std::string> link_names;      // The links, then "<attachment>".
        std::vector<std::string> obstacle_names;  // The obstacles, then "<self>".
        std::vector<std::uint64_t> counts;        // link_names.size() x obstacle_names.size(), row-major.
        std::uint64_t rejected = 0;               // Rejected blocks counted so far.
        CollisionStats *previous = nullptr;       // What was active before this one was entered, for the Python context manager.

        CollisionStats(std::vector<std::string> links, std::vector<std::string> obstacles)
          : link_names(std::move(links)), obstacle_names(std::move(obstacles))
        {
            link_names.emplace_back("<attachment>");
            obstacle_names.emplace_back("<self>");
            counts.assign(link_names.size() * obstacle_names.size(), 0);
        }

        inline auto n_links() const noexcept -> std::size_t
        {
            return link_names.size() - 1;
        }

        inline auto n_obstacles() const noexcept -> std::size_t
        {
            return obstacle_names.size() - 1;
        }

        inline void add(std::size_t row, std::size_t column) noexcept
        {
            if (row < link_names.size() and column < obstacle_names.size())
            {
                ++counts[row * obstacle_names.size() + column];
            }
        }

        // A link hit an obstacle (`unknown` when it was a kind of obstacle that has no index).
        inline void record(std::size_t link, std::size_t obstacle) noexcept
        {
            ++rejected;
            add(link, obstacle);
        }

        // Two links hit each other.
        inline void record_self(std::size_t first, std::size_t second) noexcept
        {
            ++rejected;
            add(first, n_obstacles());
            add(second, n_obstacles());
        }

        // What the robot holds hit an obstacle.
        inline void record_attachment(std::size_t obstacle) noexcept
        {
            ++rejected;
            add(n_links(), obstacle);
        }

        inline void reset() noexcept
        {
            std::fill(counts.begin(), counts.end(), 0);
            rejected = 0;
        }

        // The nonzero cells, most counted first: (link, obstacle, count).
        inline auto pairs() const -> std::vector<std::tuple<std::string, std::string, std::uint64_t>>
        {
            std::vector<std::tuple<std::string, std::string, std::uint64_t>> out;
            for (std::size_t row = 0; row < link_names.size(); ++row)
            {
                for (std::size_t column = 0; column < obstacle_names.size(); ++column)
                {
                    if (const auto count = counts[row * obstacle_names.size() + column]; count != 0)
                    {
                        out.emplace_back(link_names[row], obstacle_names[column], count);
                    }
                }
            }

            std::stable_sort(
                out.begin(), out.end(), [](const auto &a, const auto &b) { return std::get<2>(a) > std::get<2>(b); });
            return out;
        }
    };

    // The statistics being collected on this thread, if any, and the obstacle the last sphere-environment hit was.
    inline thread_local CollisionStats *active_stats = nullptr;
    inline thread_local std::size_t last_obstacle = CollisionStats::unknown;

    // Makes `stats` the one collected into on this thread, for as long as the scope lives.
    class Scope
    {
    public:
        explicit Scope(CollisionStats &stats) noexcept : previous_(active_stats)
        {
            active_stats = &stats;
        }

        ~Scope()
        {
            active_stats = previous_;
        }

        Scope(const Scope &) = delete;
        auto operator=(const Scope &) -> Scope & = delete;

    private:
        CollisionStats *previous_;
    };

    // Obstacles are numbered in the order the environment's lists are checked: spheres, capsules, z-aligned capsules,
    // cuboids, z-aligned cuboids, heightfields, point clouds. These are the numbers of their first entries.
    template <typename DataT>
    struct ObstacleOffsets
    {
        std::size_t spheres = 0;
        std::size_t capsules, z_aligned_capsules, cuboids, z_aligned_cuboids, heightfields, pointclouds, total;

        explicit ObstacleOffsets(const Environment<DataT> &e) noexcept
          : capsules(e.spheres.size())
          , z_aligned_capsules(capsules + e.capsules.size())
          , cuboids(z_aligned_capsules + e.z_aligned_capsules.size())
          , z_aligned_cuboids(cuboids + e.cuboids.size())
          , heightfields(z_aligned_cuboids + e.z_aligned_cuboids.size())
          , pointclouds(heightfields + e.heightfields.size())
          , total(pointclouds + e.pointclouds.size())
        {
        }
    };

    // The names of the obstacles of an environment, in the order they are numbered. One that was not given a name is
    // called after its kind and place in its list, such as cuboid_3.
    template <typename DataT>
    inline auto obstacle_names(const Environment<DataT> &e) -> std::vector<std::string>
    {
        std::vector<std::string> names;
        const auto add = [&names](const auto &list, const char *kind)
        {
            for (std::size_t i = 0; i < list.size(); ++i)
            {
                names.emplace_back(list[i].name.empty() ? std::string(kind) + "_" + std::to_string(i) : list[i].name);
            }
        };

        add(e.spheres, "sphere");
        add(e.capsules, "capsule");
        add(e.z_aligned_capsules, "z_aligned_capsule");
        add(e.cuboids, "cuboid");
        add(e.z_aligned_cuboids, "z_aligned_cuboid");
        add(e.heightfields, "heightfield");
        for (std::size_t i = 0; i < e.pointclouds.size(); ++i)
        {
            names.emplace_back("pointcloud_" + std::to_string(i));
        }

        return names;
    }
}  // namespace vamp::collision
