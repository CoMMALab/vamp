#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <vector>

#include <vamp/collision/math.hh>

namespace vamp::collision
{
    // CenterVox pointcloud downsampling filter, from "VCC: Efficient Voxel-Based Collision
    // Checking Framework for Real-Time Robotic Motion Planning" (Chen and Yeh, ICRA 2026).
    //
    // Subdivides the workspace into a grid of cubic voxels, keeping at most one point per voxel:
    // the point closest to the voxel's geometric center. Points beyond `max_range` from `origin`
    // or outside the workspace AABB are removed.
    struct CenterSelectiveVoxelFilter
    {
        using TableOffset = std::uint32_t;
        static constexpr TableOffset NULL_OFFSET = std::numeric_limits<TableOffset>::max();

        struct Voxel
        {
            Point stored_point = {0.F, 0.F, 0.F};
            Point voxel_center = {0.F, 0.F, 0.F};
            float stored_point_dist_sq = 0.F;

            void set_center(
                std::uint16_t vx,
                std::uint16_t vy,
                std::uint16_t vz,
                float voxel_size,
                const Point &workspace_min) noexcept
            {
                voxel_center[0] = workspace_min[0] + (static_cast<float>(vx) + 0.5F) * voxel_size;
                voxel_center[1] = workspace_min[1] + (static_cast<float>(vy) + 0.5F) * voxel_size;
                voxel_center[2] = workspace_min[2] + (static_cast<float>(vz) + 0.5F) * voxel_size;
            }

            void try_insert(const Point &point, bool first) noexcept
            {
                const float dx = point[0] - voxel_center[0];
                const float dy = point[1] - voxel_center[1];
                const float dz = point[2] - voxel_center[2];
                const float dist_sq = dx * dx + dy * dy + dz * dz;

                if (first or dist_sq < stored_point_dist_sq)
                {
                    stored_point = point;
                    stored_point_dist_sq = dist_sq;
                }
            }
        };

        Point workspace_aabb_min;
        Point origin_point;
        float voxel_size;
        float max_range_sq;
        float inverse_scale_factor{};
        std::uint16_t grid_width{};

        std::unique_ptr<std::uint8_t[], decltype(&std::free)> hierarchy_pool{nullptr, &std::free};
        std::size_t hierarchy_pool_size_bytes = 0;
        std::size_t hierarchy_pool_used_bytes = 0;
        TableOffset x_table_offset = NULL_OFFSET;

        std::vector<Voxel> voxel_storage;

        CenterSelectiveVoxelFilter(
            float voxel_size,
            float max_range,
            const Point &origin,
            const Point &workspace_min,
            const Point &workspace_max)
          : workspace_aabb_min(workspace_min)
          , origin_point(origin)
          , voxel_size(voxel_size)
          , max_range_sq(max_range * max_range)
        {
            const float workspace_width = std::max(
                {workspace_max[0] - workspace_min[0],
                 workspace_max[1] - workspace_min[1],
                 workspace_max[2] - workspace_min[2]});

            if (not(workspace_width > 0.F) or not(voxel_size > 0.F))
            {
                grid_width = 1;
                inverse_scale_factor = 0.F;
            }
            else
            {
                grid_width = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(
                    static_cast<std::uint32_t>(std::ceil(workspace_width / voxel_size)),
                    1U,
                    std::numeric_limits<std::uint16_t>::max()));
                inverse_scale_factor = 1.F / voxel_size;
            }

            // Initial estimate only; allocate_table() grows the pool on demand.
            const std::size_t table_bytes = static_cast<std::size_t>(grid_width) * sizeof(TableOffset);
            const std::size_t estimated_tables =
                1 + grid_width +
                static_cast<std::size_t>(
                    static_cast<float>(grid_width) * static_cast<float>(grid_width) * 0.3F);
            hierarchy_pool_size_bytes = estimated_tables * table_bytes;
            hierarchy_pool.reset(
                static_cast<std::uint8_t *>(aligned_alloc_checked(hierarchy_pool_size_bytes)));

            x_table_offset = allocate_table();
        }

        void insert_point(const Point &point)
        {
            const float dx = point[0] - origin_point[0];
            const float dy = point[1] - origin_point[1];
            const float dz = point[2] - origin_point[2];
            if (dx * dx + dy * dy + dz * dz >= max_range_sq)
            {
                return;
            }

            const auto ivx = static_cast<int>((point[0] - workspace_aabb_min[0]) * inverse_scale_factor);
            const auto ivy = static_cast<int>((point[1] - workspace_aabb_min[1]) * inverse_scale_factor);
            const auto ivz = static_cast<int>((point[2] - workspace_aabb_min[2]) * inverse_scale_factor);
            if (static_cast<std::uint32_t>(ivx) >= grid_width or
                static_cast<std::uint32_t>(ivy) >= grid_width or
                static_cast<std::uint32_t>(ivz) >= grid_width)
            {
                return;
            }

            const auto vx = static_cast<std::uint16_t>(ivx);
            const auto vy = static_cast<std::uint16_t>(ivy);
            const auto vz = static_cast<std::uint16_t>(ivz);

            // Table pointers are re-derived after each allocation: allocate_table() may grow and
            // relocate the pool, but offsets stay valid.
            if (table_at(x_table_offset)[vx] == NULL_OFFSET)
            {
                const auto offset = allocate_table();
                mutable_table(x_table_offset)[vx] = offset;
            }

            const auto y_offset = table_at(x_table_offset)[vx];
            if (table_at(y_offset)[vy] == NULL_OFFSET)
            {
                const auto offset = allocate_table();
                mutable_table(y_offset)[vy] = offset;
            }

            const auto z_offset = table_at(y_offset)[vy];
            auto &voxel_slot = mutable_table(z_offset)[vz];
            if (voxel_slot == NULL_OFFSET)
            {
                voxel_slot = static_cast<TableOffset>(voxel_storage.size());
                auto &voxel = voxel_storage.emplace_back();
                voxel.set_center(vx, vy, vz, voxel_size, workspace_aabb_min);
                voxel.try_insert(point, true);
                return;
            }

            voxel_storage[voxel_slot].try_insert(point, false);
        }

        [[nodiscard]] auto extract_points() const -> std::vector<Point>
        {
            std::vector<Point> result;
            result.reserve(voxel_storage.size());
            for (const auto &voxel : voxel_storage)
            {
                result.emplace_back(voxel.stored_point);
            }

            return result;
        }

    private:
        [[nodiscard]] auto table_at(TableOffset offset) const noexcept -> const TableOffset *
        {
            return reinterpret_cast<const TableOffset *>(hierarchy_pool.get() + offset);
        }

        [[nodiscard]] auto mutable_table(TableOffset offset) noexcept -> TableOffset *
        {
            return reinterpret_cast<TableOffset *>(hierarchy_pool.get() + offset);
        }

        static auto aligned_alloc_checked(std::size_t bytes) -> void *
        {
            void *ptr = nullptr;
            if (posix_memalign(&ptr, 64, std::max<std::size_t>(bytes, 64)) != 0)
            {
                throw std::bad_alloc();
            }

            return ptr;
        }

        auto allocate_table() -> TableOffset
        {
            const std::size_t size_bytes = static_cast<std::size_t>(grid_width) * sizeof(TableOffset);
            if (hierarchy_pool_used_bytes + size_bytes > hierarchy_pool_size_bytes)
            {
                auto new_size = std::max<std::size_t>(hierarchy_pool_size_bytes, 64);
                while (new_size < hierarchy_pool_used_bytes + size_bytes)
                {
                    new_size *= 2;
                }

                auto *new_pool = static_cast<std::uint8_t *>(aligned_alloc_checked(new_size));
                std::memcpy(new_pool, hierarchy_pool.get(), hierarchy_pool_used_bytes);
                hierarchy_pool.reset(new_pool);
                hierarchy_pool_size_bytes = new_size;
            }

            const auto offset = static_cast<TableOffset>(hierarchy_pool_used_bytes);
            auto *table = mutable_table(offset);
            std::fill(table, table + grid_width, NULL_OFFSET);
            hierarchy_pool_used_bytes += size_bytes;
            return offset;
        }
    };

    // Filter a pointcloud by keeping at most one representative point per cubic voxel (the point
    // nearest the voxel center). Points beyond `max_range` of `origin` or outside the workspace
    // AABB are removed. An alternative to `filter_pointcloud` for pointcloud preprocessing.
    template <typename PointCloud>
    auto filter_pointcloud_centervox(
        const PointCloud &pc,
        float voxel_size,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max) -> std::vector<Point>
    {
        if (pc.shape(0) == 0)
        {
            return std::vector<Point>();
        }

        CenterSelectiveVoxelFilter filter(voxel_size, max_range, origin, workspace_min, workspace_max);
        for (std::size_t i = 0; i < pc.shape(0); ++i)
        {
            filter.insert_point(Point{pc(i, 0), pc(i, 1), pc(i, 2)});
        }

        return filter.extract_points();
    }

    template <>
    inline auto filter_pointcloud_centervox(
        const std::vector<Point> &pc,
        float voxel_size,
        float max_range,
        Point origin,
        Point workspace_min,
        Point workspace_max) -> std::vector<Point>
    {
        struct PointcloudWrapper
        {
            inline auto shape(std::size_t dim) const noexcept -> std::size_t
            {
                if (dim == 0)
                {
                    return pc.size();
                }

                if (dim == 1)
                {
                    return 3;
                }

                return -1;
            }

            inline auto operator()(std::size_t i, std::size_t j) const noexcept ->
                typename Point::value_type
            {
                return pc[i][j];
            }

            const std::vector<Point> &pc;
        };

        return filter_pointcloud_centervox(
            PointcloudWrapper{pc},
            voxel_size,
            max_range,
            std::move(origin),
            std::move(workspace_min),
            std::move(workspace_max));
    }
}  // namespace vamp::collision
