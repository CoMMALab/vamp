#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <vector>

#include <vamp/collision/math.hh>
#include <vamp/vector.hh>

namespace vamp::collision
{
    // Multi-level Voxel Table (MVT), from "VCC: Efficient Voxel-Based Collision Checking Framework
    // for Real-Time Robotic Motion Planning" (Chen and Yeh, ICRA 2026). A three-level sparse table
    // (X -> Y -> Z offset tables in a single pooled allocation) over a fixed workspace AABB; leaf
    // voxels hold SIMD-padded SoA point buffers with per-voxel AABBs.
    //
    // All points must lie within the given workspace bounds. `max_query_radius` sizes the voxels
    // (voxel edge = max_query_radius + point_radius); queries with larger radii remain correct but
    // scan proportionally more voxels.
    struct MVT
    {
        using FVectorT = FloatVector<>;
        using VoxelIndex = std::uint32_t;
        using TableOffset = std::uint32_t;

        static constexpr VoxelIndex INVALID_VOXEL_INDEX = std::numeric_limits<VoxelIndex>::max();
        static constexpr TableOffset NULL_OFFSET = std::numeric_limits<TableOffset>::max();
        static_assert(INVALID_VOXEL_INDEX == NULL_OFFSET);
        static constexpr std::uint16_t MAX_GRID_WIDTH = std::numeric_limits<std::uint16_t>::max();

        struct alignas(32) Voxel
        {
            float *x_coords = nullptr;
            float *y_coords = nullptr;
            float *z_coords = nullptr;
            std::size_t point_count = 0;
            std::size_t capacity = 0;

            Point bbox_min = {0.F, 0.F, 0.F};
            Point bbox_max = {0.F, 0.F, 0.F};

            void add_point(const Point &point, float point_radius) noexcept
            {
                assert(point_count < capacity);

                x_coords[point_count] = point[0];
                y_coords[point_count] = point[1];
                z_coords[point_count] = point[2];

                update_bounding_box(point, point_radius);
                ++point_count;
            }

        private:
            void update_bounding_box(const Point &point, float point_radius) noexcept
            {
                const float px_min = point[0] - point_radius;
                const float py_min = point[1] - point_radius;
                const float pz_min = point[2] - point_radius;
                const float px_max = point[0] + point_radius;
                const float py_max = point[1] + point_radius;
                const float pz_max = point[2] + point_radius;

                if (point_count == 0)
                {
                    bbox_min = {px_min, py_min, pz_min};
                    bbox_max = {px_max, py_max, pz_max};
                }
                else
                {
                    bbox_min[0] = std::min(bbox_min[0], px_min);
                    bbox_min[1] = std::min(bbox_min[1], py_min);
                    bbox_min[2] = std::min(bbox_min[2], pz_min);
                    bbox_max[0] = std::max(bbox_max[0], px_max);
                    bbox_max[1] = std::max(bbox_max[1], py_max);
                    bbox_max[2] = std::max(bbox_max[2], pz_max);
                }
            }
        };

        // Query parameters
        float max_query_radius;
        float point_radius;

        // Spatial bounds
        Point workspace_aabb_min;
        Point workspace_aabb_max;
        Point global_aabb_min;
        Point global_aabb_max;

        // Grid configuration
        float inverse_scale_factor{};
        std::uint16_t grid_width{};

        // Memory pools; hierarchy tables are addressed by pool offsets so the pool can be grown
        // (and the structure copied) without pointer relocation.
        std::unique_ptr<float[], decltype(&std::free)> point_coord_pool{nullptr, &std::free};
        std::size_t point_coord_pool_size = 0;
        std::size_t point_coord_pool_used = 0;

        std::unique_ptr<std::uint8_t[], decltype(&std::free)> hierarchy_pool{nullptr, &std::free};
        std::size_t hierarchy_pool_size_bytes = 0;
        std::size_t hierarchy_pool_used_bytes = 0;

        std::vector<Voxel> voxel_storage;
        TableOffset x_table_offset = NULL_OFFSET;

        // SIMD-cached bounds
        FVectorT simd_global_min_x, simd_global_min_y, simd_global_min_z;
        FVectorT simd_global_max_x, simd_global_max_y, simd_global_max_z;
        FVectorT simd_workspace_min_x, simd_workspace_min_y, simd_workspace_min_z;

        MVT(const std::vector<Point> &points,
            float max_query_radius,
            const Point &workspace_aabb_min,
            const Point &workspace_aabb_max,
            float point_radius)
          : max_query_radius{max_query_radius}
          , point_radius{point_radius}
          , workspace_aabb_min{workspace_aabb_min}
          , workspace_aabb_max{workspace_aabb_max}
        {
            initialize_empty_bounds();
            if (points.empty())
            {
                setup_simd_vectors();
                return;
            }

            configure_grid();
            initialize_hierarchy_pool();
            initialize_voxel_storage();
            build_spatial_grid_two_phase(points);
            compute_global_bounds();
            setup_simd_vectors();
        }

        MVT(const MVT &other)
          : max_query_radius(other.max_query_radius)
          , point_radius(other.point_radius)
          , workspace_aabb_min(other.workspace_aabb_min)
          , workspace_aabb_max(other.workspace_aabb_max)
          , global_aabb_min(other.global_aabb_min)
          , global_aabb_max(other.global_aabb_max)
          , inverse_scale_factor(other.inverse_scale_factor)
          , grid_width(other.grid_width)
          , point_coord_pool_size(other.point_coord_pool_size)
          , point_coord_pool_used(other.point_coord_pool_used)
          , hierarchy_pool_size_bytes(other.hierarchy_pool_size_bytes)
          , hierarchy_pool_used_bytes(other.hierarchy_pool_used_bytes)
          , voxel_storage(other.voxel_storage)
          , x_table_offset(other.x_table_offset)
        {
            copy_memory_pools(other);
            relocate_voxel_coordinates(other);
            setup_simd_vectors();
        }

        MVT(MVT &&) noexcept = default;
        auto operator=(MVT &&) noexcept -> MVT & = default;

        auto operator=(const MVT &other) -> MVT &
        {
            if (this != &other)
            {
                *this = MVT(other);
            }

            return *this;
        }

        ~MVT() = default;

        // Scalar collision detection
        [[nodiscard]] auto collides(const Point &center, float radius) const noexcept -> bool
        {
            const float query_radius = radius + point_radius;
            const float query_radius_squared = query_radius * query_radius;

            // Early exit: global AABB check (also rejects everything for an empty cloud)
            if (center[0] + query_radius < global_aabb_min[0] or
                center[0] - query_radius > global_aabb_max[0] or
                center[1] + query_radius < global_aabb_min[1] or
                center[1] - query_radius > global_aabb_max[1] or
                center[2] + query_radius < global_aabb_min[2] or
                center[2] - query_radius > global_aabb_max[2])
            {
                return false;
            }

            const float grid_query_radius = query_radius * inverse_scale_factor;
            const float grid_center_x = (center[0] - workspace_aabb_min[0]) * inverse_scale_factor;
            const float grid_center_y = (center[1] - workspace_aabb_min[1]) * inverse_scale_factor;
            const float grid_center_z = (center[2] - workspace_aabb_min[2]) * inverse_scale_factor;

            const auto min_x = grid_index(grid_center_x - grid_query_radius);
            const auto max_x = grid_index(grid_center_x + grid_query_radius);
            const auto min_y = grid_index(grid_center_y - grid_query_radius);
            const auto max_y = grid_index(grid_center_y + grid_query_radius);
            const auto min_z = grid_index(grid_center_z - grid_query_radius);
            const auto max_z = grid_index(grid_center_z + grid_query_radius);

            const auto *x_table = table_at(x_table_offset);
            for (auto voxel_x = min_x; voxel_x <= max_x; ++voxel_x)
            {
                if (x_table[voxel_x] == NULL_OFFSET)
                {
                    continue;
                }

                const auto *y_table = table_at(x_table[voxel_x]);
                for (auto voxel_y = min_y; voxel_y <= max_y; ++voxel_y)
                {
                    if (y_table[voxel_y] == NULL_OFFSET)
                    {
                        continue;
                    }

                    const auto *z_table = table_at(y_table[voxel_y]);
                    for (auto voxel_z = min_z; voxel_z <= max_z; ++voxel_z)
                    {
                        const auto voxel_index = z_table[voxel_z];
                        if (voxel_index == INVALID_VOXEL_INDEX)
                        {
                            continue;
                        }

                        const auto &voxel = voxel_storage[voxel_index];
                        if (center[0] + query_radius < voxel.bbox_min[0] or
                            center[0] - query_radius > voxel.bbox_max[0] or
                            center[1] + query_radius < voxel.bbox_min[1] or
                            center[1] - query_radius > voxel.bbox_max[1] or
                            center[2] + query_radius < voxel.bbox_min[2] or
                            center[2] - query_radius > voxel.bbox_max[2])
                        {
                            continue;
                        }

                        for (std::size_t i = 0; i < voxel.point_count; ++i)
                        {
                            const float dx = center[0] - voxel.x_coords[i];
                            const float dy = center[1] - voxel.y_coords[i];
                            const float dz = center[2] - voxel.z_coords[i];
                            if (dx * dx + dy * dy + dz * dz <= query_radius_squared)
                            {
                                return true;
                            }
                        }
                    }
                }
            }

            return false;
        }

        // Collision check for a SIMD batch of spheres: per-lane hierarchy traversal with a
        // SIMD scan over the points inside each candidate voxel. True if any lane collides.
        auto collides_simd(const std::array<FVectorT, 3> &centers, FVectorT radii) const noexcept
            -> bool
        {
            constexpr std::size_t simd_width = FVectorT::num_scalars;

            const FVectorT query_radii = radii + FVectorT::fill(point_radius);

            // Cull lanes completely outside the global AABB (all lanes, for an empty cloud)
            const auto outside_mask =
                (centers[0] < (simd_global_min_x - query_radii)) |
                ((simd_global_max_x + query_radii) < centers[0]) |
                (centers[1] < (simd_global_min_y - query_radii)) |
                ((simd_global_max_y + query_radii) < centers[1]) |
                (centers[2] < (simd_global_min_z - query_radii)) |
                ((simd_global_max_z + query_radii) < centers[2]);

            if (outside_mask.all())
            {
                return false;
            }

            const FVectorT inv_scale = FVectorT::fill(inverse_scale_factor);
            const FVectorT grid_centers_x = (centers[0] - simd_workspace_min_x) * inv_scale;
            const FVectorT grid_centers_y = (centers[1] - simd_workspace_min_y) * inv_scale;
            const FVectorT grid_centers_z = (centers[2] - simd_workspace_min_z) * inv_scale;

            const auto centers_x_array = centers[0].to_array();
            const auto centers_y_array = centers[1].to_array();
            const auto centers_z_array = centers[2].to_array();
            const auto query_radii_array = query_radii.to_array();
            const auto outside_array = outside_mask.to_array();
            const auto grid_x_array = grid_centers_x.to_array();
            const auto grid_y_array = grid_centers_y.to_array();
            const auto grid_z_array = grid_centers_z.to_array();

            const auto *x_table = table_at(x_table_offset);
            for (std::size_t lane = 0; lane < simd_width; ++lane)
            {
                if (outside_array[lane] != 0)
                {
                    continue;
                }

                const Point center = {
                    centers_x_array[lane], centers_y_array[lane], centers_z_array[lane]};
                const float query_radius = query_radii_array[lane];
                const float query_radius_squared = query_radius * query_radius;
                const float grid_query_radius = query_radius * inverse_scale_factor;

                const auto min_x = grid_index(grid_x_array[lane] - grid_query_radius);
                const auto max_x = grid_index(grid_x_array[lane] + grid_query_radius);
                const auto min_y = grid_index(grid_y_array[lane] - grid_query_radius);
                const auto max_y = grid_index(grid_y_array[lane] + grid_query_radius);
                const auto min_z = grid_index(grid_z_array[lane] - grid_query_radius);
                const auto max_z = grid_index(grid_z_array[lane] + grid_query_radius);

                for (auto voxel_x = min_x; voxel_x <= max_x; ++voxel_x)
                {
                    if (x_table[voxel_x] == NULL_OFFSET)
                    {
                        continue;
                    }

                    const auto *y_table = table_at(x_table[voxel_x]);
                    for (auto voxel_y = min_y; voxel_y <= max_y; ++voxel_y)
                    {
                        if (y_table[voxel_y] == NULL_OFFSET)
                        {
                            continue;
                        }

                        const auto *z_table = table_at(y_table[voxel_y]);
                        for (auto voxel_z = min_z; voxel_z <= max_z; ++voxel_z)
                        {
                            const auto voxel_index = z_table[voxel_z];
                            if (voxel_index == INVALID_VOXEL_INDEX)
                            {
                                continue;
                            }

                            const auto &voxel = voxel_storage[voxel_index];
                            if (center[0] + query_radius < voxel.bbox_min[0] or
                                center[0] - query_radius > voxel.bbox_max[0] or
                                center[1] + query_radius < voxel.bbox_min[1] or
                                center[1] - query_radius > voxel.bbox_max[1] or
                                center[2] + query_radius < voxel.bbox_min[2] or
                                center[2] - query_radius > voxel.bbox_max[2])
                            {
                                continue;
                            }

                            const FVectorT sphere_x = FVectorT::fill(center[0]);
                            const FVectorT sphere_y = FVectorT::fill(center[1]);
                            const FVectorT sphere_z = FVectorT::fill(center[2]);
                            const FVectorT sphere_radius_sq = FVectorT::fill(query_radius_squared);

                            // Buffers are padded to simd_width with +inf, so full loads are safe
                            for (std::size_t i = 0; i < voxel.point_count; i += simd_width)
                            {
                                const FVectorT point_x(voxel.x_coords + i);
                                const FVectorT point_y(voxel.y_coords + i);
                                const FVectorT point_z(voxel.z_coords + i);

                                const FVectorT dx = sphere_x - point_x;
                                const FVectorT dy = sphere_y - point_y;
                                const FVectorT dz = sphere_z - point_z;
                                const FVectorT dist_sq = dx * dx + dy * dy + dz * dz;

                                if ((dist_sq <= sphere_radius_sq).any())
                                {
                                    return true;
                                }
                            }
                        }
                    }
                }
            }

            return false;
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

        // Clamps a grid-space coordinate to a valid voxel index.
        [[nodiscard]] auto grid_index(float grid_coordinate) const noexcept -> std::uint16_t
        {
            return static_cast<std::uint16_t>(
                std::clamp(grid_coordinate, 0.F, static_cast<float>(grid_width - 1)));
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

        void initialize_empty_bounds() noexcept
        {
            constexpr float max_val = std::numeric_limits<float>::max();
            constexpr float min_val = std::numeric_limits<float>::lowest();

            global_aabb_min = Point{max_val, max_val, max_val};
            global_aabb_max = Point{min_val, min_val, min_val};
        }

        void configure_grid() noexcept
        {
            const float workspace_width = std::max(
                {workspace_aabb_max[0] - workspace_aabb_min[0],
                 workspace_aabb_max[1] - workspace_aabb_min[1],
                 workspace_aabb_max[2] - workspace_aabb_min[2]});
            const float voxel_size = max_query_radius + point_radius;

            if (not(workspace_width > 0.F) or not(voxel_size > 0.F))
            {
                grid_width = 1;
                inverse_scale_factor = 0.F;
                return;
            }

            grid_width = static_cast<std::uint16_t>(std::clamp<std::uint32_t>(
                static_cast<std::uint32_t>(workspace_width / voxel_size), 1U, MAX_GRID_WIDTH));
            inverse_scale_factor = static_cast<float>(grid_width) / workspace_width;
        }

        void initialize_hierarchy_pool()
        {
            // Initial estimate only; allocate_table() grows the pool on demand.
            const std::size_t table_bytes = static_cast<std::size_t>(grid_width) * sizeof(TableOffset);
            const std::size_t estimated_tables =
                1 + grid_width +
                static_cast<std::size_t>(
                    static_cast<float>(grid_width) * static_cast<float>(grid_width) * 0.8F);

            hierarchy_pool_size_bytes = estimated_tables * table_bytes;
            hierarchy_pool_used_bytes = 0;
            hierarchy_pool.reset(
                static_cast<std::uint8_t *>(aligned_alloc_checked(hierarchy_pool_size_bytes)));
        }

        void initialize_voxel_storage()
        {
            const auto gw = static_cast<std::size_t>(grid_width);
            voxel_storage.reserve(static_cast<std::size_t>(static_cast<float>(gw * gw * gw) * 0.1F));
        }

        void setup_simd_vectors() noexcept
        {
            simd_global_min_x = FVectorT::fill(global_aabb_min[0]);
            simd_global_min_y = FVectorT::fill(global_aabb_min[1]);
            simd_global_min_z = FVectorT::fill(global_aabb_min[2]);
            simd_global_max_x = FVectorT::fill(global_aabb_max[0]);
            simd_global_max_y = FVectorT::fill(global_aabb_max[1]);
            simd_global_max_z = FVectorT::fill(global_aabb_max[2]);

            simd_workspace_min_x = FVectorT::fill(workspace_aabb_min[0]);
            simd_workspace_min_y = FVectorT::fill(workspace_aabb_min[1]);
            simd_workspace_min_z = FVectorT::fill(workspace_aabb_min[2]);
        }

        // Allocates one grid_width-wide table in the hierarchy pool, filled with NULL_OFFSET
        // (== INVALID_VOXEL_INDEX), growing the pool if needed. Returns the table's pool offset;
        // callers must re-derive any table pointers held across this call.
        auto allocate_table() -> TableOffset
        {
            const std::size_t size_bytes = static_cast<std::size_t>(grid_width) * sizeof(TableOffset);
            if (hierarchy_pool_used_bytes + size_bytes > hierarchy_pool_size_bytes)
            {
                grow_hierarchy_pool(hierarchy_pool_used_bytes + size_bytes);
            }

            const auto offset = static_cast<TableOffset>(hierarchy_pool_used_bytes);
            auto *table = mutable_table(offset);
            std::fill(table, table + grid_width, NULL_OFFSET);
            hierarchy_pool_used_bytes += size_bytes;
            return offset;
        }

        void grow_hierarchy_pool(std::size_t required_bytes)
        {
            auto new_size = std::max<std::size_t>(hierarchy_pool_size_bytes, 64);
            while (new_size < required_bytes)
            {
                new_size *= 2;
            }

            auto *new_pool = static_cast<std::uint8_t *>(aligned_alloc_checked(new_size));
            std::memcpy(new_pool, hierarchy_pool.get(), hierarchy_pool_used_bytes);
            hierarchy_pool.reset(new_pool);
            hierarchy_pool_size_bytes = new_size;
        }

        void build_spatial_grid_two_phase(const std::vector<Point> &points)
        {
            // --- Phase 1: build the table hierarchy and count points per voxel ---
            x_table_offset = allocate_table();

            std::vector<VoxelIndex> point_to_voxel(points.size());
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                const auto &point = points[i];
                const auto voxel_x =
                    grid_index((point[0] - workspace_aabb_min[0]) * inverse_scale_factor);
                const auto voxel_y =
                    grid_index((point[1] - workspace_aabb_min[1]) * inverse_scale_factor);
                const auto voxel_z =
                    grid_index((point[2] - workspace_aabb_min[2]) * inverse_scale_factor);

                // Table pointers are re-derived after each allocation: allocate_table() may grow
                // and relocate the pool, but offsets stay valid.
                if (table_at(x_table_offset)[voxel_x] == NULL_OFFSET)
                {
                    const auto offset = allocate_table();
                    mutable_table(x_table_offset)[voxel_x] = offset;
                }

                const auto y_offset = table_at(x_table_offset)[voxel_x];
                if (table_at(y_offset)[voxel_y] == NULL_OFFSET)
                {
                    const auto offset = allocate_table();
                    mutable_table(y_offset)[voxel_y] = offset;
                }

                const auto z_offset = table_at(y_offset)[voxel_y];
                auto &voxel_slot = mutable_table(z_offset)[voxel_z];
                if (voxel_slot == INVALID_VOXEL_INDEX)
                {
                    voxel_slot = static_cast<VoxelIndex>(voxel_storage.size());
                    voxel_storage.emplace_back();
                }

                voxel_storage[voxel_slot].point_count++;
                point_to_voxel[i] = voxel_slot;
            }

            // --- Phase 2: exact SoA allocation and filling ---
            std::size_t total_required_floats = 0;
            constexpr std::size_t simd_width = FVectorT::num_scalars;
            for (auto &voxel : voxel_storage)
            {
                voxel.capacity = (voxel.point_count + simd_width - 1) & ~(simd_width - 1);
                total_required_floats += voxel.capacity * 3;
            }

            allocate_exact_point_pool(total_required_floats);

            for (auto &voxel : voxel_storage)
            {
                voxel.x_coords = allocate_coords(voxel.capacity);
                voxel.y_coords = allocate_coords(voxel.capacity);
                voxel.z_coords = allocate_coords(voxel.capacity);
                voxel.point_count = 0;
            }

            for (std::size_t i = 0; i < points.size(); ++i)
            {
                voxel_storage[point_to_voxel[i]].add_point(points[i], point_radius);
            }
        }

        void compute_global_bounds() noexcept
        {
            initialize_empty_bounds();

            for (const auto &voxel : voxel_storage)
            {
                if (voxel.point_count > 0)
                {
                    global_aabb_min[0] = std::min(global_aabb_min[0], voxel.bbox_min[0]);
                    global_aabb_min[1] = std::min(global_aabb_min[1], voxel.bbox_min[1]);
                    global_aabb_min[2] = std::min(global_aabb_min[2], voxel.bbox_min[2]);
                    global_aabb_max[0] = std::max(global_aabb_max[0], voxel.bbox_max[0]);
                    global_aabb_max[1] = std::max(global_aabb_max[1], voxel.bbox_max[1]);
                    global_aabb_max[2] = std::max(global_aabb_max[2], voxel.bbox_max[2]);
                }
            }
        }

        void allocate_exact_point_pool(std::size_t total_floats)
        {
            point_coord_pool_size = total_floats;
            point_coord_pool.reset(
                static_cast<float *>(aligned_alloc_checked(total_floats * sizeof(float))));
            point_coord_pool_used = 0;

            // Pad with +inf so full-width SIMD loads past point_count never report a collision
            std::fill(
                point_coord_pool.get(),
                point_coord_pool.get() + point_coord_pool_size,
                std::numeric_limits<float>::infinity());
        }

        auto allocate_coords(std::size_t count) noexcept -> float *
        {
            assert(point_coord_pool_used + count <= point_coord_pool_size);

            auto *result = point_coord_pool.get() + point_coord_pool_used;
            point_coord_pool_used += count;
            return result;
        }

        void copy_memory_pools(const MVT &other)
        {
            if (other.point_coord_pool and other.point_coord_pool_size != 0)
            {
                point_coord_pool.reset(static_cast<float *>(
                    aligned_alloc_checked(point_coord_pool_size * sizeof(float))));
                std::memcpy(
                    point_coord_pool.get(),
                    other.point_coord_pool.get(),
                    point_coord_pool_used * sizeof(float));
            }

            if (other.hierarchy_pool and other.hierarchy_pool_size_bytes != 0)
            {
                hierarchy_pool.reset(
                    static_cast<std::uint8_t *>(aligned_alloc_checked(hierarchy_pool_size_bytes)));
                std::memcpy(
                    hierarchy_pool.get(), other.hierarchy_pool.get(), hierarchy_pool_used_bytes);
            }
        }

        void relocate_voxel_coordinates(const MVT &other) noexcept
        {
            for (std::size_t i = 0; i < voxel_storage.size(); ++i)
            {
                auto &voxel = voxel_storage[i];
                const auto &other_voxel = other.voxel_storage[i];

                if (other_voxel.x_coords != nullptr)
                {
                    voxel.x_coords =
                        point_coord_pool.get() + (other_voxel.x_coords - other.point_coord_pool.get());
                    voxel.y_coords =
                        point_coord_pool.get() + (other_voxel.y_coords - other.point_coord_pool.get());
                    voxel.z_coords =
                        point_coord_pool.get() + (other_voxel.z_coords - other.point_coord_pool.get());
                }
            }
        }
    };
}  // namespace vamp::collision
