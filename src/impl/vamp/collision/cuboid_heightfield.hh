#pragma once

#include <limits>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/sphere_heightfield.hh>

namespace vamp::collision
{
    // Robot box `c` vs. heightfield. Negative in lanes that collide.
    //
    // APPROXIMATE. sphere_heightfield only compares one height sample (the cell under the query point)
    // against the query point, so a box is tested by posing a regular lattice of `Samples` points per
    // axis over the box's volume (Samples = 2 is just the 8 corners; the default of 3 adds edge
    // midpoints, face centers and the center) and taking the lowest clearance over all of them.
    //
    // This can miss thin terrain features that fit between lattice points, e.g., a narrow bump under the
    // middle of a large flat face. Raise `Samples` for large links or rough terrain; the cost is
    // Samples^3 gathers.
    template <std::size_t Samples = 3, typename BoxT, typename DataT>
    inline auto cuboid_heightfield(const BoxT &c, const HeightField<DataT> &h) noexcept -> DataT
    {
        static_assert(Samples >= 2, "Need at least the two extremes along each axis");

        const DataT zero(0.F);
        DataT result(std::numeric_limits<float>::max());

        for (auto i = 0U; i < Samples; ++i)
        {
            const float si = -1.F + 2.F * static_cast<float>(i) / static_cast<float>(Samples - 1);
            for (auto j = 0U; j < Samples; ++j)
            {
                const float sj = -1.F + 2.F * static_cast<float>(j) / static_cast<float>(Samples - 1);
                for (auto k = 0U; k < Samples; ++k)
                {
                    const float sk = -1.F + 2.F * static_cast<float>(k) / static_cast<float>(Samples - 1);

                    const auto a = c.axis_1_r * si;
                    const auto b = c.axis_2_r * sj;
                    const auto d = c.axis_3_r * sk;

                    const auto px = c.x + c.axis_1_x * a + c.axis_2_x * b + c.axis_3_x * d;
                    const auto py = c.y + c.axis_1_y * a + c.axis_2_y * b + c.axis_3_y * d;
                    const auto pz = c.z + c.axis_1_z * a + c.axis_2_z * b + c.axis_3_z * d;

                    // Elementwise min(result, clearance), via max on the negation
                    const auto clearance = sphere_heightfield(h, px, py, pz, zero);
                    result = -((-result).max(-clearance));
                }
            }
        }

        return result;
    }
}  // namespace vamp::collision
