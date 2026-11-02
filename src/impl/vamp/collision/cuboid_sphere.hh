#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    // Squared distance from the point (x, y, z) to the (solid) box; zero inside the box.
    template <typename BoxT, typename DataT>
    inline auto
    point_cuboid_sql2(const BoxT &c, const DataT &x, const DataT &y, const DataT &z) noexcept -> DataT
    {
        const auto xs = x - c.x;
        const auto ys = y - c.y;
        const auto zs = z - c.z;

        const auto a1 = (dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs).abs() - c.axis_1_r).max(0.);
        const auto a2 = (dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs).abs() - c.axis_2_r).max(0.);
        const auto a3 = (dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs).abs() - c.axis_3_r).max(0.);

        return dot_3(a1, a2, a3, a1, a2, a3);
    }

    // Robot box `c` vs. environment sphere with center (x, y, z) and radius r.
    // Negative in lanes that collide.
    template <typename BoxT, typename DataT>
    inline auto
    cuboid_sphere(const BoxT &c, const DataT &x, const DataT &y, const DataT &z, const DataT &r) noexcept
        -> DataT
    {
        return point_cuboid_sql2(c, x, y, z) - r * r;
    }

    template <typename BoxT, typename DataT>
    inline auto cuboid_sphere(const BoxT &c, const Sphere<DataT> &s) noexcept -> DataT
    {
        return cuboid_sphere(c, s.x, s.y, s.z, s.r);
    }
}  // namespace vamp::collision
