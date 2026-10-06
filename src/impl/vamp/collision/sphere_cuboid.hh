#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_cuboid(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &rsq) noexcept -> DataT
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = (dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs).abs() - c.axis_1_r).max(0.);
        auto a2 = (dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs).abs() - c.axis_2_r).max(0.);
        auto a3 = (dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs).abs() - c.axis_3_r).max(0.);

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    template <typename DataT>
    inline constexpr auto sphere_cuboid(const Cuboid<DataT> &c, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_cuboid(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &rsq) noexcept -> DataT
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto a1 = (dot_2(c.axis_1_x, c.axis_1_y, xs, ys).abs() - c.axis_1_r).max(0.);
        auto a2 = (dot_2(c.axis_2_x, c.axis_2_y, xs, ys).abs() - c.axis_2_r).max(0.);
        auto a3 = (zs.abs() - c.axis_3_r).max(0.);

        auto sum = dot_3(a1, a2, a3, a1, a2, a3);
        return sum - rsq;
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_cuboid(const Cuboid<DataT> &c, const Sphere<DataT> &s) noexcept
        -> DataT
    {
        return sphere_z_aligned_cuboid(c, s.x, s.y, s.z, s.r * s.r);
    }

    // Signed distance + gradient (see DistanceGradient) from the query sphere (x, y, z, r)
    // to cuboid `c`, w.r.t. the query's center. `e1`/`e2`/`e3` are the signed excess of the
    // query's projection onto each local axis beyond that axis' half-extent (zero when the
    // projection already lies within the slab on that axis); their sum along the cuboid's
    // world axes is exactly (query - nearest surface point), so its norm is the true
    // distance and its normalized direction the outward gradient. When the query is fully
    // inside the cuboid, all three are zero and the gradient direction is arbitrary (a
    // known approximation, `eps` only keeps the division finite -- see also the
    // heightfield gradient below for the same kind of approximation).
    template <typename DataT>
    inline constexpr auto sphere_cuboid_gradient(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r,
        DataT eps = DataT(1e-6F)) noexcept -> DistanceGradient<DataT>
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto d1 = dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs);
        auto d2 = dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs);
        auto d3 = dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs);

        auto e1 = d1 - clamp(d1, -c.axis_1_r, c.axis_1_r);
        auto e2 = d2 - clamp(d2, -c.axis_2_r, c.axis_2_r);
        auto e3 = d3 - clamp(d3, -c.axis_3_r, c.axis_3_r);

        auto gx = c.axis_1_x * e1 + c.axis_2_x * e2 + c.axis_3_x * e3;
        auto gy = c.axis_1_y * e1 + c.axis_2_y * e2 + c.axis_3_y * e3;
        auto gz = c.axis_1_z * e1 + c.axis_2_z * e2 + c.axis_3_z * e3;

        auto norm = sqrt(dot_3(gx, gy, gz, gx, gy, gz));
        auto safe_norm = norm.max(eps);

        return DistanceGradient<DataT>{norm - r, gx / safe_norm, gy / safe_norm, gz / safe_norm};
    }

    // z-aligned counterpart of sphere_cuboid_gradient (see sphere_z_aligned_cuboid above).
    template <typename DataT>
    inline constexpr auto sphere_z_aligned_cuboid_gradient(
        const Cuboid<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r,
        DataT eps = DataT(1e-6F)) noexcept -> DistanceGradient<DataT>
    {
        auto xs = x - c.x;
        auto ys = y - c.y;
        auto zs = z - c.z;

        auto d1 = dot_2(c.axis_1_x, c.axis_1_y, xs, ys);
        auto d2 = dot_2(c.axis_2_x, c.axis_2_y, xs, ys);
        auto d3 = zs;

        auto e1 = d1 - clamp(d1, -c.axis_1_r, c.axis_1_r);
        auto e2 = d2 - clamp(d2, -c.axis_2_r, c.axis_2_r);
        auto e3 = d3 - clamp(d3, -c.axis_3_r, c.axis_3_r);

        auto gx = c.axis_1_x * e1 + c.axis_2_x * e2;
        auto gy = c.axis_1_y * e1 + c.axis_2_y * e2;
        auto gz = e3;

        auto norm = sqrt(dot_3(gx, gy, gz, gx, gy, gz));
        auto safe_norm = norm.max(eps);

        return DistanceGradient<DataT>{norm - r, gx / safe_norm, gy / safe_norm, gz / safe_norm};
    }
}  // namespace vamp::collision
