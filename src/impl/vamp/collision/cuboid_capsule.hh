#pragma once

#include <array>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    namespace detail
    {
        // Squared distance between the segment p(t) = p0 + t * d, t in [0, 1], and an axis-aligned box
        // centered at the origin with half-lengths `e`. `p0` and `d` are given in the box's frame.
        //
        // f(t) = |p(t) - clamp(p(t), -e, e)|^2 is convex and piecewise quadratic, so its derivative
        //   f'(t) / 2 = sum_k d_k * (q_k - clamp(q_k, -e_k, e_k)),   q = p(t),
        // is continuous and nondecreasing. We find its root by bisection with a fixed trip count (so every
        // SIMD lane does the same work). Since f is minimized there, the error in f is second order in
        // the bracket width 2^-Iterations: ~1e-6 relative to the segment length squared at the default
        // of 10, i.e. microns for decimeter-scale links.
        template <std::size_t Iterations, typename DataT>
        inline auto segment_aabb_sql2(
            const std::array<DataT, 3> &p0,
            const std::array<DataT, 3> &d,
            const std::array<DataT, 3> &e) noexcept -> DataT
        {
            const std::array<DataT, 3> ne = {-e[0], -e[1], -e[2]};

            auto excess = [&](const DataT &t, std::array<DataT, 3> &q, std::array<DataT, 3> &ex)
            {
                for (auto k = 0U; k < 3; ++k)
                {
                    q[k] = p0[k] + d[k] * t;
                    ex[k] = q[k] - clamp(q[k], ne[k], e[k]);
                }
            };

            DataT lo(0.F);
            DataT hi(1.F);

            std::array<DataT, 3> q;
            std::array<DataT, 3> ex;
            for (auto i = 0U; i < Iterations; ++i)
            {
                const auto mid = (lo + hi) * 0.5F;
                excess(mid, q, ex);

                // Derivative at mid > 0 => minimizer is left of mid
                const auto grad = dot_3(d[0], d[1], d[2], ex[0], ex[1], ex[2]);
                hi = hi.blend(mid, grad > 0.F);
                lo = lo.blend(mid, grad <= 0.F);
            }

            const auto t = (lo + hi) * 0.5F;
            excess(t, q, ex);
            return dot_3(ex[0], ex[1], ex[2], ex[0], ex[1], ex[2]);
        }

        template <std::size_t Iterations, typename BoxT, typename DataT>
        inline auto cuboid_segment_sql2(
            const BoxT &c,
            const DataT &x1,
            const DataT &y1,
            const DataT &z1,
            const DataT &xv,
            const DataT &yv,
            const DataT &zv) noexcept -> DataT
        {
            const auto xs = x1 - c.x;
            const auto ys = y1 - c.y;
            const auto zs = z1 - c.z;

            const std::array<DataT, 3> p0 = {
                dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xs, ys, zs),
                dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xs, ys, zs),
                dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xs, ys, zs)};

            const std::array<DataT, 3> d = {
                dot_3(c.axis_1_x, c.axis_1_y, c.axis_1_z, xv, yv, zv),
                dot_3(c.axis_2_x, c.axis_2_y, c.axis_2_z, xv, yv, zv),
                dot_3(c.axis_3_x, c.axis_3_y, c.axis_3_z, xv, yv, zv)};

            const std::array<DataT, 3> e = {c.axis_1_r, c.axis_2_r, c.axis_3_r};

            return segment_aabb_sql2<Iterations>(p0, d, e);
        }
    }  // namespace detail

    // Robot box `c` vs. environment capsule. Negative in lanes that collide.
    //
    // This is exact up to the bisection tolerance documented in detail::segment_aabb_sql2.
    template <std::size_t Iterations = 10, typename BoxT, typename DataT>
    inline auto cuboid_capsule(const BoxT &c, const Capsule<DataT> &cap) noexcept -> DataT
    {
        const auto sql2 = detail::cuboid_segment_sql2<Iterations>(
            c, cap.x1, cap.y1, cap.z1, cap.xv, cap.yv, cap.zv);
        return sql2 - cap.r * cap.r;
    }

    // As above, for a z-aligned capsule, whose axis is (0, 0, zv) from (x1, y1, z1); only z1 and zv are
    // read from the vector, matching sphere_z_aligned_capsule.
    template <std::size_t Iterations = 10, typename BoxT, typename DataT>
    inline auto cuboid_z_aligned_capsule(const BoxT &c, const Capsule<DataT> &cap) noexcept -> DataT
    {
        const DataT zero(0.F);
        const auto sql2 =
            detail::cuboid_segment_sql2<Iterations>(c, cap.x1, cap.y1, cap.z1, zero, zero, cap.zv);
        return sql2 - cap.r * cap.r;
    }
}  // namespace vamp::collision
