#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_capsule(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        auto dot = dot_3(x - c.x1, y - c.y1, z - c.z1, c.xv, c.yv, c.zv);
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1 + c.xv * cdf, c.y1 + c.yv * cdf, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_capsule(const Capsule<DataT> &c, const Sphere<DataT> &s) noexcept -> DataT
    {
        return sphere_capsule(c, s.x, s.y, s.z, s.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_capsule(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        auto dot = (z - c.z1) * c.zv;
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto sum = sql2_3(x, y, z, c.x1, c.y1, c.z1 + c.zv * cdf);
        auto rs = r + c.r;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_z_aligned_capsule(const Capsule<DataT> &c, const Sphere<DataT> &s) noexcept
        -> DataT
    {
        return sphere_z_aligned_capsule(c, s.x, s.y, s.z, s.r);
    }

    // Signed distance + gradient (see DistanceGradient) from the query sphere (x, y, z, r)
    // to capsule `c`, w.r.t. the query's center: the closest point on the segment is
    // recomputed exactly as in sphere_capsule above, and the gradient is the (normalized)
    // vector from that point to the query. `eps` only guards the degenerate case where the
    // query's center coincides with the segment.
    template <typename DataT>
    inline constexpr auto sphere_capsule_gradient(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r,
        DataT eps = DataT(1e-6F)) noexcept -> DistanceGradient<DataT>
    {
        auto dot = dot_3(x - c.x1, y - c.y1, z - c.z1, c.xv, c.yv, c.zv);
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto gx = x - (c.x1 + c.xv * cdf);
        auto gy = y - (c.y1 + c.yv * cdf);
        auto gz = z - (c.z1 + c.zv * cdf);

        auto norm = sqrt(dot_3(gx, gy, gz, gx, gy, gz));
        auto safe_norm = norm.max(eps);

        return DistanceGradient<DataT>{norm - (r + c.r), gx / safe_norm, gy / safe_norm, gz / safe_norm};
    }

    // z-aligned counterpart of sphere_capsule_gradient (see sphere_z_aligned_capsule above).
    template <typename DataT>
    inline constexpr auto sphere_z_aligned_capsule_gradient(
        const Capsule<DataT> &c,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r,
        DataT eps = DataT(1e-6F)) noexcept -> DistanceGradient<DataT>
    {
        auto dot = (z - c.z1) * c.zv;
        auto cdf = (dot * c.rdv).clamp(0.F, 1.F);

        auto gx = x - c.x1;
        auto gy = y - c.y1;
        auto gz = z - (c.z1 + c.zv * cdf);

        auto norm = sqrt(dot_3(gx, gy, gz, gx, gy, gz));
        auto safe_norm = norm.max(eps);

        return DistanceGradient<DataT>{norm - (r + c.r), gx / safe_norm, gy / safe_norm, gz / safe_norm};
    }
}  // namespace vamp::collision
