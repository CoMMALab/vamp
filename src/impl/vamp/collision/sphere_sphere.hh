#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(
        const DataT &ax,
        const DataT &ay,
        const DataT &az,
        const DataT &ar,
        const DataT &bx,
        const DataT &by,
        const DataT &bz,
        const DataT &br) noexcept -> DataT
    {
        auto sum = sql2_3(ax, ay, az, bx, by, bz);
        auto rs = ar + br;
        return sum - rs * rs;
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(
        const Sphere<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        return sphere_sphere_sql2(a.x, a.y, a.z, a.r, x, y, z, r);
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_sql2(const Sphere<DataT> &a, const Sphere<DataT> &b) noexcept -> DataT
    {
        return sphere_sphere_sql2(a, b.x, b.y, b.z, b.r);
    }

    template <typename DataT>
    inline constexpr auto sphere_sphere_l2(const Sphere<DataT> &a, const Sphere<DataT> &b) noexcept -> DataT
    {
        auto sum = sql2_3(a.x, a.y, a.z, b.x, b.y, b.z).sqrt();
        return sum - (a.r + b.r);
    }

    // Signed distance + gradient (see DistanceGradient) from the query sphere (x, y, z, r)
    // to fixed sphere `a`, w.r.t. the query's center. The gradient is exact everywhere
    // except at the query's exact center coincidence with `a`'s, where `eps` keeps the
    // normalization finite (direction is arbitrary in that degenerate case).
    template <typename DataT>
    inline constexpr auto sphere_sphere_gradient(
        const Sphere<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r,
        DataT eps = DataT(1e-6F)) noexcept -> DistanceGradient<DataT>
    {
        auto dx = x - a.x;
        auto dy = y - a.y;
        auto dz = z - a.z;
        auto norm = sqrt(dot_3(dx, dy, dz, dx, dy, dz));
        auto safe_norm = norm.max(eps);

        return DistanceGradient<DataT>{norm - (a.r + r), dx / safe_norm, dy / safe_norm, dz / safe_norm};
    }
}  // namespace vamp::collision
