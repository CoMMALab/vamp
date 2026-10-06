#pragma once

#include <algorithm>
#include <cmath>

namespace vamp::collision
{
    using Point = std::array<float, 3>;

    template <typename DataT>
    inline constexpr auto dot_2(const DataT &ax, const DataT &ay, const DataT &bx, const DataT &by) -> DataT
    {
        return (ax * bx) + (ay * by);
    }

    template <typename DataT>
    inline constexpr auto dot_3(
        const DataT &ax,
        const DataT &ay,
        const DataT &az,
        const DataT &bx,
        const DataT &by,
        const DataT &bz) -> DataT
    {
        return (ax * bx) + (ay * by) + (az * bz);
    }

    template <typename DataT>
    inline constexpr auto sql2_3(
        const DataT &ax,
        const DataT &ay,
        const DataT &az,
        const DataT &bx,
        const DataT &by,
        const DataT &bz) -> DataT
    {
        const auto xs = (ax - bx);
        const auto ys = (ay - by);
        const auto zs = (az - bz);

        return dot_3(xs, ys, zs, xs, ys, zs);
    }

    template <typename DataT>
    inline constexpr auto clamp(const DataT &v, const DataT &lower, const DataT &upper) -> DataT
    {
        return v.clamp(lower, upper);
    }

    template <>
    inline constexpr auto clamp<float>(const float &v, const float &lower, const float &upper) -> float
    {
        return std::max(std::min(v, upper), lower);
    }

    template <typename DataT>
    inline constexpr auto sqrt(const DataT &v) -> DataT
    {
        return v.sqrt();
    }

    template <>
    inline constexpr auto sqrt<float>(const float &v) -> float
    {
        return std::sqrt(v);
    }

    // Signed distance (true distance, not squared) from a query sphere to a shape's
    // surface, plus the (unit, outward-pointing) gradient of that distance w.r.t. the
    // query sphere's center -- what a `sphere_*` distance kernel's `_gradient` sibling
    // (sphere_sphere.hh, sphere_cuboid.hh, sphere_capsule.hh, sphere_heightfield.hh)
    // returns, and what `sphere_environment_min_distance_gradient` (validity.hh)
    // minimizes over every primitive in an Environment.
    template <typename DataT>
    struct DistanceGradient
    {
        DataT distance;
        DataT gx;
        DataT gy;
        DataT gz;
    };
}  // namespace vamp::collision
