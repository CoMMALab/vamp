#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

namespace vamp::collision
{
    template <typename DataT>
    inline constexpr auto sphere_heightfield(
        const HeightField<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DataT
    {
        using IndexT = IntVector<DataT::num_scalars_per_row, DataT::num_rows>;
        auto xo = a.x - x;
        auto yo = a.y - y;

        auto xs = (a.xs * xo + a.xd2).clamp(0.F, static_cast<float>(a.xd)).floor();
        auto ys = (a.ys * yo + a.yd2).clamp(0.F, static_cast<float>(a.yd)).floor();

        auto index = ys * a.xd + xs;
        IndexT indices = index.template to<IndexT>();

        auto zh = DataT::gather(a.data.data(), indices);
        auto zhs = a.zs * zh + a.z;

        return z - r - zhs;
    }

    // Signed distance (identical to sphere_heightfield above) + gradient (see
    // DistanceGradient) w.r.t. the query sphere's center. The height lookup is
    // piecewise-constant (nearest-cell `gather`), so its true x/y derivative is zero
    // almost everywhere with discontinuities at cell boundaries; only the z-derivative is
    // meaningful, so straight up is used as the gradient direction (a known approximation).
    template <typename DataT>
    inline constexpr auto sphere_heightfield_gradient(
        const HeightField<DataT> &a,
        const DataT &x,
        const DataT &y,
        const DataT &z,
        const DataT &r) noexcept -> DistanceGradient<DataT>
    {
        return DistanceGradient<DataT>{
            sphere_heightfield(a, x, y, z, r), DataT(0.F), DataT(0.F), DataT(1.F)};
    }
}  // namespace vamp::collision
