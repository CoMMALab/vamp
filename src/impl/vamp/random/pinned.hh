#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <utility>

#include <vamp/random/rng.hh>

namespace vamp::rng
{
    // Plan over a subset of joints at runtime: overwrites the pinned dimensions of every
    // sample drawn from the wrapped RNG with fixed values. Since interpolation preserves
    // dimensions shared by both endpoints and identical dimensions contribute zero to
    // distances, planners and simplification need no changes — the whole tree stays
    // constant on the pinned dimensions. `mask` is 1 on pinned dimensions, 0 on free ones.
    template <typename Robot, typename Space = Robot>
    struct PinnedRNG : public RNG<Robot, Space>
    {
        using Configuration = FloatVector<Space::dimension>;

        PinnedRNG(
            typename RNG<Robot, Space>::Ptr inner,
            const Configuration &mask,
            const Configuration &values)
          : inner(std::move(inner))
          , mask(mask)
          , values(values)
          , unmask(Configuration::fill(1.F) - mask)
          , pinned(values * mask)
        {
        }

        virtual ~PinnedRNG() = default;

        inline void reset() noexcept override final
        {
            inner->reset();
            inner->dist.reset();
        }

        inline auto next() noexcept -> Configuration override final
        {
            return inner->next() * unmask + pinned;
        }

        // Pinned-dimension flags, for consumers that mask per scalar dimension (e.g.
        // constraint-projection Jacobian columns).
        auto pinned_dims() const noexcept -> std::array<bool, Space::dimension>
        {
            alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded> ms;
            mask.to_array(ms.data());

            std::array<bool, Space::dimension> out;
            for (auto i = 0U; i < Space::dimension; ++i)
            {
                out[i] = ms[i] != 0.F;
            }

            return out;
        }

        // Check that a configuration agrees with the pinned values (e.g. planner
        // start/goal inputs, which must lie on the pinned slice).
        auto matches(const Configuration &x, float tolerance = 1e-4F) const noexcept -> bool
        {
            alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded> xs, ms, vs;
            x.to_array(xs.data());
            mask.to_array(ms.data());
            values.to_array(vs.data());

            for (auto i = 0U; i < Space::dimension; ++i)
            {
                if (ms[i] != 0.F and std::abs(xs[i] - vs[i]) > tolerance)
                {
                    return false;
                }
            }

            return true;
        }

        typename RNG<Robot, Space>::Ptr inner;
        const Configuration mask;
        const Configuration values;
        const Configuration unmask;
        const Configuration pinned;
    };
}  // namespace vamp::rng
