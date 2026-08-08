#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <utility>

#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::rng
{
    // Wraps a flask-robot sampler and rescales each sample's velocity block so that
    // kinetic energy is uniform on [0, max_energy] rather than inherited from the
    // velocity box. Box-uniform draws concentrate energy near its maximum in high
    // dimension (near-rest samples have vanishing probability), so every steer target
    // demands a fast arrival and the local planner's arcs sweep wide; shaping the
    // energy distribution rest-biases targets at the source. KE is homogeneous of
    // degree 2 in the velocity, so hitting a target energy is a single scalar rescale.
    template <typename Robot, typename Space = Robot>
    struct KineticEnergyShaped : public RNG<Robot, Space>
    {
        using Configuration = FloatVector<Space::dimension>;
        static constexpr std::size_t n_q = Space::flat_dimension;

        KineticEnergyShaped(typename RNG<Robot, Space>::Ptr inner, float max_energy) noexcept
          : inner_(std::move(inner)), max_energy_(max_energy)
        {
        }

        virtual ~KineticEnergyShaped() = default;

        inline void reset() noexcept override final
        {
            inner_->reset();
            index_ = 0;
        }

        inline auto next() noexcept -> Configuration override final
        {
            const auto z = inner_->next();
            const float target = max_energy_ * van_der_corput(++index_);

            alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded> buf;
            z.to_array(buf);

            std::array<float, Space::dimension> x;
            std::copy_n(buf.begin(), Space::dimension, x.begin());
            const float energy = Space::kinetic_energy(x);
            if (energy <= 0.F)
            {
                return z;
            }

            // Clamp scale-ups so no joint leaves its sampled velocity box.
            float scale = std::sqrt(target / energy);
            for (auto i = 0U; i < n_q; ++i)
            {
                const auto v = std::abs(buf[n_q + i]);
                if (scale * v > Space::velocity_limits[i])
                {
                    scale = Space::velocity_limits[i] / v;
                }
            }

            for (auto i = 0U; i < n_q; ++i)
            {
                buf[n_q + i] *= scale;
            }

            return Configuration(buf.data());
        }

    private:
        // Base-2 van der Corput over an internal counter: a deterministic
        // low-discrepancy uniform on (0, 1), so halton-driven runs stay reproducible
        // under skip()/reset(). Base 2 is free here since the inner Halton bases
        // start at 3.
        inline static auto van_der_corput(std::size_t n) noexcept -> float
        {
            float r = 0.F;
            float f = 0.5F;
            while (n > 0)
            {
                r += f * static_cast<float>(n & 1U);
                n >>= 1U;
                f *= 0.5F;
            }

            return r;
        }

        typename RNG<Robot, Space>::Ptr inner_;
        float max_energy_;
        std::size_t index_ = 0;
    };
}  // namespace vamp::rng
