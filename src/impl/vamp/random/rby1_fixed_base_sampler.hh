#pragma once

#include <array>
#include <memory>
#include <utility>

#include <vamp/random/rng.hh>
#include <vamp/robots/rby1.hh>

namespace vamp::rng
{
    // Problem-specific sampler for RBY1::ParameterizedSpace task-space planning: wraps an
    // inner ParameterizedSpace sampler and, on every draw, (1) fixes the mobile base at the
    // origin and (2) remaps the t_mid_pose position from ParameterizedSpace::sample()'s
    // default +/-2 box down to a +/-position_half_range box around a given start position.
    // Torso, psi_left/psi_right, and the t_mid_pose orientation are left exactly as the inner
    // sampler draws them ("sampled normally") -- this only overrides base and position.
    struct RBY1FixedBaseSampler : public RNG<vamp::robots::RBY1, vamp::robots::RBY1::ParameterizedSpace>
    {
        using Robot = vamp::robots::RBY1;
        using ParameterizedSpace = Robot::ParameterizedSpace;

        // ParameterizedSpace::sample()'s default box for t_mid_pose position (see rby1.hh's
        // generated sample(): y[12..14] = -2 + 4*u). Used to invert back to the underlying
        // uniform draw before remapping into [start - half_range, start + half_range].
        static constexpr float default_position_lower = -2.0F;
        static constexpr float default_position_range = 4.0F;

        RBY1FixedBaseSampler(
            vamp::rng::RNG<Robot, ParameterizedSpace>::Ptr inner_in,
            std::array<float, 3> start_position_in,
            float position_half_range_in = 0.4F) noexcept
          : inner(std::move(inner_in))
          , start_position(start_position_in)
          , position_half_range(position_half_range_in)
        {
        }

        virtual ~RBY1FixedBaseSampler() = default;

        inline void reset() noexcept override
        {
            inner->reset();
            inner->dist.reset();
        }

        inline auto next() noexcept -> ParameterizedSpace::State override
        {
            auto sample = inner->next();
            auto y = sample.to_array();

            // base: fixed at the origin (x, y, cos(rz), sin(rz)).
            y[0] = 0.0F;
            y[1] = 0.0F;
            y[2] = 1.0F;
            y[3] = 0.0F;

            // t_mid_pose position: recover the inner sampler's underlying uniform draw from its
            // default box, then remap into the requested range around start_position.
            for (std::size_t i = 0; i < 3; ++i)
            {
                const float u = (y[12 + i] - default_position_lower) / default_position_range;
                y[12 + i] = start_position[i] - position_half_range + 2.0F * position_half_range * u;
            }

            return ParameterizedSpace::State(y.data());
        }

        vamp::rng::RNG<Robot, ParameterizedSpace>::Ptr inner;
        std::array<float, 3> start_position;
        float position_half_range;
    };
}  // namespace vamp::rng
