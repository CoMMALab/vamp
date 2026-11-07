#pragma once

#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Renormalize the quaternion segments of quaternion-parameterized joints back onto S^3.
    // Additive updates (gradient steps, chord interpolation) leave the sphere; dividing by
    // the norm is a valid retraction, so linear-then-renormalize is nlerp.
    template <typename Robot, std::size_t rake>
    inline void renormalize_so3(typename Robot::template ConfigurationBlock<rake> &q) noexcept
    {
        if constexpr (Robot::so3_offsets.size() > 0)
        {
            for (const auto offset : Robot::so3_offsets)
            {
                auto norm = (q[offset] * q[offset] + q[offset + 1] * q[offset + 1] +
                             q[offset + 2] * q[offset + 2] + q[offset + 3] * q[offset + 3])
                                .sqrt();
                for (auto i = offset; i < offset + 4; ++i)
                {
                    q[i] = q[i] / norm;
                }
            }
        }
    }

    // First-order update along the projection gradient, then clamp to the joint limits by
    // round-tripping through the robot's unit-scaled configuration space.
    template <typename Robot, std::size_t rake>
    inline void integrate_step(
        typename Robot::template ConfigurationBlock<rake> &q,
        const typename Robot::template ConfigurationBlock<rake> &gradient,
        float alpha) noexcept
    {
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            q[i] = q[i] - gradient[i] * alpha;
        }

        Robot::descale_configuration_block(q);
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            q[i] = q[i].clamp(0.F, 1.F);
        }
        Robot::scale_configuration_block(q);

        renormalize_so3<Robot, rake>(q);
    }
}  // namespace vamp::planning::constraint
