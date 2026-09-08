#pragma once

#include <type_traits>

#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Detects robots that declare `so2_offsets` (offsets of (cos, sin) unit-circle joint
    // pairs, e.g. a planar mobile base heading). Robots that don't declare it are treated
    // as having none -- this keeps every other robot header, none of which declare
    // `so2_offsets`, compiling unchanged.
    template <typename Robot, typename = void>
    struct HasSo2Offsets : std::false_type
    {
    };

    template <typename Robot>
    struct HasSo2Offsets<Robot, std::void_t<decltype(Robot::so2_offsets)>> : std::true_type
    {
    };

    template <typename Robot>
    inline constexpr bool has_so2_offsets_v = HasSo2Offsets<Robot>::value;

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

    // Renormalize the (cos, sin) segments of circle-parameterized joints back onto S^1 --
    // the SO(2) analogue of renormalize_so3 above, for joints like a planar mobile base
    // heading that are stored as a unit-circle pair rather than a raw angle so that chord
    // interpolation and gradient steps have a well-defined linear-then-renormalize (nlerp)
    // retraction, same as the quaternion case.
    template <typename Robot, std::size_t rake>
    inline void renormalize_so2(typename Robot::template ConfigurationBlock<rake> &q) noexcept
    {
        if constexpr (has_so2_offsets_v<Robot>)
        {
            for (const auto offset : Robot::so2_offsets)
            {
                auto norm =
                    (q[offset] * q[offset] + q[offset + 1] * q[offset + 1]).sqrt();
                q[offset] = q[offset] / norm;
                q[offset + 1] = q[offset + 1] / norm;
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
        renormalize_so2<Robot, rake>(q);
    }
}  // namespace vamp::planning::constraint
