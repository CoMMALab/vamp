#pragma once

#include <vamp/utils.hh>

namespace vamp::planning
{
    VAMP_DEFINE_HAS_METHOD(cost)

    // Edge cost a -> b used as the planners' optimization objective. Robots may provide a
    // (possibly asymmetric) Robot::cost; otherwise the symmetric configuration-space distance
    // is used. Asymmetric costs must always be evaluated in execution (start -> goal) order.
    template <typename Robot>
    [[nodiscard]] inline auto cost(
        const typename Robot::Configuration &a,
        const typename Robot::Configuration &b) noexcept -> float
    {
        if constexpr (has_cost_v<Robot>)
        {
            return Robot::cost(a, b);
        }
        else
        {
            return Robot::distance(a, b);
        }
    }
}  // namespace vamp::planning
