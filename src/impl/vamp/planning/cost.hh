#pragma once

#include <vamp/utils.hh>

namespace vamp::planning
{
    VAMP_DEFINE_HAS_METHOD(cost)

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

    template <typename Robot>
    [[nodiscard]] inline auto cost_nonincreasing(
        const typename Robot::Configuration &prev,
        const typename Robot::Configuration &mid_old,
        const typename Robot::Configuration &mid_new,
        const typename Robot::Configuration &next) noexcept -> bool
    {
        if constexpr (has_cost_v<Robot>)
        {
            return cost<Robot>(prev, mid_new) + cost<Robot>(mid_new, next) <=
                   cost<Robot>(prev, mid_old) + cost<Robot>(mid_old, next);
        }
        else
        {
            return true;
        }
    }
}  // namespace vamp::planning
