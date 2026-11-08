#pragma once

#include <vamp/robots/traits.hh>
#include <vamp/planning/space.hh>
#include <vamp/utils.hh>

namespace vamp::planning
{
    VAMP_DEFINE_HAS_METHOD(cost)

    template <typename Robot>
    [[nodiscard]] inline auto cost(const space::State<Robot> &a, const space::State<Robot> &b) noexcept
        -> float
    {
        if constexpr (has_cost_v<Robot>)
        {
            return Robot::cost(a, b);
        }
        else
        {
            return space::distance<Robot>(a, b);
        }
    }

    template <typename Robot>
    [[nodiscard]] inline auto cost_nonincreasing(
        const space::State<Robot> &prev,
        const space::State<Robot> &mid_old,
        const space::State<Robot> &mid_new,
        const space::State<Robot> &next) noexcept -> bool
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
