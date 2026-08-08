#pragma once

#include <vamp/utils.hh>

namespace vamp::planning
{
    VAMP_DEFINE_HAS_METHOD(cost)
    VAMP_DEFINE_HAS_METHOD(cost_grad)

    template <typename Space>
    [[nodiscard]] inline auto
    cost(const typename Space::State &a, const typename Space::State &b) noexcept -> float
    {
        if constexpr (has_cost_v<Space>)
        {
            return Space::cost(a, b);
        }
        else
        {
            return Space::distance(a, b);
        }
    }

    template <typename Space>
    [[nodiscard]] inline auto cost_nonincreasing(
        const typename Space::State &prev,
        const typename Space::State &mid_old,
        const typename Space::State &mid_new,
        const typename Space::State &next) noexcept -> bool
    {
        if constexpr (has_cost_v<Space>)
        {
            return cost<Space>(prev, mid_new) + cost<Space>(mid_new, next) <=
                   cost<Space>(prev, mid_old) + cost<Space>(mid_old, next);
        }
        else
        {
            return true;
        }
    }
}  // namespace vamp::planning
