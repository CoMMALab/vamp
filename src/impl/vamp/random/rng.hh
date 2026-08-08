#pragma once

#include <memory>
#include <vamp/vector.hh>
#include <vamp/random/distribution.hh>

namespace vamp::rng
{
    template <typename Robot, typename Space = Robot>
    struct RNG
    {
        using Ptr = std::shared_ptr<RNG<Robot, Space>>;
        virtual inline void reset() noexcept = 0;
        virtual inline auto next() noexcept -> FloatVector<Space::dimension> = 0;

        Distribution dist;
    };
}  // namespace vamp::rng
