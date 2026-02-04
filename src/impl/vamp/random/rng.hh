#pragma once

#include <memory>
#include <vamp/vector.hh>
#include <vamp/random/distribution.hh>
#include <vamp/random/uniform.hh>

namespace vamp::rng
{
    template <typename Robot>
    struct RNG
    {
        using Ptr = std::shared_ptr<RNG<Robot>>;
        virtual inline void reset() noexcept = 0;
        virtual inline auto next() noexcept -> FloatVector<Robot::dimension> = 0;

        Distribution dist;
        Uniform<Robot> uniform;
    };
}  // namespace vamp::rng
