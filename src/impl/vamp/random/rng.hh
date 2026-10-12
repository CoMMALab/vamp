#pragma once

#include <memory>
#include <optional>
#include <vamp/vector.hh>
#include <vamp/random/distribution.hh>

namespace vamp::rng
{
    template <typename Robot>
    struct RNG
    {
        using Ptr = std::shared_ptr<RNG<Robot>>;
        virtual inline void reset() noexcept = 0;
        virtual inline auto next() noexcept -> FloatVector<Robot::dimension> = 0;

        // A draw near `from` (OMPL's sampleUniformNear) for planners that expand locally; samplers
        // without a local model return nothing and the planner pulls next() toward `from`.
        virtual inline auto next_near(const FloatVector<Robot::dimension> &, float) noexcept
            -> std::optional<FloatVector<Robot::dimension>>
        {
            return std::nullopt;
        }

        Distribution dist;
    };
}  // namespace vamp::rng
