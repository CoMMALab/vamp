#pragma once

#include <cstddef>
#include <memory>
#include <utility>

#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::rng
{
    // Samples of the robot's SplineSpace (Robot::SplineSpace, generated with "spline_constraint": true): a
    // configuration from the robot's own RNG, and the curve parameter from the base-2 van der Corput sequence (the
    // Halton sequence leaves base 2 out of the joints' bases, so t is stratified without correlating with them). The
    // configuration is not on the manifold: a planner that wants a state on it projects the sample at its t.
    template <typename Robot>
    struct SplineRNG : public RNG<Robot, typename Robot::SplineSpace>
    {
        using Space = typename Robot::SplineSpace;

        explicit SplineRNG(typename RNG<Robot>::Ptr inner) noexcept : inner_(std::move(inner))
        {
        }

        virtual ~SplineRNG() = default;

        inline void reset() noexcept override final
        {
            inner_->reset();
            index_ = 0;
        }

        inline auto next() noexcept -> FloatVector<Space::dimension> override final
        {
            index_++;

            float t = 0.F;
            float scale = 0.5F;
            for (auto bits = index_; bits != 0; bits >>= 1)
            {
                if (bits & 1U)
                {
                    t += scale;
                }

                scale *= 0.5F;
            }

            return Space::make_state(inner_->next(), t);
        }

    private:
        typename RNG<Robot>::Ptr inner_;
        std::size_t index_ = 0;
    };
}  // namespace vamp::rng
