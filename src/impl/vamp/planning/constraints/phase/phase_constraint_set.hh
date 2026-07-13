#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include <vamp/planning/constraints/phase/phase_constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Runtime collection of phase-space inequality gates. An empty set is valid and
    // satisfied everywhere. Not thread-safe (constraints cache per-evaluation state): use
    // one set (with unshared constraints) per thread.
    template <typename Robot, std::size_t rake>
    class PhaseConstraintSet
    {
    public:
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        using Ptr = std::shared_ptr<const PhaseConstraint<Robot, rake>>;

        PhaseConstraintSet() = default;

        explicit PhaseConstraintSet(std::vector<Ptr> constraints) noexcept
          : constraints_(std::move(constraints))
        {
        }

        auto empty() const noexcept -> bool
        {
            return constraints_.empty();
        }

        // Run every gate's kernel on the whole block, caching all lanes for extract().
        void evaluate(const Block &x) const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->evaluate(x);
            }
        }

        // Whether one SIMD lane of the last evaluate() block satisfies every gate.
        auto extract(std::size_t lane) const noexcept -> bool
        {
            for (const auto &c : constraints_)
            {
                if (not c->extract(lane))
                {
                    return false;
                }
            }

            return true;
        }

        // Whether every SIMD lane of a block satisfies every gate: one batched kernel
        // evaluation serves all lanes.
        auto satisfied_block(const Block &x) const noexcept -> bool
        {
            evaluate(x);
            for (auto lane = 0U; lane < rake; ++lane)
            {
                if (not extract(lane))
                {
                    return false;
                }
            }

            return true;
        }

        // Largest s in (0, 1] such that (q, s * qd) satisfies every gate; exactly 1 on
        // feasible states, so applying the scale is idempotent.
        auto velocity_scale(const Configuration &z) const noexcept -> float
        {
            float s = 1.0F;
            for (const auto &c : constraints_)
            {
                s = std::min(s, c->velocity_scale(z));
            }

            // The closed-form scales are exact in real arithmetic but can land a hair
            // outside a gate in float (the scale comes from the scalar kernels, feasibility
            // is judged by the block kernels); back off so scaled states always pass.
            return (s < 1.0F) ? s * (1.0F - 1e-5F) : 1.0F;
        }

        // Whether a single flat state satisfies every gate, judged by the same block
        // kernels as extract() so steer/gate decisions agree bit-for-bit.
        auto satisfied(const Configuration &z) const noexcept -> bool
        {
            if (empty())
            {
                return true;
            }

            Block block;
            for (auto i = 0U; i < Block::num_rows; ++i)
            {
                block[i] = (i < Robot::dimension) ? z.broadcast(i) : FloatVector<rake, 1>::fill(0.0F);
            }

            evaluate(block);
            return extract(0);
        }

    private:
        std::vector<Ptr> constraints_;
    };
}  // namespace vamp::planning::constraint
