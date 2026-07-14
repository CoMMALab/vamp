#pragma once

#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Type-erased phase-space inequality constraint g(q, qd) <= 0 over a rake-wide block of
    // flat states z = (q, qd). Unlike Constraint, a phase constraint defines no manifold:
    // states are checked against it, never projected onto it, and feasibility is restored by scaling
    // velocities (every instance is homogeneous in qd, so scaling qd down always reaches
    // the feasible set).
    //
    // Implementations cache per-lane results of the last evaluate() call for extract(), so
    // batch callers pay each kernel once per block instead of per lane. Instances are not
    // thread-safe: use one instance per thread.
    template <typename Robot, std::size_t rake>
    struct PhaseConstraint
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;

        virtual ~PhaseConstraint() = default;

        // Run the kernel on the whole block, caching every lane for extract(). Only the
        // [q; qd] rows of the block are read; acceleration rows are ignored.
        virtual void evaluate(const Block &x) const noexcept = 0;

        // Whether one SIMD lane of the last evaluate() block satisfies the constraint.
        virtual auto extract(std::size_t lane) const noexcept -> bool = 0;

        // Largest s in (0, 1] such that (q, s * qd) satisfies the constraint; exactly 1 if
        // z = (q, qd) is already feasible, so applying the scale is idempotent.
        virtual auto velocity_scale(const Configuration &z) const noexcept -> float = 0;
    };
}  // namespace vamp::planning::constraint
