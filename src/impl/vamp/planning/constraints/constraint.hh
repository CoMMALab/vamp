#pragma once

#include <vamp/planning/constraints/settings.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Type-erased manifold constraint over a rake-wide block of configurations.
    //
    // Implementations cache the Jacobian and hinged error of the last squared_error() call;
    // step() consumes that cache, so callers must call squared_error(q) before step(q, ...)
    // on the same q. This saves one full error/Jacobian evaluation per projection iteration,
    // since convergence checks already evaluate at the stepped configuration. Instances are
    // not thread-safe: use one instance per thread.
    template <typename Robot, std::size_t rake>
    struct Constraint
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;

        virtual ~Constraint() = default;

        // Per-lane squared hinged constraint error at q. Caches the Jacobian and hinged
        // error for step().
        virtual auto squared_error(const Block &q) const noexcept -> FloatVector<rake, 1> = 0;

        // One descent step in place, using the cache of the last squared_error() call.
        virtual void step(Block &q, ProjMethod method, float alpha) const noexcept = 0;
    };
}  // namespace vamp::planning::constraint
