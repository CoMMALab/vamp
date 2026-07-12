#pragma once

#include <vamp/planning/constraints/settings.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Bound width below which a constraint row counts as manifold-defining (equality-like)
    // for active_rows(): tight [lower, upper] pins a task-space value and restricts the
    // tangent space, while wider rows are slabs that leave it free.
    inline constexpr float tight_row_width = 0.5F;

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

        // Number of scalar rows this constraint contributes to a stacked error/Jacobian.
        virtual auto n_rows() const noexcept -> std::size_t = 0;

        // Manifold-defining row flags (n_rows() entries): rows whose bounds are tighter
        // than tight_row_width. Determined by the bounds alone, so configuration-independent.
        virtual void active_rows(bool *rows) const noexcept = 0;

        // Hinged error (n_rows() entries) and *raw*-error Jacobian (n_rows() x
        // Robot::dimension, row-major) of one SIMD lane of q, for tangent-space (chart)
        // construction. The hinge mask of squared_error() cannot apply here: on the
        // manifold the hinged error vanishes, so masking would zero every row. Overwrites
        // the cache of squared_error(); re-call squared_error() before step().
        virtual void error_jacobian(const Block &q, std::size_t lane, float *err, float *jac)
            const noexcept = 0;
    };
}  // namespace vamp::planning::constraint
