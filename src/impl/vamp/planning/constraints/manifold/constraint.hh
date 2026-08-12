#pragma once

#include <array>

#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Bound width below which a constraint row counts as manifold-defining (equality-like)
    // for active_rows(): tight [lower, upper] pins a task-space value and restricts the
    // tangent space, while wider rows are slabs that leave it free.
    inline constexpr float tight_row_width = 0.5F;

    // Type-erased manifold constraint over a rake-wide block of configurations.
    //
    // Implementations cache the Jacobian and violation error of the last squared_error() call;
    // step() consumes that cache, so callers must call squared_error(q) before step(q, ...)
    // on the same q. This saves one full error/Jacobian evaluation per projection iteration,
    // since convergence checks already evaluate at the stepped configuration. Instances are
    // not thread-safe: use one instance per thread.
    template <typename Robot, std::size_t rake>
    struct Constraint
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;

        virtual ~Constraint() = default;

        // Per-lane squared constraint-violation error at q. Caches the Jacobian and
        // violation error for step().
        virtual auto squared_error(const Block &q) const noexcept -> FloatVector<rake, 1> = 0;

        // One descent step in place, using the cache of the last squared_error() call.
        virtual void step(Block &q, ProjMethod method, float alpha) const noexcept = 0;

        // Number of scalar rows this constraint contributes to a stacked error/Jacobian.
        virtual auto n_rows() const noexcept -> std::size_t = 0;

        // Manifold-defining row flags (n_rows() entries): rows whose bounds are tighter
        // than tight_row_width. Determined by the bounds alone, so configuration-independent.
        virtual void active_rows(bool *rows) const noexcept = 0;

        // Pfaffian row flags (n_rows() entries): rows that restrict
        // velocity direction only and carry no position error, so executed edges can
        // drift along their normals (see ChartSettings::max_slip_fraction). Default: none.
        virtual void pfaffian_rows(bool *rows) const noexcept
        {
            for (std::size_t i = 0, n = n_rows(); i < n; ++i)
            {
                rows[i] = false;
            }
        }

        // Zero the pinned dimensions' columns (pinned[j] true, Robot::dimension entries) of
        // the Jacobian cached by the last squared_error() call, so the next step() solves
        // for the optimal descent within the active subspace and cannot move pinned joints.
        // Default: no-op, for constraints whose step() is already a no-op.
        virtual void mask_jacobian(const bool * /*pinned*/) const noexcept
        {
        }

        // Whether this constraint contributes a position error to manifold projection. Pfaffian
        // (velocity-only) constraints do not -- their step() is a no-op -- so the coupled
        // Gauss-Newton step (ConstraintSet::step_coupled_in_place) excludes them. Default: true.
        virtual auto projects() const noexcept -> bool
        {
            return true;
        }

        // Append this constraint's cached SIMD error (n_rows() rows) and Jacobian (n_rows() x
        // Robot::dimension, row-major) from the last squared_error() call to the stacked buffers,
        // for the coupled Gauss-Newton step. The cache is already hinged + active-masked per this
        // constraint's semantics (satisfied hinge rows dropped; equality rows kept), so it is read
        // directly -- no re-evaluation, no re-masking. Default no-op (only projecting constraints
        // override). Requires squared_error() to have run at the current q.
        virtual void stacked_cache(
            FloatVector<rake, 1> * /*err*/, FloatVector<rake, 1> * /*jac*/) const noexcept
        {
        }

        // Run the error/Jacobian kernel on the whole block, caching every lane for
        // extract_error_jacobian(). One evaluation serves all rake lanes, so batch callers
        // (chart construction over a block of samples) pay the kernel once instead of per
        // lane. Overwrites the cache of squared_error(); re-call squared_error() before
        // step().
        virtual void evaluate_error_jacobian(const Block &q) const noexcept = 0;

        // Violation error (n_rows() entries) and *raw*-error Jacobian (n_rows() x
        // Robot::dimension, row-major) of one SIMD lane of the last
        // evaluate_error_jacobian() block, for tangent-space (chart) construction. The
        // hinge mask of squared_error() cannot apply to the Jacobian: on the manifold the
        // violation error vanishes, so masking would zero every row.
        virtual void extract_error_jacobian(std::size_t lane, float *err, float *jac)
            const noexcept = 0;

        // Error and Jacobian of one lane of q; batch callers should evaluate once and
        // extract per lane instead.
        void error_jacobian(const Block &q, std::size_t lane, float *err, float *jac)
            const noexcept
        {
            evaluate_error_jacobian(q);
            extract_error_jacobian(lane, err, jac);
        }
    };

    // Output layout of the generated error kernels and input layout of the generated
    // solvers: the Jacobian d(err)/dq (row-major), then the error.
    template <std::size_t rake, std::size_t err_size, std::size_t jac_size>
    struct SolveBuffer
    {
        FloatVector<rake, jac_size> jac;
        FloatVector<rake, err_size> err;

        auto operator[](std::size_t index) noexcept -> FloatVector<rake, 1> &
        {
            return (index < jac_size) ? jac[index] : err[index - jac_size];
        }

        auto operator[](std::size_t index) const noexcept -> FloatVector<rake, 1>
        {
            return (index < jac_size) ? jac[index] : err[index - jac_size];
        }
    };

    // Zero the pinned columns of a cached solve buffer's Jacobian (mask_jacobian()
    // implementations of constraints with generated solvers).
    template <typename Robot, std::size_t rake, std::size_t err_size, std::size_t jac_size>
    inline void mask_pinned_columns(
        SolveBuffer<rake, err_size, jac_size> &solve,
        const bool *pinned) noexcept
    {
        for (auto j = 0U; j < Robot::dimension; ++j)
        {
            if (not pinned[j])
            {
                continue;
            }

            for (auto i = 0U; i < err_size; ++i)
            {
                solve.jac[i * Robot::dimension + j] = FloatVector<rake, 1>::fill(0.F);
            }
        }
    }

    // Shared machinery of bounded task-space (TSR-style) constraints, whose rows are pose
    // errors hinged against [lb, ub] bounds. The derived constraint supplies, as private
    // members visible through `friend Base`:
    //   - a mutable `input` with the generated kernel's operator[] layout, holding `q` and
    //     per-row `lb`/`ub` bound rows (filled, with `tight_rows`, by its constructor);
    //   - run_kernel(): evaluate the error/Jacobian kernel of `input` into `solve`;
    //   - solve_step(gradient, method): dispatch to the generated projection solvers.
    template <typename Derived, typename Robot, std::size_t rake, std::size_t err_size_>
    struct HingedTSRConstraint : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = err_size_;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            derived().input.q = q;
            derived().run_kernel();

            for (auto i = 0U; i < err_size; ++i)
            {
                solve.err[i] = (solve.err[i] - derived().input.lb[i]).min(0.F) +
                               (solve.err[i] - derived().input.ub[i]).max(0.F);

                // The hinge is flat inside the bounds, so satisfied rows must drop out of
                // the Jacobian too: keeping them turns the LM solve's zero-residual rows
                // into "hold this pose value" equality constraints that block projection.
                const auto active = solve.err[i] != 0.F;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    solve.jac[i * Robot::dimension + j] =
                        active & solve.jac[i * Robot::dimension + j];
                }
            }

            auto d = solve.err[0] * solve.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve.err[i] * solve.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept final
        {
            Block gradient;
            derived().solve_step(gradient, method);
            integrate_step<Robot, rake>(q, gradient, alpha);
        }

        auto n_rows() const noexcept -> std::size_t final
        {
            return err_size;
        }

        void active_rows(bool *rows) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = tight_rows[i];
            }
        }

        void mask_jacobian(const bool *pinned) const noexcept final
        {
            mask_pinned_columns<Robot>(solve, pinned);
        }

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            derived().input.q = q;
            derived().run_kernel();
        }

        void extract_error_jacobian(std::size_t lane, float *err, float *jac)
            const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                // Same SIMD min/max hinge as squared_error: NaN from the log map at
                // exactly-satisfied orientations masks to zero.
                const auto violation = (solve.err[i] - derived().input.lb[i]).min(0.F) +
                                       (solve.err[i] - derived().input.ub[i]).max(0.F);
                err[i] = violation[{0, lane}];
            }

            for (auto i = 0U; i < jac_size; ++i)
            {
                jac[i] = solve.jac[{i, lane}];
            }
        }

        // squared_error() left solve.err hinged and solve.jac active-masked; read them directly.
        void stacked_cache(FloatVector<rake, 1> *err, FloatVector<rake, 1> *jac) const noexcept final
        {
            for (std::size_t i = 0; i < err_size; ++i) err[i] = solve.err[i];
            for (std::size_t i = 0; i < jac_size; ++i) jac[i] = solve.jac[i];
        }

    protected:
        mutable SolveBuffer<rake, err_size, jac_size> solve;
        std::array<bool, err_size> tight_rows{};

    private:
        auto derived() const noexcept -> const Derived &
        {
            return static_cast<const Derived &>(*this);
        }
    };
}  // namespace vamp::planning::constraint
