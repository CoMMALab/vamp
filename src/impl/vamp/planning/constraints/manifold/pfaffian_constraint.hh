#pragma once

#include <cmath>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Pfaffian constraint: rows a_i(q)^T qdot = 0, linear in velocity
    // with configuration-dependent coefficients. The rows carry no position error --
    // every configuration is admissible, so the projection loops and satisfied() see
    // straight through them -- but every row is chart-active: the tangent basis becomes
    // ker [J_holonomic; A(q)], and all planner velocities (LQMT cubics at the chart
    // center, per-sample execution-frame reprojection, arrival velocities) satisfy
    // A(q) qdot = 0 by construction.
    //
    // The derived constraint supplies, as private members visible through `friend Base`:
    //   - a mutable `input` with the generated kernel's operator[] layout, holding `q`
    //     and the row parameters (filled by its constructor);
    //   - run_kernel(): evaluate the row kernel of `input` into `solve`. Only the
    //     Jacobian rows are consumed; the kernel's error output is ignored here (an
    //     integrable row's error is its conserved quantity, useful as an external drift
    //     diagnostic but not a projection target).
    template <typename Derived, typename Robot, std::size_t rake, std::size_t n_rows_>
    struct PfaffianConstraint : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = n_rows_;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        auto squared_error(const Block &) const noexcept -> Row final
        {
            return Row::fill(0.F);
        }

        void step(Block &, ProjMethod, float) const noexcept final
        {
        }

        // Velocity-only: carries no position error, so it is excluded from the coupled
        // Gauss-Newton projection step (as its step() is already a no-op for block-Jacobi).
        auto projects() const noexcept -> bool final
        {
            return false;
        }

        auto n_rows() const noexcept -> std::size_t final
        {
            return err_size;
        }

        void active_rows(bool *rows) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
        }

        void pfaffian_rows(bool *rows) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
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
                err[i] = 0.F;

                // Unit-normalize each row: scaling cannot change ker A, but the chart
                // rank cutoff is relative to the largest QR column, so a row far from
                // unit scale would be spuriously rank-dropped (or drown the task-space
                // rows). Degenerate (near-zero) rows are left as-is: a vanishing row
                // restricts nothing, and boosting one to unit norm would manufacture a
                // constraint from noise. NaN rows pass through untouched to trip the
                // chart builder's finiteness fallback.
                float norm2 = 0.F;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    const float v = solve.jac[{i * Robot::dimension + j, lane}];
                    jac[i * Robot::dimension + j] = v;
                    norm2 += v * v;
                }

                if (norm2 > min_row_norm2)
                {
                    const float inv = 1.F / std::sqrt(norm2);
                    for (auto j = 0U; j < Robot::dimension; ++j)
                    {
                        jac[i * Robot::dimension + j] *= inv;
                    }
                }
            }
        }

    protected:
        static constexpr float min_row_norm2 = 1e-12F;

        mutable SolveBuffer<rake, err_size, jac_size> solve;

    private:
        auto derived() const noexcept -> const Derived &
        {
            return static_cast<const Derived &>(*this);
        }
    };
}  // namespace vamp::planning::constraint
