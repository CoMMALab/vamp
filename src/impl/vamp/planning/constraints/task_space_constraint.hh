#pragma once

#include <array>

#include <vamp/planning/constraints/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Task Space Region constraint (Berenson et al., "Task Space Regions", IJRR 2011): for
    // each end-effector, the pose of an offset frame e (rTe, expressed in the end-effector
    // frame) must lie within [lower, upper] se(3) bounds of a reference frame r (wTr,
    // expressed in the world frame). Transforms are wxyz quaternion + xyz translation,
    // matching the input layout of the generated Robot::tsr_error.
    template <typename Robot, std::size_t rake>
    struct TaskSpaceConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t n_eef = Robot::n_eef;
        static constexpr std::size_t err_size = 6 * n_eef;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        using Transform = std::array<float, 7>;
        using Bound = std::array<float, 6>;

        TaskSpaceConstraint(
            const std::array<Transform, n_eef> &eef_to_offset,       // rTe per end-effector
            const std::array<Transform, n_eef> &world_to_reference,  // wTr per end-effector
            const std::array<Bound, n_eef> &lower,
            const std::array<Bound, n_eef> &upper) noexcept
        {
            for (auto i = 0U; i < n_eef; ++i)
            {
                for (auto j = 0U; j < 7; ++j)
                {
                    input.rTe[i * 7 + j] = Row::fill(eef_to_offset[i][j]);
                    input.wTr[i * 7 + j] = Row::fill(world_to_reference[i][j]);
                }

                for (auto j = 0U; j < 6; ++j)
                {
                    input.lb[i * 6 + j] = Row::fill(lower[i][j]);
                    input.ub[i * 6 + j] = Row::fill(upper[i][j]);
                    tight_rows[i * 6 + j] = (upper[i][j] - lower[i][j]) < tight_row_width;
                }
            }
        }

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            input.q = q;
            Robot::template tsr_error<rake>(input, solve);

            for (auto i = 0U; i < err_size; ++i)
            {
                solve.err[i] = (solve.err[i] - input.lb[i]).min(0.F) +
                               (solve.err[i] - input.ub[i]).max(0.F);

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
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_tsr_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_tsr_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_tsr_error_gradient_descent<rake>(solve, gradient);
                    break;
            }

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

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            input.q = q;
            Robot::template tsr_error<rake>(input, solve);
        }

        void extract_error_jacobian(std::size_t lane, float *err, float *jac)
            const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                // Same SIMD min/max hinge as squared_error: NaN from the log map at
                // exactly-satisfied orientations masks to zero.
                const auto hinged = (solve.err[i] - input.lb[i]).min(0.F) +
                                    (solve.err[i] - input.ub[i]).max(0.F);
                err[i] = hinged[{0, lane}];
            }

            for (auto i = 0U; i < jac_size; ++i)
            {
                jac[i] = solve.jac[{i, lane}];
            }
        }

    private:
        // Input layout of the generated tsr_error: q, then per end-effector rTe (7), wTr (7),
        // lower (6), upper (6).
        struct Input
        {
            Block q;
            FloatVector<rake, 7 * n_eef> rTe;
            FloatVector<rake, 7 * n_eef> wTr;
            FloatVector<rake, 6 * n_eef> lb;
            FloatVector<rake, 6 * n_eef> ub;

            static constexpr std::size_t per_eef = 7 + 7 + 6 + 6;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                const std::size_t eef = (index - Robot::dimension) / per_eef;
                const std::size_t i = (index - Robot::dimension) % per_eef;

                if (i < 7)
                {
                    return rTe[eef * 7 + i];
                }

                if (i < 14)
                {
                    return wTr[eef * 7 + i - 7];
                }

                if (i < 20)
                {
                    return lb[eef * 6 + i - 14];
                }

                return ub[eef * 6 + i - 20];
            }
        };

        // Output layout of tsr_error and input layout of the generated solvers: the Jacobian
        // d(err)/dq (row-major), then the error.
        struct Solve
        {
            FloatVector<rake, jac_size> jac;
            FloatVector<rake, err_size> err;

            auto operator[](std::size_t index) noexcept -> Row &
            {
                return (index < jac_size) ? jac[index] : err[index - jac_size];
            }

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return (index < jac_size) ? jac[index] : err[index - jac_size];
            }
        };

        mutable Input input;
        mutable Solve solve;
        std::array<bool, err_size> tight_rows{};
    };
}  // namespace vamp::planning::constraint
