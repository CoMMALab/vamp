#pragma once

#include <array>

#include <vamp/planning/constraints/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Relative pose constraint between two end-effectors: the pose of end-effector 1 (right)
    // expressed in the frame of end-effector 0 (left) must lie within [lower, upper] se(3)
    // bounds of the target relative transform lTr. Transforms are wxyz quaternion + xyz
    // translation, matching the input layout of the generated Robot::tsr_bimanual_error.
    template <typename Robot, std::size_t rake>
    struct BimanualTaskSpaceConstraint final : Constraint<Robot, rake>
    {
        static_assert(Robot::n_eef >= 2, "BimanualTaskSpaceConstraint requires two end-effectors");

        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = 6;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        using Transform = std::array<float, 7>;
        using Bound = std::array<float, 6>;

        BimanualTaskSpaceConstraint(
            const Transform &right_in_left,  // lTr
            const Bound &lower,
            const Bound &upper) noexcept
        {
            for (auto j = 0U; j < 7; ++j)
            {
                input.lTr[j] = Row::fill(right_in_left[j]);
            }

            for (auto j = 0U; j < 6; ++j)
            {
                input.lb[j] = Row::fill(lower[j]);
                input.ub[j] = Row::fill(upper[j]);
                tight_rows[j] = (upper[j] - lower[j]) < tight_row_width;
            }
        }

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            input.q = q;
            Robot::template tsr_bimanual_error<rake>(input, solve);

            for (auto i = 0U; i < err_size; ++i)
            {
                solve.err[i] = (solve.err[i] - input.lb[i]).min(0.F) +
                               (solve.err[i] - input.ub[i]).max(0.F);

                // Mask satisfied rows out of the Jacobian: the hinge is flat inside the
                // bounds (see TaskSpaceConstraint::squared_error).
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
                    Robot::template solve_tsr_relative_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_tsr_relative_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_tsr_relative_error_gradient_descent<rake>(solve, gradient);
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
            Robot::template tsr_bimanual_error<rake>(input, solve);
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
        // Input layout of the generated tsr_bimanual_error: q, then lTr (7), lower (6),
        // upper (6).
        struct Input
        {
            Block q;
            FloatVector<rake, 7> lTr;
            FloatVector<rake, 6> lb;
            FloatVector<rake, 6> ub;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                const std::size_t i = index - Robot::dimension;

                if (i < 7)
                {
                    return lTr[i];
                }

                if (i < 13)
                {
                    return lb[i - 7];
                }

                return ub[i - 13];
            }
        };

        // Output layout of tsr_bimanual_error and input layout of the generated relative
        // solvers: the Jacobian d(err)/dq (row-major), then the error.
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
