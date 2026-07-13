#pragma once

#include <vamp/planning/constraints/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Loop-closure constraint: each cut kinematic loop of the robot contributes one equality
    // row, the deviation of the distance between the loop's cut frames from its fixed length
    // (e.g. a rigid connecting rod). Error and Jacobian come from the generated
    // Robot::closed_loop_error.
    template <typename Robot, std::size_t rake>
    struct ClosedLoopConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = Robot::n_closed_loops;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        ClosedLoopConstraint() noexcept = default;

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            input.q = q;
            Robot::template closed_loop_error<rake>(input, solve);

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
                    Robot::template solve_closed_loop_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_closed_loop_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_closed_loop_error_gradient_descent<rake>(solve, gradient);
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
            // Equality rows are always manifold-defining.
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
        }

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            input.q = q;
            Robot::template closed_loop_error<rake>(input, solve);
        }

        void extract_error_jacobian(std::size_t lane, float *err, float *jac)
            const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = solve.err[{i, lane}];
            }

            for (auto i = 0U; i < jac_size; ++i)
            {
                jac[i] = solve.jac[{i, lane}];
            }
        }

    private:
        // Input layout of the generated closed_loop_error: just the configuration.
        struct Input
        {
            Block q;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return q[index];
            }
        };

        // Output layout of closed_loop_error and input layout of the generated solvers: the
        // Jacobian d(err)/dq (row-major), then the error.
        using Solve = SolveBuffer<rake, err_size, jac_size>;

        mutable Input input;
        mutable Solve solve;
    };
}  // namespace vamp::planning::constraint
