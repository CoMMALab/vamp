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
    struct BimanualTaskSpaceConstraint final
      : HingedTSRConstraint<BimanualTaskSpaceConstraint<Robot, rake>, Robot, rake, 6>
    {
        static_assert(Robot::n_eef >= 2, "BimanualTaskSpaceConstraint requires two end-effectors");

        using Base =
            HingedTSRConstraint<BimanualTaskSpaceConstraint<Robot, rake>, Robot, rake, 6>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;

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
                this->tight_rows[j] = (upper[j] - lower[j]) < tight_row_width;
            }
        }

    private:
        friend Base;

        void run_kernel() const noexcept
        {
            Robot::template tsr_bimanual_error<rake>(input, this->solve);
        }

        void solve_step(Block &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_tsr_relative_error_lm_inner<rake>(this->solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_tsr_relative_error_lm_outer<rake>(this->solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_tsr_relative_error_gradient_descent<rake>(
                        this->solve, gradient);
                    break;
            }
        }

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

        mutable Input input;
    };
}  // namespace vamp::planning::constraint
