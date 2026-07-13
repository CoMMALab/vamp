#pragma once

#include <array>

#include <vamp/planning/constraints/manifold/constraint.hh>
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
    struct TaskSpaceConstraint final
      : HingedTSRConstraint<TaskSpaceConstraint<Robot, rake>, Robot, rake, 6 * Robot::n_eef>
    {
        using Base =
            HingedTSRConstraint<TaskSpaceConstraint<Robot, rake>, Robot, rake, 6 * Robot::n_eef>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;

        static constexpr std::size_t n_eef = Robot::n_eef;

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
                    this->tight_rows[i * 6 + j] = (upper[i][j] - lower[i][j]) < tight_row_width;
                }
            }
        }

    private:
        friend Base;

        void run_kernel() const noexcept
        {
            Robot::template tsr_error<rake>(input, this->solve);
        }

        void solve_step(Block &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_tsr_error_lm_inner<rake>(this->solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_tsr_error_lm_outer<rake>(this->solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_tsr_error_gradient_descent<rake>(this->solve, gradient);
                    break;
            }
        }

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

        mutable Input input;
    };
}  // namespace vamp::planning::constraint
