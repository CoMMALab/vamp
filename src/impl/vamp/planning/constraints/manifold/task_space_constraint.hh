#pragma once

#include <array>
#include <type_traits>
#include <utility>

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

            eq_input.rTe = input.rTe;
            eq_input.wTr = input.wTr;
        }

        // Re-target end-effector `eef`'s reference frame (wTr); bounds and offsets are unchanged.
        void set_reference(std::size_t eef, const Transform &world_to_reference) noexcept
        {
            for (auto j = 0U; j < 7; ++j)
            {
                input.wTr[eef * 7 + j] = Row::fill(world_to_reference[j]);
            }

            eq_input.wTr = input.wTr;
        }

        // Equality form of the tight rows: the offset frame pinned to the reference frame (bound
        // centre) along each tight row, via the smooth error [p; vee(R - R^T) / 2] of the
        // generated tsr_equality kernel. Near the pin, each row changes at the rate of the
        // offset frame's twist expressed in the reference frame (linear velocity first).
        auto equality_rows() const noexcept -> std::size_t final
        {
            if constexpr (has_equality_kernel())
            {
                std::size_t n = 0;
                for (auto i = 0U; i < 6 * n_eef; ++i)
                {
                    n += this->tight_rows[i];
                }

                return n;
            }

            return 0;
        }

        void evaluate_equality(const Block &q, const Block &qd) const noexcept final
        {
            if constexpr (has_equality_kernel())
            {
                eq_input.q = q;
                eq_input.qd = qd;
                Robot::template tsr_equality<rake>(eq_input, eq_out);
            }
        }

        void extract_equality(std::size_t lane, float *F, float *J, float *gamma) const noexcept final
        {
            constexpr std::size_t rows = 6 * n_eef;
            constexpr std::size_t n_jac = rows * Robot::dimension;
            std::size_t k = 0;
            for (auto r = 0U; r < rows; ++r)
            {
                if (not this->tight_rows[r])
                {
                    continue;
                }

                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    J[k * Robot::dimension + j] = eq_out[r * Robot::dimension + j][{0, lane}];
                }

                F[k] = eq_out[n_jac + r][{0, lane}];
                gamma[k] = eq_out[n_jac + rows + r][{0, lane}];
                ++k;
            }
        }

        void extract_equality_rows(Row *F, Row *J, Row *gamma) const noexcept final
        {
            constexpr std::size_t rows = 6 * n_eef;
            constexpr std::size_t n_jac = rows * Robot::dimension;
            std::size_t k = 0;
            for (auto r = 0U; r < rows; ++r)
            {
                if (not this->tight_rows[r])
                {
                    continue;
                }

                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    J[k * Robot::dimension + j] = eq_out[r * Robot::dimension + j];
                }

                F[k] = eq_out[n_jac + r];
                gamma[k] = eq_out[n_jac + rows + r];
                ++k;
            }
        }

    private:
        friend Base;

        using EqualityOutput = std::array<Row, 6 * n_eef * Robot::dimension + 12 * n_eef>;

        // Robots generated before the equality kernel existed have no equality form.
        template <typename R, typename In, typename = void>
        struct equality_kernel : std::false_type
        {
        };

        template <typename R, typename In>
        struct equality_kernel<
            R,
            In,
            std::void_t<decltype(R::template tsr_equality<rake>(
                std::declval<const In &>(), std::declval<EqualityOutput &>()))>> : std::true_type
        {
        };

        static constexpr auto has_equality_kernel() noexcept -> bool
        {
            return equality_kernel<Robot, EqualityInput>::value;
        }

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

        // Input layout of the generated tsr_equality: q, qd, then per end-effector rTe (7), wTr (7).
        struct EqualityInput
        {
            Block q;
            Block qd;
            FloatVector<rake, 7 * n_eef> rTe;
            FloatVector<rake, 7 * n_eef> wTr;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                if (index < 2 * Robot::dimension)
                {
                    return qd[index - Robot::dimension];
                }

                const std::size_t eef = (index - 2 * Robot::dimension) / 14;
                const std::size_t i = (index - 2 * Robot::dimension) % 14;
                return (i < 7) ? rTe[eef * 7 + i] : wTr[eef * 7 + i - 7];
            }
        };

        mutable Input input;
        mutable EqualityInput eq_input;
        mutable EqualityOutput eq_out;
    };
}  // namespace vamp::planning::constraint
