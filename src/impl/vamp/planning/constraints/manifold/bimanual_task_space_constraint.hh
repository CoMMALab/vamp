#pragma once

#include <type_traits>
#include <utility>

#include <algorithm>
#include <array>

#include <vamp/planning/constraints/manifold/constraint.hh>
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
                all_tight_ = all_tight_ and this->tight_rows[j];
            }

            eq_input.lTr = input.lTr;
        }

        // Equality form when every row is tight: the rigid relative pose (bound centre) as the
        // smooth error [p; vee(R - R^T) / 2] of the generated tsr_bimanual_equality kernel.
        auto equality_rows() const noexcept -> std::size_t final
        {
            return (has_equality_kernel() and all_tight_) ? 6 : 0;
        }

        void evaluate_equality(const Block &q, const Block &qd) const noexcept final
        {
            if constexpr (has_equality_kernel())
            {
                eq_input.q = q;
                eq_input.qd = qd;
                Robot::template tsr_bimanual_equality<rake>(eq_input, eq_out);
            }
        }

        void extract_equality(std::size_t lane, float *F, float *J, float *gamma) const noexcept final
        {
            constexpr std::size_t n_jac = 6 * Robot::dimension;
            for (auto i = 0U; i < n_jac; ++i)
            {
                J[i] = eq_out[i][{0, lane}];
            }

            for (auto i = 0U; i < 6; ++i)
            {
                F[i] = eq_out[n_jac + i][{0, lane}];
                gamma[i] = eq_out[n_jac + 6 + i][{0, lane}];
            }
        }

        void extract_equality_rows(Row *F, Row *J, Row *gamma) const noexcept final
        {
            constexpr std::size_t n_jac = 6 * Robot::dimension;
            for (auto i = 0U; i < n_jac; ++i)
            {
                J[i] = eq_out[i];
            }

            for (auto i = 0U; i < 6; ++i)
            {
                F[i] = eq_out[n_jac + i];
                gamma[i] = eq_out[n_jac + 6 + i];
            }
        }

        void evaluate_equality_position(const Block &q) const noexcept final
        {
            if constexpr (position_kernel<Robot, EqualityInput>::value)
            {
                constexpr std::size_t n_out = 6 * Robot::dimension + 6;
                eq_input.q = q;
                Robot::template tsr_bimanual_equality_fj<rake>(eq_input, eq_out);
                for (auto i = n_out; i < n_out + 6; ++i)
                {
                    eq_out[i] = Row::fill(0.F);
                }
            }
            else
            {
                evaluate_equality(q, Block::zero_vector());
            }
        }

        auto has_normal_kernel() const noexcept -> bool final
        {
            return normal_kernel<Robot>::value;
        }

        auto solve_normal_rows(const Row *J, const Row *rhs, const Row *N, Row *out) const noexcept
            -> bool final
        {
            if constexpr (normal_kernel<Robot>::value)
            {
                constexpr std::size_t n_jac = 6 * Robot::dimension;
                NormalInput in;
                std::copy_n(J, n_jac, in.begin());
                std::copy_n(rhs, 6, in.begin() + n_jac);
                std::copy_n(N, n_jac, in.begin() + n_jac + 6);
                NormalOutput o;
                Robot::template solve_tsr_bimanual_exact<rake>(in, o);
                std::copy(o.begin(), o.end(), out);
                return true;
            }

            return false;
        }

    private:
        friend Base;

        using EqualityOutput = std::array<Row, 6 * Robot::dimension + 12>;
        using NormalInput = std::array<Row, 12 * Robot::dimension + 6>;
        using NormalOutput = std::array<Row, Robot::dimension + 6>;

        // Robots generated before the drift-free equality kernel existed evaluate the full one.
        template <typename R, typename In, typename = void>
        struct position_kernel : std::false_type
        {
        };

        template <typename R, typename In>
        struct position_kernel<
            R,
            In,
            std::void_t<decltype(R::template tsr_bimanual_equality_fj<rake>(
                std::declval<const In &>(), std::declval<EqualityOutput &>()))>> : std::true_type
        {
        };

        // Robots generated before the exact-chart step kernel existed solve generically.
        template <typename R, typename = void>
        struct normal_kernel : std::false_type
        {
        };

        template <typename R>
        struct normal_kernel<
            R,
            std::void_t<decltype(R::template solve_tsr_bimanual_exact<rake>(
                std::declval<const NormalInput &>(), std::declval<NormalOutput &>()))>> : std::true_type
        {
        };

        // Robots generated before the equality kernel existed have no equality form.
        template <typename R, typename In, typename = void>
        struct equality_kernel : std::false_type
        {
        };

        template <typename R, typename In>
        struct equality_kernel<
            R,
            In,
            std::void_t<decltype(R::template tsr_bimanual_equality<rake>(
                std::declval<const In &>(), std::declval<EqualityOutput &>()))>> : std::true_type
        {
        };

        static constexpr auto has_equality_kernel() noexcept -> bool
        {
            return equality_kernel<Robot, EqualityInput>::value;
        }

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

        // Input layout of the generated tsr_bimanual_equality: q, qd, then lTr (7).
        struct EqualityInput
        {
            Block q;
            Block qd;
            FloatVector<rake, 7> lTr;

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

                return lTr[index - 2 * Robot::dimension];
            }
        };

        mutable Input input;
        mutable EqualityInput eq_input;
        mutable EqualityOutput eq_out;
        bool all_tight_ = true;
    };
}  // namespace vamp::planning::constraint
