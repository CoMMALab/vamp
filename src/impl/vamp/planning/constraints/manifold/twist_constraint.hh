#pragma once

#include <array>
#include <cstddef>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/pfaffian_constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Constant-coefficient Pfaffian velocity constraint over the end-effector twist:
    // rows a_i(q)^T qdot = c_ref_i^T twist_ref + c_loc_i^T twist_loc = 0, where the
    // generated Robot::twist_jacobians kernel provides the twist Jacobian [linear;
    // angular] of the offset end-effector frame (eef * rTe^-1) expressed in the
    // reference frame wTr's axes (twist_ref) and in the frame's own body axes
    // (twist_loc). The kernel is purely geometric -- no log map -- so the rows stay
    // smooth for unbounded rotation, and one traced kernel per robot covers the whole
    // family (lead screw, knife-edge, no-slip) through the runtime coefficients.
    template <typename Robot, std::size_t rake, std::size_t n_rows_ = 1>
    struct TwistConstraint
      : PfaffianConstraint<TwistConstraint<Robot, rake, n_rows_>, Robot, rake, n_rows_>
    {
        using Base = PfaffianConstraint<TwistConstraint<Robot, rake, n_rows_>, Robot, rake, n_rows_>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;

        using Transform = std::array<float, 7>;  // wxyz quaternion + xyz translation
        using Coefficients = std::array<std::array<float, 6>, n_rows_>;

        TwistConstraint(
            const Transform &eef_to_offset,
            const Transform &world_to_reference,
            const Coefficients &reference_coefficients,
            const Coefficients &body_coefficients) noexcept
          : cref(reference_coefficients), cloc(body_coefficients)
        {
            for (auto j = 0U; j < 7; ++j)
            {
                input.rTe[j] = Row::fill(eef_to_offset[j]);
                input.wTr[j] = Row::fill(world_to_reference[j]);
            }
        }

    private:
        friend Base;

        void run_kernel() const noexcept
        {
            Robot::template twist_jacobians<rake>(input, twist);

            for (auto i = 0U; i < n_rows_; ++i)
            {
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    Row acc = Row::fill(0.F);
                    for (auto a = 0U; a < 6; ++a)
                    {
                        if (cref[i][a] != 0.F)
                        {
                            acc = acc + twist[a * Robot::dimension + j] * cref[i][a];
                        }

                        if (cloc[i][a] != 0.F)
                        {
                            acc = acc + twist[(6 + a) * Robot::dimension + j] * cloc[i][a];
                        }
                    }

                    this->solve.jac[i * Robot::dimension + j] = acc;
                }
            }
        }

        // Input layout of the generated twist_jacobians: q, then rTe (7), wTr (7).
        struct Input
        {
            Block q;
            FloatVector<rake, 7> rTe;
            FloatVector<rake, 7> wTr;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                const std::size_t i = index - Robot::dimension;
                if (i < 7)
                {
                    return rTe[i];
                }

                return wTr[i - 7];
            }
        };

        mutable Input input;
        mutable FloatVector<rake, 12 * Robot::dimension> twist;
        Coefficients cref;
        Coefficients cloc;
    };

    // Lead-screw coupling of the end-effector as a twist row: advance along the
    // reference frame's z-axis locked to rotation about it,
    //     twist_ref_z - (pitch / 2 pi) omega_ref_z = 0.
    // Unlike the log3-based screw invariant this form has no half-turn wrap; multi-turn
    // screws are fine. The integrable LeadScrewLevelConstraint remains its holonomic
    // correctness oracle (valid while the level target stays within a half turn).
    template <typename Robot, std::size_t rake>
    struct LeadScrewConstraint final : TwistConstraint<Robot, rake, 1>
    {
        using Base = TwistConstraint<Robot, rake, 1>;
        using Transform = typename Base::Transform;
        using Coefficients = typename Base::Coefficients;

        LeadScrewConstraint(
            const Transform &eef_to_offset,
            const Transform &world_to_reference,
            float pitch) noexcept
          : Base(
                eef_to_offset,
                world_to_reference,
                Coefficients{{{0.F, 0.F, 1.F, 0.F, 0.F, -pitch / (2.F * pi)}}},
                Coefficients{})
        {
        }

    private:
        static constexpr float pi = 3.14159265358979323846F;
    };
}  // namespace vamp::planning::constraint
