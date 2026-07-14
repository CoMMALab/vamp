#pragma once

#include <array>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Holonomic form of the lead-screw coupling: one equality row pinning the
    // screw invariant
    //     h(q) = [t]_z - (pitch / 2 pi) [log3(R)]_z
    // of the offset end-effector frame expressed in the reference frame to a target
    // level, via the generated Robot::lead_screw_error kernel and projection solvers.
    // Note the log map wraps at a half turn: |rotation| must stay below pi for h to be
    // continuous. This is the integrable correctness oracle for the Pfaffian
    // LeadScrewConstraint (twist_constraint.hh), whose h-drift along planned
    // trajectories measures the Pfaffian machinery's edge-level violation.
    template <typename Robot, std::size_t rake>
    struct LeadScrewLevelConstraint final
      : HingedTSRConstraint<LeadScrewLevelConstraint<Robot, rake>, Robot, rake, 1>
    {
        using Base = HingedTSRConstraint<LeadScrewLevelConstraint<Robot, rake>, Robot, rake, 1>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;

        using Transform = std::array<float, 7>;

        LeadScrewLevelConstraint(
            const Transform &eef_to_offset,
            const Transform &world_to_reference,
            float pitch,
            float target) noexcept
        {
            for (auto j = 0U; j < 7; ++j)
            {
                input.rTe[j] = Row::fill(eef_to_offset[j]);
                input.wTr[j] = Row::fill(world_to_reference[j]);
            }

            input.pitch = Row::fill(pitch);
            input.lb[0] = Row::fill(target);
            input.ub[0] = Row::fill(target);
            this->tight_rows[0] = true;
        }

    private:
        friend Base;

        void run_kernel() const noexcept
        {
            Robot::template lead_screw_error<rake>(input, this->solve);
        }

        void solve_step(Block &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_lead_screw_error_lm_inner<rake>(this->solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_lead_screw_error_lm_outer<rake>(this->solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_lead_screw_error_gradient_descent<rake>(
                        this->solve, gradient);
                    break;
            }
        }

        // Kernel input layout as LeadScrewConstraint::Input; lb/ub are runtime hinge
        // bounds only, never on the tape.
        struct Input
        {
            Block q;
            FloatVector<rake, 7> rTe;
            FloatVector<rake, 7> wTr;
            Row pitch;
            FloatVector<rake, 1> lb;
            FloatVector<rake, 1> ub;

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

                if (i < 14)
                {
                    return wTr[i - 7];
                }

                return pitch;
            }
        };

        mutable Input input;
    };
}  // namespace vamp::planning::constraint
