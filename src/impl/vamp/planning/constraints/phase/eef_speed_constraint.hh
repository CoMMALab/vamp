#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <vamp/planning/constraints/phase/phase_constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Phase gate bounding the workspace speed of every end-effector origin:
    // ||v_eef|| <= max_speed (linear velocity only), via the robot-generated eef_velocity
    // kernels. The velocity is homogeneous of degree 1 in qd, so the feasibility scale is
    // max_speed / ||v_eef||.
    template <typename Robot, std::size_t rake>
    struct EEFSpeedConstraint final : PhaseConstraint<Robot, rake>
    {
        using Base = PhaseConstraint<Robot, rake>;
        using Block = typename Base::Block;
        using Configuration = typename Base::Configuration;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t n_eef = Robot::n_end_effectors;

        explicit EEFSpeedConstraint(float max_speed) noexcept
          : max_speed_(max_speed), max_speed2_(max_speed * max_speed)
        {
        }

        void evaluate(const Block &x) const noexcept final
        {
            std::array<Row, 3 * n_eef> eev;
            Robot::template eef_velocity_block<rake>(x, eev);

            speed2_ = eev[0] * eev[0] + eev[1] * eev[1] + eev[2] * eev[2];
            for (std::size_t e = 1; e < n_eef; ++e)
            {
                speed2_ = speed2_.max(
                    eev[3 * e] * eev[3 * e] + eev[3 * e + 1] * eev[3 * e + 1] +
                    eev[3 * e + 2] * eev[3 * e + 2]);
            }
        }

        auto extract(std::size_t lane) const noexcept -> bool final
        {
            return speed2_[{0, lane}] <= max_speed2_;
        }

        auto velocity_scale(const Configuration &z) const noexcept -> float final
        {
            std::array<float, Robot::dimension> x;
            const auto buf = z.to_array();
            std::copy_n(buf.begin(), Robot::dimension, x.begin());

            const auto eev = Robot::eef_velocity(x);
            float worst2 = 0.0F;
            for (std::size_t e = 0; e < n_eef; ++e)
            {
                worst2 = std::max(
                    worst2,
                    eev[3 * e] * eev[3 * e] + eev[3 * e + 1] * eev[3 * e + 1] +
                        eev[3 * e + 2] * eev[3 * e + 2]);
            }

            if (worst2 <= max_speed2_)
            {
                return 1.0F;
            }

            return max_speed_ / std::sqrt(worst2);
        }

        auto max_speed() const noexcept -> float
        {
            return max_speed_;
        }

    private:
        float max_speed_;
        float max_speed2_;
        mutable Row speed2_;
    };
}  // namespace vamp::planning::constraint
