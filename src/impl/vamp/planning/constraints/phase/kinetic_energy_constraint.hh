#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include <vamp/planning/constraints/phase/phase_constraint.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Phase constraint bounding total kinetic energy: (1/2) qd^T M(q) qd <= max_energy,
    // via the robot-generated kinetic_energy kernels. KE is homogeneous of degree 2 in qd,
    // so velocity_scale is sqrt(max_energy / KE).
    template <typename Robot, std::size_t rake>
    struct KineticEnergyConstraint final : PhaseConstraint<Robot, rake>
    {
        using Base = PhaseConstraint<Robot, rake>;
        using Block = typename Base::Block;
        using Configuration = typename Base::Configuration;
        using Row = FloatVector<rake, 1>;

        explicit KineticEnergyConstraint(float max_energy) noexcept : max_energy_(max_energy)
        {
        }

        void evaluate(const Block &x) const noexcept final
        {
            energy_ = Robot::template kinetic_energy_block<rake>(x);
        }

        auto extract(std::size_t lane) const noexcept -> bool final
        {
            return energy_[{0, lane}] <= max_energy_;
        }

        auto velocity_scale(const Configuration &z) const noexcept -> float final
        {
            std::array<float, Robot::dimension> x;
            const auto buf = z.to_array();
            std::copy_n(buf.begin(), Robot::dimension, x.begin());

            const float energy = Robot::kinetic_energy(x);
            if (energy <= max_energy_)
            {
                return 1.0F;
            }

            return std::sqrt(max_energy_ / energy);
        }

        auto max_energy() const noexcept -> float
        {
            return max_energy_;
        }

    private:
        float max_energy_;
        mutable Row energy_;
    };
}  // namespace vamp::planning::constraint
