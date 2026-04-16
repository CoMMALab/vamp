#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
#include <vamp/vector/math.hh>
#include <iomanip>
#include <vamp/planning/constraints/block_utils.hh>

namespace vamp::planning::constraint
{

    template <typename Robot, std::size_t rake>
    class ClosedLinkConstraint
    {
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, Robot::num_closed_link_chains * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, Robot::num_closed_link_chains> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < Robot::num_closed_link_chains * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_closed_link_chains * Robot::dimension &&
                    index < Robot::num_closed_link_chains * Robot::dimension + Robot::num_closed_link_chains)
                {
                    return err[index - Robot::num_closed_link_chains * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < Robot::num_closed_link_chains * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_closed_link_chains * Robot::dimension &&
                    index < Robot::num_closed_link_chains * Robot::dimension + Robot::num_closed_link_chains)
                {
                    return err[index - Robot::num_closed_link_chains * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<
                      rake,
                      Robot::num_closed_link_chains + Robot::num_closed_link_chains * Robot::dimension> y)
            {
                for (size_t i = 0; i < Robot::num_closed_link_chains; i++)
                {
                    err[i] = y[Robot::num_closed_link_chains * Robot::dimension + i];
                }
                for (size_t i = 0; i < Robot::num_closed_link_chains * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        mutable JacobianProjectInp jac_proj_inp;
        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        static constexpr char *name = "ClosedLinkConstraint";

        ClosedLinkConstraint()
        {
            ;
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            std::cout << "Closed linkage error : " << std::endl;
            for (auto i = 0U; i < Robot::num_closed_link_chains * Robot::dimension; i++)
            {
                if (i % Robot::dimension == 0)
                {
                    std::cout << std::endl << " J[" << i << "]: ";
                }
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl << "Error : ";
            for (auto i = 0U; i < Robot::num_closed_link_chains; i++)
            {
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            }
            std::cout << std::endl;
            return dist;
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            Robot::template closed_link_joints_error<rake>(q, jac_proj_inp);
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < Robot::num_closed_link_chains; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }
            return d;
        }

        void projectStepInPlace(
            ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            distanceToConstraint(q);
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_closed_link_error_lm_inner<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_closed_link_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_closed_link_error_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else
            {
                std::cout << "Invalid projection method: " << projection_method << std::endl;
                throw std::runtime_error("Invalid projection method");
            }
            integrateJointConfiguration<Robot, rake>(q, q, grad, alpha);
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            ConfigurationBlock q_new = q;
            projectStepInPlace(q_new, projection_method, alpha);
            return q_new;
        }
    };

}  // namespace vamp::planning::constraint