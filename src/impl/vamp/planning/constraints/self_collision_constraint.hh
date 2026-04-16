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
    class SelfCollisionConstraint
    {
        /**
         * A self collision constraint that uses the robot's bounding spheres to compute a differentiable
         * error and jacobian for self collision avoidance.
         *
         */
        /**
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, Robot::num_bounding_spheres * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, Robot::num_bounding_spheres> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < Robot::num_bounding_spheres * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_bounding_spheres * Robot::dimension &&
                    index < Robot::num_bounding_spheres * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - Robot::num_bounding_spheres * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < Robot::num_bounding_spheres * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_bounding_spheres * Robot::dimension &&
                    index < Robot::num_bounding_spheres * Robot::dimension +
                                Robot::num_bounding_spheres * Robot::n_eef)
                {
                    return err[index - Robot::num_bounding_spheres * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<
                      rake,
                      Robot::num_bounding_spheres * Robot::n_eef +
                          Robot::num_bounding_spheres * Robot::n_eef * Robot::dimension> y)
            {
                for (size_t i = 0; i < Robot::num_bounding_spheres; i++)
                {
                    err[i] = y[Robot::num_bounding_spheres * Robot::dimension + i];
                }
                for (size_t i = 0; i < Robot::num_bounding_spheres * Robot::dimension; i++)
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
        static constexpr char *name = "SelfCollisionConstraint";

        SelfCollisionConstraint()
        {
            ;
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            // for(auto i=0U; i < Robot::dimension + 19; i++)
            //     std::cout << tsr_function_inp[i] << " ";

            auto dist = distanceToConstraint(q);
            std::cout << "Self collision error : " << std::endl;
            for (auto i = 0U; i < Robot::num_bounding_spheres * Robot::dimension; i++)
            {
                if (i % Robot::dimension == 0)
                {
                    std::cout << std::endl << " J[" << i << "]: ";
                }
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl << "Error : ";
            for (auto i = 0U; i < Robot::num_bounding_spheres; i++)
            {
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            }
            std::cout << std::endl;
            return dist;
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            Robot::template bounding_spheres_self_collision_error<rake>(q, jac_proj_inp);

            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < Robot::num_bounding_spheres; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }
            // std::cout << "Error : ";
            // for(auto i=0U; i < Robot::num_bounding_spheres; i++){
            //     std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            // }
            // std::cout << std::endl;

            return d * 0.0;  // NOTE: Chatgpt pointed it out, maybe get rid of the * 0.0
        }

        void projectStepInPlace(
            ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            distanceToConstraint(q);
            ConfigurationBlock grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_self_collision_error_lm_inner<rake>(jac_proj_inp, grad);
                // grad = grad.zero_out_nans();
                // std::cout << "Grad for selfcoll constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << q[{i, 0}] << " -- " <<grad[{i, 0}] << " ";
                // std::cout << std::endl;
            }
            if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_self_collision_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_self_collision_error_gradient_descent<rake>(jac_proj_inp, grad);
            }
            integrateJointConfiguration<Robot, rake>(q, q, grad, alpha);
        }

        vamp::FloatVector<rake, 1> projectStep(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            bool update_q = true,
            float alpha = 1.0F)
        {
            auto dist = print_robot_tsr_error(q);
            if (update_q)
            {
                q_new = q;
                projectStepInPlace(q_new, projection_method, alpha);
            }
            return dist;
        }

    };  // namespace vamp::planning::constraint
}  // namespace vamp::planning::constraint