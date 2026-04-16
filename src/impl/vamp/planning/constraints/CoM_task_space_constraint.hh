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

    template <typename Robot, std::size_t rake, std::size_t num_polygons>
    class CoMTaskSpaceConstraint
    {
        /**
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;

        struct CoMConstraintInp
        {
            vamp::FloatVector<rake, 3> CoM;
            vamp::FloatVector<rake, 3 * Robot::dimension> com_jacobian;

            vamp::FloatVector<rake, 2 * num_polygons> polygon_points;  // arranged as (x1, y1, x2, y2, x3,
                                                                       // y3...)

            auto &operator[](size_t index)
            {
                if (index < 3)
                {
                    return CoM[index];
                }

                if (index >= 3 && index < 3 + 3 * Robot::dimension)  // jacobian
                {
                    return com_jacobian[index - 3];
                }

                if (index >= 3 + 3 * Robot::dimension)  // jacobian
                {
                    return polygon_points[index - (3 + 3 * Robot::dimension)];
                }

                else
                {
                    return CoM[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 3)
                {
                    return CoM[index];
                }

                if (index >= 3 && index < 3 + 3 * Robot::dimension)  // jacobian
                {
                    return com_jacobian[index - 3];
                }

                if (index >= 3 + 3 * Robot::dimension)  // jacobian
                {
                    return polygon_points[index - (3 + 3 * Robot::dimension)];
                }

                else
                {
                    return CoM[0];
                }
            }

            CoMConstraintInp &operator=(vamp::FloatVector<rake, 3 + 3 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 3; i++)
                {
                    CoM[i] = y[i];
                }
                for (size_t i = 0; i < 3 * Robot::dimension; i++)
                {
                    com_jacobian[i] = y[i + 3];
                }

                for (size_t i = 0; i < 2 * num_polygons; i++)
                {
                    polygon_points[i] = y[i + 3 + 3 * Robot::dimension];
                }

                return *this;
            }
        };

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 2 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 2> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (index >= 2 * Robot::dimension && index < 2 * Robot::dimension + 2)
                {
                    return err[index - 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (index >= 2 * Robot::dimension && index < 2 * Robot::dimension + 2)
                {
                    return err[index - 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &operator=(vamp::FloatVector<rake, 2 + 2 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 2; i++)
                {
                    err[i] = y[2 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 2 * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        mutable JacobianProjectInp jac_proj_inp;
        mutable CoMConstraintInp com_jac_polygons;

        // size_t num_polygons;

        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        static constexpr char *name = "CoMTaskSpaceConstraint";

        CoMTaskSpaceConstraint(const std::array<float, 2 * num_polygons> polygon_points)
        {
            // for (size_t i = 0; i < 2 * num_polygons; i++)
            //     std::cout << polygon_points[i] << " ";
            // std::cout << std::endl;

            assignBlock<rake, 2 * num_polygons>(polygon_points, com_jac_polygons.polygon_points);
        }

        auto print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            std::cout << "COM  : ";
            for (auto i = 0U; i < 3; i++)
            {
                std::cout << std::setprecision(5) << com_jac_polygons.CoM[{i, 0}] << " ";
            }
            std::cout << std::endl;

            std::cout << "COM Error : ";
            for (auto i = 0U; i < 2; i++)
            {
                std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            }
            std::cout << std::endl;
            std::cout << "Error Jac : ";
            for (auto i = 0U; i < 2 * Robot::dimension; i++)
            {
                if (i % Robot::dimension == 0)
                {
                    std::cout << std::endl;
                }
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            return dist;
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            Robot::template compute_com<rake>(q, com_jac_polygons);
            Robot::template com_constraint_error<rake>(com_jac_polygons, num_polygons, jac_proj_inp);

            // deviation from constraint is just x-y error
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0] + jac_proj_inp.err[1] * jac_proj_inp.err[1];

            return d;
        }

        void projectStepInPlace(
            ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            distanceToConstraint(q);
            // std::cout << "COM constraint distance: " << dist << std::endl;
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_com_function_lm_inner<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_com_function_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_com_function_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else
            {
                throw std::invalid_argument("Invalid projection method");
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
