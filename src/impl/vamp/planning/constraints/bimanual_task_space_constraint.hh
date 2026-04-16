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
    class BimanualTaskSpaceConstraint
    {
        /**
         * A TSR constraint is expressed as 2 transformation matrices
         * with a lower and upper bound, as per
         * https://personalrobotics.cs.washington.edu/publications/berenson2011task.pdf
         *
         *
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;

        struct TSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7> rTlB;
            vamp::FloatVector<rake, 6> lbB;
            vamp::FloatVector<rake, 6> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t index_e = (index - Robot::dimension);

                if (index_e < 7)  // rTl
                {
                    return rTlB[index_e];
                }

                else if (index_e >= 7 && index_e < (1 * 7 + 6))
                {
                    return lbB[index_e - 7];
                }

                else if (index_e >= (1 * 7 + 6) && index_e < (1 * 7 + 6 * 2))
                {
                    return ubB[index_e - (7 + 6)];
                }
                else
                {
                    return q[0];
                }
            }

            void print() const
            {
                std::cout << "rTlB: ";
                for (int i = 0U; i < 7; i++)
                {
                    std::cout << rTlB[{i, 0}] << " ";
                }
                std::cout << std::endl;

                std::cout << "lbB: ";
                for (int i = 0U; i < 6; i++)
                {
                    std::cout << lbB[{i, 0}] << " ";
                }
                std::cout << std::endl;

                std::cout << "ubB: ";
                for (int i = 0U; i < 6; i++)
                {
                    std::cout << ubB[{i, 0}] << " ";
                }
                std::cout << std::endl;
            }
        };

        mutable TSRComputeInput tsr_function_inp;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 6 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 6> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (index >= 6 * Robot::dimension && index < 6 * Robot::dimension + 6)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (index >= 6 * Robot::dimension && index < 6 * Robot::dimension + 6)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &operator=(vamp::FloatVector<rake, 6 + 6 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    err[i] = y[6 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 6 * Robot::dimension; i++)
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
        static constexpr char *name = "BimanualTaskSpaceConstraint";

        BimanualTaskSpaceConstraint(
            const std::array<float, 7> right_eef_pose_w_ref_left_eef,  // rTl qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6> lower_bound,
            const std::array<float, 6> upper_bound)
        {
            assignBlock<rake, 7>(right_eef_pose_w_ref_left_eef, tsr_function_inp.rTlB);
            assignBlock<rake, 6>(lower_bound, tsr_function_inp.lbB);
            assignBlock<rake, 6>(upper_bound, tsr_function_inp.ubB);
            // tsr_function_inp.print();
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            // for(auto i=0U; i < Robot::dimension + 19; i++)
            //     std::cout << tsr_function_inp[i] << " ";

            auto dist = distanceToConstraint(q);
            std::cout << "Bimanual Error : " << std::endl;
            for (auto i = 0U; i < 6 * Robot::dimension; i++)
            {
                if (i % Robot::dimension == 0)
                {
                    std::cout << std::endl << " J[" << i << "]: ";
                }
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl << "Error : ";
            for (auto i = 0U; i < 6; i++)
            {
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            }
            std::cout << std::endl;
            return dist;
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            Robot::template tsr_bimanual_error<rake>(tsr_function_inp, jac_proj_inp);

            const size_t jac_offset = 6 * Robot::dimension;
            for (size_t i = 0; i < 6; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            }
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6; i++)
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
                Robot::template solve_tsr_relative_error_lm_inner<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_tsr_relative_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_tsr_relative_error_gradient_descent<rake>(jac_proj_inp, grad);
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