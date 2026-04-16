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

    // exposing to public so we can set this in settings maybe?
    enum ProjMethod
    {
        InnerLM,  // J.T * ((J.JT + l)-1 . err) --> (6,6) matrix inv
        OuterLM,  // (JT.T + l)-1 . (J.T * err) -> (nq,nq) matrix inv
        GradDesc  // J.T * err
    };

    template <typename Robot, std::size_t rake>
    class TaskSpaceConstraint
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
            vamp::FloatVector<rake, 7 * Robot::n_eef> rTeB;
            vamp::FloatVector<rake, 7 * Robot::n_eef> wTrB;
            vamp::FloatVector<rake, 6 * Robot::n_eef> lbB;
            vamp::FloatVector<rake, 6 * Robot::n_eef> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t eef_id = (index - Robot::dimension) / size_per_eef;
                size_t index_e = (index - Robot::dimension) % size_per_eef;

                if (index_e < 7)  // rtE
                {
                    return rTeB[eef_id * 7 + index_e];
                }

                else if (index_e >= 7 && index_e < (2 * 7))
                {
                    return wTrB[eef_id * 7 + index_e - 7];
                }

                else if (index_e >= (2 * 7) && index_e < (2 * 7 + 6))
                {
                    return lbB[eef_id * 6 + index_e - (2 * 7)];
                }

                else if (index_e >= (2 * 7 + 6) && index_e < (2 * 7 + 6 * 2))
                {
                    return ubB[eef_id * 6 + index_e - (2 * 7 + 6)];
                }
                else
                {
                    return q[0];
                }
            }
        };

        mutable TSRComputeInput tsr_function_inp;

        struct JacobianProjectInp
        {
            static constexpr size_t J_size = 6 * Robot::n_eef * Robot::dimension;
            static constexpr size_t err_size = 6 * Robot::n_eef;

            vamp::FloatVector<rake, J_size> J;      // jacobian
            vamp::FloatVector<rake, err_size> err;  // error vector

            auto &operator[](size_t index)
            {
                if (index < J_size)
                {
                    return J[index];
                }
                else if (index >= J_size && index < J_size + err_size)
                {
                    return err[index - J_size];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < J_size)
                {
                    return J[index];
                }
                else if (index >= J_size && index < J_size + err_size)
                {
                    return err[index - J_size];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &operator=(vamp::FloatVector<rake, err_size + J_size> y)
            {
                for (size_t i = 0; i < err_size; i++)
                {
                    err[i] = y[J_size + i];
                }
                for (size_t i = 0; i < J_size; i++)
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
        static constexpr char *name = "TaskSpaceConstraint";

        TaskSpaceConstraint(
            std::array<std::array<float, 7>, Robot::n_eef> eef_pose_w_ref_reference,  // qw, qx, qy, qz, tx,
                                                                                      // ty, tz
            std::array<std::array<float, 7>, Robot::n_eef> ref_frame_w_world,  // qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6 * Robot::n_eef> lower_bound,
            const std::array<float, 6 * Robot::n_eef> upper_bound)
        {
            std::array<float, 7 * Robot::n_eef> transform1;
            std::memcpy(transform1.data(), eef_pose_w_ref_reference.data(), sizeof(float) * 7 * Robot::n_eef);

            std::array<float, 7 * Robot::n_eef> transform2;
            std::memcpy(transform2.data(), ref_frame_w_world.data(), sizeof(float) * 7 * Robot::n_eef);

            assignBlock<rake, 7 * Robot::n_eef>(transform1, tsr_function_inp.rTeB);
            assignBlock<rake, 7 * Robot::n_eef>(transform2, tsr_function_inp.wTrB);

            assignBlock<rake, 6 * Robot::n_eef>(lower_bound, tsr_function_inp.lbB);
            assignBlock<rake, 6 * Robot::n_eef>(upper_bound, tsr_function_inp.ubB);
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "Q input: ";
            // for(auto i=0U; i < Robot::dimension; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.q[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.rTeB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.wTrB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Lower limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.lbB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Upper limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.ubB[{i, 0}] << " ";
            // std::cout << std::endl;

            // std::cout << "TSR Error : " << std::endl;
            // for(auto i=0U; i < 6 * 2 * Robot::dimension; i++){
            //     if(i % Robot::dimension == 0)
            //         std::cout << std::endl << "J[" << i / Robot::dimension << "] : ";
            //     std::cout << short_jac_proj_inp.J[{i, 0}] << " ";
            // }
            // std::cout << std::endl;
            std::cout << "TSR Error : ";
            for (auto i = 0U; i < 6 * Robot::n_eef; i++)
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

            Robot::template tsr_error<rake>(tsr_function_inp, jac_proj_inp);

            const size_t jac_offset = 6 * Robot::n_eef * Robot::dimension;

            for (size_t i = 0; i < 6 * Robot::n_eef; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            }

            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6 * Robot::n_eef; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }

            return d;
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            (void)distanceToConstraint(q);
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else
            {
                throw std::runtime_error("Invalid projection method");
            }
            ConfigurationBlock q_new;
            integrateJointConfiguration<Robot, rake>(q, q_new, grad, alpha);
            return q_new;
        }
    };

}  // namespace vamp::planning::constraint