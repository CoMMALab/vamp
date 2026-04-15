#pragma once

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <iostream>
#include <iomanip>

namespace vamp::planning::constraint
{
    template <std::size_t rake, std::size_t dim>
    inline static auto assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
    {
        for (size_t i = 0; i < dim; i++)
        {
            dest[i] = src[i];
        }
    }

    template <typename Robot, std::size_t rake>
    inline static void integrateJointConfiguration(
        const typename Robot::ConfigurationBlock<rake> &q,
        typename Robot::ConfigurationBlock<rake> &q_new,
        const typename Robot::ConfigurationBlock<rake> &gradient,
        float alpha = 1.0F)
    {
        for (size_t i = 0; i < Robot::dimension; i++)
        {
            q_new[i] = q[i] - gradient[i] * alpha;
        }
        Robot::descale_configuration_block(q_new);
        for (auto i = 0U; i < Robot::dimension; i++)
            q_new[i] = q_new[i].clamp(0.F, 1.F);
        Robot::scale_configuration_block(q_new);
    }

}  // namespace vamp::planning::constraint
