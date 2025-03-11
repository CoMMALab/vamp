#pragma once

#include <algorithm>
#include <vamp/random/rng.hh>
#include <cmath>

namespace vamp::rng
{
    template <std::size_t dim>
    struct Gaussian : public RNG<dim>, public Distribution
    {
        explicit Gaussian(FloatVector<dim> mean_in, FloatVector<dim> stddev_in) noexcept 
            : mean(mean_in), stddev(stddev_in)
        {
        }

        Gaussian(std::initializer_list<FloatT> mean_list, std::initializer_list<FloatT> stddev_list) noexcept 
            : Gaussian(FloatVector<dim>::pack_and_pad(mean_list), FloatVector<dim>::pack_and_pad(stddev_list))
        {
        }

        explicit Gaussian() : Gaussian(FloatVector<dim>::fill(0.0F), FloatVector<dim>::fill(1.0F))
        {
        }

        inline void reset() noexcept override final
        {
            has_spare = false;
        }

        inline auto next() noexcept -> FloatVector<dim> override final
        {
            if (has_spare)
            {
                has_spare = false;
                return mean + stddev * spare;
            }
            
            float u1, u2, r, theta;
            do
            {
                u1 = uniform_01();
                u2 = uniform_01();
            } while (u1 <= 0.0F);
            
            r = std::sqrt(-2.0F * std::log(u1));
            theta = 2.0F * M_PI * u2;
            
            float z0 = r * std::cos(theta);
            spare = r * std::sin(theta);
            has_spare = true;
            
            return mean + stddev * FloatVector<dim>::fill(z0);
        }

        FloatVector<dim> mean;
        FloatVector<dim> stddev;
        float spare;
        bool has_spare = false;
    };
} // namespace vamp::rng
