#pragma once

#include <algorithm>
#include <array>
#include <vamp/random/rng.hh>
#include <cmath>

namespace vamp::rng
{
    template <typename Robot>
    struct Gaussian : public RNG<Robot>, public Distribution
    {
        static constexpr auto dim = Robot::dimension;

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
        }

        // Independent standard-normal draw per dimension (Box-Muller, two draws per pair).
        inline auto next() noexcept -> FloatVector<dim> override final
        {
            std::array<FloatT, dim> z;
            for (std::size_t d = 0; d < dim; d += 2)
            {
                float u1;
                do
                {
                    u1 = uniform_01();
                } while (u1 <= 0.0F);
                const float u2 = uniform_01();

                const float r = std::sqrt(-2.0F * std::log(u1));
                const float theta = 2.0F * static_cast<float>(M_PI) * u2;

                z[d] = r * std::cos(theta);
                if (d + 1 < dim)
                {
                    z[d + 1] = r * std::sin(theta);
                }
            }

            return mean + stddev * FloatVector<dim>(z);
        }

        FloatVector<dim> mean;
        FloatVector<dim> stddev;
    };
} // namespace vamp::rng
