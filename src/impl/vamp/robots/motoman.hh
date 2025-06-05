#pragma once

#include <vamp/robots/motoman/fk.hh>
#include <vamp/vector.hh>

namespace vamp::robots
{
    struct Motoman
    {
        static constexpr auto name = "motoman";
        static constexpr auto dimension = 16;
        static constexpr auto resolution = 64;
        static constexpr auto n_spheres = motoman::n_spheres;
        static constexpr auto space_measure = motoman::space_measure;

        using Configuration = FloatVector<dimension>;
        using ConfigurationArray = std::array<FloatT, dimension>;

        struct alignas(FloatVectorAlignment) ConfigurationBuffer
          : std::array<float, Configuration::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using ConfigurationBlock = motoman::ConfigurationBlock<rake>;

        template <std::size_t rake>
        using Spheres = motoman::Spheres<rake>;

        static constexpr auto scale_configuration = motoman::scale_configuration;
        static constexpr auto descale_configuration = motoman::descale_configuration;

        template <std::size_t rake>
        static constexpr auto scale_configuration_block = motoman::scale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto descale_configuration_block = motoman::descale_configuration_block<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc = motoman::interleaved_sphere_fk<rake>;

        template <std::size_t rake>
        static constexpr auto fkcc_attach = motoman::interleaved_sphere_fk_attachment<rake>;

        template <std::size_t rake>
        static constexpr auto sphere_fk = motoman::sphere_fk<rake>;

        static constexpr auto eefk = motoman::eefk;
    };
}  // namespace vamp::robots
