#pragma once

#include <array>
#include <cstddef>
#include <type_traits>
#include <utility>

#include <vamp/vector.hh>

// Optional robot hooks for planning in non-Euclidean configuration spaces (e.g. floating bases with
// quaternion dimensions). A robot may define:
//
//   static constexpr bool euclidean;                        // joint space is a flat L2 space
//   static constexpr std::size_t sample_dimension;          // dimension of the sampler's unit cube
//   static auto sample(FloatVector<sample_dimension>) -> Configuration;   // unit cube -> configuration
//   static auto distance(const Configuration &, const Configuration &) -> float;
//   static auto interpolate(const Configuration &, const Configuration &, float t) -> Configuration;
//   static constexpr std::array<std::size_t, N> so3_offsets;  // start index of each quaternion block
//
// Robots that define none of them are Euclidean, sample the configuration cube directly, and use
// Configuration::distance and Configuration::interpolate. The helpers below resolve each hook with
// that fallback, so planning code can be written once for both kinds of robot.
namespace vamp::robots
{
    namespace detail
    {
        template <typename Robot, typename = void>
        struct euclidean : std::true_type
        {
        };

        template <typename Robot>
        struct euclidean<Robot, std::void_t<decltype(Robot::euclidean)>>
          : std::bool_constant<Robot::euclidean>
        {
        };

        template <typename Robot, typename = void>
        struct so3_offsets
        {
            static constexpr std::array<std::size_t, 0> value{};
        };

        template <typename Robot>
        struct so3_offsets<Robot, std::void_t<decltype(Robot::so3_offsets)>>
        {
            static constexpr auto value = Robot::so3_offsets;
        };

        template <typename Robot, typename = void>
        struct sample_dimension : std::integral_constant<std::size_t, Robot::dimension>
        {
        };

        template <typename Robot>
        struct sample_dimension<Robot, std::void_t<decltype(Robot::sample_dimension)>>
          : std::integral_constant<std::size_t, Robot::sample_dimension>
        {
        };

        template <typename Robot, typename = void>
        struct has_sample : std::false_type
        {
        };

        template <typename Robot>
        struct has_sample<
            Robot,
            std::void_t<decltype(Robot::sample(
                std::declval<const FloatVector<sample_dimension<Robot>::value> &>()))>> : std::true_type
        {
        };

        template <typename Robot, typename = void>
        struct has_distance : std::false_type
        {
        };

        template <typename Robot>
        struct has_distance<
            Robot,
            std::void_t<decltype(Robot::distance(
                std::declval<const typename Robot::Configuration &>(),
                std::declval<const typename Robot::Configuration &>()))>> : std::true_type
        {
        };

        template <typename Robot, typename = void>
        struct has_interpolate : std::false_type
        {
        };

        template <typename Robot>
        struct has_interpolate<
            Robot,
            std::void_t<decltype(Robot::interpolate(
                std::declval<const typename Robot::Configuration &>(),
                std::declval<const typename Robot::Configuration &>(),
                0.F))>> : std::true_type
        {
        };
    }  // namespace detail

    template <typename Robot>
    inline constexpr bool is_euclidean_v = detail::euclidean<Robot>::value;

    // Start index of each quaternion (SO(3)) block in the configuration; empty if there are none.
    template <typename Robot>
    inline constexpr auto so3_offsets_v = detail::so3_offsets<Robot>::value;

    template <typename Robot>
    inline constexpr std::size_t sample_dimension_v = detail::sample_dimension<Robot>::value;

    // Map a point of the sampler's unit cube to a configuration.
    template <typename Robot>
    [[nodiscard]] inline auto sample(const FloatVector<sample_dimension_v<Robot>> &unit) noexcept ->
        typename Robot::Configuration
    {
        if constexpr (detail::has_sample<Robot>::value)
        {
            return Robot::sample(unit);
        }
        else
        {
            auto configuration = unit;
            Robot::scale_configuration(configuration);
            return configuration;
        }
    }

    template <typename Robot>
    [[nodiscard]] inline auto
    distance(const typename Robot::Configuration &a, const typename Robot::Configuration &b) noexcept -> float
    {
        if constexpr (detail::has_distance<Robot>::value)
        {
            return Robot::distance(a, b);
        }
        else
        {
            return a.distance(b);
        }
    }

    template <typename Robot>
    [[nodiscard]] inline auto interpolate(
        const typename Robot::Configuration &a,
        const typename Robot::Configuration &b,
        float t) noexcept -> typename Robot::Configuration
    {
        if constexpr (detail::has_interpolate<Robot>::value)
        {
            return Robot::interpolate(a, b, t);
        }
        else
        {
            return a.interpolate(b, t);
        }
    }
}  // namespace vamp::robots
