#pragma once

#include <array>
#include <cstddef>
#include <type_traits>
#include <utility>

#include <vamp/robots/traits.hh>
#include <vamp/vector.hh>

// A planning *space* is the state space a planner searches. A robot is its own space (its joint
// configuration space), but a space can be something else: for example a task-space
// parameterization whose states are resolved into the robot's configurations (an IK-parameterized
// space). Planning code that is written against a space instead of a robot works for both.
//
// A space S provides:
//
//   S::dimension                      number of scalars in a state
//   S::State                          FloatVector<S::dimension>      (robots: Configuration)
//
// and may provide, with the robot-hook fallbacks of robots/traits.hh:
//
//   static constexpr bool euclidean;                       default true
//   static constexpr std::array so3_offsets;               default empty
//   static auto distance(State, State) -> float;           default State::distance
//   static auto interpolate(State, State, float) -> State; default State::interpolate
//
// A space that is not the robot's own sits over an *ambient* robot (S::Ambient; a robot is its own
// ambient): collision checking, forward kinematics and constraints work on the ambient configuration,
// and a state is mapped to it by *resolving* it. A space provides, when it is not its own ambient:
//
//   using Ambient = Robot;                                         the robot it sits over
//   template <size_t rake> using StateBlock = FloatVector<rake, dimension>;
//   static auto to_ambient(State) -> Ambient::Configuration;       one state -> configuration
//   static auto with_ambient(State, Ambient::Configuration) -> State;   replace the configuration part
//   template <size_t rake> static auto resolve_block(StateBlock<rake>)
//       -> std::pair<bool, Ambient::ConfigurationBlock<rake>>;     a block of states -> configurations;
//                                                                  the bool is false if any lane cannot be
//                                                                  resolved (an IK failure, say)
//   static auto in_bounds(State) -> bool;
//
// Every robot is a space as is: `State` falls back to `Configuration` and `Ambient` to the robot itself
// (to_ambient/with_ambient are the identity and resolve is the identity with every lane valid), so no
// regeneration of robot headers is needed. Examples: a robot, a curve space (q, t) whose resolve drops t, and an
// IK-parameterized task-space space whose resolve runs the inverse kinematics.
namespace vamp::planning::space
{
    namespace detail
    {
        template <typename S, typename = void>
        struct state_of
        {
            using type = typename S::Configuration;
        };

        template <typename S>
        struct state_of<S, std::void_t<typename S::State>>
        {
            using type = typename S::State;
        };

        template <typename S>
        using state_t = typename state_of<S>::type;

        template <typename S, typename = void>
        struct ambient_of
        {
            using type = S;
        };

        template <typename S>
        struct ambient_of<S, std::void_t<typename S::Ambient>>
        {
            using type = typename S::Ambient;
        };

        // An aligned scalar buffer for a state of S, for spaces that do not define one.
        template <typename S>
        struct alignas(FloatVectorAlignment) GenericStateBuffer : std::array<float, state_t<S>::num_scalars_rounded>
        {
        };

        template <typename S, typename = void>
        struct configuration_buffer_of
        {
            using type = GenericStateBuffer<S>;
        };

        template <typename S>
        struct configuration_buffer_of<S, std::void_t<typename S::ConfigurationBuffer>>
        {
            using type = typename S::ConfigurationBuffer;
        };

        template <typename S, typename = void>
        struct buffer_of
        {
            using type = typename configuration_buffer_of<S>::type;
        };

        template <typename S>
        struct buffer_of<S, std::void_t<typename S::StateBuffer>>
        {
            using type = typename S::StateBuffer;
        };

        template <typename S, std::size_t rake, typename = void>
        struct block_of
        {
            using type = typename S::template ConfigurationBlock<rake>;
        };

        template <typename S, std::size_t rake>
        struct block_of<S, rake, std::void_t<typename S::template StateBlock<rake>>>
        {
            using type = typename S::template StateBlock<rake>;
        };

        template <typename S, std::size_t rake>
        using block_of_t = typename block_of<S, rake>::type;

        template <typename S, typename = void>
        struct has_to_ambient : std::false_type
        {
        };

        template <typename S>
        struct has_to_ambient<S, std::void_t<decltype(S::to_ambient(std::declval<const state_t<S> &>()))>>
          : std::true_type
        {
        };

        template <typename S, typename = void>
        struct has_in_bounds : std::false_type
        {
        };

        template <typename S>
        struct has_in_bounds<S, std::void_t<decltype(S::in_bounds(std::declval<const state_t<S> &>()))>>
          : std::true_type
        {
        };

        template <typename S, std::size_t rake, typename = void>
        struct has_interpolate_block : std::false_type
        {
        };

        template <typename S, std::size_t rake>
        struct has_interpolate_block<
            S,
            rake,
            std::void_t<decltype(S::template interpolate_block<rake>(
                std::declval<const state_t<S> &>(),
                std::declval<const state_t<S> &>(),
                std::declval<const FloatVector<rake> &>(),
                std::declval<block_of_t<S, rake> &>()))>> : std::true_type
        {
        };

        template <typename S, typename = void>
        struct has_scale : std::false_type
        {
        };

        template <typename S>
        struct has_scale<S, std::void_t<decltype(S::scale_configuration(std::declval<state_t<S> &>()))>>
          : std::true_type
        {
        };

        template <typename S, typename = void>
        struct has_space_measure : std::false_type
        {
        };

        template <typename S>
        struct has_space_measure<S, std::void_t<decltype(S::space_measure())>> : std::true_type
        {
        };

        template <typename S, typename = void>
        struct has_distance : std::false_type
        {
        };

        template <typename S>
        struct has_distance<
            S,
            std::void_t<decltype(S::distance(
                std::declval<const state_t<S> &>(),
                std::declval<const state_t<S> &>()))>> : std::true_type
        {
        };

        template <typename S, typename = void>
        struct has_interpolate : std::false_type
        {
        };

        template <typename S>
        struct has_interpolate<
            S,
            std::void_t<decltype(S::interpolate(
                std::declval<const state_t<S> &>(),
                std::declval<const state_t<S> &>(),
                0.F))>> : std::true_type
        {
        };
    }  // namespace detail

    // The state type of a space (a robot's Configuration when the space is a robot).
    template <typename S>
    using State = detail::state_t<S>;

    template <typename S>
    inline constexpr std::size_t dimension_v = S::dimension;

    // The robot a space sits over (the space itself when it is a robot).
    template <typename S>
    using Ambient = typename detail::ambient_of<S>::type;

    template <typename S>
    using StateBuffer = typename detail::buffer_of<S>::type;

    template <typename S, std::size_t rake>
    using StateBlock = typename detail::block_of<S, rake>::type;

    template <typename S>
    using AmbientConfiguration = typename Ambient<S>::Configuration;

    template <typename S, std::size_t rake>
    using AmbientBlock = typename Ambient<S>::template ConfigurationBlock<rake>;

    template <typename S>
    inline constexpr bool is_own_ambient_v = std::is_same_v<Ambient<S>, S>;

    template <typename S>
    inline constexpr bool euclidean_v = robots::detail::euclidean<S>::value;

    template <typename S>
    inline constexpr auto so3_offsets_v = robots::detail::so3_offsets<S>::value;

    // True when S looks like a planning space: it has a dimension and its states are vectors of
    // that dimension.
    template <typename S, typename = void>
    struct is_space : std::false_type
    {
    };

    template <typename S>
    struct is_space<S, std::void_t<decltype(S::dimension), detail::state_t<S>>>
      : std::is_same<detail::state_t<S>, FloatVector<S::dimension>>
    {
    };

    template <typename S>
    inline constexpr bool is_space_v = is_space<S>::value;

    template <typename S>
    [[nodiscard]] inline auto distance(const State<S> &a, const State<S> &b) noexcept -> float
    {
        if constexpr (detail::has_distance<S>::value)
        {
            return S::distance(a, b);
        }
        else
        {
            return a.distance(b);
        }
    }

    template <typename S>
    [[nodiscard]] inline auto interpolate(const State<S> &a, const State<S> &b, float t) noexcept -> State<S>
    {
        if constexpr (detail::has_interpolate<S>::value)
        {
            return S::interpolate(a, b, t);
        }
        else
        {
            return a.interpolate(b, t);
        }
    }

    // The ambient configuration of a state. The identity for a robot.
    template <typename S>
    [[nodiscard]] inline auto to_ambient(const State<S> &s) noexcept -> AmbientConfiguration<S>
    {
        if constexpr (detail::has_to_ambient<S>::value)
        {
            return S::to_ambient(s);
        }
        else
        {
            static_assert(is_own_ambient_v<S>, "a space over an ambient robot must provide to_ambient");
            return s;
        }
    }

    // The state with its configuration part replaced by q. A robot's state is its configuration.
    template <typename S>
    [[nodiscard]] inline auto with_ambient(const State<S> &s, const AmbientConfiguration<S> &q) noexcept
        -> State<S>
    {
        if constexpr (not is_own_ambient_v<S>)
        {
            return S::with_ambient(s, q);
        }
        else
        {
            (void)s;
            return q;
        }
    }

    // Resolve a block of states into the ambient robot's configurations. The bool is false when any lane
    // could not be resolved. For a robot every lane is its own configuration and always resolves.
    template <typename S, std::size_t rake>
    [[nodiscard]] inline auto resolve(const StateBlock<S, rake> &block) noexcept
        -> std::pair<bool, AmbientBlock<S, rake>>
    {
        if constexpr (not is_own_ambient_v<S>)
        {
            return S::template resolve_block<rake>(block);
        }
        else
        {
            return {true, block};
        }
    }

    // Whether a state is inside the space's limits.
    template <typename S>
    [[nodiscard]] inline auto in_bounds(const State<S> &s) noexcept -> bool
    {
        if constexpr (detail::has_in_bounds<S>::value)
        {
            return S::in_bounds(s);
        }
        else if constexpr (is_own_ambient_v<S>)
        {
            return true;
        }
        else
        {
            return in_bounds<Ambient<S>>(to_ambient<S>(s));
        }
    }

    // The states a -> b at the fractions in t, as a block (lane k holds the state at t[k]). Uses the space's
    // interpolate_block when it has one; otherwise the states lie on the straight line, which is what a
    // Euclidean space interpolates along.
    template <typename S, std::size_t rake>
    inline void interpolate_block(
        const State<S> &a,
        const State<S> &b,
        const FloatVector<rake> &t,
        StateBlock<S, rake> &out) noexcept
    {
        if constexpr (detail::has_interpolate_block<S, rake>::value)
        {
            S::template interpolate_block<rake>(a, b, t, out);
        }
        else
        {
            static_assert(euclidean_v<S>, "a non-Euclidean space must provide interpolate_block");
            for (auto j = 0U; j < S::dimension; ++j)
            {
                out[j] = a.broadcast(j) + (b.broadcast(j) - a.broadcast(j)) * t;
            }
        }
    }

    // Size of the sampler's unit cube for a space (its dimension unless the space says otherwise).
    template <typename S>
    inline constexpr std::size_t sample_dimension_v = robots::sample_dimension_v<S>;

    // Map a point of the sampler's unit cube to a state of the space.
    template <typename S>
    [[nodiscard]] inline auto sample(const FloatVector<sample_dimension_v<S>> &unit) noexcept -> State<S>
    {
        if constexpr (robots::detail::has_sample<S>::value)
        {
            return S::sample(unit);
        }
        else
        {
            static_assert(detail::has_scale<S>::value, "a space needs sample() or scale_configuration() to be sampled");
            auto state = unit;
            S::scale_configuration(state);
            return state;
        }
    }

    // Whether a space can be scaled from the unit cube and back (what informed sampling needs in a Euclidean space).
    template <typename S>
    inline constexpr bool has_scale_v = detail::has_scale<S>::value;

    // The measure of the space, used for the connection radius of asymptotically optimal planners.
    template <typename S>
    [[nodiscard]] inline auto space_measure() noexcept -> float
    {
        static_assert(detail::has_space_measure<S>::value, "a space needs space_measure() for GRRT*");
        return S::space_measure();
    }
}  // namespace vamp::planning::space
