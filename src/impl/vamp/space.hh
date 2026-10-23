#pragma once

#include <cstddef>

namespace vamp
{
    // What planners, samplers and local planners need to know about the Space they plan in, behind one name.
    //
    // A Space names its points State and sits over an ambient robot (Space::Ambient, the Robot the planner is
    // templated on): collision checking and forward kinematics always work on the ambient Configuration, which a state
    // maps to with to_ambient(). A Space provides
    //
    //   State, StateBuffer, StateBlock<rake>   the state vector, an aligned scalar buffer of it, a rake of states
    //   to_ambient(state) -> Configuration     the robot configuration of a state
    //   with_ambient(state, q) -> State        the state with its configuration replaced by q
    //   distance(a, b), in_bounds(state)       the metric the planner measures extensions with, and the limits
    //
    // plus whatever extra coordinates it adds to the configuration (see SplineSpace).
    //
    // The robot is its own Space (the default of every `typename Space = Robot` parameter). That case is the
    // specialization below, written in terms of the robot's Configuration types so that robots without State aliases
    // keep working.
    template <typename Robot, typename Space = Robot>
    struct SpaceTraits
    {
        using State = typename Space::State;
        using StateBuffer = typename Space::StateBuffer;

        template <std::size_t rake>
        using StateBlock = typename Space::template StateBlock<rake>;

        static constexpr std::size_t dimension = State::num_scalars;

        // Coordinates beyond the robot's joints
        static constexpr std::size_t extra = dimension - Robot::dimension;

        static inline auto ambient(const State &s) noexcept -> typename Robot::Configuration
        {
            return Space::to_ambient(s);
        }

        static inline auto with_ambient(const State &s, const typename Robot::Configuration &q) noexcept -> State
        {
            return Space::with_ambient(s, q);
        }

        static inline auto distance(const State &a, const State &b) noexcept -> float
        {
            return Space::distance(a, b);
        }

        static inline auto in_bounds(const State &s) noexcept -> bool
        {
            return Space::in_bounds(s);
        }
    };

    template <typename Robot>
    struct SpaceTraits<Robot, Robot>
    {
        using State = typename Robot::Configuration;
        using StateBuffer = typename Robot::ConfigurationBuffer;

        template <std::size_t rake>
        using StateBlock = typename Robot::template ConfigurationBlock<rake>;

        static constexpr std::size_t dimension = Robot::dimension;
        static constexpr std::size_t extra = 0;

        static inline auto ambient(const State &s) noexcept -> const State &
        {
            return s;
        }

        static inline auto with_ambient(const State & /*s*/, const typename Robot::Configuration &q) noexcept
            -> State
        {
            return q;
        }

        static inline auto distance(const State &a, const State &b) noexcept -> float
        {
            return Robot::distance(a, b);
        }

        static inline auto in_bounds(const State &s) noexcept -> bool
        {
            return Robot::in_bounds(s);
        }
    };
}  // namespace vamp
