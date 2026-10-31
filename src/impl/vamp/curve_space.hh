#pragma once

#include <array>
#include <cstddef>

#include <vamp/vector.hh>

namespace vamp
{
    // The space (q, t) of a robot's configurations extended with a curve parameter t in [0, 1], for planning along a
    // curve in task space. It knows nothing of the curve: the constraints that hold the end-effector on a curve read
    // the t of a state (see ConstrainedLocalPlanner, which hands it to them before every projection). Header-only: the
    // robot is only used through its Configuration and its distance, interpolation and bounds.
    //
    // The state is the joints, then t. distance() here is the robot's on the joints and ignores t: it is the length of a
    // path (its cost), which is the joints' travel. The metric the planner measures extensions and nearest neighbours
    // with is the weighted one, sqrt(|dq|^2 + (w dt)^2), which lives in the nearest-neighbour tree (KDTree's parameter
    // weight) and in the planner's step clamp, since the weight is a setting of the planning run and a Space is static.
    // Interpolation moves t linearly along with the joints, for any weight (a weighted norm has the same straight lines).
    // See SpaceTraits (vamp/space.hh) for what a planner reads off a Space.
    template <typename Robot>
    struct CurveSpace
    {
        static constexpr const char *name = "curve_space";

        // The robot this space sits over: what collision checking and the constraint kernels work on
        using Ambient = Robot;
        using Configuration = typename Robot::Configuration;

        // The state is the joints, then t
        static constexpr std::size_t t_index = Robot::dimension;
        static constexpr std::size_t dimension = Robot::dimension + 1;
        static constexpr std::size_t sample_dimension = Robot::sample_dimension;

        using State = FloatVector<dimension>;

        struct alignas(FloatVectorAlignment) StateArray : std::array<FloatT, dimension>
        {
        };

        struct alignas(FloatVectorAlignment) StateBuffer : std::array<float, State::num_scalars_rounded>
        {
        };

        template <std::size_t rake>
        using StateBlock = FloatVector<rake, dimension>;

        static inline auto make_state(const Configuration &q, float t) noexcept -> State
        {
            std::array<float, dimension> state;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                state[j] = array[j];
            }

            state[t_index] = t;
            return State(state);
        }

        static inline auto to_ambient(const State &s) noexcept -> Configuration
        {
            std::array<float, Robot::dimension> q;
            const auto array = s.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                q[j] = array[j];
            }

            return Configuration(q);
        }

        static inline auto parameter(const State &s) noexcept -> float
        {
            return s.to_array()[t_index];
        }

        static inline auto with_ambient(const State &s, const Configuration &q) noexcept -> State
        {
            return make_state(q, parameter(s));
        }

        // The metric ignores t
        static inline auto distance(const State &a, const State &b) noexcept -> float
        {
            return Robot::distance(to_ambient(a), to_ambient(b));
        }

        static inline auto interpolate(const State &a, const State &b, float f) noexcept -> State
        {
            const auto ta = parameter(a);
            const auto tb = parameter(b);
            return make_state(Robot::interpolate(to_ambient(a), to_ambient(b), f), ta + (tb - ta) * f);
        }

        static inline auto in_bounds(const State &s) noexcept -> bool
        {
            const auto t = parameter(s);
            return Robot::in_bounds(to_ambient(s)) and t >= 0.0F and t <= 1.0F;
        }
    };
}  // namespace vamp
