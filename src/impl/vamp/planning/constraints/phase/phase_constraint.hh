#pragma once

#include <array>
#include <type_traits>

#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // End-effector payload sizes: Robot::payload_dimension on flask robots, zero otherwise
    // (geometric robots instantiate empty phase-constraint sets too).
    template <typename Robot, typename = void>
    struct payload_size : std::integral_constant<std::size_t, 0>
    {
    };

    template <typename Robot>
    struct payload_size<Robot, std::void_t<decltype(Robot::payload_dimension)>>
      : std::integral_constant<std::size_t, Robot::payload_dimension>
    {
    };

    // Same types as Robot::Payload / Robot::PayloadBlock<rake> where those exist.
    template <typename Robot>
    using PayloadOf = std::array<float, payload_size<Robot>::value>;
    template <typename Robot, std::size_t rake>
    using PayloadBlockOf = std::array<FloatVector<rake, 1>, payload_size<Robot>::value>;

    // Type-erased phase-space inequality constraint g(q, qd) <= 0 over a rake-wide block of
    // flat states z = (q, qd). Unlike Constraint, a phase constraint defines no manifold:
    // states are checked against it, never projected onto it, and feasibility is restored by scaling
    // velocities (every instance is homogeneous in qd, so scaling qd down always reaches
    // the feasible set).
    //
    // Implementations cache per-lane results of the last evaluate() call for extract(), so
    // batch callers pay each kernel once per block instead of per lane. Instances are not
    // thread-safe: use one instance per thread.
    template <typename Robot, std::size_t rake>
    struct PhaseConstraint
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;

        // End-effector payload dynamic parameters (Robot::Payload, from the environment's
        // attachments): kernels that depend on the robot's inertia read them, purely kinematic
        // ones ignore them. The owning PhaseConstraintSet supplies them.
        using Payload = PayloadOf<Robot>;
        using PayloadBlock = PayloadBlockOf<Robot, rake>;

        virtual ~PhaseConstraint() = default;

        // Run the kernel on the whole block, caching every lane for extract(). Only the
        // [q; qd] rows of the block are read; acceleration rows are ignored.
        virtual void evaluate(const Block &x, const PayloadBlock &payload) const noexcept = 0;

        // Whether one SIMD lane of the last evaluate() block satisfies the constraint.
        virtual auto extract(std::size_t lane) const noexcept -> bool = 0;

        // Largest s in (0, 1] such that (q, s * qd) satisfies the constraint; exactly 1 if
        // z = (q, qd) is already feasible, so applying the scale is idempotent.
        virtual auto velocity_scale(const Configuration &z, const Payload &payload) const noexcept -> float = 0;
    };
}  // namespace vamp::planning::constraint
