#pragma once

#include <cstddef>
#include <cstdint>

namespace vamp::planning::constraint
{
    enum struct ProjMethod : std::uint8_t
    {
        InnerLM,   // J^T (J J^T + lambda I)^-1 e: solves in task space (6 n_eef square system)
        OuterLM,   // (J^T J + lambda I)^-1 J^T e: solves in configuration space (nq square system)
        GradDesc,  // J^T e
    };

    struct ConstraintSettings
    {
        ProjMethod method = ProjMethod::InnerLM;

        // Step size along the projection gradient.
        float descend_rate = 1.F;

        // Convergence threshold on the squared constraint-violation error.
        float tolerance = 1e-6F;

        // Iteration cap for a single projection.
        std::size_t max_iterations = 15;

        // Scale of the per-lane target perturbations applied before projecting a steer
        // candidate: each SIMD lane projects a slightly different target and the first lane
        // to converge wins.
        float perturbation_scale = 0.1F;

        // Emit every waypoint of a projected chain from steer, not just the endpoint.
        bool emit_all_waypoints = true;

        // Multiplier on a connect loop's direct-path step budget: projection drift makes
        // constrained connects wander, so allow extra steps before giving up.
        float connect_slack = 2.F;

        // Squared radius around the steer target within which the endpoint counts as
        // Reached: projection rarely lands exactly on the target.
        float reached_radius2 = 1e-2F;

        // Squared tolerance for a traced chain to count as attaining its endpoint.
        float endpoint_tolerance2 = 1e-6F;

        // Keep rows that are already satisfied (error exactly zero inside their bounds) in the
        // projection Jacobian instead of dropping them (default: keep). Kept rows act as "hold this
        // pose value" equalities, so projection corrects only the violated rows and leaves everything
        // else where the sample put it (steps keep their length, which explores faster on some
        // problems, e.g. a pinned-height pen on a plane or a humanoid's feet). Dropped rows let
        // projection slide along any directions that are within bounds, which finds the nearest
        // manifold point but shortens steps, and can be more reliable when few free directions
        // remain (measured: Digit, the sphere cage, a hand-held mug, task-space starts). Affects the
        // bounded task-space constraints (TaskSpace, BimanualTaskSpace); the manifold is unchanged.
        bool hold_satisfied_rows = true;
    };
}  // namespace vamp::planning::constraint
