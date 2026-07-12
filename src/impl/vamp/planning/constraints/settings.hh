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

        // Convergence threshold on the squared hinged constraint error.
        float tolerance = 1e-6F;

        // Iteration cap for a single projection.
        std::size_t max_iterations = 15;

        // Scale of the per-lane target perturbations applied before projecting a steer
        // candidate: each SIMD lane projects a slightly different target and the first lane
        // to converge wins.
        float perturbation_scale = 0.1F;

        // Emit every waypoint of a projected chain from steer, not just the frontier.
        bool emit_all_waypoints = true;
    };
}  // namespace vamp::planning::constraint
