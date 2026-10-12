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
    };

    // Knobs for chart construction and chart-LQMT edge validation (ChartLocalPlanner).
    struct ChartSettings
    {
        // Maximum ambient displacement between a lifted sample and its chart pre-image:
        // bounds how far a chart is trusted before an edge is rejected.
        float eps_chart = 0.25F;

        // Consecutive-lifted-sample jump guard multiplier: the allowed jump between
        // adjacent lifted samples is cont_factor * max(chart step, 1e-3), rejecting lifts
        // the projection tears apart.
        float cont_factor = 4.F;

        // Position/velocity tolerances for an exact connect to count as attained. TSR
        // bounds define a slab, not a zero-thickness manifold: on-constraint states can
        // differ by the bound width along the normal space, so the position tolerance
        // must cover the widest manifold-defining bound. The velocity tolerance is
        // speed-relative: tol * (1 + |target velocity|).
        float reached_pos_tol = 5e-2F;
        float reached_vel_tol = 5e-2F;

        // Shooting iterations for exact connects: u_f += B^T (q_t - psi(u_f)), whose
        // fixed point lifts exactly onto the target.
        std::size_t shoot_iters = 4;

        // Cap on lifted collision-check samples per edge (rounded down to a multiple of
        // the SIMD width).
        std::size_t max_edge_samples = 512;

        // Relative cutoff on the pivoted-QR R diagonal for the rank of the stacked
        // active-row Jacobian.
        float rank_tolerance = 1e-4F;

        // Per-edge slip tolerance: budget on the drift along Pfaffian row normals, as a
        // fraction of the executed path length: sum_j |A(q_j) (q_j - q_{j-1})| over the
        // executed samples must stay within max_slip_fraction of sum_j |q_j - q_{j-1}|.
        // Those rows carry no position error for the per-sample projection to correct,
        // so an arc in the chart fixed at the edge start slips along their normals to
        // first order in the rows' rotation across the edge -- and cost shortcutting
        // will exploit that slip in place of genuine steering unless it is bounded.
        // Rows are unit-normalized, so the fraction never exceeds 1: values >= 1
        // disable the bound. At 0.02 the net (signed) drift of a rapidly rotating row
        // drops to projection-noise level while planning still connects; looser values
        // leave shortcutting a proportional budget to cheat with.
        float max_slip_fraction = 0.02F;

        // Exact charts: lift chart samples along the chart's normal space onto the zero set
        // of the constraints' smooth equality form, q = q0 + B u + N w with F(q) = 0 (the
        // tangent-space parametrization of atlas methods), instead of projecting q0 + B u.
        // The executed velocity and acceleration then follow exactly by implicit
        // differentiation of F(sigma(t)) = 0 -- including the curvature (centripetal) term the
        // projection chart's chart-frame accelerations omit -- so torque and velocity checks
        // judge the executed trajectory, and an edge lands exactly on an on-manifold target
        // with no shooting. Requires every constraint to provide an equality form (e.g. a
        // rigid bimanual grasp: all relative-pose rows tight) and no Pfaffian rows.
        bool exact_chart = false;

        // Exact-chart Newton convergence threshold on the squared equality error, and the
        // per-sample iteration cap. Newton converges quadratically, so the tight default
        // (|F| <= 1e-6, near the float evaluation floor) costs about one extra iteration and
        // keeps sample positions smooth enough that finite differences of the executed
        // trajectory agree with the exact velocities.
        float exact_tolerance = 1e-12F;
        std::size_t exact_iterations = 8;

        // Exact-chart lifts run all rake lanes through SIMD rows: one kernel evaluation, normal-space
        // system, LU and solve per Newton iteration for the whole batch (false: the per-lane scalar
        // path, kept for comparison).
        bool batched_exact = true;

        // Batched lifts use a constraint's generated exact-chart step kernel (straight-line SIMD
        // normal-space solve) when the set is a single constraint that has one (false: the batched
        // generic LU).
        bool step_kernel = true;

        // Before lifting an edge, one SIMD batch of samples spread over the whole edge is checked on
        // the tangent-plane approximation (chart point q0 + B u, chart-frame velocity and
        // acceleration, no lift) against the effort limits inflated by this margin; edges that fail
        // are rejected without lifting. Most rejected edges fail on torque. <= 0 disables.
        float prescreen_margin = 0.15F;
    };
}  // namespace vamp::planning::constraint
