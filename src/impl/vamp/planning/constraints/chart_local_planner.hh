#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/chart.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/phase/phase_constraint_set.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/flask.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/validate.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Local planner for manifold-constrained kinodynamic (flask) planning, pairing a flask
    // z-robot (states z = (q, qdot)) with its ambient position-space sibling
    // ZRobot::Ambient, whose constraint kernels define the manifold (MCFLASK).
    //
    // The manifold-restricted system is differentially flat with flat output equal to
    // tangent-space (chart) coordinates u in R^{d-k}: local paths are LQMT cubics solved
    // in the chart at the from-node -- an orthonormal basis B_0 of ker J(q_0) from a
    // pivoted QR of the transposed stacked active-row constraint Jacobian -- and lifted
    // onto the manifold by
    // batch projection, sigma(t) = P_M(q_0 + B_0 u(t)). Tree nodes are lifted states
    // (positions exactly on the manifold, velocities in its tangent space). Samples are
    // raw steering targets and need no projection, but states synthesized outside the
    // chart machinery -- simplify's B-spline subdivision midpoints and interpolants are
    // flat z-space cubics -- leave the manifold, so `projecting` is true. Extensions have
    // no interior waypoints: an edge is reconstructible from its endpoint states, since
    // the executed chart target is a fixed point of the shooting iteration at the
    // parent's chart.
    //
    // Directedness: goal-tree extensions (`forward` false) are validated as the
    // time-reversed edge -- same geometric path, negated velocities -- which leaves LQMT
    // durations/costs, velocity bounds, and rigid-body RNEA torques invariant.
    //
    // Holds mutable per-evaluation state (constraint caches, Jacobian buffers): not
    // thread-safe, use one instance (with unshared constraints) per thread.
    template <typename ZRobot, std::size_t rake, std::size_t resolution>
    struct ChartLocalPlanner
    {
        using Ambient = typename ZRobot::Ambient;

        static constexpr std::size_t d = Ambient::dimension;
        static_assert(ZRobot::flask, "ChartLocalPlanner requires a flask z-robot");
        static_assert(
            ZRobot::flat_dimension == d and ZRobot::dimension == 2 * d,
            "flask z-robot states must be (q, qdot) over the ambient robot's configuration space");

        using Configuration = typename ZRobot::Configuration;  // state (q, qdot)
        using Environment = collision::Environment<FloatVector<rake>>;
        using AmbientConfiguration = typename Ambient::Configuration;
        using AmbientBlock = typename Ambient::template ConfigurationBlock<rake>;
        using ZBlock = typename ZRobot::template ConfigurationBlock<rake>;  // (y, yd, ydd)

        // Configurations synthesized by flat z-space interpolation (simplify's B-spline
        // subdivision midpoints, interpolants) leave the manifold and must be projected.
        static constexpr bool projecting = true;

        // Lift displacement drifts connects sideways: allow twice the direct-path step
        // count before a connect loop gives up.
        static constexpr float connect_slack = 2.F;

        explicit ChartLocalPlanner(
            ConstraintSet<Ambient, rake> constraint_set,
            ChartSettings chart_settings = {},
            PhaseConstraintSet<ZRobot, rake> phase_constraint_set = {}) noexcept
          : constraints(std::move(constraint_set))
          , phase_constraints(std::move(phase_constraint_set))
          , settings(chart_settings)
          , builder_(constraints, settings.rank_tolerance)
          , err_(builder_.rows())
          , jac_batch_(rake * builder_.rows() * d)
        {
        }

        // Validity of the exact local path a -> b in execution order. The chart lives at
        // a (a tree node in both directions); backward paths validate the time reversal.
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward = true) const noexcept -> bool
        {
            float cost;
            return exact_edge(a, b, e, forward, cost);
        }

        // Admit the exact local path a -> b (execution order) only if it is valid, attains
        // b within the reached tolerances, and its LQMT cost is within `budget()`. The
        // extension has no interior waypoints (the edge is endpoint-reconstructible).
        template <typename Budget>
        inline auto connect_within(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Budget &&budget,
            std::size_t max_states) const noexcept -> Extension<ZRobot>
        {
            chain_.clear();

            float cost;
            const bool valid =
                0 < max_states and exact_edge(a, b, e, true, cost) and cost <= budget();
            return {(valid) ? SteerStatus::Reached : SteerStatus::Trapped, chain_};
        }

        // Steer from `from` toward `target` by at most `range` in chart coordinates (the
        // caller's state-space `distance` is ignored: reach is decided by the chart-space
        // displacement). Within range the edge shoots to hit the target exactly; beyond
        // it the chart target is clipped and the endpoint velocity still aims at the
        // target's. The endpoint is the lifted terminal state with its velocity
        // tangent-projected at its own chart.
        template <typename Accept = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float,
            float range,
            bool forward,
            const Environment &e,
            Accept &&accept = Accept()) const noexcept -> Extension<ZRobot>
        {
            chain_.clear();

            std::array<float, d> q0, v0, qt, vt;
            split(from, q0, v0);
            split(target, qt, vt);

            // Scale the target velocity toward phase feasibility: the scale is exactly 1
            // on feasible targets (stored nodes are unchanged); the per-sample phase
            // check in make_edge remains the soundness mechanism.
            phase_clamp(qt, vt);

            Edge edge;
            const auto status =
                make_edge(q0, v0, qt, vt, range, forward, e, std::forward<Accept>(accept), edge);
            if (status == EdgeStatus::Pruned)
            {
                return {SteerStatus::Rejected, chain_};
            }

            if (status != EdgeStatus::Valid or edge.trivial)
            {
                return {SteerStatus::Trapped, chain_};
            }

            // Degenerate (no-motion) extensions would insert duplicate nodes.
            float step2 = 0.F;
            for (auto i = 0U; i < d; ++i)
            {
                const float diff = edge.qf[i] - q0[i];
                step2 += diff * diff;
            }

            if (step2 < 1e-8F)
            {
                return {SteerStatus::Trapped, chain_};
            }

            std::array<float, d> vf;
            if (not arrival_velocity(edge, vf))
            {
                return {SteerStatus::Trapped, chain_};
            }

            chain_.emplace_back(join(edge.qf, vf));
            const bool reached = edge.reach and attained(edge.qf, vf, qt, vt);
            return {(reached) ? SteerStatus::Reached : SteerStatus::Advanced, chain_};
        }

        // Project a state onto the manifold: position by constraint projection, velocity
        // into the tangent space at the projected position, then scaled into the phase-
        // feasible set.
        inline auto project(Configuration &z) const noexcept -> bool
        {
            std::array<float, d> q, v;
            split(z, q, v);

            if (not lift_point(q, q))
            {
                return false;
            }

            const auto chart = builder_.make_chart(constraints, q);
            if (not chart.valid)
            {
                return false;
            }

            auto vt = tangent_project(chart, v);
            phase_clamp(q, vt);
            z = join(q, vt);
            return true;
        }

        // Whether a state's position satisfies every manifold constraint within tolerance
        // and the state satisfies every phase constraint.
        inline auto satisfied(const Configuration &z) const noexcept -> bool
        {
            std::array<float, d> q, v;
            split(z, q, v);
            return constraints.satisfied(AmbientConfiguration(q)) and phase_constraints.satisfied(z);
        }

        // Debug: the orthonormal tangent-space basis at a state's position, as nc rows of
        // d ambient components each. Empty when no chart exists there.
        inline auto debug_chart(const Configuration &z) const noexcept
            -> std::vector<std::array<float, d>>
        {
            std::array<float, d> q, v;
            split(z, q, v);
            const auto chart = builder_.make_chart(constraints, q);

            std::vector<std::array<float, d>> rows;
            if (chart.valid)
            {
                rows.assign(chart.basis.begin(), chart.basis.begin() + chart.nc);
            }

            return rows;
        }

        // Reconstruct the executed trajectory of the edge from -> target through the same
        // chart/LQMT/lift machinery, without any validation. Emits n_samples lifted states
        // at uniform times in [0, T] (velocities in the execution frame, tangent-projected
        // at each sample's own chart) and each sample's stacked constraint-violation
        // error norm. Samples run in chart time from `from`; backward edges are executed by
        // traversing them in reverse. A trivial (no-motion) edge emits the single lifted
        // from-state. False only when no chart exists at `from`, the initial chart target
        // cannot be lifted, or the time-optimal solve fails; per-sample lift failures
        // degrade to best-effort samples instead (see emit below).
        inline auto lift_edge(
            const Configuration &from,
            const Configuration &target,
            bool forward,
            std::size_t n_samples,
            std::vector<Configuration> &states,
            std::vector<float> &errors,
            float &duration,
            float &cost) const noexcept -> bool
        {
            states.clear();
            errors.clear();
            duration = 0.F;
            cost = 0.F;

            std::array<float, d> q0, v0, qt, vt;
            split(from, q0, v0);
            split(target, qt, vt);

            const float sign = (forward) ? 1.F : -1.F;

            EdgeSolve es;
            if (not solve_edge<true>(q0, v0, qt, vt, no_range, sign, es))
            {
                return false;
            }

            // Best-effort per-sample lift: a sample whose projection misses the caller's
            // budget (or has no chart for the velocity pushforward) is emitted
            // unprojected, its residual visible in the error norms, rather than failing
            // the whole replay of a known-valid edge.
            const auto emit = [&](const std::array<float, d> &q_amb,
                                  const std::array<float, d> &v_amb)
            {
                std::array<float, d> q_lift;
                if (not lift_point(q_amb, q_lift))
                {
                    q_lift = q_amb;
                }

                const auto chart_t = builder_.make_chart(constraints, q_lift);
                auto v_lift = chart_t.valid ? tangent_project(chart_t, v_amb) : v_amb;
                for (auto dim = 0U; dim < d; ++dim)
                {
                    v_lift[dim] *= sign;
                }

                states.emplace_back(join(q_lift, v_lift));
                errors.emplace_back(builder_.error_norm(constraints, q_lift));
            };

            if (es.trivial)
            {
                emit(q0, v0);
                return true;
            }

            duration = es.T;
            cost = es.cost;

            const std::size_t n = std::max<std::size_t>(n_samples, 2);
            states.reserve(n);
            errors.reserve(n);

            std::array<float, d> u{}, ud{}, udd{};
            for (std::size_t j = 0; j < n; ++j)
            {
                const float t = es.T * static_cast<float>(j) / static_cast<float>(n - 1);
                eval_cubic(es, t, u, ud, udd);
                emit(chart_point(es.chart, u), from_chart(es.chart, ud));
            }

            return true;
        }

        ConstraintSet<Ambient, rake> constraints;
        PhaseConstraintSet<ZRobot, rake> phase_constraints;
        ChartSettings settings;

    private:
        // Effectively unbounded chart radius: exact edges never clip.
        static constexpr float no_range = 1e9F;

        enum struct EdgeStatus : std::uint8_t
        {
            Invalid,  // chart, LQMT, lift, or validity failure
            Pruned,   // the caller's cost bound pruned the candidate endpoint
            Valid,
        };

        struct Edge
        {
            float T = 0.F;
            float cost = 0.F;
            std::array<float, d> qf{};  // lifted terminal position (execution order)
            std::array<float, d> vf{};  // terminal chart velocity, execution frame
            bool reach = false;         // chart target within range: exact shooting applied
            bool trivial = false;       // zero-length edge (target at the chart center, at rest)
        };

        // LQMT boundary scalars in chart coordinates (dimension-free).
        struct LQMTScalars
        {
            double v00 = 0., v0f = 0., vff = 0., dyv0 = 0., dyvf = 0., dy2 = 0.;

            // Zero-length edge (target at the chart center, at rest).
            auto trivial() const noexcept -> bool
            {
                return dy2 < 1e-10 and v00 + vff < 1e-10;
            }
        };

        // Front half of an edge evaluation, shared by make_edge and lift_edge: the chart
        // at q0, boundary data in the execution frame, cubic coefficients, the
        // shooting-corrected lifted endpoint, and the time-optimal solve.
        struct EdgeSolve
        {
            Chart<d> chart{};
            LQMTScalars s{};
            std::array<float, d> ud0{}, uf{}, udf{};
            std::array<float, d> a3{}, a2{};
            std::array<float, d> q_pre{};  // shooting-corrected lifted endpoint, valid iff `lifted`
            float T = 0.F;
            float cost = 0.F;
            bool reach = false;
            bool lifted = false;
            bool trivial = false;
        };

        inline static void
        split(const Configuration &z, std::array<float, d> &q, std::array<float, d> &v) noexcept
        {
            const auto arr = z.to_array();
            for (auto i = 0U; i < d; ++i)
            {
                q[i] = arr[i];
                v[i] = arr[d + i];
            }
        }

        inline static auto join(const std::array<float, d> &q, const std::array<float, d> &v) noexcept
            -> Configuration
        {
            typename ZRobot::ConfigurationArray arr{};
            for (auto i = 0U; i < d; ++i)
            {
                arr[i] = q[i];
                arr[d + i] = v[i];
            }

            return Configuration(arr);
        }

        // Scale the velocity half into the phase-feasible set: phase-constraint kernels
        // are homogeneous in qd, so scaling always reaches it, and the scale is exactly
        // 1 on feasible states.
        inline void phase_clamp(const std::array<float, d> &q, std::array<float, d> &v)
            const noexcept
        {
            if (phase_constraints.empty())
            {
                return;
            }

            const float s = phase_constraints.velocity_scale(join(q, v));
            if (s < 1.F)
            {
                for (auto i = 0U; i < d; ++i)
                {
                    v[i] *= s;
                }
            }
        }

        // Project a single ambient configuration onto the manifold (deterministic:
        // broadcast across lanes inside ConstraintSet::project).
        inline auto lift_point(const std::array<float, d> &q_in, std::array<float, d> &q_out)
            const noexcept -> bool
        {
            AmbientConfiguration cfg(q_in);
            if (not constraints.project(cfg))
            {
                return false;
            }

            const auto arr = cfg.to_array();
            for (auto i = 0U; i < d; ++i)
            {
                q_out[i] = arr[i];
            }

            return true;
        }

        // Shooting iteration toward the exact target: iterate u_f += B^T (q_t - psi(u_f)),
        // whose fixed point lifts exactly onto the target; `lifted` reports whether q_pre
        // holds a successful lift. Strict mode (make_edge) fails on any diverged lift.
        // Best-effort mode (debug replay -- the caller judges the endpoint miss) verifies
        // the final target with one extra lift and falls back to the last liftable target
        // on divergence, failing only if the initial target is unliftable.
        template <bool best_effort>
        auto shoot(
            const Chart<d> &chart,
            const std::array<float, d> &qt,
            std::array<float, d> &uf,
            std::array<float, d> &q_pre,
            bool &lifted) const noexcept -> bool
        {
            auto uf_good = uf;
            const std::size_t last = settings.shoot_iters;
            for (auto it = 0U; it < last + best_effort; ++it)
            {
                if (not lift_point(chart_point(chart, uf), q_pre))
                {
                    if constexpr (best_effort)
                    {
                        if (lifted)
                        {
                            uf = uf_good;
                            return true;
                        }
                    }

                    return false;
                }

                lifted = true;
                if constexpr (best_effort)
                {
                    uf_good = uf;
                    if (it == last)
                    {
                        break;
                    }
                }

                float delta2 = 0.F;
                for (auto c = 0U; c < chart.nc; ++c)
                {
                    float dot = 0.F;
                    for (auto j = 0U; j < d; ++j)
                    {
                        dot += chart.basis[c][j] * (qt[j] - q_pre[j]);
                    }

                    uf[c] += dot;
                    delta2 += dot * dot;
                }

                if (delta2 < 1e-10F)
                {
                    break;
                }
            }

            return true;
        }

        inline static auto lqmt_scalars(
            std::size_t nc,
            const std::array<float, d> &ud0,
            const std::array<float, d> &uf,
            const std::array<float, d> &udf) noexcept -> LQMTScalars
        {
            LQMTScalars s;
            for (auto c = 0U; c < nc; ++c)
            {
                s.v00 += static_cast<double>(ud0[c]) * ud0[c];
                s.v0f += static_cast<double>(ud0[c]) * udf[c];
                s.vff += static_cast<double>(udf[c]) * udf[c];
                s.dyv0 += static_cast<double>(uf[c]) * ud0[c];
                s.dyvf += static_cast<double>(uf[c]) * udf[c];
                s.dy2 += static_cast<double>(uf[c]) * uf[c];
            }

            return s;
        }

        // Shared front half of make_edge and lift_edge: chart at q0, boundary conversion
        // into the execution frame (sign negates velocities for time-reversed edges),
        // clipping the chart target to `range`, shooting toward the exact target when
        // within range (strict or best-effort, see shoot), and the time-optimal LQMT
        // solve with cubic coefficients. False when no chart exists at q0, the shooting
        // fails, or the solve does; on success `es.trivial` marks a zero-length edge
        // (T, cost, and coefficients unset).
        template <bool best_effort>
        auto solve_edge(
            const std::array<float, d> &q0,
            const std::array<float, d> &v0,
            const std::array<float, d> &qt,
            const std::array<float, d> &vt,
            float range,
            float sign,
            EdgeSolve &es) const noexcept -> bool
        {
            es.chart = builder_.make_chart(constraints, q0);
            if (not es.chart.valid)
            {
                return false;
            }

            const std::size_t nc = es.chart.nc;

            std::array<float, d> diff;
            for (auto j = 0U; j < d; ++j)
            {
                diff[j] = qt[j] - q0[j];
            }

            es.uf = to_chart(es.chart, diff);
            es.ud0 = to_chart(es.chart, v0);
            es.udf = to_chart(es.chart, vt);
            for (auto c = 0U; c < nc; ++c)
            {
                es.ud0[c] *= sign;
                es.udf[c] *= sign;
            }

            float norm2 = 0.F;
            for (auto c = 0U; c < nc; ++c)
            {
                norm2 += es.uf[c] * es.uf[c];
            }

            es.reach = norm2 <= range * range;
            if (not es.reach)
            {
                const float scale = range / std::sqrt(norm2);
                for (auto c = 0U; c < nc; ++c)
                {
                    es.uf[c] *= scale;
                }
            }

            es.q_pre = q0;
            if (es.reach and
                not shoot<best_effort>(es.chart, qt, es.uf, es.q_pre, es.lifted))
            {
                return false;
            }

            es.s = lqmt_scalars(nc, es.ud0, es.uf, es.udf);
            if (es.s.trivial())
            {
                es.trivial = true;
                return true;
            }

            const auto sol = flask::solve_scalars(
                es.s.v00,
                es.s.v0f,
                es.s.vff,
                es.s.dyv0,
                es.s.dyvf,
                es.s.dy2,
                static_cast<double>(ZRobot::rho));
            es.T = sol.time;
            es.cost = sol.cost;
            if (not (es.T > 0.F) or not std::isfinite(es.cost))
            {
                return false;
            }

            // Cubic coefficients u(t) = a3 t^3 + a2 t^2 + ud0 t (u(0) = 0) hitting
            // (uf, udf) at time T.
            for (auto c = 0U; c < nc; ++c)
            {
                const float d1 = es.uf[c] - es.T * es.ud0[c];
                const float d2 = es.udf[c] - es.ud0[c];
                es.a3[c] = (-2.F * d1 + es.T * d2) / (es.T * es.T * es.T);
                es.a2[c] = (3.F * d1 - es.T * d2) / (es.T * es.T);
            }

            return true;
        }

        // The chart cubic and its first two derivatives at time t.
        inline static void eval_cubic(
            const EdgeSolve &es,
            float t,
            std::array<float, d> &u,
            std::array<float, d> &ud,
            std::array<float, d> &udd) noexcept
        {
            for (auto c = 0U; c < es.chart.nc; ++c)
            {
                u[c] = ((es.a3[c] * t + es.a2[c]) * t + es.ud0[c]) * t;
                ud[c] = (3.F * es.a3[c] * t + 2.F * es.a2[c]) * t + es.ud0[c];
                udd[c] = 6.F * es.a3[c] * t + 2.F * es.a2[c];
            }
        }

        // The stored (and physically executed) arrival velocity of an edge. The lift's
        // pushforward is the tangent projector at the arrival point, so the raw
        // chart-frame terminal velocity carries a curvature-order normal error that
        // projection removes; residue below float noise is zeroed so rest states stay
        // exactly at rest. False when no chart exists at the arrival point.
        inline auto arrival_velocity(const Edge &edge, std::array<float, d> &vf) const noexcept
            -> bool
        {
            const auto chart_f = builder_.make_chart(constraints, edge.qf);
            if (not chart_f.valid)
            {
                return false;
            }

            vf = tangent_project(chart_f, edge.vf);
            for (auto i = 0U; i < d; ++i)
            {
                if (std::abs(vf[i]) < 1e-6F)
                {
                    vf[i] = 0.F;
                }
            }

            return true;
        }

        // Whether an edge's arrival state attains a target within the reached tolerances.
        // The velocity tolerance is speed-relative (tol * (1 + |vt|)): the residual
        // velocity error between nearby tangent spaces scales with speed, while rest
        // targets keep the strict absolute tolerance.
        inline auto attained(
            const std::array<float, d> &qf,
            const std::array<float, d> &vf,
            const std::array<float, d> &qt,
            const std::array<float, d> &vt) const noexcept -> bool
        {
            float dq2 = 0.F, dv2 = 0.F, vt2 = 0.F;
            for (auto i = 0U; i < d; ++i)
            {
                const float dq = qf[i] - qt[i];
                const float dv = vf[i] - vt[i];
                dq2 += dq * dq;
                dv2 += dv * dv;
                vt2 += vt[i] * vt[i];
            }

            const float vtol = settings.reached_vel_tol * (1.F + std::sqrt(vt2));
            return dq2 <= settings.reached_pos_tol * settings.reached_pos_tol and dv2 <= vtol * vtol;
        }

        // Build and fully validate the exact edge a -> b (execution order), requiring
        // attainment of b; reports the edge's LQMT cost on success.
        inline auto exact_edge(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward,
            float &cost) const noexcept -> bool
        {
            std::array<float, d> qa, va, qb, vb;
            split(a, qa, va);
            split(b, qb, vb);

            Edge edge;
            if (make_edge(qa, va, qb, vb, no_range, forward, e, AlwaysTrue(), edge) !=
                EdgeStatus::Valid)
            {
                return false;
            }

            std::array<float, d> vf;
            if (not (arrival_velocity(edge, vf) and attained(edge.qf, vf, qb, vb)))
            {
                return false;
            }

            cost = edge.cost;
            return true;
        }

        // Generate and validate one chart-LQMT edge from the lifted state (q0, v0) toward
        // the target state (qt, vt), in execution order; `forward` false runs the time
        // reversal (negated boundary velocities, terminal velocity negated back). The
        // chart target is clipped to `range`; within range the edge shoots to land on qt
        // exactly. Each lifted sample must stay within eps_chart of its pre-image and
        // within the continuity guard of its predecessor; its velocity is re-projected
        // into the tangent space at its own lifted point (the execution frame), and the
        // assembled (y, yd, ydd) blocks must pass the z-robot's fused limit/torque/
        // collision check. The accept predicate sees the candidate endpoint after the
        // LQMT solve but before batch validation.
        template <typename Accept>
        auto make_edge(
            const std::array<float, d> &q0,
            const std::array<float, d> &v0_in,
            const std::array<float, d> &qt,
            const std::array<float, d> &vt_in,
            float range,
            bool forward,
            const Environment &environment,
            Accept &&accept,
            Edge &out) const noexcept -> EdgeStatus
        {
            const float sign = (forward) ? 1.F : -1.F;

            EdgeSolve es;
            if (not solve_edge<false>(q0, v0_in, qt, vt_in, range, sign, es))
            {
                return EdgeStatus::Invalid;
            }

            out.reach = es.reach;
            if (es.trivial)
            {
                out.T = 0.F;
                out.cost = 0.F;
                out.qf = q0;
                out.vf = v0_in;
                out.trivial = true;
                return EdgeStatus::Valid;
            }

            const std::size_t nc = es.chart.nc;
            const float T = es.T;

            // Terminal chart velocity in the execution frame.
            std::array<float, d> u_T{}, ud_T{}, udd_T{};
            eval_cubic(es, T, u_T, ud_T, udd_T);
            auto v_T = from_chart(es.chart, ud_T);
            for (auto dim = 0U; dim < d; ++dim)
            {
                v_T[dim] *= sign;
            }

            // Apply the cost bound to the candidate endpoint before paying for batch
            // validation.
            if constexpr (not std::is_same_v<std::decay_t<Accept>, AlwaysTrue>)
            {
                if (not es.lifted and not lift_point(chart_point(es.chart, es.uf), es.q_pre))
                {
                    return EdgeStatus::Invalid;
                }

                if (not accept(join(es.q_pre, v_T)))
                {
                    return EdgeStatus::Pruned;
                }
            }

            // Time samples: chart path-length heuristic at the collision resolution
            const float L = static_cast<float>(
                std::sqrt(es.s.dy2) + 0.25 * T * (std::sqrt(es.s.v00) + std::sqrt(es.s.vff)));
            std::size_t n_total = static_cast<std::size_t>(
                std::max(std::ceil(L * static_cast<float>(resolution)), static_cast<float>(rake)));
            n_total = std::min(n_total, std::max(settings.max_edge_samples, rake));
            n_total = ((n_total + rake - 1) / rake) * rake;
            const std::size_t n_batches = n_total / rake;
            const std::size_t jac_stride = builder_.rows() * d;

            std::array<float, d> prev_q = q0;
            std::array<float, d> prev_u{};
            std::array<float, d> u_now{}, ud_now{}, udd_now{};
            std::array<float, rake> du_step;

            // Pfaffian rows have no position error for the per-sample projection to
            // correct, so the executed polyline drifts along their normals to first
            // order in the rows' rotation across the edge -- and cost shortcutting
            // exploits that drift in place of genuine steering. Bound the accumulated
            // normal drift by a fraction of the executed length (the slip tolerance).
            const bool bound_slip =
                builder_.n_pfaffian() != 0 and settings.max_slip_fraction < 1.F;
            float slip_sum = 0.F;
            float exec_len = 0.F;
            std::array<std::array<float, d>, rake> pre_scalar;
            alignas(FloatVectorAlignment) std::array<FloatT, d * rake> amb;
            alignas(FloatVectorAlignment) std::array<FloatT, 3 * d * rake> zarr;

            for (auto b = 0U; b < n_batches; ++b)
            {
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    const std::size_t j = b * rake + lane;
                    const float t = T * static_cast<float>(j + 1) / static_cast<float>(n_total);
                    eval_cubic(es, t, u_now, ud_now, udd_now);

                    float du2 = 0.F;
                    for (auto c = 0U; c < nc; ++c)
                    {
                        const float step = u_now[c] - prev_u[c];
                        du2 += step * step;
                        prev_u[c] = u_now[c];
                    }

                    du_step[lane] = std::sqrt(du2);

                    const auto q_amb = chart_point(es.chart, u_now);
                    const auto v_amb = from_chart(es.chart, ud_now);
                    const auto a_amb = from_chart(es.chart, udd_now);
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        amb[lane + dim * rake] = q_amb[dim];
                        pre_scalar[lane][dim] = q_amb[dim];
                        // Provisional chart-center-frame velocities; re-projected into each
                        // sample's own tangent space after the lift, below. The reversed sign
                        // is immaterial to the (even) velocity bounds and rigid-body RNEA.
                        zarr[lane + (d + dim) * rake] = sign * v_amb[dim];
                        zarr[lane + (2 * d + dim) * rake] = a_amb[dim];
                    }
                }

                AmbientBlock ablock(amb);
                if (not constraints.project_all(ablock))
                {
                    return EdgeStatus::Invalid;
                }

                // One constraint-kernel evaluation covers the whole batch of lifted
                // samples; extract every lane's Jacobian up front, since the NaN-fallback
                // make_chart below overwrites the kernel caches.
                if (builder_.n_active() != 0)
                {
                    constraints.evaluate_error_jacobian(ablock);
                    for (auto lane = 0U; lane < rake; ++lane)
                    {
                        constraints.extract_error_jacobian(
                            lane, err_.data(), jac_batch_.data() + lane * jac_stride);
                    }
                }

                // Chart validity (lift displacement) and lift continuity guards
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    std::array<float, d> q_lift;
                    std::array<float, d> chord;
                    float disp2 = 0.F, jump2 = 0.F;
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        const float pj = ablock[{dim, lane}];
                        const float dd = pj - pre_scalar[lane][dim];
                        disp2 += dd * dd;

                        const float jj = pj - prev_q[dim];
                        jump2 += jj * jj;
                        chord[dim] = jj;

                        zarr[lane + dim * rake] = pj;
                        q_lift[dim] = pj;
                        prev_q[dim] = pj;
                    }

                    if (disp2 > settings.eps_chart * settings.eps_chart)
                    {
                        return EdgeStatus::Invalid;
                    }

                    const float jump_tol = settings.cont_factor * std::max(du_step[lane], 1e-3F);
                    if (jump2 > jump_tol * jump_tol)
                    {
                        return EdgeStatus::Invalid;
                    }

                    if (bound_slip)
                    {
                        exec_len += std::sqrt(jump2);
                        slip_sum +=
                            builder_.slip(jac_batch_.data() + lane * jac_stride, chord.data());
                    }

                    // Execution-frame velocity: the lift's pushforward is the tangent
                    // projector at the lifted sample (as in arrival_velocity), and per-joint
                    // components can exceed their chart-center-frame values by a
                    // curvature-order amount, so the limit check must see the executed
                    // values. Accelerations keep the chart frame: the executed correction
                    // is the same order but needs the second fundamental form, which is not
                    // economically available here.
                    if (builder_.n_active() != 0)
                    {
                        const float *jac_lane = jac_batch_.data() + lane * jac_stride;
                        const auto chart_t = builder_.active_rows_finite(jac_lane) ?
                                                 builder_.chart_from_jacobian(q_lift, jac_lane) :
                                                 builder_.make_chart(constraints, q_lift);
                        if (not chart_t.valid)
                        {
                            return EdgeStatus::Invalid;
                        }

                        std::array<float, d> v_s;
                        for (auto dim = 0U; dim < d; ++dim)
                        {
                            v_s[dim] = zarr[lane + (d + dim) * rake];
                        }

                        const auto v_exec = tangent_project(chart_t, v_s);
                        for (auto dim = 0U; dim < d; ++dim)
                        {
                            zarr[lane + (d + dim) * rake] = v_exec[dim];
                        }
                    }
                }

                ZBlock zblock(zarr);

                // Phase constraints on the execution-frame samples: every lane's
                // velocities are already tangent-projected at their own lifted points
                // above, so the constraints judge the executed velocities. One batched
                // kernel evaluation serves all lanes.
                if (not phase_constraints.empty() and not phase_constraints.satisfied_block(zblock))
                {
                    return EdgeStatus::Invalid;
                }

                // Fused z-robot check: position/velocity bounds, RNEA torque limits, and
                // sphere collision in one pass over the (y, yd, ydd) block.
                if (not fkcc_block<ZRobot, rake>(environment, zblock))
                {
                    return EdgeStatus::Invalid;
                }
            }

            if (bound_slip and slip_sum > settings.max_slip_fraction * exec_len)
            {
                return EdgeStatus::Invalid;
            }

            out.T = T;
            out.cost = es.cost;
            out.qf = prev_q;  // last lifted sample, t = T
            out.vf = v_T;
            return EdgeStatus::Valid;
        }

        ChartBuilder<Ambient, rake> builder_;
        mutable std::vector<float> err_;
        mutable std::vector<float> jac_batch_;
        mutable std::vector<Configuration> chain_;
    };
}  // namespace vamp::planning::constraint
