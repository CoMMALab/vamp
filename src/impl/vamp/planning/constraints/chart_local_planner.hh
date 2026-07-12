#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/constraint_set.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/flask.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Local planner for manifold-constrained kinodynamic (flask) planning, pairing a flask
    // z-robot (states z = (q, qdot)) with its ambient position-space sibling
    // ZRobot::Ambient, whose constraint kernels define the manifold (MCFLASK).
    //
    // The manifold-restricted system is differentially flat with flat output equal to
    // tangent-space (chart) coordinates u in R^{d-k}: local paths are LQMT cubics solved
    // in the chart at the from-node -- an orthonormal basis B_0 of ker J(q_0) from an SVD
    // of the stacked active-row constraint Jacobian -- and lifted onto the manifold by
    // batch projection, sigma(t) = P_M(q_0 + B_0 u(t)). Tree nodes are lifted states
    // (positions exactly on the manifold, velocities in its tangent space). Samples are
    // raw steering targets and need no projection, but states synthesized outside the
    // chart machinery -- simplify's B-spline subdivision midpoints and interpolants are
    // flat z-space cubics -- leave the manifold, so `projecting` is true. Extensions have
    // no interior waypoints: an edge is reconstructible from its endpoint states, since
    // the flown chart target is a fixed point of the shooting iteration at the parent's
    // chart.
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
            ChartSettings chart_settings = {}) noexcept
          : constraints(std::move(constraint_set))
          , settings(chart_settings)
          , m_rows_(constraints.total_rows())
          , active_(std::make_unique<bool[]>(m_rows_))
          , err_(m_rows_)
          , jac_(m_rows_ * d)
        {
            constraints.active_rows(active_.get());
            for (auto i = 0U; i < m_rows_; ++i)
            {
                n_active_ += active_[i];
            }
        }

        // Validity of the exact local path a -> b in execution order. The chart lives at
        // a (a tree node in both directions); backward paths validate the time reversal.
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward = true) const noexcept -> bool
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
            return arrival_velocity(edge, vf) and attained(edge.qf, vf, qb, vb);
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

            std::array<float, d> qa, va, qb, vb;
            split(a, qa, va);
            split(b, qb, vb);

            Edge edge;
            std::array<float, d> vf;
            const bool valid =
                0 < max_states and
                make_edge(qa, va, qb, vb, no_range, true, e, AlwaysTrue(), edge) ==
                    EdgeStatus::Valid and
                arrival_velocity(edge, vf) and attained(edge.qf, vf, qb, vb) and
                edge.cost <= budget();

            return {(valid) ? SteerStatus::Reached : SteerStatus::Trapped, chain_};
        }

        // Steer from `from` toward `target` by at most `range` in chart coordinates (the
        // caller's state-space `distance` is ignored: reach is decided by the chart-space
        // displacement). Within range the edge shoots to hit the target exactly; beyond
        // it the chart target is clipped and the endpoint velocity still aims at the
        // target's. The frontier is the lifted terminal state with its velocity
        // tangent-projected at its own chart.
        template <typename Gate = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float,
            float range,
            bool forward,
            const Environment &e,
            Gate &&gate = Gate()) const noexcept -> Extension<ZRobot>
        {
            chain_.clear();

            std::array<float, d> q0, v0, qt, vt;
            split(from, q0, v0);
            split(target, qt, vt);

            Edge edge;
            const auto status =
                make_edge(q0, v0, qt, vt, range, forward, e, std::forward<Gate>(gate), edge);
            if (status == EdgeStatus::Gated)
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
        // into the tangent space at the projected position.
        inline auto project(Configuration &z) const noexcept -> bool
        {
            std::array<float, d> q, v;
            split(z, q, v);

            AmbientConfiguration qa(q);
            if (not constraints.project(qa))
            {
                return false;
            }

            const auto arr = qa.to_array();
            for (auto i = 0U; i < d; ++i)
            {
                q[i] = arr[i];
            }

            const auto chart = make_chart(q);
            if (not chart.valid)
            {
                return false;
            }

            z = join(q, tangent_project(chart, v));
            return true;
        }

        // Whether a state's position satisfies every constraint within tolerance.
        inline auto satisfied(const Configuration &z) const noexcept -> bool
        {
            std::array<float, d> q, v;
            split(z, q, v);
            return constraints.satisfied(AmbientConfiguration(q));
        }

        // Debug: the orthonormal tangent-space basis at a state's position, as nc rows of
        // d ambient components each. Empty when no chart exists there.
        inline auto debug_chart(const Configuration &z) const noexcept
            -> std::vector<std::array<float, d>>
        {
            std::array<float, d> q, v;
            split(z, q, v);
            const auto chart = make_chart(q);

            std::vector<std::array<float, d>> rows;
            if (chart.valid)
            {
                rows.assign(chart.basis.begin(), chart.basis.begin() + chart.nc);
            }

            return rows;
        }

        // Debug: densify the exact edge from -> target through the same chart/LQMT/lift
        // machinery, without any validation. Emits n_samples lifted states at uniform
        // times in [0, T] (velocities in the execution frame, tangent-projected at each
        // sample's own chart) and each sample's stacked hinged constraint error norm.
        // Samples run in chart time from `from`; backward edges are executed by
        // traversing them in reverse. A trivial (no-motion) edge emits the single lifted
        // from-state. False when no chart exists, a lift diverges, or the solve fails.
        inline auto debug_lift_edge(
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

            const auto chart = make_chart(q0);
            if (not chart.valid)
            {
                return false;
            }

            const float sign = (forward) ? 1.F : -1.F;
            const std::size_t nc = chart.nc;

            std::array<float, d> ud0{}, uf{}, udf{};
            for (auto c = 0U; c < nc; ++c)
            {
                float a = 0.F, b = 0.F, e = 0.F;
                for (auto j = 0U; j < d; ++j)
                {
                    a += chart.basis[c][j] * v0[j];
                    b += chart.basis[c][j] * (qt[j] - q0[j]);
                    e += chart.basis[c][j] * vt[j];
                }

                ud0[c] = sign * a;
                uf[c] = b;
                udf[c] = sign * e;
            }

            // Chart retargeting toward the exact target, as in make_edge
            std::array<float, d> q_pre;
            for (auto it = 0U; it < settings.shoot_iters; ++it)
            {
                auto q_amb = q0;
                for (auto c = 0U; c < nc; ++c)
                {
                    for (auto j = 0U; j < d; ++j)
                    {
                        q_amb[j] += chart.basis[c][j] * uf[c];
                    }
                }

                if (not lift_point(q_amb, q_pre))
                {
                    return false;
                }

                float delta2 = 0.F;
                for (auto c = 0U; c < nc; ++c)
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

            double v00 = 0., v0f = 0., vff = 0., dyv0 = 0., dyvf = 0., dy2 = 0.;
            for (auto c = 0U; c < nc; ++c)
            {
                v00 += static_cast<double>(ud0[c]) * ud0[c];
                v0f += static_cast<double>(ud0[c]) * udf[c];
                vff += static_cast<double>(udf[c]) * udf[c];
                dyv0 += static_cast<double>(uf[c]) * ud0[c];
                dyvf += static_cast<double>(uf[c]) * udf[c];
                dy2 += static_cast<double>(uf[c]) * uf[c];
            }

            const auto emit = [&](const std::array<float, d> &q_amb,
                                  const std::array<float, d> &v_amb) -> bool
            {
                std::array<float, d> q_lift;
                if (not lift_point(q_amb, q_lift))
                {
                    return false;
                }

                const auto chart_t = make_chart(q_lift);
                if (not chart_t.valid)
                {
                    return false;
                }

                auto v_lift = tangent_project(chart_t, v_amb);
                for (auto dim = 0U; dim < d; ++dim)
                {
                    v_lift[dim] *= sign;
                }

                constraints.error_jacobian(broadcast(q_lift), 0, err_.data(), jac_.data());
                float e2 = 0.F;
                for (auto r = 0U; r < m_rows_; ++r)
                {
                    if (std::isfinite(err_[r]))
                    {
                        e2 += err_[r] * err_[r];
                    }
                }

                states.emplace_back(join(q_lift, v_lift));
                errors.emplace_back(std::sqrt(e2));
                return true;
            };

            if (dy2 < 1e-10 and v00 + vff < 1e-10)
            {
                return emit(q0, v0);
            }

            const auto sol =
                flask::solve_scalars(v00, v0f, vff, dyv0, dyvf, dy2, static_cast<double>(ZRobot::rho));
            const float T = sol.time;
            if (not (T > 0.F) or not std::isfinite(sol.cost))
            {
                return false;
            }

            duration = T;
            cost = sol.cost;

            std::array<float, d> a3{}, a2{};
            for (auto c = 0U; c < nc; ++c)
            {
                const float d1 = uf[c] - T * ud0[c];
                const float d2 = udf[c] - ud0[c];
                a3[c] = (-2.F * d1 + T * d2) / (T * T * T);
                a2[c] = (3.F * d1 - T * d2) / (T * T);
            }

            const std::size_t n = std::max<std::size_t>(n_samples, 2);
            states.reserve(n);
            errors.reserve(n);
            for (std::size_t j = 0; j < n; ++j)
            {
                const float t = T * static_cast<float>(j) / static_cast<float>(n - 1);

                std::array<float, d> q_amb = q0, v_amb{};
                for (auto c = 0U; c < nc; ++c)
                {
                    const float u_c = ((a3[c] * t + a2[c]) * t + ud0[c]) * t;
                    const float ud_c = (3.F * a3[c] * t + 2.F * a2[c]) * t + ud0[c];
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        q_amb[dim] += chart.basis[c][dim] * u_c;
                        v_amb[dim] += chart.basis[c][dim] * ud_c;
                    }
                }

                if (not emit(q_amb, v_amb))
                {
                    return false;
                }
            }

            return true;
        }

        ConstraintSet<Ambient, rake> constraints;
        ChartSettings settings;

    private:
        // Effectively unbounded chart radius: exact edges never clip.
        static constexpr float no_range = 1e9F;

        enum struct EdgeStatus : std::uint8_t
        {
            Invalid,  // chart, LQMT, lift, or validity failure
            Gated,    // the caller's gate rejected the candidate frontier
            Valid,
        };

        struct Chart
        {
            std::array<float, d> q0{};
            // basis[c][j]: c-th orthonormal tangent vector, ambient component j
            std::array<std::array<float, d>, d> basis{};
            std::size_t nc = 0;
            bool valid = false;
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

        inline static auto broadcast(const std::array<float, d> &q) noexcept -> AmbientBlock
        {
            AmbientConfiguration cfg(q);
            AmbientBlock block;
            for (auto i = 0U; i < d; ++i)
            {
                block[i] = cfg.broadcast(i);
            }

            return block;
        }

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

        // Orthonormal tangent basis of ker J_active at q0 via SVD of the stacked
        // active-row constraint Jacobian. An empty or slab-only constraint set yields the
        // identity chart (nc = d), reducing every edge to an unconstrained ambient LQMT.
        auto make_chart(const std::array<float, d> &q0) const noexcept -> Chart
        {
            Chart chart;
            chart.q0 = q0;

            if (n_active_ == 0)
            {
                chart.nc = d;
                for (auto c = 0U; c < d; ++c)
                {
                    chart.basis[c][c] = 1.F;
                }

                chart.valid = true;
                return chart;
            }

            // The traced rotation-error Jacobian is NaN exactly on the manifold (acos at
            // argument 1). J is continuous, so evaluate at a deterministically perturbed
            // point when needed; the O(1e-3) offset is far below the chart error budget.
            bool finite = false;
            for (auto attempt = 0U; attempt < 4 and not finite; ++attempt)
            {
                auto q_eval = q0;
                if (attempt > 0)
                {
                    const float delta = 1e-3F * static_cast<float>(attempt);
                    for (auto j = 0U; j < d; ++j)
                    {
                        q_eval[j] += (j % 2 == 0) ? delta : -delta;
                    }
                }

                constraints.error_jacobian(broadcast(q_eval), 0, err_.data(), jac_.data());

                finite = true;
                for (auto r = 0U; r < m_rows_ and finite; ++r)
                {
                    if (not active_[r])
                    {
                        continue;
                    }

                    for (auto c = 0U; c < d; ++c)
                    {
                        if (not std::isfinite(jac_[r * d + c]))
                        {
                            finite = false;
                            break;
                        }
                    }
                }
            }

            if (not finite)
            {
                return chart;
            }

            Eigen::MatrixXf Ja(n_active_, d);
            std::size_t k = 0;
            for (auto r = 0U; r < m_rows_; ++r)
            {
                if (not active_[r])
                {
                    continue;
                }

                for (auto c = 0U; c < d; ++c)
                {
                    Ja(k, c) = jac_[r * d + c];
                }

                ++k;
            }

            Eigen::JacobiSVD<Eigen::MatrixXf> svd(Ja, Eigen::ComputeFullV);
            const auto &sv = svd.singularValues();

            std::size_t rank = 0;
            const float tol = settings.rank_tolerance * std::max(sv(0), 1e-6F);
            for (auto i = 0; i < sv.size(); ++i)
            {
                rank += sv(i) > tol;
            }

            chart.nc = d - rank;
            const auto &V = svd.matrixV();  // d x d
            for (auto c = 0U; c < chart.nc; ++c)
            {
                for (auto j = 0U; j < d; ++j)
                {
                    chart.basis[c][j] = V(j, rank + c);
                }
            }

            chart.valid = chart.nc > 0;
            return chart;
        }

        // Project v into the tangent space at the chart center: v <- B B^T v.
        inline static auto tangent_project(const Chart &chart, const std::array<float, d> &v) noexcept
            -> std::array<float, d>
        {
            std::array<float, d> out{};
            for (auto c = 0U; c < chart.nc; ++c)
            {
                float dot = 0.F;
                for (auto j = 0U; j < d; ++j)
                {
                    dot += chart.basis[c][j] * v[j];
                }

                for (auto j = 0U; j < d; ++j)
                {
                    out[j] += chart.basis[c][j] * dot;
                }
            }

            return out;
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

        // The stored (and physically executed) arrival velocity of an edge. The lift's
        // pushforward is the tangent projector at the arrival point, so the raw
        // chart-frame terminal velocity carries a curvature-order normal seam that
        // projection removes; residue below float noise is zeroed so rest states stay
        // exactly at rest. False when no chart exists at the arrival point.
        inline auto arrival_velocity(const Edge &edge, std::array<float, d> &vf) const noexcept
            -> bool
        {
            const auto chart_f = make_chart(edge.qf);
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
        // The velocity tolerance is speed-relative (tol * (1 + |vt|)): the residual chart
        // seam between nearby tangent spaces scales with speed, while rest targets keep
        // the strict absolute tolerance.
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

        // Generate and validate one chart-LQMT edge from the lifted state (q0, v0) toward
        // the target state (qt, vt), in execution order; `forward` false runs the time
        // reversal (negated boundary velocities, terminal velocity negated back). The
        // chart target is clipped to `range`; within range the edge shoots to land on qt
        // exactly. Each lifted sample must stay within eps_chart of its pre-image and
        // within the continuity guard of its predecessor, and the assembled (y, yd, ydd)
        // blocks must pass the z-robot's fused limit/torque/collision check. The gate sees
        // the candidate frontier after the LQMT solve but before batch validation.
        template <typename Gate>
        auto make_edge(
            const std::array<float, d> &q0,
            const std::array<float, d> &v0_in,
            const std::array<float, d> &qt,
            const std::array<float, d> &vt_in,
            float range,
            bool forward,
            const Environment &environment,
            Gate &&gate,
            Edge &out) const noexcept -> EdgeStatus
        {
            const auto chart = make_chart(q0);
            if (not chart.valid)
            {
                return EdgeStatus::Invalid;
            }

            const float sign = (forward) ? 1.F : -1.F;
            const std::size_t nc = chart.nc;

            // Boundary data in chart coordinates, execution frame (time-reversed when
            // backward: the cubic always starts at the chart center)
            std::array<float, d> ud0{}, uf{}, udf{};
            for (auto c = 0U; c < nc; ++c)
            {
                float a = 0.F, b = 0.F, e = 0.F;
                for (auto j = 0U; j < d; ++j)
                {
                    a += chart.basis[c][j] * v0_in[j];
                    b += chart.basis[c][j] * (qt[j] - q0[j]);
                    e += chart.basis[c][j] * vt_in[j];
                }

                ud0[c] = sign * a;
                uf[c] = b;
                udf[c] = sign * e;
            }

            float norm2 = 0.F;
            for (auto c = 0U; c < nc; ++c)
            {
                norm2 += uf[c] * uf[c];
            }

            out.reach = norm2 <= range * range;
            if (not out.reach)
            {
                const float scale = range / std::sqrt(norm2);
                for (auto c = 0U; c < nc; ++c)
                {
                    uf[c] *= scale;
                }
            }

            std::array<float, d> q_pre = q0;
            bool have_pre = false;
            if (out.reach)
            {
                // Chart retargeting: iterate u_f += B^T (q_t - psi(u_f))
                for (auto it = 0U; it < settings.shoot_iters; ++it)
                {
                    auto q_amb = q0;
                    for (auto c = 0U; c < nc; ++c)
                    {
                        for (auto j = 0U; j < d; ++j)
                        {
                            q_amb[j] += chart.basis[c][j] * uf[c];
                        }
                    }

                    if (not lift_point(q_amb, q_pre))
                    {
                        return EdgeStatus::Invalid;
                    }

                    have_pre = true;
                    float delta2 = 0.F;
                    for (auto c = 0U; c < nc; ++c)
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
            }

            // LQMT in chart coordinates (dimension-free scalars)
            double v00 = 0., v0f = 0., vff = 0., dyv0 = 0., dyvf = 0., dy2 = 0.;
            for (auto c = 0U; c < nc; ++c)
            {
                v00 += static_cast<double>(ud0[c]) * ud0[c];
                v0f += static_cast<double>(ud0[c]) * udf[c];
                vff += static_cast<double>(udf[c]) * udf[c];
                dyv0 += static_cast<double>(uf[c]) * ud0[c];
                dyvf += static_cast<double>(uf[c]) * udf[c];
                dy2 += static_cast<double>(uf[c]) * uf[c];
            }

            if (dy2 < 1e-10 and v00 + vff < 1e-10)
            {
                out.T = 0.F;
                out.cost = 0.F;
                out.qf = q0;
                out.vf = v0_in;
                out.trivial = true;
                return EdgeStatus::Valid;
            }

            const auto sol =
                flask::solve_scalars(v00, v0f, vff, dyv0, dyvf, dy2, static_cast<double>(ZRobot::rho));
            const float T = sol.time;
            if (not (T > 0.F) or not std::isfinite(sol.cost))
            {
                return EdgeStatus::Invalid;
            }

            // Cubic coefficients u(t) = a3 t^3 + a2 t^2 + ud0 t (u(0) = 0)
            std::array<float, d> a3{}, a2{};
            for (auto c = 0U; c < nc; ++c)
            {
                const float d1 = uf[c] - T * ud0[c];
                const float d2 = udf[c] - ud0[c];
                a3[c] = (-2.F * d1 + T * d2) / (T * T * T);
                a2[c] = (3.F * d1 - T * d2) / (T * T);
            }

            const auto terminal_v = [&](std::array<float, d> &v_out)
            {
                for (auto dim = 0U; dim < d; ++dim)
                {
                    float v_amb = 0.F;
                    for (auto c = 0U; c < nc; ++c)
                    {
                        const float ud_T = (3.F * a3[c] * T + 2.F * a2[c]) * T + ud0[c];
                        v_amb += chart.basis[c][dim] * ud_T;
                    }

                    v_out[dim] = sign * v_amb;
                }
            };

            // Cost-gate on the candidate frontier before paying for batch validation.
            if constexpr (not std::is_same_v<std::decay_t<Gate>, AlwaysTrue>)
            {
                if (not have_pre)
                {
                    auto q_amb = q0;
                    for (auto c = 0U; c < nc; ++c)
                    {
                        for (auto j = 0U; j < d; ++j)
                        {
                            q_amb[j] += chart.basis[c][j] * uf[c];
                        }
                    }

                    if (not lift_point(q_amb, q_pre))
                    {
                        return EdgeStatus::Invalid;
                    }
                }

                std::array<float, d> v_pre;
                terminal_v(v_pre);
                if (not gate(join(q_pre, v_pre)))
                {
                    return EdgeStatus::Gated;
                }
            }

            // Time samples: chart path-length heuristic at the collision resolution
            const float L =
                static_cast<float>(std::sqrt(dy2) + 0.25 * T * (std::sqrt(v00) + std::sqrt(vff)));
            std::size_t n_total = static_cast<std::size_t>(
                std::max(std::ceil(L * static_cast<float>(resolution)), static_cast<float>(rake)));
            n_total = std::min(n_total, std::max(settings.max_edge_samples, rake));
            n_total = ((n_total + rake - 1) / rake) * rake;
            const std::size_t n_batches = n_total / rake;

            std::array<float, d> prev_q = q0;
            std::array<float, d> prev_u{};
            std::array<float, d> u_now{}, ud_now{}, udd_now{};
            std::array<float, rake> du_step;
            std::array<std::array<float, d>, rake> pre_scalar;
            alignas(FloatVectorAlignment) std::array<FloatT, d * rake> amb;
            alignas(FloatVectorAlignment) std::array<FloatT, 3 * d * rake> zarr;

            for (auto b = 0U; b < n_batches; ++b)
            {
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    const std::size_t j = b * rake + lane;
                    const float t = T * static_cast<float>(j + 1) / static_cast<float>(n_total);

                    float du2 = 0.F;
                    for (auto c = 0U; c < nc; ++c)
                    {
                        const float u_c = ((a3[c] * t + a2[c]) * t + ud0[c]) * t;
                        const float ud_c = (3.F * a3[c] * t + 2.F * a2[c]) * t + ud0[c];
                        const float step = u_c - prev_u[c];
                        du2 += step * step;
                        prev_u[c] = u_c;
                        u_now[c] = u_c;
                        ud_now[c] = ud_c;
                        udd_now[c] = 6.F * a3[c] * t + 2.F * a2[c];
                    }

                    du_step[lane] = std::sqrt(du2);

                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        float q_amb = q0[dim];
                        float v_amb = 0.F;
                        float a_amb = 0.F;
                        for (auto c = 0U; c < nc; ++c)
                        {
                            q_amb += chart.basis[c][dim] * u_now[c];
                            v_amb += chart.basis[c][dim] * ud_now[c];
                            a_amb += chart.basis[c][dim] * udd_now[c];
                        }

                        amb[lane + dim * rake] = q_amb;
                        pre_scalar[lane][dim] = q_amb;
                        // Velocities in the chart-center tangent frame; the reversed sign
                        // is immaterial to the (even) velocity bounds and rigid-body RNEA.
                        zarr[lane + (d + dim) * rake] = sign * v_amb;
                        zarr[lane + (2 * d + dim) * rake] = a_amb;
                    }
                }

                AmbientBlock ablock(amb);
                if (not constraints.project_all(ablock))
                {
                    return EdgeStatus::Invalid;
                }

                // Chart validity (lift displacement) and lift continuity guards
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    float disp2 = 0.F, jump2 = 0.F;
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        const float pj = ablock[{dim, lane}];
                        const float dd = pj - pre_scalar[lane][dim];
                        disp2 += dd * dd;

                        const float jj = pj - prev_q[dim];
                        jump2 += jj * jj;

                        zarr[lane + dim * rake] = pj;
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
                }

                // Fused z-robot check: position/velocity bounds, RNEA torque limits, and
                // sphere collision in one pass over the (y, yd, ydd) block.
                ZBlock zblock(zarr);
                const bool valid = (environment.attachments) ?
                                       ZRobot::template fkcc_attach<rake>(environment, zblock) :
                                       ZRobot::template fkcc<rake>(environment, zblock);
                if (not valid)
                {
                    return EdgeStatus::Invalid;
                }
            }

            out.T = T;
            out.cost = sol.cost;
            out.qf = prev_q;  // last lifted sample, t = T
            terminal_v(out.vf);
            return EdgeStatus::Valid;
        }

        std::size_t m_rows_;
        std::size_t n_active_ = 0;
        mutable std::unique_ptr<bool[]> active_;
        mutable std::vector<float> err_;
        mutable std::vector<float> jac_;
        mutable std::vector<Configuration> chain_;
    };
}  // namespace vamp::planning::constraint
