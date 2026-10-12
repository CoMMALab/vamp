#pragma once

#include <algorithm>
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
#include <vamp/planning/plan.hh>
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
        using Row = FloatVector<rake, 1>;

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
          , exact_(settings.exact_chart and constraints.exact_form())
          , builder_(constraints, settings.rank_tolerance, exact_)
          , err_(builder_.rows())
          , jac_batch_(rake * builder_.rows() * d)
          , eq_F_(builder_.equality_rows())
          , eq_gamma_(builder_.equality_rows())
          , eq_J_(rake * builder_.equality_rows() * d)
          , rF_(builder_.equality_rows())
          , rG_(builder_.equality_rows())
          , rJ_(builder_.equality_rows() * d)
          , rM_(builder_.equality_rows() * builder_.equality_rows())
          , rA_(builder_.equality_rows() * builder_.equality_rows())
          , rB_(builder_.equality_rows())
          , rX_(builder_.equality_rows())
          , rN_(builder_.equality_rows() * d)
          , rD_(d + builder_.equality_rows())
        {
            const auto k = static_cast<Eigen::Index>(builder_.equality_rows());
            M_.resize(k, k);
            lu_ = Eigen::PartialPivLU<Eigen::MatrixXf>(k);
            rhs_.resize(k);
            sol_.resize(k);
        }

        // Whether edges use exact charts (ChartSettings::exact_chart on a constraint set with
        // an equality form).
        auto exact() const noexcept -> bool
        {
            return exact_;
        }

        // Validity of the exact local path a -> b in execution order. The chart lives at
        // a (a tree node in both directions); backward paths validate the time reversal.
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward = true) const noexcept -> bool
        {
            ++stats_.motions_attempted;
            float cost;
            const bool valid = exact_edge(a, b, e, forward, cost);
            stats_.motions_valid += valid;
            return valid;
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

            if (max_states == 0)
            {
                return {SteerStatus::Trapped, chain_};
            }

            ++stats_.motions_attempted;
            float cost;
            const bool edge_valid = exact_edge(a, b, e, true, cost);
            stats_.motions_valid += edge_valid;
            const bool valid = edge_valid and cost <= budget();
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

            ++stats_.motions_attempted;
            stats_.motions_valid += (status == EdgeStatus::Valid);

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

            // Exact charts project onto the equality form's zero set jointly across all
            // constraints (the per-constraint projector composes stacked constraints by summing
            // independent steps, which can fail to converge), as tight as their edges land.
            if (exact_ ? not exact_project(q) : not lift_point(q, q))
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

        // Exact projection (exact charts only) of up to rake positions at once: lane i starts at q[i]
        // for i < n (lanes past n replicate lane n - 1). The damped Gauss-Newton steps of the scalar
        // projection run on every lane as SIMD rows -- one equality-kernel evaluation and one
        // batched Cholesky solve of J J^T per iteration for the whole batch; a lane stops once it
        // converges or diverges. As project(), a converged lane counts only where its equality rows
        // have full rank. ok[i] reports success; returns the number of successes.
        auto exact_project_batch(std::array<std::array<float, d>, rake> &q, std::size_t n, std::array<bool, rake> &ok)
            const noexcept -> std::size_t
        {
            ok.fill(false);
            const std::size_t e = builder_.equality_rows();
            if (not exact_ or e == 0 or n == 0)
            {
                return 0;
            }

            n = std::min(n, rake);
            for (auto lane = n; lane < rake; ++lane)
            {
                q[lane] = q[n - 1];
            }

            const std::size_t cap = std::max(settings.exact_iterations, constraints.settings().max_iterations);
            std::array<bool, rake> active{};
            std::fill_n(active.begin(), n, true);
            std::size_t remaining = n, converged = 0;
            alignas(FloatVectorAlignment) std::array<FloatT, d * rake> qarr;
            auto &L = rM_;  // e x e, lower Cholesky factor of J J^T + damping I
            auto &y = rX_;
            std::array<Row, d> step;

            // Factor J J^T + damping I in place for every lane; the extreme pivots (the diagonal of L,
            // the R diagonal of an unpivoted QR of J^T) return per lane.
            const auto factor = [&](float damping) -> std::pair<Row, Row>
            {
                for (auto r = 0U; r < e; ++r)
                {
                    for (auto c = 0U; c <= r; ++c)
                    {
                        Row sum = Row::fill((r == c) ? damping : 0.F);
                        for (auto j = 0U; j < d; ++j)
                        {
                            sum = sum + rJ_[r * d + j] * rJ_[c * d + j];
                        }

                        L[r * e + c] = sum;
                    }
                }

                Row lo = Row::fill(std::numeric_limits<float>::infinity());
                Row hi = Row::fill(0.F);
                for (auto p = 0U; p < e; ++p)
                {
                    for (auto c = 0U; c < p; ++c)
                    {
                        L[p * e + p] = L[p * e + p] - L[p * e + c] * L[p * e + c];
                    }

                    L[p * e + p] = L[p * e + p].max(0.F).sqrt();
                    lo = lo.min(L[p * e + p]);
                    hi = hi.max(L[p * e + p]);
                    const Row inv = Row::fill(1.F) / L[p * e + p];
                    for (auto r = p + 1; r < e; ++r)
                    {
                        for (auto c = 0U; c < p; ++c)
                        {
                            L[r * e + p] = L[r * e + p] - L[r * e + c] * L[p * e + c];
                        }

                        L[r * e + p] = L[r * e + p] * inv;
                    }
                }

                return {lo, hi};
            };

            for (std::size_t it = 0; remaining > 0; ++it)
            {
                constraints.evaluate_equality_position(load_block(q, qarr));
                constraints.extract_equality_rows(rF_.data(), rJ_.data(), rG_.data());

                Row err2 = Row::fill(0.F);
                for (auto r = 0U; r < e; ++r)
                {
                    err2 = err2 + rF_[r] * rF_[r];
                }

                const auto err = err2.to_array();
                bool landed = false;
                for (auto lane = 0U; lane < n; ++lane)
                {
                    landed = landed or (active[lane] and err[lane] <= settings.exact_tolerance);
                }

                // As project(): a converged lane counts only where its equality rows have full rank
                // (the chart's rank test, on the undamped factor).
                std::array<float, rake> pivot_lo{}, pivot_hi{};
                if (landed)
                {
                    const auto [lo, hi] = factor(0.F);
                    pivot_lo = lo.to_array();
                    pivot_hi = hi.to_array();
                }

                for (auto lane = 0U; lane < n; ++lane)
                {
                    if (not active[lane])
                    {
                        continue;
                    }

                    if (err[lane] <= settings.exact_tolerance)
                    {
                        ok[lane] = pivot_lo[lane] > settings.rank_tolerance * std::max(pivot_hi[lane], 1e-6F);
                        converged += ok[lane];
                    }
                    else if (it < cap and std::isfinite(err[lane]))
                    {
                        continue;
                    }

                    active[lane] = false;
                    --remaining;
                }

                if (remaining == 0)
                {
                    break;
                }

                factor(1e-6F);

                for (auto r = 0U; r < e; ++r)
                {
                    y[r] = rF_[r];
                    for (auto c = 0U; c < r; ++c)
                    {
                        y[r] = y[r] - L[r * e + c] * y[c];
                    }

                    y[r] = y[r] / L[r * e + r];
                }

                for (auto r = e; r-- > 0;)
                {
                    for (auto c = r + 1; c < e; ++c)
                    {
                        y[r] = y[r] - L[c * e + r] * y[c];
                    }

                    y[r] = y[r] / L[r * e + r];
                }

                Row norm2 = Row::fill(0.F);
                for (auto j = 0U; j < d; ++j)
                {
                    Row s = Row::fill(0.F);
                    for (auto r = 0U; r < e; ++r)
                    {
                        s = s + rJ_[r * d + j] * y[r];
                    }

                    step[j] = s;
                    norm2 = norm2 + s * s;
                }

                // Trust region: clip long steps so far seeds descend instead of overshooting.
                const auto n2 = norm2.to_array();
                for (auto j = 0U; j < d; ++j)
                {
                    const auto sj = step[j].to_array();
                    for (auto lane = 0U; lane < n; ++lane)
                    {
                        if (active[lane])
                        {
                            const float norm = std::sqrt(n2[lane]);
                            const float scale = (norm > max_projection_step) ? max_projection_step / norm : 1.F;
                            q[lane][j] = std::clamp(
                                q[lane][j] - scale * sj[lane], Ambient::lower_bound[j], Ambient::upper_bound[j]);
                        }
                    }
                }
            }

            return converged;
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
            float &cost,
            std::vector<std::array<float, d>> *accelerations = nullptr) const noexcept -> bool
        {
            states.clear();
            errors.clear();
            if (accelerations != nullptr)
            {
                accelerations->clear();
            }

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
                if (accelerations != nullptr)
                {
                    accelerations->emplace_back();
                }

                return true;
            }

            duration = es.T;
            cost = es.cost;

            const std::size_t n = std::max<std::size_t>(n_samples, 2);
            states.reserve(n);
            errors.reserve(n);

            if (exact_)
            {
                // Exact charts: the executed states and accelerations themselves, in chunks
                // of SIMD width.
                Lanes u, ud, udd, q, qd, qdd;
                std::array<float, rake> w2;
                for (std::size_t j0 = 0; j0 < n; j0 += rake)
                {
                    const std::size_t m = std::min<std::size_t>(rake, n - j0);
                    for (std::size_t l = 0; l < m; ++l)
                    {
                        const float t = es.T * static_cast<float>(j0 + l) / static_cast<float>(n - 1);
                        eval_cubic(es, t, u[l], ud[l], udd[l]);
                    }

                    if (not exact_samples(es.chart, m, u, ud, udd, q, qd, qdd, w2))
                    {
                        return false;
                    }

                    for (std::size_t l = 0; l < m; ++l)
                    {
                        for (auto dim = 0U; dim < d; ++dim)
                        {
                            qd[l][dim] *= sign;
                        }

                        states.emplace_back(join(q[l], qd[l]));
                        errors.emplace_back(builder_.error_norm(constraints, q[l]));
                        if (accelerations != nullptr)
                        {
                            accelerations->emplace_back(qdd[l]);
                        }
                    }
                }

                return true;
            }

            std::array<float, d> u{}, ud{}, udd{};
            for (std::size_t j = 0; j < n; ++j)
            {
                const float t = es.T * static_cast<float>(j) / static_cast<float>(n - 1);
                eval_cubic(es, t, u, ud, udd);
                emit(chart_point(es.chart, u), from_chart(es.chart, ud));
                if (accelerations != nullptr)
                {
                    // What make_edge validates: the chart-frame acceleration B udd.
                    accelerations->emplace_back(from_chart(es.chart, udd));
                }
            }

            return true;
        }

        ConstraintSet<Ambient, rake> constraints;
        PhaseConstraintSet<ZRobot, rake> phase_constraints;
        ChartSettings settings;

        // Work counters accumulated across validate/connect_within/steer (motions_attempted/
        // motions_valid/rake_calls). Mutable scratch like chain_: planners hold the local
        // planner const but reset this per query.
        inline auto reset_stats() const noexcept -> void
        {
            stats_ = PlanningStats();
        }

        [[nodiscard]] inline auto stats() const noexcept -> const PlanningStats &
        {
            return stats_;
        }

    private:
        // Effectively unbounded chart radius: exact edges never clip.
        static constexpr float no_range = 1e9F;

        // Joint-space trust radius of one exact-projection Gauss-Newton step.
        static constexpr float max_projection_step = 0.3F;

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

        using Lanes = std::array<std::array<float, d>, rake>;

        inline static auto load_block(const Lanes &x, std::array<FloatT, d * rake> &arr) noexcept
            -> AmbientBlock
        {
            for (auto lane = 0U; lane < rake; ++lane)
            {
                for (auto dim = 0U; dim < d; ++dim)
                {
                    arr[lane + dim * rake] = x[lane][dim];
                }
            }

            return AmbientBlock(arr);
        }

        // LU-factor the normal-space Newton system M = J N of a (rows x d, row-major)
        // equality Jacobian; false if M is singular (the chart's normal directions no longer
        // cross the manifold transversally).
        inline auto factor_normal_system(const Chart<d> &chart, const float *J) const noexcept -> bool
        {
            const std::size_t k = chart.nk;
            for (auto r = 0U; r < k; ++r)
            {
                for (auto i = 0U; i < k; ++i)
                {
                    float dot = 0.F;
                    for (auto j = 0U; j < d; ++j)
                    {
                        dot += J[r * d + j] * chart.normal[i][j];
                    }

                    M_(r, i) = dot;
                }
            }

            lu_.compute(M_);
            const auto diag = lu_.matrixLU().diagonal().cwiseAbs();
            const float hi = diag.maxCoeff();
            return std::isfinite(hi) and diag.minCoeff() > 1e-6F * std::max(hi, 1e-6F);
        }

        // Solve M x = rhs_ with the last factorization and accumulate N x into `out`.
        inline void add_normal_solution(const Chart<d> &chart, std::array<float, d> &out) const noexcept
        {
            sol_ = lu_.solve(rhs_);
            for (auto i = 0U; i < chart.nk; ++i)
            {
                for (auto j = 0U; j < d; ++j)
                {
                    out[j] += chart.normal[i][j] * sol_[i];
                }
            }
        }

        // Exact-chart samples (ChartSettings::exact_chart) for the first n lanes: chart
        // coordinates u lift to sigma = q0 + B u + N w with F(sigma) = 0 by Newton steps along
        // the chart's normal space (M = J N), and differentiating F(sigma(t)) = 0 gives the
        // executed derivatives exactly:
        //   M wd  = -J B ud,                    qd  = B ud  + N wd,
        //   M wdd = -(J B udd + gamma(q, qd)),  qdd = B udd + N wdd,
        // where gamma = d²F/dq²[qd, qd] is the curvature (centripetal) term. w2 reports the
        // lift displacement |N w|^2. Lanes past n replicate lane n - 1. With `derivatives`
        // false only q is computed. False if a lane fails to converge or M is singular.
        // Exact-chart samples: positions sigma(u) = q0 + B u + N w with F(sigma) = 0 by Newton in w,
        // and (with `derivatives`) the executed velocity and acceleration by implicit
        // differentiation. ChartSettings::batched_exact selects the SIMD-batched path.
        auto exact_samples(
            const Chart<d> &chart,
            std::size_t n,
            const Lanes &u,
            const Lanes &ud,
            const Lanes &udd,
            Lanes &q,
            Lanes &qd,
            Lanes &qdd,
            std::array<float, rake> &w2,
            bool derivatives = true) const noexcept -> bool
        {
            return settings.batched_exact ? exact_samples_batched(chart, n, u, ud, udd, q, qd, qdd, w2, derivatives) :
                                            exact_samples_scalar(chart, n, u, ud, udd, q, qd, qdd, w2, derivatives);
        }

        // The batched path: every Newton iteration and both implicit differentiations run on all rake
        // lanes at once as SIMD rows -- one equality-kernel evaluation, one normal-space system
        // M = J N, one LU and one solve per step for the whole batch. Each lane's M is solved by LU
        // without pivoting after preconditioning with P = M^-1 of lane 0 at the first iteration
        // (one scalar inverse per batch): a batch's samples lie close together on the edge, so P M
        // stays near the identity, where elimination without pivoting is stable; a lane whose pivot
        // collapses rejects the batch, as a singular M does in the scalar path.
        auto exact_samples_batched(
            const Chart<d> &chart,
            std::size_t n,
            const Lanes &u,
            const Lanes &ud,
            const Lanes &udd,
            Lanes &q,
            Lanes &qd,
            Lanes &qdd,
            std::array<float, rake> &w2,
            bool derivatives) const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            if (chart.nk != e or e == 0 or n == 0)
            {
                return false;
            }

            const auto rows_of = [&](const auto &lane_value, std::array<Row, d> &out)
            {
                alignas(FloatVectorAlignment) std::array<FloatT, d * rake> arr;
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    const auto v = lane_value(std::min<std::size_t>(lane, n - 1));
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        arr[lane + dim * rake] = v[dim];
                    }
                }

                for (auto dim = 0U; dim < d; ++dim)
                {
                    out[dim] = Row(arr.data() + dim * rake);
                }
            };
            const auto store = [&](const std::array<Row, d> &rows, Lanes &out)
            {
                for (auto dim = 0U; dim < d; ++dim)
                {
                    const auto a = rows[dim].to_array();
                    for (auto lane = 0U; lane < rake; ++lane)
                    {
                        out[lane][dim] = a[lane];
                    }
                }
            };

            std::array<Row, d> pre, qr;
            rows_of([&](std::size_t lane) { return chart_point(chart, u[lane]); }, pre);
            qr = pre;

            AmbientBlock qb, vb;
            const auto evaluate = [&](const std::array<Row, d> &qq, const std::array<Row, d> *vv)
            {
                for (auto dim = 0U; dim < d; ++dim)
                {
                    qb[dim] = qq[dim];
                }

                if (vv)
                {
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        vb[dim] = (*vv)[dim];
                    }

                    constraints.evaluate_equality(qb, vb);
                }
                else
                {
                    constraints.evaluate_equality_position(qb);  // Newton: no drift needed
                }

                constraints.extract_equality_rows(rF_.data(), rJ_.data(), rG_.data());
            };

            bool have_p = false;
            std::array<Row, d> step;
            for (std::size_t it = 0;; ++it)
            {
                evaluate(qr, nullptr);
                ++stats_.newton_iterations;
                Row err2 = Row::fill(0.F);
                for (auto r = 0U; r < e; ++r)
                {
                    err2 = err2 + rF_[r] * rF_[r];
                }

                const Row done = err2 <= settings.exact_tolerance;
                if (done.all())
                {
                    break;
                }

                if (it >= settings.exact_iterations)
                {
                    return false;
                }

                for (auto r = 0U; r < e; ++r)
                {
                    rB_[r] = Row::fill(0.F) - rF_[r];
                }

                if (not normal_step(chart, have_p, true, step))
                {
                    return false;
                }

                for (auto dim = 0U; dim < d; ++dim)
                {
                    qr[dim] = (qr[dim] + step[dim]).blend(qr[dim], done);
                }
            }

            Row m2 = Row::fill(0.F);
            for (auto dim = 0U; dim < d; ++dim)
            {
                const Row dw = qr[dim] - pre[dim];
                m2 = m2 + dw * dw;
            }

            const auto m2a = m2.to_array();
            for (auto lane = 0U; lane < rake; ++lane)
            {
                w2[lane] = m2a[lane];
            }

            store(qr, q);
            if (not derivatives)
            {
                return true;
            }

            // J is current at sigma; one factorization serves both differentiations.
            const auto tangent = [&](const Lanes &x, std::array<Row, d> &out)
            { rows_of([&](std::size_t lane) { return from_chart(chart, x[lane]); }, out); };
            const auto correct = [&](std::array<Row, d> &x, const Row *offset, bool refactor)
            {
                // x <- x + N w with M w = -(J x + offset)
                for (auto r = 0U; r < e; ++r)
                {
                    Row dot = (offset) ? offset[r] : Row::fill(0.F);
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        dot = dot + rJ_[r * d + dim] * x[dim];
                    }

                    rB_[r] = Row::fill(0.F) - dot;
                }

                if (not normal_step(chart, have_p, refactor, step))
                {
                    return false;
                }

                for (auto dim = 0U; dim < d; ++dim)
                {
                    x[dim] = x[dim] + step[dim];
                }

                return true;
            };

            std::array<Row, d> qdr, qddr;
            tangent(ud, qdr);
            if (not correct(qdr, nullptr, true))
            {
                return false;
            }

            evaluate(qr, &qdr);  // gamma = d2F[qd, qd] at sigma (J unchanged)
            tangent(udd, qddr);
            if (not correct(qddr, rG_.data(), false))
            {
                return false;
            }

            store(qdr, qd);
            store(qddr, qdd);
            return true;
        }

        // P = M^-1 of lane 0 (M = J N from the current rows of J): the batch's preconditioner. False
        // if lane 0's M is singular.
        auto compute_preconditioner(const Chart<d> &chart) const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            for (auto r = 0U; r < e; ++r)
            {
                for (auto c = 0U; c < e; ++c)
                {
                    float m = 0.F;
                    for (auto j = 0U; j < d; ++j)
                    {
                        m += rJ_[r * d + j][{0, 0}] * chart.normal[c][j];
                    }

                    M_(r, c) = m;
                }
            }

            lu_.compute(M_);
            const auto diag = lu_.matrixLU().diagonal().cwiseAbs();
            const float hi = diag.maxCoeff();
            if (not (std::isfinite(hi) and diag.minCoeff() > 1e-6F * std::max(hi, 1e-6F)))
            {
                return false;
            }

            P_ = lu_.inverse();
            return true;
        }

        // Joint-space normal step delta = N M^-1 rB_ for every lane (M = J N from the current rows of
        // J). A single constraint with a generated step kernel runs it on N' = N P (P = lane 0's
        // M^-1, built at the batch's first step, so J N' stays near the identity and the kernel's
        // pivot-free LU is stable); otherwise the batched generic LU, refactored when J changed.
        auto normal_step(const Chart<d> &chart, bool &have_p, bool refactor, std::array<Row, d> &delta)
            const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            if (settings.step_kernel and constraints.has_normal_kernel())
            {
                if (not have_p)
                {
                    if (not compute_preconditioner(chart))
                    {
                        return false;
                    }

                    for (auto c = 0U; c < e; ++c)
                    {
                        for (auto j = 0U; j < d; ++j)
                        {
                            float v = 0.F;
                            for (auto m = 0U; m < e; ++m)
                            {
                                v += chart.normal[m][j] * P_(m, c);
                            }

                            rN_[c * d + j] = Row::fill(v);
                        }
                    }

                    have_p = true;
                }

                if (not constraints.solve_normal_rows(rJ_.data(), rB_.data(), rN_.data(), rD_.data()))
                {
                    return false;
                }

                Row lo = Row::fill(std::numeric_limits<float>::infinity());
                Row hi = Row::fill(0.F);
                for (auto r = 0U; r < e; ++r)
                {
                    const Row mag = rD_[d + r].abs();
                    lo = lo.min(mag);
                    hi = hi.max(mag);
                }

                if (not (lo > hi * 1e-6F).all())
                {
                    return false;
                }

                for (auto j = 0U; j < d; ++j)
                {
                    delta[j] = rD_[j];
                }

                return true;
            }

            if (refactor and not factor_rows(chart, have_p))
            {
                return false;
            }

            solve_rows();
            for (auto j = 0U; j < d; ++j)
            {
                Row v = Row::fill(0.F);
                for (auto c = 0U; c < e; ++c)
                {
                    v = v + rX_[c] * chart.normal[c][j];
                }

                delta[j] = v;
            }

            return true;
        }

        // Form M = J N for every lane from the current rows of J, precondition it with P (lane 0's
        // M^-1, computed on the first call of a batch), and LU-factor A = P M in place without
        // pivoting. False if lane 0's M is singular or any lane's pivots collapse.
        auto factor_rows(const Chart<d> &chart, bool &have_p) const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            for (auto r = 0U; r < e; ++r)
            {
                for (auto c = 0U; c < e; ++c)
                {
                    Row m = Row::fill(0.F);
                    for (auto j = 0U; j < d; ++j)
                    {
                        m = m + rJ_[r * d + j] * chart.normal[c][j];
                    }

                    rM_[r * e + c] = m;
                }
            }

            if (not have_p)
            {
                if (not compute_preconditioner(chart))
                {
                    return false;
                }

                have_p = true;
            }

            for (auto r = 0U; r < e; ++r)
            {
                for (auto c = 0U; c < e; ++c)
                {
                    Row a = Row::fill(0.F);
                    for (auto m = 0U; m < e; ++m)
                    {
                        a = a + rM_[m * e + c] * P_(r, m);
                    }

                    rA_[r * e + c] = a;
                }
            }

            Row lo = Row::fill(std::numeric_limits<float>::infinity());
            Row hi = Row::fill(0.F);
            for (auto p = 0U; p < e; ++p)
            {
                const Row pivot = rA_[p * e + p];
                const Row mag = pivot.abs();
                lo = lo.min(mag);
                hi = hi.max(mag);
                const Row inv = Row::fill(1.F) / pivot;
                for (auto r = p + 1; r < e; ++r)
                {
                    const Row l = rA_[r * e + p] * inv;
                    rA_[r * e + p] = l;
                    for (auto c = p + 1; c < e; ++c)
                    {
                        rA_[r * e + c] = rA_[r * e + c] - l * rA_[p * e + c];
                    }
                }
            }

            return (lo > hi * 1e-6F).all();
        }

        // rX_ = M^-1 rB_ for every lane, with the factorization of the last factor_rows: precondition
        // (P rB_), then unit-lower forward and upper back substitution.
        void solve_rows() const noexcept
        {
            const std::size_t e = builder_.equality_rows();
            for (auto r = 0U; r < e; ++r)
            {
                Row y = Row::fill(0.F);
                for (auto m = 0U; m < e; ++m)
                {
                    y = y + rB_[m] * P_(r, m);
                }

                rX_[r] = y;
            }

            for (auto r = 1U; r < e; ++r)
            {
                for (auto c = 0U; c < r; ++c)
                {
                    rX_[r] = rX_[r] - rA_[r * e + c] * rX_[c];
                }
            }

            for (auto r = e; r-- > 0;)
            {
                for (auto c = r + 1; c < e; ++c)
                {
                    rX_[r] = rX_[r] - rA_[r * e + c] * rX_[c];
                }

                rX_[r] = rX_[r] / rA_[r * e + r];
            }
        }

        auto exact_samples_scalar(
            const Chart<d> &chart,
            std::size_t n,
            const Lanes &u,
            const Lanes &ud,
            const Lanes &udd,
            Lanes &q,
            Lanes &qd,
            Lanes &qdd,
            std::array<float, rake> &w2,
            bool derivatives = true) const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            if (chart.nk != e or e == 0 or n == 0)
            {
                return false;
            }

            alignas(FloatVectorAlignment) std::array<FloatT, d * rake> qarr, varr;
            Lanes pre;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                pre[lane] = chart_point(chart, u[std::min<std::size_t>(lane, n - 1)]);
                q[lane] = pre[lane];
            }

            // Newton on F(pre + N w) = 0; lanes that converged keep refreshing their Jacobian at
            // the converged point, so on exit every J in eq_J_ is evaluated at sigma.
            for (std::size_t it = 0;; ++it)
            {
                constraints.evaluate_equality_position(load_block(q, qarr));
                ++stats_.newton_iterations;
                bool all = true;
                for (auto lane = 0U; lane < n; ++lane)
                {
                    float *J = eq_J_.data() + lane * e * d;
                    constraints.extract_equality(lane, eq_F_.data(), J, eq_gamma_.data());

                    float err2 = 0.F;
                    for (auto r = 0U; r < e; ++r)
                    {
                        err2 += eq_F_[r] * eq_F_[r];
                    }

                    if (err2 <= settings.exact_tolerance)
                    {
                        continue;
                    }

                    all = false;
                    if (it >= settings.exact_iterations or not factor_normal_system(chart, J))
                    {
                        return false;
                    }

                    for (auto r = 0U; r < e; ++r)
                    {
                        rhs_[r] = -eq_F_[r];
                    }

                    add_normal_solution(chart, q[lane]);
                }

                if (all)
                {
                    break;
                }

                for (auto lane = n; lane < rake; ++lane)
                {
                    q[lane] = q[n - 1];
                }
            }

            for (auto lane = 0U; lane < n; ++lane)
            {
                float m2 = 0.F;
                for (auto j = 0U; j < d; ++j)
                {
                    const float dw = q[lane][j] - pre[lane][j];
                    m2 += dw * dw;
                }

                w2[lane] = m2;
            }

            if (not derivatives)
            {
                return true;
            }

            for (auto lane = 0U; lane < n; ++lane)
            {
                const float *J = eq_J_.data() + lane * e * d;
                if (not factor_normal_system(chart, J))
                {
                    return false;
                }

                qd[lane] = from_chart(chart, ud[lane]);
                for (auto r = 0U; r < e; ++r)
                {
                    float dot = 0.F;
                    for (auto j = 0U; j < d; ++j)
                    {
                        dot += J[r * d + j] * qd[lane][j];
                    }

                    rhs_[r] = -dot;
                }

                add_normal_solution(chart, qd[lane]);
            }

            for (auto lane = n; lane < rake; ++lane)
            {
                qd[lane] = qd[n - 1];
            }

            constraints.evaluate_equality(load_block(q, qarr), load_block(qd, varr));
            for (auto lane = 0U; lane < n; ++lane)
            {
                float *J = eq_J_.data() + lane * e * d;
                constraints.extract_equality(lane, eq_F_.data(), J, eq_gamma_.data());
                if (not factor_normal_system(chart, J))
                {
                    return false;
                }

                qdd[lane] = from_chart(chart, udd[lane]);
                for (auto r = 0U; r < e; ++r)
                {
                    float dot = 0.F;
                    for (auto j = 0U; j < d; ++j)
                    {
                        dot += J[r * d + j] * qdd[lane][j];
                    }

                    rhs_[r] = -(dot + eq_gamma_[r]);
                }

                add_normal_solution(chart, qdd[lane]);
            }

            return true;
        }

        // Damped Gauss-Newton on the stacked equality form from q: q <- q - J^T (J J^T + mu I)^-1 F,
        // trust-region clipped and clamped to the joint bounds, until |F|^2 <= exact_tolerance,
        // within the constraint settings' iteration cap.
        auto exact_project(std::array<float, d> &q) const noexcept -> bool
        {
            const std::size_t e = builder_.equality_rows();
            const std::size_t cap = std::max(settings.exact_iterations, constraints.settings().max_iterations);
            alignas(FloatVectorAlignment) std::array<FloatT, d * rake> qarr;
            Lanes lanes;
            Eigen::MatrixXf JJt(e, e);
            Eigen::VectorXf F(e);
            for (std::size_t it = 0; it <= cap; ++it)
            {
                lanes.fill(q);
                constraints.evaluate_equality_position(load_block(lanes, qarr));
                constraints.extract_equality(0, eq_F_.data(), eq_J_.data(), eq_gamma_.data());

                float err2 = 0.F;
                for (auto r = 0U; r < e; ++r)
                {
                    err2 += eq_F_[r] * eq_F_[r];
                    F[r] = eq_F_[r];
                }

                if (err2 <= settings.exact_tolerance)
                {
                    return true;
                }

                if (it == cap or not std::isfinite(err2))
                {
                    return false;
                }

                const Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> J(
                    eq_J_.data(), e, d);
                JJt.noalias() = J * J.transpose();
                JJt.diagonal().array() += 1e-6F;
                // Trust region: clip long steps so far seeds descend instead of overshooting.
                Eigen::VectorXf step = J.transpose() * JJt.ldlt().solve(F);
                const float norm = step.norm();
                if (norm > max_projection_step)
                {
                    step *= max_projection_step / norm;
                }

                for (auto j = 0U; j < d; ++j)
                {
                    q[j] = std::clamp(q[j] - step[j], Ambient::lower_bound[j], Ambient::upper_bound[j]);
                }
            }

            return false;
        }

        // Exact-chart lift of a single chart point u (no derivatives).
        inline auto exact_point(const Chart<d> &chart, const std::array<float, d> &u, std::array<float, d> &q_out)
            const noexcept -> bool
        {
            Lanes uu{}, q, qd, qdd;
            std::array<float, rake> w2;
            uu[0] = u;
            if (not exact_samples(chart, 1, uu, uu, uu, q, qd, qdd, w2, false))
            {
                return false;
            }

            q_out = q[0];
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

            // Exact charts need no shooting: B^T (psi(u_f) - q0) = u_f = B^T (q_t - q0), so
            // psi(u_f) is the on-manifold target itself.
            es.q_pre = q0;
            if (es.reach and not exact_ and
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
                const bool lifted = es.lifted or
                                    ((exact_) ? exact_point(es.chart, es.uf, es.q_pre) :
                                                lift_point(chart_point(es.chart, es.uf), es.q_pre));
                if (not lifted)
                {
                    return EdgeStatus::Invalid;
                }

                if (not accept(join(es.q_pre, v_T)))
                {
                    return EdgeStatus::Pruned;
                }
            }

            // Pre-screen: one SIMD batch spread over the whole edge, on the tangent-plane approximation
            // (chart point, chart-frame velocity and acceleration -- no lift), against the effort
            // limits inflated by prescreen_margin. Rejects most doomed edges before any lift.
            if (settings.prescreen_margin > 0.F)
            {
                alignas(FloatVectorAlignment) std::array<FloatT, 3 * d * rake> pz;
                std::array<float, d> up{}, udp{}, uddp{};
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    eval_cubic(es, T * static_cast<float>(lane + 1) / static_cast<float>(rake), up, udp, uddp);
                    const auto qa = chart_point(es.chart, up);
                    const auto va = from_chart(es.chart, udp);
                    const auto aa = from_chart(es.chart, uddp);
                    for (auto dim = 0U; dim < d; ++dim)
                    {
                        pz[lane + dim * rake] = qa[dim];
                        pz[lane + (d + dim) * rake] = sign * va[dim];
                        pz[lane + (2 * d + dim) * rake] = aa[dim];
                    }
                }

                if (not ZRobot::template limits_check<rake>(
                        ZBlock(pz),
                        collision::attachment_payload<ZRobot::payload_dimension / 10>(environment),
                        1.F + settings.prescreen_margin,
                        false))
                {
                    return EdgeStatus::Invalid;
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

            // Exact charts: per-lane chart samples and the executed states they lift to.
            Lanes ub, udb, uddb, ql, qdl, qddl;
            std::array<float, rake> w2;
            std::array<float, d> v_exact_T{};

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
                    ub[lane] = u_now;
                    udb[lane] = ud_now;
                    uddb[lane] = udd_now;

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

                // Exact charts replace projection with the normal-space lift and give the
                // executed velocity and acceleration exactly (execution frame by construction).
                if (exact_)
                {
                    if (not exact_samples(es.chart, rake, ub, udb, uddb, ql, qdl, qddl, w2))
                    {
                        return EdgeStatus::Invalid;
                    }

                    for (auto lane = 0U; lane < rake; ++lane)
                    {
                        for (auto dim = 0U; dim < d; ++dim)
                        {
                            amb[lane + dim * rake] = ql[lane][dim];
                            zarr[lane + (d + dim) * rake] = sign * qdl[lane][dim];
                            zarr[lane + (2 * d + dim) * rake] = qddl[lane][dim];
                        }
                    }

                    if (b + 1 == n_batches)
                    {
                        for (auto dim = 0U; dim < d; ++dim)
                        {
                            v_exact_T[dim] = sign * qdl[rake - 1][dim];
                        }
                    }
                }

                AmbientBlock ablock(amb);
                if (not exact_ and not constraints.project_all(ablock))
                {
                    return EdgeStatus::Invalid;
                }

                // One constraint-kernel evaluation covers the whole batch of lifted
                // samples; extract every lane's Jacobian up front, since the NaN-fallback
                // make_chart below overwrites the kernel caches.
                if (builder_.n_active() != 0 and not exact_)
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
                    // is the same order but needs the second fundamental form -- which
                    // exact charts supply (exact_samples), so they skip this block.
                    if (builder_.n_active() != 0 and not exact_)
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
                if (not fkcc_block<ZRobot, rake>(environment, zblock, &stats_.rake_calls))
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
            out.vf = (exact_) ? v_exact_T : v_T;
            return EdgeStatus::Valid;
        }

        bool exact_;
        ChartBuilder<Ambient, rake> builder_;
        mutable std::vector<float> err_;
        mutable std::vector<float> jac_batch_;
        mutable std::vector<float> eq_F_;
        mutable std::vector<float> eq_gamma_;
        mutable std::vector<float> eq_J_;  // per-lane equality Jacobians (rake x rows x d)
        mutable Eigen::MatrixXf M_;        // J N, the normal-space Newton system
        mutable Eigen::PartialPivLU<Eigen::MatrixXf> lu_;
        mutable Eigen::VectorXf rhs_;
        mutable Eigen::VectorXf sol_;
        mutable std::vector<Configuration> chain_;
        mutable PlanningStats stats_;
        // Batched exact-chart buffers (SIMD rows over the rake lanes): equality F, gamma, J
        // (row-major), the normal-space system M = J N, its preconditioned LU A = P M, and the
        // right-hand side / solution; P is the scalar preconditioner of the current batch.
        mutable std::vector<Row> rF_, rG_, rJ_, rM_, rA_, rB_, rX_;
        // Generated-kernel path: preconditioned normals N P (broadcast rows) and the kernel output
        // (joint-space step, then LU pivots).
        mutable std::vector<Row> rN_, rD_;
        mutable Eigen::MatrixXf P_;
    };
}  // namespace vamp::planning::constraint
