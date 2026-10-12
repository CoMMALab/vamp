#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <vamp/planning/plan.hh>

namespace vamp::planning
{
    // Time-parameterized joint trajectory of a flask robot: knots (t, q, qd, qdd), queried at any
    // time by interpolating the enclosing knot interval (position, velocity and acceleration match
    // at the knots; see state). Torques come from the robot's RNEA kernel with a payload.
    template <typename ZRobot>
    struct Trajectory
    {
        static constexpr std::size_t n = ZRobot::flat_dimension;
        using Joints = std::array<float, n>;

        std::vector<float> times;
        std::vector<Joints> q;
        std::vector<Joints> qd;
        std::vector<Joints> qdd;

        auto size() const noexcept -> std::size_t
        {
            return times.size();
        }

        auto duration() const noexcept -> float
        {
            return times.empty() ? 0.F : times.back() - times.front();
        }

        // Append a knot. A knot at (numerically) the previous knot's time is kept only if its
        // acceleration differs -- a junction where piecewise segments join with an acceleration jump:
        // queries just before it interpolate with the earlier acceleration, at and after it with the
        // later one. Knots earlier than the last are dropped.
        void append(float t, const Joints &qk, const Joints &qdk, const Joints &qddk)
        {
            if (not times.empty())
            {
                if (t < times.back() - 1e-7F)
                {
                    return;
                }

                if (t <= times.back() + 1e-7F)
                {
                    float jump = 0.F;
                    for (auto j = 0U; j < n; ++j)
                    {
                        jump = std::max(jump, std::abs(qddk[j] - qdd.back()[j]));
                    }

                    if (jump < 1e-6F)
                    {
                        return;
                    }

                    t = times.back();
                }
            }

            times.emplace_back(t);
            q.emplace_back(qk);
            qd.emplace_back(qdk);
            qdd.emplace_back(qddk);
        }

        // (q, qd, qdd) at time t, clamped to the trajectory's time span.
        void state(float t, Joints &qo, Joints &qdo, Joints &qddo) const noexcept
        {
            if (times.size() == 1 or t <= times.front())
            {
                qo = q.front();
                qdo = qd.front();
                qddo = qdd.front();
                return;
            }

            if (t >= times.back())
            {
                qo = q.back();
                qdo = qd.back();
                qddo = qdd.back();
                return;
            }

            const auto k =
                static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), t) - times.begin() - 1);
            const float h = times[k + 1] - times[k];
            const float s = (t - times[k]) / h;
            const float s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;

            // Position: quintic Hermite (p0, h v0, h^2 a0, h^2 a1, h v1, p1). Velocity and acceleration
            // come from the knots' own exact values -- cubic Hermite of (v, a) and linear a -- rather than
            // differentiating the position interpolant, whose 1 / h^2 sensitivity turns float rounding of
            // closely spaced knot positions into large acceleration errors.
            const float H[6] = {
                1.F - 10.F * s3 + 15.F * s4 - 6.F * s5,
                s - 6.F * s3 + 8.F * s4 - 3.F * s5,
                0.5F * (s2 - 3.F * s3 + 3.F * s4 - s5),
                0.5F * (s3 - 2.F * s4 + s5),
                -4.F * s3 + 7.F * s4 - 3.F * s5,
                10.F * s3 - 15.F * s4 + 6.F * s5};
            const float V[4] = {2.F * s3 - 3.F * s2 + 1.F, s3 - 2.F * s2 + s, -2.F * s3 + 3.F * s2, s3 - s2};

            for (auto j = 0U; j < n; ++j)
            {
                const float c[6] = {
                    q[k][j],
                    h * qd[k][j],
                    h * h * qdd[k][j],
                    h * h * qdd[k + 1][j],
                    h * qd[k + 1][j],
                    q[k + 1][j]};
                float p = 0.F;
                for (auto i = 0U; i < 6; ++i)
                {
                    p += H[i] * c[i];
                }

                qo[j] = p;
                qdo[j] =
                    V[0] * qd[k][j] + V[1] * h * qdd[k][j] + V[2] * qd[k + 1][j] + V[3] * h * qdd[k + 1][j];
                qddo[j] = (1.F - s) * qdd[k][j] + s * qdd[k + 1][j];
            }
        }

        // Joint torques at time t with the given payload.
        auto torques(float t, const typename ZRobot::Payload &payload) const noexcept -> Joints
        {
            Joints qo, qdo, qddo;
            state(t, qo, qdo, qddo);
            std::array<float, 3 * n> x;
            std::copy(qo.begin(), qo.end(), x.begin());
            std::copy(qdo.begin(), qdo.end(), x.begin() + n);
            std::copy(qddo.begin(), qddo.end(), x.begin() + 2 * n);
            return ZRobot::torques(x, payload);
        }
    };

    namespace detail
    {
        template <typename LP, typename = void>
        struct lifts_edges : std::false_type
        {
        };

        template <typename LP>
        struct lifts_edges<LP, std::void_t<decltype(&LP::lift_edge)>> : std::true_type
        {
        };

        // max_i |tau_i| / (effort_scale * limit_i) over (q, qd, qdd) with a payload.
        template <typename ZRobot, typename Joints>
        inline auto torque_ratio(
            const Joints &q,
            const Joints &qd,
            const Joints &qdd,
            const typename ZRobot::Payload &payload) noexcept -> float
        {
            constexpr std::size_t n = ZRobot::flat_dimension;
            std::array<float, 3 * n> x;
            std::copy(q.begin(), q.end(), x.begin());
            std::copy(qd.begin(), qd.end(), x.begin() + n);
            std::copy(qdd.begin(), qdd.end(), x.begin() + 2 * n);
            const auto tau = ZRobot::torques(x, payload);
            float r = 0.F;
            for (auto i = 0U; i < n; ++i)
            {
                r = std::max(r, std::abs(tau[i]) / (ZRobot::effort_limits[i] * ZRobot::effort_scale));
            }

            return r;
        }
    }  // namespace detail

    // The executed motion of a flask path, sampled at `samples` points per edge. Unconstrained flask
    // edges are the robot's LQMT cubics at their optimal durations. Edges of a chart local planner
    // are its exact lifts; a planner tree may have built an edge from either end (goal-tree edges
    // are time-reversed lifts from the later state), and both lifts land on the edge's endpoints, so
    // the one with the lower peak torque ratio is kept (the validated one passes the torque check).
    // Empty if an edge cannot be lifted.
    template <typename ZRobot, typename LocalPlanner>
    inline auto trajectory_from_path(
        const Path<ZRobot> &path,
        const LocalPlanner &lp,
        const typename ZRobot::Payload &payload,
        std::size_t samples = 64) -> std::optional<Trajectory<ZRobot>>
    {
        constexpr std::size_t n = ZRobot::flat_dimension;
        using Joints = typename Trajectory<ZRobot>::Joints;
        Trajectory<ZRobot> out;
        if (path.empty())
        {
            return std::nullopt;
        }

        const auto split = [](const auto &z, Joints &q, Joints &qd)
        {
            const auto arr = z.to_array();
            std::copy_n(arr.begin(), n, q.begin());
            std::copy_n(arr.begin() + n, n, qd.begin());
        };

        float t0 = 0.F;
        samples = std::max<std::size_t>(samples, 2);
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            if constexpr (detail::lifts_edges<LocalPlanner>::value)
            {
                using Configuration = typename ZRobot::Configuration;
                std::optional<std::pair<float, Trajectory<ZRobot>>> best;
                for (const bool forward : {true, false})
                {
                    const Configuration &from = forward ? path[i] : path[i + 1];
                    const Configuration &to = forward ? path[i + 1] : path[i];
                    std::vector<Configuration> states;
                    std::vector<float> errors;
                    std::vector<std::array<float, n>> acc;
                    float T = 0.F, cost = 0.F;
                    if (not lp.lift_edge(from, to, forward, samples, states, errors, T, cost, &acc))
                    {
                        continue;
                    }

                    if (not forward)
                    {
                        std::reverse(states.begin(), states.end());
                        std::reverse(acc.begin(), acc.end());
                    }

                    Trajectory<ZRobot> seg;
                    float peak = 0.F;
                    for (std::size_t k = 0; k < states.size(); ++k)
                    {
                        Joints q, qd;
                        split(states[k], q, qd);
                        const float t = (states.size() > 1) ? T * k / (states.size() - 1) : 0.F;
                        seg.append(t, q, qd, acc[k]);
                        peak = std::max(peak, detail::torque_ratio<ZRobot>(q, qd, acc[k], payload));
                    }

                    if (not best or peak < best->first)
                    {
                        best = std::make_pair(peak, std::move(seg));
                    }
                }

                if (not best)
                {
                    return std::nullopt;
                }

                const auto &seg = best->second;
                for (std::size_t k = 0; k < seg.size(); ++k)
                {
                    out.append(t0 + seg.times[k], seg.q[k], seg.qd[k], seg.qdd[k]);
                }

                t0 += seg.duration();
            }
            else
            {
                (void)lp;
                const float T = ZRobot::optimal_time(path[i], path[i + 1]);
                for (std::size_t k = 0; k < samples; ++k)
                {
                    const float frac = static_cast<float>(k) / (samples - 1);
                    const auto x = ZRobot::eval(path[i], path[i + 1], T, frac);
                    Joints q, qd, qdd;
                    std::copy_n(x.begin(), n, q.begin());
                    std::copy_n(x.begin() + n, n, qd.begin());
                    std::copy_n(x.begin() + 2 * n, n, qdd.begin());
                    out.append(t0 + frac * T, q, qd, qdd);
                }

                t0 += T;
            }
        }

        if (out.size() == 0)
        {
            Joints q, qd, zero{};
            split(path.front(), q, qd);
            out.append(0.F, q, qd, zero);
        }

        return out;
    }

    struct RetimeSettings
    {
        std::size_t grid = 800;         // path grid points
        float effort_fraction = 0.98F;  // torque bound as a fraction of the (scaled) effort limits
        float end_speed = 0.F;          // path speed bound at the end (path parameter per second)
        std::size_t substeps = 4;       // output knots per grid interval
        // Constraints hold at the grid points; between them torque can overshoot where the path's
        // curvature changes quickly. Each repair round evaluates every output knot and splits the grid
        // intervals that overshoot the torque bound or a velocity limit (by more than repair_tolerance) into
        // quarters before solving again; a refined grid that loses feasibility keeps the previous timing. 0
        // disables.
        std::size_t repair_iterations = 6;
        float repair_tolerance = 0.005F;
    };

    namespace detail
    {
        // Natural cubic spline through (s_k, y_k) per column; evaluates value, first and second
        // derivative.
        template <std::size_t n>
        struct CubicSpline
        {
            std::vector<float> s;
            std::vector<std::array<float, n>> y, m;  // values and second derivatives at the knots

            CubicSpline(std::vector<float> knots, std::vector<std::array<float, n>> values)
              : s(std::move(knots)), y(std::move(values)), m(s.size())
            {
                const std::size_t K = s.size();
                for (auto &mk : m)
                {
                    mk.fill(0.F);
                }

                if (K < 3)
                {
                    return;
                }

                // Thomas algorithm on the interior second derivatives (natural ends: m_0 = m_K-1 = 0).
                std::vector<double> c(K, 0.0), dprime(K, 0.0);
                for (auto j = 0U; j < n; ++j)
                {
                    std::vector<double> d(K, 0.0);
                    for (std::size_t k = 1; k + 1 < K; ++k)
                    {
                        const double h0 = s[k] - s[k - 1], h1 = s[k + 1] - s[k];
                        const double a = h0 / 6.0, b = (h0 + h1) / 3.0, cc = h1 / 6.0;
                        const double rhs = (y[k + 1][j] - y[k][j]) / h1 - (y[k][j] - y[k - 1][j]) / h0;
                        const double denom = b - a * ((k > 1) ? c[k - 1] : 0.0);
                        c[k] = cc / denom;
                        d[k] = (rhs - a * ((k > 1) ? d[k - 1] : 0.0)) / denom;
                    }

                    double next = 0.0;
                    for (std::size_t k = K - 2; k >= 1; --k)
                    {
                        next = d[k] - c[k] * next;
                        m[k][j] = static_cast<float>(next);
                        if (k == 1)
                        {
                            break;
                        }
                    }
                }
            }

            void
            eval(float x, std::array<float, n> &v, std::array<float, n> &d1, std::array<float, n> &d2) const
            {
                const auto k = std::min<std::size_t>(
                    static_cast<std::size_t>(
                        std::max<std::ptrdiff_t>(std::upper_bound(s.begin(), s.end(), x) - s.begin() - 1, 0)),
                    s.size() - 2);
                const float h = s[k + 1] - s[k];
                const float a = (s[k + 1] - x) / h, b = (x - s[k]) / h;
                for (auto j = 0U; j < n; ++j)
                {
                    const float m0 = m[k][j], m1 = m[k + 1][j];
                    v[j] = a * y[k][j] + b * y[k + 1][j] +
                           ((a * a * a - a) * m0 + (b * b * b - b) * m1) * h * h / 6.F;
                    d1[j] = (y[k + 1][j] - y[k][j]) / h +
                            ((1.F - 3.F * a * a) * m0 + (3.F * b * b - 1.F) * m1) * h / 6.F;
                    d2[j] = a * m0 + b * m1;
                }
            }
        };

        // Optimum (u, x) of objective . (u, x) over {A (u, x) <= rhs} by vertex enumeration.
        inline auto
        lp2(const std::vector<std::array<double, 2>> &A,
            const std::vector<double> &rhs,
            double ou,
            double ox,
            bool maximize) -> std::optional<std::array<double, 2>>
        {
            std::optional<std::array<double, 2>> best;
            double best_value = 0.0;
            const std::size_t m = A.size();
            for (std::size_t i = 0; i < m; ++i)
            {
                for (std::size_t k = i + 1; k < m; ++k)
                {
                    const double det = A[i][0] * A[k][1] - A[i][1] * A[k][0];
                    if (std::abs(det) < 1e-12)
                    {
                        continue;
                    }

                    const double u = (rhs[i] * A[k][1] - A[i][1] * rhs[k]) / det;
                    const double x = (A[i][0] * rhs[k] - rhs[i] * A[k][0]) / det;
                    bool feasible = true;
                    for (std::size_t r = 0; r < m and feasible; ++r)
                    {
                        feasible = A[r][0] * u + A[r][1] * x <= rhs[r] + 1e-9 * (1.0 + std::abs(rhs[r]));
                    }

                    if (not feasible)
                    {
                        continue;
                    }

                    const double value = ou * u + ox * x;
                    if (not best or (maximize ? value > best_value : value < best_value))
                    {
                        best = std::array<double, 2>{u, x};
                        best_value = value;
                    }
                }
            }

            return best;
        }
    }  // namespace detail

    // Time-optimal retiming of a dense joint path under torque and joint-velocity limits:
    // time-optimal path parameterization by reachability analysis (Pham & Pham, T-RO 2018). The
    // path q(s) is a chord-length natural cubic spline through `positions` (near-duplicate points,
    // closer than a quarter of the median spacing, are dropped: junction repeats would spike its
    // curvature), sampled at settings.grid points. Torques are affine in (sdd, sd^2),
    // tau = a sdd + b sd^2 + c, with a, b, c from the payload-aware RNEA; at both ends of every
    // grid interval (first-order interpolation discretization) they stay within
    // effort_fraction of the effort limits, and joint speeds within their limits. Starts at rest
    // and ends with path speed at most end_speed. Statically infeasible stretches are kept when the
    // velocity terms make them feasible. The trajectory is the executed motion (sdd constant per
    // interval, sd^2 linear in s) at `substeps` knots per interval. Empty with a reason if no
    // feasible timing exists.
    template <typename ZRobot>
    inline auto topp_retime(
        const std::vector<std::array<float, ZRobot::flat_dimension>> &positions,
        const typename ZRobot::Payload &payload,
        const RetimeSettings &settings,
        std::string *reason = nullptr) -> std::optional<Trajectory<ZRobot>>
    {
        constexpr std::size_t n = ZRobot::flat_dimension;
        using Joints = std::array<float, n>;
        const auto fail = [&](std::string why) -> std::optional<Trajectory<ZRobot>>
        {
            if (reason)
            {
                *reason = std::move(why);
            }

            return std::nullopt;
        };

        if (positions.size() < 2 or settings.grid < 3)
        {
            return fail("need at least two positions and three grid points");
        }

        const auto dist = [](const Joints &a, const Joints &b)
        {
            float d2 = 0.F;
            for (auto j = 0U; j < n; ++j)
            {
                d2 += (a[j] - b[j]) * (a[j] - b[j]);
            }

            return std::sqrt(d2);
        };

        std::vector<float> gaps;
        for (std::size_t k = 1; k < positions.size(); ++k)
        {
            gaps.emplace_back(dist(positions[k], positions[k - 1]));
        }

        std::nth_element(gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end());
        const float gap = 0.25F * gaps[gaps.size() / 2];
        std::vector<Joints> kept{positions.front()};
        for (std::size_t k = 1; k + 1 < positions.size(); ++k)
        {
            if (dist(positions[k], kept.back()) >= gap)
            {
                kept.emplace_back(positions[k]);
            }
        }

        if (kept.size() > 1 and dist(positions.back(), kept.back()) < gap)
        {
            kept.pop_back();
        }

        kept.emplace_back(positions.back());
        if (kept.size() < 2)
        {
            return fail("the path has no extent");
        }

        std::vector<float> chord{0.F};
        for (std::size_t k = 1; k < kept.size(); ++k)
        {
            chord.emplace_back(chord.back() + dist(kept[k], kept[k - 1]));
        }

        const float length = chord.back();
        if (not(length > 0.F))
        {
            return fail("the path has no extent");
        }

        for (auto &c : chord)
        {
            c /= length;
        }

        const detail::CubicSpline<n> spline(chord, kept);
        std::array<double, n> limit;
        for (auto j = 0U; j < n; ++j)
        {
            limit[j] = settings.effort_fraction * ZRobot::effort_limits[j] * ZRobot::effort_scale;
        }

        const Joints zero{};
        const auto tau = [&](const Joints &q, const Joints &qd, const Joints &qdd)
        {
            std::array<float, 3 * n> x;
            std::copy(q.begin(), q.end(), x.begin());
            std::copy(qd.begin(), qd.end(), x.begin() + n);
            std::copy(qdd.begin(), qdd.end(), x.begin() + 2 * n);
            return ZRobot::torques(x, payload);
        };

        // Grid in s (refined where executed torque overshoots between grid points).
        std::vector<double> sg(settings.grid);
        for (std::size_t i = 0; i < settings.grid; ++i)
        {
            sg[i] = static_cast<double>(i) / (settings.grid - 1);
        }

        std::optional<Trajectory<ZRobot>> result;
        for (std::size_t round = 0; round <= settings.repair_iterations; ++round)
        {
            const std::size_t G = sg.size();
            std::vector<Joints> qs(G), dq(G), ddq(G);
            std::vector<std::array<double, n>> a(G), b(G), c(G);
            std::vector<double> x_cap(G, std::numeric_limits<double>::infinity());
            for (std::size_t i = 0; i < G; ++i)
            {
                spline.eval(static_cast<float>(sg[i]), qs[i], dq[i], ddq[i]);
                const auto tc = tau(qs[i], zero, zero);
                const auto ta = tau(qs[i], zero, dq[i]);
                const auto tb = tau(qs[i], dq[i], ddq[i]);
                for (auto j = 0U; j < n; ++j)
                {
                    c[i][j] = tc[j];
                    a[i][j] = ta[j] - tc[j];
                    b[i][j] = tb[j] - tc[j];
                    if (std::abs(dq[i][j]) > 1e-12F)
                    {
                        const double cap = ZRobot::velocity_limits[j] / std::abs(dq[i][j]);
                        x_cap[i] = std::min(x_cap[i], cap * cap);
                    }
                }
            }

            // Interval i's constraints on (u, x): torque at its start and, with
            // sd^2 = x + 2 delta_i u, at its end; 0 <= x <= x_cap at both ends.
            const auto rows =
                [&](std::size_t i, std::vector<std::array<double, 2>> &A, std::vector<double> &rhs)
            {
                const std::size_t j1 = i + 1;
                const double delta = sg[j1] - sg[i];
                A.clear();
                rhs.clear();
                for (auto j = 0U; j < n; ++j)
                {
                    A.push_back({a[i][j], b[i][j]});
                    rhs.push_back(limit[j] - c[i][j]);
                    A.push_back({-a[i][j], -b[i][j]});
                    rhs.push_back(limit[j] + c[i][j]);
                    const double ae = a[j1][j] + 2.0 * delta * b[j1][j];
                    A.push_back({ae, b[j1][j]});
                    rhs.push_back(limit[j] - c[j1][j]);
                    A.push_back({-ae, -b[j1][j]});
                    rhs.push_back(limit[j] + c[j1][j]);
                }

                A.push_back({0.0, -1.0});
                rhs.push_back(0.0);
                A.push_back({0.0, 1.0});
                rhs.push_back(x_cap[i]);
                A.push_back({2.0 * delta, 1.0});
                rhs.push_back(x_cap[j1]);
            };

            // Backward pass: controllable sets [lo_i, hi_i] of sd^2.
            std::vector<double> lo(G, 0.0), hi(G, 0.0);
            hi[G - 1] = std::min(static_cast<double>(settings.end_speed) * settings.end_speed, x_cap[G - 1]);
            std::vector<std::array<double, 2>> A;
            std::vector<double> rhs;
            bool feasible = true;
            for (std::size_t i = G - 1; i-- > 0;)
            {
                const double delta = sg[i + 1] - sg[i];
                rows(i, A, rhs);
                A.push_back({2.0 * delta, 1.0});
                rhs.push_back(hi[i + 1]);
                A.push_back({-2.0 * delta, -1.0});
                rhs.push_back(-lo[i + 1]);
                const auto top = detail::lp2(A, rhs, 0.0, 1.0, true);
                const auto bottom = detail::lp2(A, rhs, 0.0, 1.0, false);
                if (not top or not bottom)
                {
                    if (not result)
                    {
                        return fail(
                            "no feasible timing: the path is uncontrollable at s = " + std::to_string(sg[i]));
                    }

                    feasible = false;
                    break;
                }

                lo[i] = std::max((*bottom)[1], 0.0);
                hi[i] = (*top)[1];
            }

            if (feasible and lo[0] > 1e-12)
            {
                if (not result)
                {
                    return fail("no feasible timing from rest at the start");
                }

                feasible = false;
            }

            if (not feasible)
            {
                return result;  // a refined grid lost feasibility: keep the previous timing
            }

            // Forward pass: greedy maximum sdd within the controllable sets.
            std::vector<double> x(G, 0.0), u(G, 0.0);
            for (std::size_t i = 0; i + 1 < G; ++i)
            {
                const double delta = sg[i + 1] - sg[i];
                rows(i, A, rhs);
                A.push_back({2.0 * delta, 1.0});
                rhs.push_back(hi[i + 1]);
                A.push_back({-2.0 * delta, -1.0});
                rhs.push_back(-lo[i + 1]);
                double upper = std::numeric_limits<double>::infinity();
                double lower = -std::numeric_limits<double>::infinity();
                for (std::size_t r = 0; r < A.size(); ++r)
                {
                    const double bound = rhs[r] - A[r][1] * x[i];
                    if (A[r][0] > 1e-12)
                    {
                        upper = std::min(upper, bound / A[r][0]);
                    }
                    else if (A[r][0] < -1e-12)
                    {
                        lower = std::max(lower, bound / A[r][0]);
                    }
                }

                u[i] = (upper >= lower) ? upper : lower;
                x[i + 1] = std::min(std::max(x[i] + 2.0 * delta * u[i], lo[i + 1]), hi[i + 1]);
                u[i] = (x[i + 1] - x[i]) / (2.0 * delta);
            }

            // Executed motion: within interval i, sdd = u_i and sd^2 = x_i + 2 u_i (s - s_i).
            Trajectory<ZRobot> out;
            const std::size_t sub = std::max<std::size_t>(settings.substeps, 1);
            std::vector<std::size_t> interval;  // grid interval of each knot
            double t = 0.0, s_prev = 0.0, sd_prev = 0.0;
            const auto emit = [&](double sv, double xs, double us, std::size_t i)
            {
                const double sd = std::sqrt(std::max(xs, 0.0));
                if (sv > 0.0)
                {
                    t += 2.0 * (sv - s_prev) / std::max(sd_prev + sd, 1e-12);
                }

                Joints q, d1, d2, qd, qdd;
                spline.eval(static_cast<float>(sv), q, d1, d2);
                for (auto j = 0U; j < n; ++j)
                {
                    qd[j] = static_cast<float>(d1[j] * sd);
                    qdd[j] = static_cast<float>(d1[j] * us + d2[j] * xs);
                }

                const std::size_t before = out.size();
                out.append(static_cast<float>(t), q, qd, qdd);
                if (out.size() > before)
                {
                    interval.push_back(i);
                }

                s_prev = sv;
                sd_prev = sd;
            };

            for (std::size_t i = 0; i + 1 < G; ++i)
            {
                const double delta = sg[i + 1] - sg[i];
                for (std::size_t k = 0; k < sub; ++k)
                {
                    const double f = static_cast<double>(k) / sub;
                    emit(sg[i] + f * delta, x[i] + 2.0 * u[i] * delta * f, u[i], i);
                }
            }

            emit(1.0, x[G - 1], u[G - 2], G - 2);
            result = std::move(out);

            // Refine: split every grid interval whose executed knots overshoot the torque bound or a
            // joint-velocity limit.
            std::vector<bool> split(G - 1, false);
            bool any = false;
            const double tol = 1.0 + settings.repair_tolerance;
            for (std::size_t k = 0; k < result->size(); ++k)
            {
                const auto tk = tau(result->q[k], result->qd[k], result->qdd[k]);
                for (auto j = 0U; j < n; ++j)
                {
                    if (std::abs(tk[j]) > tol * limit[j] or
                        std::abs(result->qd[k][j]) > tol * ZRobot::velocity_limits[j])
                    {
                        split[interval[k]] = true;
                        any = true;
                        break;
                    }
                }
            }

            if (not any)
            {
                break;
            }

            std::vector<double> refined;
            for (std::size_t i = 0; i + 1 < G; ++i)
            {
                refined.push_back(sg[i]);
                if (split[i])
                {
                    for (int m = 1; m < 4; ++m)
                    {
                        refined.push_back(sg[i] + 0.25 * m * (sg[i + 1] - sg[i]));
                    }
                }
            }

            refined.push_back(sg.back());
            sg = std::move(refined);
        }

        return result;
    }
}  // namespace vamp::planning
