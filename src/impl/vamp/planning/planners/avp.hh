#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planners/bkpiece.hh>
#include <vamp/planning/trajectory.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>

namespace vamp::planning
{
    struct AVPStats
    {
        std::size_t propagations = 0;  // edges whose speed interval was propagated
        std::size_t empty = 0;         // propagations that left no admissible speed
    };

    namespace detail
    {
        // Path-speed interval of an edge in TOPP form: along an edge's lifted samples (execution
        // order, parameter s = the edge's own time), a timing s(t) executes q(s(t)) with velocity
        // q' sd and torque a(s) sdd + b(s) sd^2 + c(s) (a = M q', b = M q'' + C(q, q') q', c the
        // static load, payloads included). With x = sd^2, sdd = u constant per collocation step,
        // x_{k+1} = x_k + 2 delta_k u, the sets of admissible x are intervals: forward() maps an
        // interval reachable at the first sample to the one reachable at the last; backward() maps an
        // interval to reach at the last sample to the interval it can be reached from at the first.
        template <typename ZRobot>
        struct SpeedPropagator
        {
            static constexpr std::size_t d = ZRobot::flat_dimension;
            static constexpr double unbounded = 1e6;
            using Joints = std::array<float, d>;
            using Interval = std::array<double, 2>;

            // Load the edge samples (q, q' at times[k], q'' in acc[k]).
            void load(
                const std::vector<Joints> &q,
                const std::vector<Joints> &qd,
                const std::vector<Joints> &qdd,
                const std::vector<float> &times,
                const typename ZRobot::Payload &payload,
                float effort_fraction)
            {
                const std::size_t n = q.size();
                a_.resize(n);
                b_.resize(n);
                c_.resize(n);
                xmax_.resize(n);
                delta_.resize(n);
                for (std::size_t k = 0; k < n; ++k)
                {
                    std::array<float, 3 * d> x{};
                    std::copy(q[k].begin(), q[k].end(), x.begin());
                    const auto g = ZRobot::torques(x, payload);
                    std::copy(qd[k].begin(), qd[k].end(), x.begin() + 2 * d);
                    const auto mq = ZRobot::torques(x, payload);
                    std::copy(qd[k].begin(), qd[k].end(), x.begin() + d);
                    std::copy(qdd[k].begin(), qdd[k].end(), x.begin() + 2 * d);
                    const auto full = ZRobot::torques(x, payload);
                    double xmax = unbounded;
                    for (auto j = 0U; j < d; ++j)
                    {
                        a_[k][j] = mq[j] - g[j];
                        b_[k][j] = full[j] - g[j];
                        c_[k][j] = g[j];
                        const double v = std::abs(qd[k][j]);
                        if (v > 1e-9)
                        {
                            const double cap = ZRobot::velocity_limits[j] / v;
                            xmax = std::min(xmax, cap * cap);
                        }
                    }

                    xmax_[k] = xmax;
                    delta_[k] = (k + 1 < n) ? times[k + 1] - times[k] : 0.0;
                }

                for (auto j = 0U; j < d; ++j)
                {
                    limit_[j] = effort_fraction * ZRobot::effort_limits[j] * ZRobot::effort_scale;
                }
            }

            auto forward(Interval in) const -> std::optional<Interval>
            {
                const std::size_t n = a_.size();
                Interval r = in;
                for (std::size_t k = 0; k < n; ++k)
                {
                    auto poly = region(k, r);
                    if (poly.empty())
                    {
                        return std::nullopt;
                    }

                    if (k + 1 == n)
                    {
                        return extent(poly, 1.0, 0.0);
                    }

                    r = extent(poly, 1.0, 2.0 * delta_[k]);
                    r[0] = std::max(r[0], 0.0);
                    r[1] = std::min(r[1], xmax_[k + 1]);
                    if (r[0] > r[1])
                    {
                        return std::nullopt;
                    }
                }

                return std::nullopt;
            }

            auto backward(Interval in) const -> std::optional<Interval>
            {
                const std::size_t n = a_.size();
                if (n == 0)
                {
                    return std::nullopt;
                }

                auto last = region(n - 1, in);
                if (last.empty())
                {
                    return std::nullopt;
                }

                Interval k_next = extent(last, 1.0, 0.0);
                for (std::size_t k = n - 1; k-- > 0;)
                {
                    auto poly = region(k, {0.0, unbounded});
                    // x + 2 delta u within the interval to reach at k + 1.
                    clip(poly, 1.0, 2.0 * delta_[k], k_next[1]);
                    clip(poly, -1.0, -2.0 * delta_[k], -k_next[0]);
                    if (poly.empty())
                    {
                        return std::nullopt;
                    }

                    k_next = extent(poly, 1.0, 0.0);
                }

                return k_next;
            }

            // Time-optimal timing along the loaded samples from `start` (admissible scales at the
            // first sample) to `end` (at the last): controllable sets backward, then the greedy
            // forward pass (largest admissible u each step). Fills x (sd^2) and u per sample; false if
            // no timing exists.
            auto optimal(Interval start, Interval end, std::vector<double> &x, std::vector<double> &u) const -> bool
            {
                const std::size_t n = a_.size();
                if (n == 0)
                {
                    return false;
                }

                std::vector<Interval> K(n);
                auto last = region(n - 1, end);
                if (last.empty())
                {
                    return false;
                }

                K[n - 1] = extent(last, 1.0, 0.0);
                for (std::size_t k = n - 1; k-- > 0;)
                {
                    auto poly = region(k, {0.0, unbounded});
                    clip(poly, 1.0, 2.0 * delta_[k], K[k + 1][1]);
                    clip(poly, -1.0, -2.0 * delta_[k], -K[k + 1][0]);
                    if (poly.empty())
                    {
                        return false;
                    }

                    K[k] = extent(poly, 1.0, 0.0);
                }

                x.assign(n, 0.0);
                u.assign(n, 0.0);
                const double lo = std::max(start[0], K[0][0]), hi = std::min(start[1], K[0][1]);
                if (lo > hi)
                {
                    return false;
                }

                x[0] = hi;
                for (std::size_t k = 0; k < n; ++k)
                {
                    const double tol = 1e-9 * (1.0 + x[k]);
                    auto poly = region(k, {x[k] - tol, x[k] + tol});
                    if (k + 1 < n)
                    {
                        clip(poly, 1.0, 2.0 * delta_[k], K[k + 1][1]);
                        clip(poly, -1.0, -2.0 * delta_[k], -K[k + 1][0]);
                    }

                    if (poly.empty())
                    {
                        return false;
                    }

                    const Interval us = extent(poly, 0.0, 1.0);
                    u[k] = (k + 1 < n) ? us[1] : std::clamp(0.0, us[0], us[1]);
                    if (k + 1 < n)
                    {
                        x[k + 1] = std::clamp(x[k] + 2.0 * delta_[k] * u[k], K[k + 1][0], K[k + 1][1]);
                    }
                }

                return true;
            }

        private:
            using Point = std::array<double, 2>;  // (x, u)
            using Polygon = std::vector<Point>;

            // Admissible (x, u) at sample k with x in r: torque and velocity limits at k and -- the
            // interpolation discretization (TOPP-RA) -- at k + 1 under the same u, where
            // x_{k+1} = x + 2 delta u. Constraining both ends of each step stops a chattering u from
            // meeting the limits at the samples while violating them in between.
            auto region(std::size_t k, const Interval &r) const -> Polygon
            {
                const double lo = std::max(r[0], 0.0), hi = std::min({r[1], xmax_[k], unbounded});
                if (lo > hi)
                {
                    return {};
                }

                constexpr double U = 1e5;
                Polygon poly{{lo, -U}, {hi, -U}, {hi, U}, {lo, U}};
                for (auto j = 0U; j < d && not poly.empty(); ++j)
                {
                    // -limit <= a u + b x + c <= limit
                    clip(poly, b_[k][j], a_[k][j], limit_[j] - c_[k][j]);
                    clip(poly, -b_[k][j], -a_[k][j], limit_[j] + c_[k][j]);
                }

                if (k + 1 < a_.size() and delta_[k] > 0.0)
                {
                    const double h = 2.0 * delta_[k];
                    clip(poly, 1.0, h, xmax_[k + 1]);
                    clip(poly, -1.0, -h, 0.0);
                    for (auto j = 0U; j < d && not poly.empty(); ++j)
                    {
                        const double bx = b_[k + 1][j], bu = a_[k + 1][j] + h * b_[k + 1][j];
                        clip(poly, bx, bu, limit_[j] - c_[k + 1][j]);
                        clip(poly, -bx, -bu, limit_[j] + c_[k + 1][j]);
                    }
                }

                return poly;
            }

            // Keep alpha x + beta u <= gamma (Sutherland-Hodgman on a convex polygon).
            static void clip(Polygon &poly, double alpha, double beta, double gamma)
            {
                if (poly.empty())
                {
                    return;
                }

                Polygon out;
                out.reserve(poly.size() + 2);
                const auto value = [&](const Point &p) { return alpha * p[0] + beta * p[1] - gamma; };
                for (std::size_t i = 0; i < poly.size(); ++i)
                {
                    const Point &p = poly[i], &q = poly[(i + 1) % poly.size()];
                    const double vp = value(p), vq = value(q);
                    if (vp <= 0.0)
                    {
                        out.push_back(p);
                    }

                    if ((vp <= 0.0) != (vq <= 0.0))
                    {
                        const double t = vp / (vp - vq);
                        out.push_back({p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])});
                    }
                }

                poly = std::move(out);
            }

            // [min, max] of wx x + wu u over the polygon.
            static auto extent(const Polygon &poly, double wx, double wu) -> Interval
            {
                Interval e{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
                for (const auto &p : poly)
                {
                    const double v = wx * p[0] + wu * p[1];
                    e[0] = std::min(e[0], v);
                    e[1] = std::max(e[1], v);
                }

                return e;
            }

            std::vector<std::array<double, d>> a_, b_, c_;
            std::vector<double> xmax_, delta_;
            std::array<double, d> limit_{};
        };
    }  // namespace detail

    // Admissible velocity propagation (Pham, Caron & Nakamura, "Kinodynamic planning in the
    // configuration space via admissible velocity propagation", RSS 2013) in BKPIECE's
    // bidirectional coverage-grid search over flask states. Tree nodes are states z = (q, qd) whose
    // velocity is a direction: each carries an interval of path-speed scales x = sd^2 at which
    // (q, sd qd) is admissible -- reachable from the start (start tree) or able to reach a goal
    // (goal tree). Edges are the local planner's (which should check geometry only: build it
    // without payload); each new edge is lifted at edge_samples points and its interval propagated
    // TOPP-style under the true payload (forward reachability on the start tree, backward
    // controllability on the goal tree). Nodes whose interval empties are dropped; trees connect
    // when the propagated interval meets the other tree's. Edges join C1 (both sides of a node
    // share its velocity direction), so the returned path is timed by TOPP along its geometry
    // (each node's velocity scaled by its sd). Root intervals: states at rest admit any scale,
    // moving ones x in [0, 1] (goals: at most their velocity; the start: exactly its velocity).
    template <typename ZRobot, std::size_t rake, std::size_t resolution>
    struct AVP
    {
        using Configuration = typename ZRobot::Configuration;
        using RNG = vamp::rng::RNG<ZRobot>;
        using Projection = detail::KPIECEProjection<ZRobot>;
        using Grid = detail::Grid<Projection::max_dim>;
        using Coord = typename Grid::Coord;
        using Cell = typename Grid::Cell;
        using Propagator = detail::SpeedPropagator<ZRobot>;
        using Interval = typename Propagator::Interval;
        using Joints = typename Propagator::Joints;
        static constexpr std::size_t d = ZRobot::flat_dimension;

        // An edge as the local planner steers it: lift_edge(from, to, forward), executed reversed when
        // backward.
        struct Segment
        {
            Configuration from, to;
            bool forward;
        };

        // Time-optimal timing of a chain of segments (execution order) under the payload, from the
        // admissible scales `start` at its first state to `end` at its last; empty if none exists.
        template <typename LocalPlanner>
        inline static auto time(
            const std::vector<Segment> &segments,
            const Interval &start,
            const Interval &end,
            const LocalPlanner &lp,
            const typename ZRobot::Payload &payload,
            std::size_t edge_samples,
            float effort_fraction) -> std::optional<Trajectory<ZRobot>>
        {
            std::vector<Joints> qs, qds, qdds;
            std::vector<float> s;
            std::vector<Configuration> states;
            std::vector<float> errors;
            std::vector<Joints> acc;
            float s0 = 0.F;
            for (const auto &seg : segments)
            {
                float T = 0.F, cost = 0.F;
                if (not lp.lift_edge(seg.from, seg.to, seg.forward, edge_samples, states, errors, T, cost, &acc) or
                    states.size() < 2)
                {
                    return std::nullopt;
                }

                if (not seg.forward)
                {
                    std::reverse(states.begin(), states.end());
                    std::reverse(acc.begin(), acc.end());
                }

                for (std::size_t k = 0; k < states.size(); ++k)
                {
                    const auto arr = states[k].to_array();
                    Joints q, qd;
                    std::copy_n(arr.begin(), d, q.begin());
                    std::copy_n(arr.begin() + d, d, qd.begin());
                    qs.push_back(q);
                    qds.push_back(qd);
                    qdds.push_back(acc[k]);
                    s.push_back(s0 + T * k / (states.size() - 1));
                }

                s0 += T;
            }

            Propagator prop;
            prop.load(qs, qds, qdds, s, payload, effort_fraction);
            std::vector<double> x, u;
            if (not prop.optimal(start, end, x, u))
            {
                return std::nullopt;
            }

            Trajectory<ZRobot> out;
            double t = 0.0;
            for (std::size_t k = 0; k < qs.size(); ++k)
            {
                if (k > 0)
                {
                    const double ds = s[k] - s[k - 1];
                    const double rate = std::sqrt(std::max(x[k - 1], 0.0)) + std::sqrt(std::max(x[k], 0.0));
                    t += (ds > 0.0) ? 2.0 * ds / std::max(rate, 1e-9) : 0.0;
                }

                const double sd = std::sqrt(std::max(x[k], 0.0));
                Joints qd, qdd;
                for (auto j = 0U; j < d; ++j)
                {
                    qd[j] = static_cast<float>(qds[k][j] * sd);
                    qdd[j] = static_cast<float>(qdds[k][j] * x[k] + qds[k][j] * u[k]);
                }

                out.append(static_cast<float>(t), qs[k], qd, qdd);
            }

            return out;
        }

        // Scale interval of a root state: at rest any scale, moving ones [0, 1] (goal: at most its
        // velocity) or exactly 1 (start: its velocity).
        static auto root_interval(const Configuration &z, bool goal) noexcept -> Interval
        {
            const auto arr = z.to_array();
            float v2 = 0.F;
            for (auto j = 0U; j < d; ++j)
            {
                v2 += arr[d + j] * arr[d + j];
            }

            if (v2 < 1e-12F)
            {
                return {0.0, Propagator::unbounded};
            }

            return goal ? Interval{0.0, 1.0} : Interval{1.0, 1.0};
        }

        template <typename LocalPlanner>
        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const typename ZRobot::Payload &payload,
            const AVPSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp,
            AVPStats *avp_stats = nullptr,
            std::optional<Trajectory<ZRobot>> *timed = nullptr) noexcept -> PlanningResult<ZRobot>
        {
            PlanningResult<ZRobot> result;
            AVPStats local_stats;
            AVPStats &stats = avp_stats ? *avp_stats : local_stats;
            lp.reset_stats();
            const auto start_time = std::chrono::steady_clock::now();
            constexpr std::size_t root = std::numeric_limits<std::size_t>::max();
            Propagator prop;

            // Lift the edge the local planner steered from `from` toward `to` (direction `edge_forward`,
            // the chart at `from`), in execution order, and propagate `in`: forward maps the interval at
            // the execution start to the one at its end, backward the reverse.
            std::vector<Configuration> states;
            std::vector<float> errors;
            std::vector<Joints> acc, qs, qds;
            std::vector<float> times;
            const auto propagate = [&](const Configuration &from,
                                       const Configuration &to,
                                       bool edge_forward,
                                       const Interval &in,
                                       bool forward) -> std::optional<Interval>
            {
                float T = 0.F, cost = 0.F;
                if (not lp.lift_edge(from, to, edge_forward, settings.edge_samples, states, errors, T, cost, &acc) or
                    states.size() < 2)
                {
                    return std::nullopt;
                }

                if (not edge_forward)
                {
                    std::reverse(states.begin(), states.end());
                    std::reverse(acc.begin(), acc.end());
                }

                qs.resize(states.size());
                qds.resize(states.size());
                times.resize(states.size());
                for (std::size_t k = 0; k < states.size(); ++k)
                {
                    const auto arr = states[k].to_array();
                    std::copy_n(arr.begin(), d, qs[k].begin());
                    std::copy_n(arr.begin() + d, d, qds[k].begin());
                    times[k] = T * k / (states.size() - 1);
                }

                prop.load(qs, qds, acc, times, payload, settings.effort_fraction);
                ++stats.propagations;
                auto out = forward ? prop.forward(in) : prop.backward(in);
                stats.empty += not out.has_value();
                return out;
            };

            const Projection project(settings);
            struct Tree
            {
                std::vector<Configuration> nodes;
                std::vector<std::size_t> parents;
                std::vector<Interval> speeds;
                Grid grid;
                unsigned iteration;
                bool forward;
            };

            std::array<Tree, 2> trees{
                Tree{{}, {}, {}, Grid(project.dim()), 1, true}, Tree{{}, {}, {}, Grid(project.dim()), 1, false}};
            const auto total = [&]() { return trees[0].nodes.size() + trees[1].nodes.size(); };

            const auto add = [&](Tree &t, const Configuration &c, std::size_t parent, const Interval &speed)
                -> std::size_t
            {
                const std::size_t index = t.nodes.size();
                t.nodes.emplace_back(c);
                t.parents.emplace_back((parent == root) ? index : parent);
                t.speeds.emplace_back(speed);
                const Coord coord = project(c);
                Cell *cell = t.grid.getCell(coord);
                if (cell)
                {
                    cell->coverage += 1.0;
                }
                else
                {
                    cell = t.grid.createCell(coord, t.iteration, 1.0 + std::log(static_cast<double>(t.iteration)));
                }

                cell->motion_indices.emplace_back(index);
                t.grid.updateImportance(cell, coord);
                return index;
            };

            add(trees[0], start, root, root_interval(start, false));
            for (const auto &goal : goals)
            {
                add(trees[1], goal, root, root_interval(goal, true));
            }

            std::normal_distribution<float> normal;
            const auto half_normal = [&](std::size_t n) -> std::size_t
            {
                const float v = static_cast<float>(n) * (1.F - std::abs(normal(rng->dist.rng)) / 3.F);
                return (v <= 0.F) ? 0 : std::min(static_cast<std::size_t>(v), n - 1);
            };

            const auto overlaps = [](const Interval &x, const Interval &y)
            { return std::max(x[0], y[0]) <= std::min(x[1], y[1]); };

            std::size_t samples_drawn = 0;
            std::size_t iter = 0;
            std::size_t active = (settings.start_tree_first) ? 0 : 1;
            std::vector<Configuration> chain;
            while (iter++ < settings.max_iterations and total() < settings.max_samples)
            {
                Tree &a = trees[active];
                Tree &b = trees[1 - active];
                active = 1 - active;
                ++a.iteration;

                const auto selected = a.grid.selectCell(settings.border_fraction, rng->dist);
                if (not selected)
                {
                    break;
                }

                Cell *cell = a.grid.getCell(*selected);
                if (cell->score < std::numeric_limits<double>::epsilon())
                {
                    a.grid.rescoreAllCells(a.iteration);
                }

                ++cell->selections;
                const std::size_t from_index = cell->motion_indices[half_normal(cell->motion_indices.size())];
                const Configuration from = a.nodes[from_index];

                std::optional<FloatVector<ZRobot::dimension>> near;
                if (settings.local_targets)
                {
                    near = rng->next_near(from, settings.range);
                }

                auto target = near ? Configuration(*near) : Configuration(rng->next());
                ++samples_drawn;
                if (settings.local_targets)
                {
                    const float dist = ZRobot::distance(from, target);
                    if (dist > settings.range)
                    {
                        target = from + (target - from) * (settings.range / dist);
                    }
                }

                const auto steered =
                    lp.steer(from, target, ZRobot::distance(from, target), settings.range, a.forward, environment);
                bool grew = false;
                std::size_t last = from_index;
                if (steered.status != SteerStatus::Trapped and steered.status != SteerStatus::Rejected)
                {
                    chain.assign(steered.waypoints.begin(), steered.waypoints.end());
                    for (const auto &w : chain)
                    {
                        if (total() >= settings.max_samples)
                        {
                            break;
                        }

                        const auto speed = propagate(a.nodes[last], w, a.forward, a.speeds[last], a.forward);
                        if (not speed)
                        {
                            break;
                        }

                        last = add(a, w, last, *speed);
                        grew = true;
                    }
                }

                if (not grew)
                {
                    cell->score *= settings.failed_expansion_score_factor;
                    a.grid.updateImportance(cell, *selected);
                    continue;
                }

                a.grid.updateImportance(cell, *selected);

                // Connect through the other tree's cell at the new state.
                Cell *other = b.grid.getCell(project(a.nodes[last]));
                if (not other)
                {
                    continue;
                }

                for (std::size_t k = 0; k < settings.connect_attempts; ++k)
                {
                    const std::size_t j = other->motion_indices[rng->dist.template uniform_integer<std::size_t>(
                        0, other->motion_indices.size() - 1)];
                    const Configuration here = a.nodes[last];
                    const Configuration there = b.nodes[j];
                    const auto step = lp.steer(
                        here, there, ZRobot::distance(here, there), std::numeric_limits<float>::max(), a.forward,
                        environment);
                    if (step.status != SteerStatus::Reached)
                    {
                        continue;
                    }

                    // Execution order: start tree -> goal tree. The exact edge attains `there`.
                    const std::size_t fwd_node = a.forward ? last : j;
                    const std::size_t bwd_node = a.forward ? j : last;
                    const Tree &ft = a.forward ? a : b;
                    const Tree &bt = a.forward ? b : a;
                    const auto reach = propagate(here, there, a.forward, ft.speeds[fwd_node], true);
                    if (not reach or not overlaps(*reach, bt.speeds[bwd_node]))
                    {
                        continue;
                    }

                    for (std::size_t cur = fwd_node;; cur = ft.parents[cur])
                    {
                        result.path.emplace_back(ft.nodes[cur]);
                        if (ft.parents[cur] == cur)
                        {
                            break;
                        }
                    }

                    std::reverse(result.path.begin(), result.path.end());

                    // Segments as propagated: start-tree edges charted at the parent (forward), the
                    // connection charted at `here`, goal-tree edges charted at the parent (backward).
                    std::vector<Segment> segments;
                    for (std::size_t i = 0; i + 1 < result.path.size(); ++i)
                    {
                        segments.push_back({result.path[i], result.path[i + 1], true});
                    }

                    segments.push_back({here, there, a.forward});
                    std::size_t goal_root = bwd_node;
                    for (std::size_t cur = bwd_node;; cur = bt.parents[cur])
                    {
                        result.path.emplace_back(bt.nodes[cur]);
                        if (bt.parents[cur] == cur)
                        {
                            goal_root = cur;
                            break;
                        }

                        segments.push_back({bt.nodes[bt.parents[cur]], bt.nodes[cur], false});
                    }

                    if (timed)
                    {
                        *timed = time(
                            segments, root_interval(start, false), root_interval(bt.nodes[goal_root], true), lp,
                            payload, settings.edge_samples, settings.effort_fraction);
                    }

                    result.cost = result.path.cost();
                    result.solved = true;
                    break;
                }

                if (result.solved)
                {
                    break;
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size = {trees[0].nodes.size(), trees[1].nodes.size()};
            result.stats = lp.stats();
            result.stats.samples_drawn = samples_drawn;
            return result;
        }
    };
}  // namespace vamp::planning
