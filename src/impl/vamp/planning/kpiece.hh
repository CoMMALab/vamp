#pragma once

// kpiece.hh — KPIECE1 ported to a VAMP-style compile-time-projection template.
//
// Design decisions vs. OMPL's KPIECE1
// ────────────────────────────────────
// 1. The projection is supplied as a *template parameter* (Projection) instead
//    of a runtime ompl::base::ProjectionEvaluator.  The concept required of
//    Projection is documented below.
// 2. The grid dimension is derived at compile time from
//    Projection::proj_dim  (a compile-time std::size_t constant).
// 3. Grid coordinates are std::array<int, proj_dim> instead of
//    std::vector<int>, eliminating heap allocation per node.
// 4. The planner is a plain struct with a static `solve()` method, mirroring
//    the VAMP RRTC style — no inheritance, no virtual dispatch.
// 5. Memory for motions is managed via a flat pool (std::vector<Motion>),
//    avoiding per-node `new`/`delete`.
// 6. The Grid / Discretization are defined in grid.hh.
// 7. All random number generation goes through rng->dist (vamp::rng::Distribution)
//    so that the planner shares the caller's RNG state and seed.
//
// ─── Projection concept ───────────────────────────────────────────────────────
//
//  struct MyProjection {
//      // Compile-time projection dimensionality.
//      static constexpr std::size_t proj_dim = 2;
//
//      // Map a robot Configuration to a projection-space float coordinate.
//      // Binning into integer grid coordinates happens in the planner using
//      // settings.cell_size.
//      static void project(const Configuration &q,
//                          std::array<float, proj_dim> &coord) noexcept;
//  };
//
// ─── Robot concept (same as VAMP) ─────────────────────────────────────────────
//
//  struct MyRobot {
//      using Configuration = ...;   // must support copy, distance(), interpolate()
//      static constexpr std::size_t dimension = ...;
//  };
//
// ─── Usage ────────────────────────────────────────────────────────────────────
//
//  using Planner = vamp::planning::KPIECE<MyRobot, MyProjection, rake, resolution>;
//  auto result   = Planner::solve(start, goal, env, settings, rng);
//
// ─────────────────────────────────────────────────────────────────────────────

#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/grid.hh>
#include <vamp/planning/kpiece_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{

// ─── KPIECE planner ──────────────────────────────────────────────────────────

template <
    typename Robot,
    typename Projection,
    std::size_t rake,
    std::size_t resolution>
struct KPIECE
{
    using Configuration = typename Robot::Configuration;
    using RNG           = typename vamp::rng::RNG<Robot>;

    static constexpr std::size_t proj_dim = Projection::proj_dim;
    using Coord = std::array<int, proj_dim>;
    using Grid  = detail::Grid<proj_dim>;
    using Cell  = typename Grid::Cell;

    struct Motion
    {
        Configuration state;
        std::size_t   parent{std::size_t(-1)};
    };

    // ── solve() ───────────────────────────────────────────────────────────

    inline static auto solve(
        const Configuration                              &start,
        const Configuration                              &goal,
        const collision::Environment<FloatVector<rake>>  &environment,
        const KPIECESettings                             &settings,
        typename RNG::Ptr                                 rng) noexcept
        -> PlanningResult<Robot>
    {
        return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
    }

    inline static auto solve(
        const Configuration                              &start,
        const std::vector<Configuration>                &goals,
        const collision::Environment<FloatVector<rake>>  &environment,
        const KPIECESettings                             &settings,
        typename RNG::Ptr                                 rng) noexcept
        -> PlanningResult<Robot>
    {
        PlanningResult<Robot> result;
        auto start_time = std::chrono::steady_clock::now();

        // ── Quick direct-connection check ──────────────────────────────
        for (const auto &goal : goals)
        {
            if (validate_motion<Robot, rake, resolution>(start, goal, environment))
            {
                result.path.emplace_back(start);
                result.path.emplace_back(goal);
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                result.iterations  = 0;
                return result;
            }
        }

        // ── Motion pool and grid ───────────────────────────────────────
        std::vector<Motion> pool;
        pool.reserve(settings.max_samples);

        Grid         grid;
        unsigned int iteration = 1;

        auto projectToGrid = [&](const Configuration &q) -> Coord
        {
            std::array<float, proj_dim> proj;
            Projection::project(q, proj);
            Coord coord;
            for (std::size_t i = 0; i < proj_dim; ++i)
                coord[i] = static_cast<int>(std::floor(proj[i] / settings.cell_size));
            return coord;
        };

        auto addMotion = [&](const Configuration &q, std::size_t parent_idx,
                             double dist_to_goal) -> std::size_t
        {
            const std::size_t idx = pool.size();
            pool.push_back({q, parent_idx});

            Coord coord = projectToGrid(q);

            Cell *cell = grid.getCell(coord);
            if (cell)
            {
                cell->motion_indices.push_back(idx);
                cell->coverage += 1.0;
            }
            else
            {
                const double init_score =
                    (1.0 + std::log(static_cast<double>(iteration))) /
                    (1.0 + dist_to_goal);
                cell = grid.createCell(coord, iteration, init_score);
                cell->motion_indices.push_back(idx);
            }
            grid.updateImportance(cell, coord);
            return idx;
        };

        addMotion(start, std::size_t(-1), std::numeric_limits<double>::infinity());

        // ── Half-normal motion selection ───────────────────────────────
        // Mirrors OMPL's rng_.halfNormalInt(0, n-1, focus=3.0):
        // samples |N(0,1)| * n / focus, clamped to [0, n).
        // This biases toward index 0 (the oldest motion in the cell).
        std::normal_distribution<float> nd;  // N(0, 1)

        auto halfNormalIndex = [&](std::size_t n) -> std::size_t
        {
            if (n == 1)
                return 0;
            constexpr float focus = 3.0F;
            const float     raw   = std::abs(nd(rng->dist.rng));
            const auto      idx   = static_cast<std::size_t>(
                std::floor(raw * static_cast<float>(n) / focus));
            return std::min(idx, n - 1);
        };

        auto pickMotionIdx = [&](const Cell *cell) -> std::size_t
        {
            return cell->motion_indices[halfNormalIndex(cell->motion_indices.size())];
        };

        // ── Goal sampling helpers ──────────────────────────────────────
        std::size_t goal_rr = 0;
        auto sampleGoal = [&]() -> const Configuration &
        {
            return goals[(goal_rr++) % goals.size()];
        };

        auto goalDist = [&](const Configuration &q) -> float
        {
            float best = std::numeric_limits<float>::infinity();
            for (const auto &g : goals)
                best = std::min(best, q.distance(g));
            return best;
        };

        // ── Book-keeping ───────────────────────────────────────────────
        std::size_t solution_idx   = std::size_t(-1);
        std::size_t approx_sol_idx = std::size_t(-1);
        float       approx_dif     = std::numeric_limits<float>::infinity();

        // ── Main loop ─────────────────────────────────────────────────
        for (std::size_t iter = 0;
             iter < settings.max_iterations && pool.size() < settings.max_samples;
             ++iter)
        {
            ++iteration;

            // 1. Select cell and motion.
            auto maybe_coord = grid.selectCell(settings.border_fraction, rng->dist);
            if (!maybe_coord)
                break;

            const Coord ecell_coord = *maybe_coord;
            Cell       *ecell       = grid.getCell(ecell_coord);
            assert(ecell && !ecell->motion_indices.empty());

            if (ecell->score < std::numeric_limits<double>::epsilon())
                grid.rescoreAllCells(iteration);

            ++ecell->selections;

            const std::size_t existing_idx = pickMotionIdx(ecell);
            const Motion     &existing     = pool[existing_idx];

            // 2. Sample target state (goal-biased).
            Configuration xstate =
                (rng->dist.uniform_01() < settings.goal_bias)
                    ? sampleGoal()
                    : rng->next();

            // 3. Steer.
            const float d        = existing.state.distance(xstate);
            const bool  is_reach = (d <= settings.range);
            if (!is_reach)
                xstate = existing.state.interpolate(xstate, settings.range / d);

            // 4. Validate (with partial fallback).
            Configuration last_valid;
            bool keep = validate_motion<Robot, rake, resolution>(
                existing.state, xstate, environment);

            if (!keep)
            {
                const float frac = validate_motion_partial<Robot, rake, resolution>(
                    existing.state, xstate, environment, last_valid);
                if (frac >= settings.min_valid_path_fraction)
                {
                    xstate = last_valid;
                    keep   = true;
                }
            }

            if (keep)
            {
                const float       dist_goal = goalDist(xstate);
                const std::size_t new_idx   =
                    addMotion(xstate, existing_idx, static_cast<double>(dist_goal));

                // Check goal satisfaction.
                bool solved = false;
                for (const auto &goal : goals)
                {
                    if (xstate.distance(goal) < 1e-4F)
                    {
                        solved       = true;
                        solution_idx = new_idx;
                        break;
                    }
                }
                if (solved)
                    break;

                if (dist_goal < approx_dif)
                {
                    approx_dif     = dist_goal;
                    approx_sol_idx = new_idx;
                }
            }
            else
            {
                ecell->score *= settings.failed_expansion_score_factor;
            }
            grid.updateImportance(ecell, ecell_coord);
        }

        // ── Reconstruct path ──────────────────────────────────────────
        const bool        approximate = (solution_idx == std::size_t(-1));
        const std::size_t trace       = approximate ? approx_sol_idx : solution_idx;

        if (trace != std::size_t(-1))
        {
            std::vector<std::size_t> indices;
            for (std::size_t cur = trace; cur != std::size_t(-1); cur = pool[cur].parent)
                indices.push_back(cur);
            std::reverse(indices.begin(), indices.end());
            for (std::size_t idx : indices)
                result.path.emplace_back(pool[idx].state);
        }

        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        result.iterations  = iteration;
        return result;
    }
};

}  // namespace vamp::planning