#pragma once

// bkpiece.hh — Bidirectional KPIECE (BKPIECE1) ported to a VAMP-style
// compile-time-projection template.
//
// This file includes the shared infrastructure from separate headers:
//   . detail::Grid<D>       — hash-map grid with lazy importance heaps (grid.hh)
//   . KPIECESettings        — all tuning parameters (goal_bias is unused here) (kpiece_settings.hh)
//
// All random number generation uses rng->dist (vamp::rng::Distribution) so
// that both trees share the caller's RNG state and seed.
//
// See kpiece.hh for Projection / Robot concept documentation and usage.

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <limits>
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

template <
    typename Robot,
    typename Projection,
    std::size_t rake,
    std::size_t resolution>
struct BKPIECE
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

    // ── TreeContext ───────────────────────────────────────────────────────
    // Bundles all mutable per-tree state.  The two trees are symmetric and
    // the main loop operates on them through active/passive references.
    struct TreeContext
    {
        std::vector<Motion> pool;
        Grid                grid;
        unsigned int        iteration{1};

        explicit TreeContext(std::size_t reserve_size)
        {
            pool.reserve(reserve_size);
        }

        // Project `q` to float coords, bin to grid integer coords,
        // add to the pool, insert into the grid, update importance.
        // Returns the new motion's pool index.
        Coord projectToGrid(const Configuration &q, float cell_size) const
        {
            std::array<float, proj_dim> proj;
            Projection::project(q, proj);
            Coord coord;
            for (std::size_t i = 0; i < proj_dim; ++i)
                coord[i] = static_cast<int>(std::floor(proj[i] / cell_size));
            return coord;
        }

        std::size_t addMotion(
            const Configuration &q,
            std::size_t          parent_idx,
            float                cell_size,
            double               dist_to_goal = 0.0)
        {
            const std::size_t idx = pool.size();
            pool.push_back({q, parent_idx});

            Coord coord = projectToGrid(q, cell_size);

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
        }

        // Select a cell using the border/internal importance heaps.
        // `dist` is the caller's Distribution (rng->dist); no RNG state is
        // stored inside TreeContext or Grid.
        std::optional<std::pair<Coord, Cell *>>
        selectCell(float border_fraction, vamp::rng::Distribution &dist)
        {
            auto maybe = grid.selectCell(border_fraction, dist);
            if (!maybe)
                return std::nullopt;
            Cell *cell = grid.getCell(*maybe);
            assert(cell && !cell->motion_indices.empty());
            return std::make_pair(*maybe, cell);
        }

        std::size_t size() const noexcept { return pool.size(); }
        bool        empty() const noexcept { return pool.empty(); }
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
                result.size.emplace_back(1);
                result.size.emplace_back(1);
                return result;
            }
        }

        // ── Tree initialisation ────────────────────────────────────────
        const std::size_t half_budget = settings.max_samples / 2;
        TreeContext start_ctx(half_budget);
        TreeContext goal_ctx(half_budget);

        start_ctx.addMotion(start, std::size_t(-1), settings.cell_size);
        for (const auto &g : goals)
            goal_ctx.addMotion(g, std::size_t(-1), settings.cell_size);

        // ── Half-normal motion selection ───────────────────────────────
        // Mirrors OMPL's rng_.halfNormalInt(0, n-1, focus=3.0):
        // samples |N(0,1)| * n / focus, clamped to [0, n).
        // Biases toward index 0 (the oldest motion in the cell).
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

        // ── Path reconstruction helper ─────────────────────────────────
        // Returns the chain from the root to `idx` in root-first order.
        auto traceBack = [](const std::vector<Motion> &pool_ref,
                            std::size_t                idx)
            -> std::vector<std::size_t>
        {
            std::vector<std::size_t> chain;
            for (std::size_t cur = idx; cur != std::size_t(-1); cur = pool_ref[cur].parent)
                chain.push_back(cur);
            std::reverse(chain.begin(), chain.end());
            return chain;  // [root, ..., idx]
        };

        // ── Main loop ─────────────────────────────────────────────────
        // `extending_start` tracks which tree is active THIS iteration
        // (before the flip used for path reconstruction at connection time).
        bool extending_start = true;

        for (std::size_t iter = 0;
             iter < settings.max_iterations &&
             start_ctx.size() + goal_ctx.size() < settings.max_samples;
             ++iter)
        {
            // 1. Select active and passive trees; flip for next iteration.
            TreeContext &active  = extending_start ? start_ctx : goal_ctx;
            TreeContext &passive = extending_start ? goal_ctx  : start_ctx;
            extending_start = !extending_start;

            ++active.iteration;

            // 2. Select cell and motion in the active tree.
            auto maybe_cell = active.selectCell(settings.border_fraction, rng->dist);
            if (!maybe_cell)
                break;

            auto &[ecell_coord, ecell] = *maybe_cell;
            ++ecell->selections;

            const std::size_t existing_idx = pickMotionIdx(ecell);
            const Motion     &existing     = active.pool[existing_idx];

            // 3. Sample a random target state (no goal bias in BKPIECE).
            Configuration xstate = rng->next();

            // 4. Steer.
            const float d        = existing.state.distance(xstate);
            const bool  is_reach = (d <= settings.range);
            if (!is_reach)
                xstate = existing.state.interpolate(xstate, settings.range / d);

            // 5. Validate (with partial fallback).
            Configuration last_valid;
            bool keep = validate_motion<Robot, rake, resolution>(
                existing.state, xstate, environment);

            if (!keep and settings.min_valid_path_fraction < 1.0)
            {
                const float frac = validate_motion_partial<Robot, rake, resolution>(
                    existing.state, xstate, environment, last_valid);
                if (frac >= settings.min_valid_path_fraction)
                {
                    xstate = last_valid;
                    keep   = true;
                }
            }

            if (!keep)
            {
                ecell->score *= settings.failed_expansion_score_factor;
                active.grid.updateImportance(ecell, ecell_coord);
                continue;
            }

            // 6. Add new motion to active tree.
            const std::size_t new_idx = active.addMotion(xstate, existing_idx, settings.cell_size);

            // 7. Connection attempt: check if the passive tree has a cell at
            //    the same grid coordinate as the new state.
            Coord new_coord = active.projectToGrid(xstate, settings.cell_size);

            Cell *passive_cell = passive.grid.getCell(new_coord);
            if (passive_cell && !passive_cell->motion_indices.empty())
            {
                // Pick a random motion from the matching cell using uniform
                // integer distribution — the connection target is chosen
                // uniformly, not by importance (matching OMPL's uniformInt).
                const std::size_t n_passive  = passive_cell->motion_indices.size();
                const std::size_t connect_pool_idx =
                    passive_cell->motion_indices[
                        rng->dist.uniform_integer(std::size_t(0), n_passive - 1)];

                const Motion &connect_motion = passive.pool[connect_pool_idx];

                // Full motion check — connections are all-or-nothing.
                if (validate_motion<Robot, rake, resolution>(
                        xstate, connect_motion.state, environment))
                {
                    // 8. Reconstruct path.
                    //
                    // At this point `extending_start` has already been flipped
                    // for the next iteration, so:
                    //   extending_start == true  → active was goal_ctx
                    //   extending_start == false → active was start_ctx
                    const bool active_is_start = !extending_start;

                    auto active_chain  = traceBack(active.pool,  new_idx);
                    auto passive_chain = traceBack(passive.pool, connect_pool_idx);

                    // Route to canonical (start-first) order.
                    const auto &start_chain  = active_is_start ? active_chain  : passive_chain;
                    const auto &goal_chain   = active_is_start ? passive_chain : active_chain;
                    const auto &start_pool_r = active_is_start ? active.pool   : passive.pool;
                    const auto &goal_pool_r  = active_is_start ? passive.pool  : active.pool;

                    // start_chain: [start_root, ..., start_side_of_connection]
                    for (std::size_t idx : start_chain)
                        result.path.emplace_back(start_pool_r[idx].state);

                    // goal_chain:  [goal_root, ..., goal_side_of_connection]
                    // Reverse so the connection point is adjacent to the start chain.
                    for (int i = static_cast<int>(goal_chain.size()) - 1; i >= 0; --i)
                        result.path.emplace_back(goal_pool_r[goal_chain[i]].state);

                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations  = iter;
                    result.size.emplace_back(start_ctx.size());
                    result.size.emplace_back(goal_ctx.size());
                    return result;
                }
                // Connection attempt failed — do NOT penalise ecell;
                // the extension itself succeeded.
            }
        }

        // BKPIECE does not produce approximate solutions.
        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        result.iterations  = settings.max_iterations;
        result.size.emplace_back(start_ctx.size());
        result.size.emplace_back(goal_ctx.size());
        return result;
    }
};

}  // namespace vamp::planning