#pragma once

#include <limits>

#include <vamp/collision/environment.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/traits.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot>
    inline static auto segment_cost(const Path<Robot> &path, std::size_t from, std::size_t to) -> float
    {
        float total = 0.F;
        for (auto i = from; i < to; ++i)
        {
            total += cost<Robot>(path[i], path[i + 1]);
        }

        return total;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename LocalPlanner>
    inline static auto smooth_bspline(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const BSplineSettings &settings,
        const LocalPlanner &lp) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        // Subdivided edges inherit validity from the edge they split only when
        // interpolation is a straight line.
        constexpr bool preserves_edges = robots::is_euclidean_v<Robot>;

        // Subdivision can also lengthen the path even when every later midpoint move is
        // cost-nonincreasing, so snapshot the input and revert the whole pass if it
        // does not pay off.
        [[maybe_unused]] Path<Robot> original;
        [[maybe_unused]] float original_cost = 0.F;
        if constexpr (not preserves_edges)
        {
            original = path;
            original_cost = path.cost();
        }

        bool changed = false;
        for (auto step = 0U; step < settings.max_steps; ++step)
        {
            // Bail out (reverting this step's subdivision) unless every new edge
            // revalidates.
            if constexpr (not preserves_edges)
            {
                auto backup = path;
                path.subdivide();

                bool admitted = true;
                for (auto index = 1U; admitted and index < path.size(); index += 2)
                {
                    admitted = lp.validate(path[index - 1], path[index], environment) and
                               lp.validate(path[index], path[index + 1], environment);
                }

                if (not admitted)
                {
                    path = std::move(backup);
                    break;
                }
            }
            else
            {
                path.subdivide();
            }

            bool updated = false;
            for (auto index = 2U; index < path.size() - 1; index += 2)
            {
                const auto temp_1 =
                    robots::interpolate<Robot>(path[index], path[index - 1], settings.midpoint_interpolation);
                const auto temp_2 =
                    robots::interpolate<Robot>(path[index], path[index + 1], settings.midpoint_interpolation);
                auto midpoint = robots::interpolate<Robot>(temp_1, temp_2, 0.5);

                if (robots::distance<Robot>(path[index], midpoint) > settings.min_change and
                    cost_nonincreasing<Robot>(path[index - 1], path[index], midpoint, path[index + 1]) and
                    lp.validate(path[index - 1], midpoint, environment) and
                    lp.validate(midpoint, path[index + 1], environment))
                {
                    path[index] = midpoint;
                    changed |= (updated = true);
                }
            }

            if (not updated)
            {
                break;
            }
        }

        if constexpr (not preserves_edges)
        {
            if (path.cost() > original_cost)
            {
                path = std::move(original);
                return false;
            }
        }

        return changed;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename LocalPlanner>
    inline static auto reduce_path_vertices(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ReduceSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng,
        const LocalPlanner &lp) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
        const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() : settings.max_empty_steps;

        bool result = false;
        for (auto i = 0U, no_change = 0U; i < max_steps and no_change < max_empty_steps; ++i, ++no_change)
        {
            int initial_size = path.size();
            int max_n = initial_size - 1;

            int range = 1 + static_cast<int>(
                                std::floor(0.5F + static_cast<float>(initial_size) * settings.range_ratio));

            auto point_0 = rng->dist.uniform_integer(0, max_n);
            auto point_1 =
                rng->dist.uniform_integer(std::max(point_0 - range, 0), std::min(max_n, point_0 + range));

            if (std::abs(point_0 - point_1) < 2)
            {
                if (point_0 < max_n - 1)
                {
                    point_1 = point_0 + 2;
                }
                else if (point_0 > 1)
                {
                    point_1 = point_0 - 2;
                }
                else
                {
                    continue;
                }
            }

            if (point_0 > point_1)
            {
                std::swap(point_0, point_1);
            }

            // Reject non-shrinking chains (see shortcut_path): every success resets
            // no_change, so accepting growth would loop forever. The cost budget rejects
            // chains that cost more than the segment despite having fewer states.
            const auto extension = lp.connect_within(
                path[point_0],
                path[point_1],
                environment,
                [&] { return segment_cost<Robot>(path, point_0, point_1); },
                static_cast<std::size_t>(point_1 - point_0 - 1));

            if (extension.status == SteerStatus::Reached)
            {
                path.erase(path.begin() + point_0 + 1, path.begin() + point_1);
                path.insert(
                    path.begin() + point_0 + 1, extension.waypoints.begin(), extension.waypoints.end());
                no_change = 0;
                result = true;
            }
        }

        return result;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename LocalPlanner>
    inline static auto shortcut_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ShortcutSettings & /*settings*/,
        const LocalPlanner &lp) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        // Deterministic longest-first shortcutting: rank the current path edges by
        // planning::cost<Robot> and try to collapse the widest valid skip starting from the
        // most expensive edge. After each successful shortcut the ranking changes, so we
        // re-score and re-sort. Terminates because every iteration either shrinks the path
        // by >= 1 or exits: shortcuts whose emitted waypoint chain does not shrink the
        // segment are rejected, since accepting them would grow the path and re-shortcut
        // its own output forever.
        bool result = false;
        while (path.size() >= 3)
        {
            std::vector<std::pair<float, std::size_t>> ranked;
            ranked.reserve(path.size() - 1);
            for (std::size_t i = 0; i + 1 < path.size(); ++i)
            {
                ranked.emplace_back(planning::cost<Robot>(path[i], path[i + 1]), i);
            }
            std::sort(
                ranked.begin(), ranked.end(), [](const auto &a, const auto &b) { return a.first > b.first; });

            bool did_shortcut = false;
            for (const auto &entry : ranked)
            {
                const std::size_t i = entry.second;
                for (auto j = path.size() - 1; j > i + 1; --j)
                {
                    // The cost budget rejects chains that cost more than the segment
                    // despite having fewer states.
                    const auto extension = lp.connect_within(
                        path[i],
                        path[j],
                        environment,
                        [&] { return segment_cost<Robot>(path, i, j); },
                        j - i - 1);

                    if (extension.status == SteerStatus::Reached)
                    {
                        path.erase(path.begin() + i + 1, path.begin() + j);
                        path.insert(
                            path.begin() + i + 1, extension.waypoints.begin(), extension.waypoints.end());
                        result = true;
                        did_shortcut = true;
                        break;
                    }
                }
                if (did_shortcut)
                {
                    break;
                }
            }
            if (not did_shortcut)
            {
                break;
            }
        }

        return result;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename LocalPlanner>
    inline static auto perturb_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const PerturbSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng,
        const LocalPlanner &lp) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
        const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() : settings.max_empty_steps;

        bool changed = false;
        for (auto step = 0U, no_change = 0U; step < max_steps and no_change < max_empty_steps;
             ++step, ++no_change)
        {
            auto to_perturb_idx = rng->dist.uniform_integer(1UL, path.size() - 2);
            auto perturb_state = path[to_perturb_idx];
            auto before_state = path[to_perturb_idx - 1];
            auto after_state = path[to_perturb_idx + 1];

            float old_cost =
                cost<Robot>(before_state, perturb_state) + cost<Robot>(perturb_state, after_state);

            for (auto attempt = 0U; attempt < settings.perturbation_attempts; ++attempt)
            {
                // next() already returns a joint-space configuration; do not rescale it.
                const auto perturbation = rng->next();
                const auto new_state =
                    robots::interpolate<Robot>(perturb_state, perturbation, settings.range);

                float new_cost = cost<Robot>(before_state, new_state) + cost<Robot>(new_state, after_state);

                if (new_cost < old_cost and lp.validate(before_state, new_state, environment) and
                    lp.validate(new_state, after_state, environment))
                {
                    no_change = 0;
                    changed = true;
                    path[to_perturb_idx] = new_state;
                    break;
                }
            }
        }

        return changed;
    }

    template <
        typename Robot,
        std::size_t rake,
        std::size_t resolution,
        typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
    inline auto simplify(
        const Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const SimplifySettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng,
        const LocalPlanner &lp = LocalPlanner()) -> PlanningResult<Robot>
    {
        auto start_time = std::chrono::steady_clock::now();

        PlanningResult<Robot> result;

        const auto run = [&](SimplifyRoutine op) -> bool
        {
            switch (op)
            {
                case BSPLINE:
                    return smooth_bspline<Robot, rake, resolution>(
                        result.path, environment, settings.bspline, lp);
                case REDUCE:
                    return reduce_path_vertices<Robot, rake, resolution>(
                        result.path, environment, settings.reduce, rng, lp);
                case SHORTCUT:
                    return shortcut_path<Robot, rake, resolution>(
                        result.path, environment, settings.shortcut, lp);
                case PERTURB:
                    return perturb_path<Robot, rake, resolution>(
                        result.path, environment, settings.perturb, rng, lp);
            }

            return false;
        };

        // Check if the direct local path from start to goal is valid; the cost budget
        // rejects a direct chain that costs more than the input path.
        if (path.size() > 2)
        {
            const auto direct = lp.connect_within(
                path.front(),
                path.back(),
                environment,
                [&] { return path.cost(); },
                std::numeric_limits<std::size_t>::max());

            if (direct.status == SteerStatus::Reached)
            {
                result.path.emplace_back(path.front());
                for (const auto &c : direct.waypoints)
                {
                    result.path.emplace_back(c);
                }
                result.path.emplace_back(path.back());
                result.solved = true;
                result.cost = result.path.cost();
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                return result;
            }
        }
        else if (path.size() == 2)
        {
            result.path = path;
            result.solved = true;
            result.cost = result.path.cost();
            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            return result;
        }

        result.path = path;

        if (settings.interpolate)
        {
            result.path.interpolate_to_n_states(settings.interpolate);
        }

        if (path.size() > 2)
        {
            for (auto i = 0U; i < settings.max_iterations; ++i)
            {
                result.iterations++;

                bool any = false;
                for (const auto &op : settings.operations)
                {
                    any |= run(op);
                }

                if (not any)
                {
                    break;
                }
            }
        }

        result.solved = true;
        result.cost = result.path.cost();
        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        return result;
    }
}  // namespace vamp::planning
