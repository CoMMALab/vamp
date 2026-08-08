#pragma once

#include <limits>

#include <vamp/collision/environment.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    // Interpolate the path to n states; for projecting local planners, every interpolated
    // state is projected back onto the constraint manifold and every resulting edge is
    // revalidated (projection bends states off their validated chords, so the new edges
    // do not inherit validity), reverting on any failure.
    template <typename Robot, std::size_t rake, typename LocalPlanner>
    inline static auto interpolate_and_project(
        Path<Robot> &path,
        std::size_t n,
        const collision::Environment<FloatVector<rake>> &environment,
        const LocalPlanner &lp) -> bool
    {
        if constexpr (LocalPlanner::projecting)
        {
            auto backup = path;
            path.interpolate_to_n_states(n);

            bool admitted = true;
            for (auto &state : path)
            {
                if (not lp.project(state))
                {
                    admitted = false;
                    break;
                }
            }

            for (auto i = 0U; admitted and i + 1 < path.size(); ++i)
            {
                admitted = lp.validate(path[i], path[i + 1], environment);
            }

            if (not admitted)
            {
                path = std::move(backup);
                return false;
            }

            return true;
        }
        else
        {
            (void)environment;
            path.interpolate_to_n_states(n);
            return true;
        }
    }

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
        // interpolation is a straight line and midpoints stay on it: projection bends
        // midpoints off their validated chords, and non-euclidean robots (flask)
        // re-solve each half edge as its own steering problem (own T*), so the halves
        // are different curves than the validated whole.
        constexpr bool preserves_edges = Robot::euclidean and not LocalPlanner::projecting;

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
            // Bail out (reverting this step's subdivision) unless every midpoint
            // projects and every new edge revalidates.
            if constexpr (not preserves_edges)
            {
                auto backup = path;
                path.subdivide();

                bool admitted = true;
                for (auto index = 1U; admitted and index < path.size(); index += 2)
                {
                    admitted = lp.project(path[index]);
                }

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
                    Robot::interpolate(path[index], path[index - 1], settings.midpoint_interpolation);
                const auto temp_2 =
                    Robot::interpolate(path[index], path[index + 1], settings.midpoint_interpolation);
                auto midpoint = Robot::interpolate(temp_1, temp_2, 0.5);

                if (lp.project(midpoint) and Robot::distance(path[index], midpoint) > settings.min_change and
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
            // projected chains that cost more than the segment despite having fewer states.
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
        // by >= 1 or exits: shortcuts whose emitted waypoint chain (rake - 1 interior
        // waypoints for projecting local planners) does not shrink the segment are
        // rejected, since accepting them would grow the path and re-shortcut its own
        // output forever.
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
                    // The cost budget rejects projected chains that cost more than the
                    // segment despite having fewer states.
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

    // Velocity-only vertex re-optimization for flask robots.
    //
    // For each interior waypoint z_i with fixed q_i, re-choose v_i to (approximately) minimize
    // C_loc(z_{i-1} -> z_i) + C_loc(z_i -> z_{i+1}) via backtracking gradient descent.
    // Analytical gradients come from Robot::cost_grad (envelope theorem on T*).
    //
    // Changing v_i does NOT preserve validity: the two adjacent cubics reshape even though q_i
    // is fixed, so intermediate collisions can appear. Each accepted trial (a) clamps v_i to
    // the joint velocity limits, (b) for projecting local planners is projected back onto the
    // manifold (tangent-projecting its velocity), and (c) revalidates both incident edges.
    // No-op for robots without Robot::cost_grad.
    template <typename Robot, std::size_t rake, std::size_t resolution, typename LocalPlanner>
    inline static auto velopt_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const VeloptSettings &settings,
        const LocalPlanner &lp) -> bool
    {
        if constexpr (not has_cost_grad_v<Robot>)
        {
            (void)path;
            (void)environment;
            (void)settings;
            (void)lp;
            return false;
        }
        else
        {
            if (path.size() < 3)
            {
                return false;
            }

            constexpr std::size_t n = Robot::flat_dimension;
            constexpr std::size_t dim = 2 * n;
            using Configuration = typename Robot::Configuration;

            bool any_changed = false;

            for (std::size_t step = 0; step < settings.max_steps; ++step)
            {
                float total_before = 0.F;
                float total_after = 0.F;

                for (std::size_t i = 1; i + 1 < path.size(); ++i)
                {
                    const auto &prev = path[i - 1];
                    const auto &next = path[i + 1];
                    const Configuration cur = path[i];

                    const auto g_left = Robot::cost_grad(prev, cur);
                    const auto g_right = Robot::cost_grad(cur, next);
                    const float c_current = g_left.cost + g_right.cost;
                    total_before += c_current;

                    std::array<float, n> grad_v{};
                    float grad_norm2 = 0.F;
                    for (std::size_t j = 0; j < n; ++j)
                    {
                        grad_v[j] = g_left.grad_b[n + j] + g_right.grad_a[n + j];
                        grad_norm2 += grad_v[j] * grad_v[j];
                    }
                    if (grad_norm2 < 1e-16F)
                    {
                        total_after += c_current;
                        continue;
                    }

                    alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded> buf{};
                    const auto cur_arr = cur.to_array();
                    for (std::size_t j = 0; j < dim; ++j)
                    {
                        buf[j] = cur_arr[j];
                    }

                    float best_cost = c_current;
                    Configuration best = cur;
                    float alpha = settings.initial_step;
                    for (std::size_t ls = 0; ls < settings.line_search_max; ++ls)
                    {
                        for (std::size_t j = 0; j < n; ++j)
                        {
                            const float v_new = cur_arr[n + j] - alpha * grad_v[j];
                            const float v_lim = Robot::velocity_limits[j];
                            buf[n + j] = std::clamp(v_new, -v_lim, v_lim);
                        }
                        Configuration trial(buf.data());
                        if constexpr (LocalPlanner::projecting)
                        {
                            // Raw gradient steps have velocity components normal to the
                            // constraint manifold, and chart edges tangent-project their
                            // arrival velocity — an unprojected trial can never be attained,
                            // so validation would reject every step.
                            if (not lp.project(trial))
                            {
                                alpha *= 0.5F;
                                continue;
                            }
                        }
                        const float trial_cost =
                            planning::cost<Robot>(prev, trial) + planning::cost<Robot>(trial, next);
                        if (trial_cost < best_cost and lp.validate(prev, trial, environment) and
                            lp.validate(trial, next, environment))
                        {
                            best_cost = trial_cost;
                            best = trial;
                            break;
                        }
                        alpha *= 0.5F;
                    }

                    total_after += best_cost;
                    if (best_cost < c_current)
                    {
                        path[i] = best;
                        any_changed = true;
                    }
                }

                if (total_before <= 0.F)
                {
                    break;
                }
                const float improvement = (total_before - total_after) / total_before;
                if (improvement < settings.min_improvement)
                {
                    break;
                }
            }

            return any_changed;
        }
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
                auto new_state = Robot::interpolate(perturb_state, perturbation, settings.range);
                if (not lp.project(new_state))
                {
                    continue;
                }

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
                case INTERP:
                    return interpolate_and_project<Robot>(result.path, settings.interpolate, environment, lp);
                case VELOPT:
                    return velopt_path<Robot, rake, resolution>(
                        result.path, environment, settings.velopt, lp);
            }

            return false;
        };

        // Check if the direct local path from start to goal is valid; the cost budget
        // rejects a direct projected chain that costs more than the input path.
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
            interpolate_and_project<Robot>(result.path, settings.interpolate, environment, lp);
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
