#pragma once

#include <map>

#include <vamp/collision/environment.hh>
#include <vamp/planning/cost.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto smooth_bspline(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const BSplineSettings &settings) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        bool changed = false;
        for (auto step = 0U; step < settings.max_steps; ++step)
        {
            path.subdivide();

            bool updated = false;
            for (auto index = 2U; index < path.size() - 1; index += 2)
            {
                const auto temp_1 = Robot::interpolate(path[index], path[index - 1], settings.midpoint_interpolation);
                const auto temp_2 = Robot::interpolate(path[index], path[index + 1], settings.midpoint_interpolation);
                const auto midpoint = Robot::interpolate(temp_1, temp_2, 0.5);

                if (Robot::distance(path[index], midpoint) > settings.min_change and
                    cost_nonincreasing<Robot>(path[index - 1], path[index], midpoint, path[index + 1]) and
                    validate_motion<Robot, rake, resolution>(path[index - 1], midpoint, environment) and
                    validate_motion<Robot, rake, resolution>(midpoint, path[index + 1], environment))
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

        return changed;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto reduce_path_vertices(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ReduceSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
        const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() : settings.max_empty_steps;

        bool result = false;
        for (auto i = 0U, no_change = 0U; i < max_steps or no_change < max_empty_steps; ++i, ++no_change)
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

            if (validate_motion<Robot, rake, resolution>(path[point_0], path[point_1], environment))
            {
                path.erase(path.begin() + point_0 + 1, path.begin() + point_1);
                no_change = 0;
                result = true;
            }
        }

        return result;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto shortcut_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const ShortcutSettings & /*settings*/) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        // Deterministic longest-first shortcutting: rank the current path edges by
        // planning::cost<Robot> and try to collapse the widest valid skip starting from the
        // most expensive edge. After each successful shortcut the ranking changes, so we
        // re-score and re-sort. Terminates because every iteration either shrinks the path
        // by >= 1 or exits.
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
                ranked.begin(),
                ranked.end(),
                [](const auto &a, const auto &b) { return a.first > b.first; });

            bool did_shortcut = false;
            for (const auto &entry : ranked)
            {
                const std::size_t i = entry.second;
                for (auto j = path.size() - 1; j > i + 1; --j)
                {
                    if (validate_motion<Robot, rake, resolution>(path[i], path[j], environment))
                    {
                        path.erase(path.begin() + i + 1, path.begin() + j);
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
    // the joint velocity limits and (b) revalidates both incident edges. No-op for robots
    // without Robot::cost_grad.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto velopt_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const VeloptSettings &settings) -> bool
    {
        if constexpr (not has_cost_grad_v<Robot>)
        {
            (void)path;
            (void)environment;
            (void)settings;
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

                    alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded>
                        buf{};
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
                        const float trial_cost =
                            planning::cost<Robot>(prev, trial) + planning::cost<Robot>(trial, next);
                        if (trial_cost < best_cost and
                            validate_motion<Robot, rake, resolution>(prev, trial, environment) and
                            validate_motion<Robot, rake, resolution>(trial, next, environment))
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

    // Trajectory polish (iLQR-flavored): simultaneous steepest-descent update of all interior
    // waypoints in (q, v) using analytical C_loc gradients from Robot::cost_grad. Each iteration
    //   1. Computes edge grads once (2n-dim per endpoint, envelope-theorem exact at T*).
    //   2. Sums per-interior-vertex grads = grad_b(edge left) + grad_a(edge right).
    //   3. Backtracks alpha over a full-path candidate that clamps every waypoint to joint /
    //      velocity limits, checks total-cost improvement, then revalidates every edge.
    // Unlike VELOPT this moves q as well, so the two adjacent cubics reshape entirely; the
    // whole-path revalidation is what buys correctness. No-op for robots without cost_grad.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto polish_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const PolishSettings &settings) -> bool
    {
        if constexpr (not has_cost_grad_v<Robot>)
        {
            (void)path;
            (void)environment;
            (void)settings;
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

            bool changed = false;

            for (std::size_t step = 0; step < settings.max_steps; ++step)
            {
                const std::size_t N = path.size();
                if (N < 3)
                {
                    break;
                }

                // Edge grads and current cost
                using EdgeGrad = decltype(Robot::cost_grad(path[0], path[1]));
                std::vector<EdgeGrad> edge_grads;
                edge_grads.reserve(N - 1);
                float total_cost = 0.F;
                for (std::size_t i = 0; i + 1 < N; ++i)
                {
                    edge_grads.push_back(Robot::cost_grad(path[i], path[i + 1]));
                    total_cost += edge_grads.back().cost;
                }

                // Per-interior-vertex grad (2n each)
                std::vector<std::array<float, dim>> gradv(N);
                float grad_norm2 = 0.F;
                for (std::size_t i = 1; i + 1 < N; ++i)
                {
                    for (std::size_t j = 0; j < dim; ++j)
                    {
                        const float gi =
                            edge_grads[i - 1].grad_b[j] + edge_grads[i].grad_a[j];
                        gradv[i][j] = gi;
                        grad_norm2 += gi * gi;
                    }
                }
                if (grad_norm2 < 1e-16F)
                {
                    break;
                }

                // Backtracking line search over the whole path
                float alpha = settings.initial_step;
                bool accepted = false;
                Path<Robot> trial;
                trial.reserve(N);
                for (std::size_t ls = 0; ls < settings.line_search_max; ++ls)
                {
                    trial.clear();
                    trial.emplace_back(path.front());
                    for (std::size_t i = 1; i + 1 < N; ++i)
                    {
                        alignas(FloatVectorAlignment) std::array<
                            float, Configuration::num_scalars_rounded>
                            buf{};
                        const auto cur_arr = path[i].to_array();
                        for (std::size_t j = 0; j < dim; ++j)
                        {
                            buf[j] = cur_arr[j] - alpha * gradv[i][j];
                        }
                        Configuration z(buf.data());
                        Robot::descale_configuration(z);
                        z = z.clamp(0.F, 1.F);
                        Robot::scale_configuration(z);
                        trial.emplace_back(z);
                    }
                    trial.emplace_back(path.back());

                    float trial_cost = 0.F;
                    for (std::size_t i = 0; i + 1 < N; ++i)
                    {
                        trial_cost += planning::cost<Robot>(trial[i], trial[i + 1]);
                    }

                    if (trial_cost < total_cost)
                    {
                        bool all_valid = true;
                        for (std::size_t i = 0; i + 1 < N; ++i)
                        {
                            if (not validate_motion<Robot, rake, resolution>(
                                    trial[i], trial[i + 1], environment))
                            {
                                all_valid = false;
                                break;
                            }
                        }
                        if (all_valid)
                        {
                            const float improvement = (total_cost - trial_cost) / total_cost;
                            for (std::size_t i = 1; i + 1 < N; ++i)
                            {
                                path[i] = trial[i];
                            }
                            changed = true;
                            accepted = true;
                            if (improvement < settings.min_improvement)
                            {
                                return changed;
                            }
                            break;
                        }
                    }
                    alpha *= 0.5F;
                }

                if (not accepted)
                {
                    break;
                }
            }

            return changed;
        }
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline static auto perturb_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const PerturbSettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
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
                auto perturbation = rng->next();
                Robot::scale_configuration(perturbation);

                const auto new_state = Robot::interpolate(perturb_state, perturbation, settings.range);
                float new_cost =
                    cost<Robot>(before_state, new_state) + cost<Robot>(new_state, after_state);

                if (new_cost < old_cost and
                    validate_motion<Robot, rake, resolution>(before_state, new_state, environment) and
                    validate_motion<Robot, rake, resolution>(new_state, after_state, environment))
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

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline auto simplify(
        const Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const SimplifySettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng) -> PlanningResult<Robot>
    {
        auto start_time = std::chrono::steady_clock::now();

        PlanningResult<Robot> result;

        const auto bspline = [&result, &environment, settings]()
        { return smooth_bspline<Robot, rake, resolution>(result.path, environment, settings.bspline); };
        const auto reduce = [&result, &environment, settings, rng]()
        {
            return reduce_path_vertices<Robot, rake, resolution>(
                result.path, environment, settings.reduce, rng);
        };
        const auto shortcut = [&result, &environment, settings]()
        { return shortcut_path<Robot, rake, resolution>(result.path, environment, settings.shortcut); };
        const auto perturb = [&result, &environment, settings, rng]()
        { return perturb_path<Robot, rake, resolution>(result.path, environment, settings.perturb, rng); };
        const auto velopt = [&result, &environment, settings]()
        { return velopt_path<Robot, rake, resolution>(result.path, environment, settings.velopt); };
        const auto polish = [&result, &environment, settings]()
        { return polish_path<Robot, rake, resolution>(result.path, environment, settings.polish); };

        const auto interpolate = [&result, settings]()
        { result.path.interpolate_to_n_states(settings.interpolate);
          return true; };

        const std::map<SimplifyRoutine, std::function<bool()>> operations = {
            {BSPLINE, bspline},
            {REDUCE, reduce},
            {SHORTCUT, shortcut},
            {PERTURB, perturb},
            {INTERP, interpolate},
            {VELOPT, velopt},
            {POLISH, polish},
        };

        // Check if straight line is valid
        if (path.size() == 2 or (path.size() > 2 and validate_motion<Robot, rake, resolution>(
                                                         path.front(), path.back(), environment)))
        {
            result.path.emplace_back(path.front());
            result.path.emplace_back(path.back());
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
                    any |= operations.find(op)->second();
                }

                if (not any)
                {
                    break;
                }
            }
        }

        if (settings.polish_at_end)
        {
            polish();
        }

        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        return result;
    }
}  // namespace vamp::planning
