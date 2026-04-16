#pragma once

#include <map>

#include <vamp/collision/environment.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <fstream>

namespace vamp::planning::constraint
{
    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline static auto smooth_bspline(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        vamp::planning::constraint::ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const BSplineSettings &settings,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0F,
        int num_projection_iterations = 25,
        bool insert_all_to_tree = false) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }

        bool changed = false;
        std::vector<typename Robot::Configuration> projected_vector;
        for (auto step = 0U; step < settings.max_steps; ++step)
        {
            path.subdivide();

            bool updated = false;
            for (auto index = 2U; index < path.size() - 1; index += 2)
            {
                const auto temp_1 = path[index].interpolate(path[index - 1], settings.midpoint_interpolation);
                const auto temp_2 = path[index].interpolate(path[index + 1], settings.midpoint_interpolation);
                const auto midpoint = temp_1.interpolate(temp_2, 0.5);

                if (path[index].distance(midpoint) > settings.min_change)
                {
                    // std::cout << "Attempting to bspline between " << index - 1 << " and " << index + 1 <<
                    // std::endl;

                    if (project_constraint_motion<Robot, rake, resolution>(
                            path[index - 1],
                            midpoint,
                            projected_vector,
                            constraint,
                            environment,
                            projection_method,
                            projection_descent_rate,
                            num_projection_iterations,
                            0.001F,
                            insert_all_to_tree) &&
                        ((path[index - 1] - midpoint).squared_l2_norm() >
                         (projected_vector.back() - path[index - 1]).squared_l2_norm()))
                    {
                        // std::cout << "Attempting to bspline between checking to goal" << index - 1 << " and
                        // " << index + 1 << std::endl;
                        if (project_constraint_motion<Robot, rake, resolution>(
                                midpoint,
                                path[index + 1],
                                projected_vector,
                                constraint,
                                environment,
                                projection_method,
                                projection_descent_rate,
                                num_projection_iterations,
                                0.001F,
                                insert_all_to_tree) &&
                            ((path[index + 1] - midpoint).squared_l2_norm() >
                             (projected_vector.back() - path[index + 1]).squared_l2_norm()))
                        {
                            // std::cout << "Bsplining between " << index - 1 << " and " << index + 1 <<
                            // std::endl;
                            path[index] = projected_vector.back();
                            changed |= (updated = true);
                        }
                    }
                }
            }

            if (not updated)
            {
                break;
            }
        }

        return changed;
    }

    // template <typename Robot, std::size_t rake, std::size_t resolution>
    // inline static auto reduce_path_vertices(
    //     Path<Robot> &path,
    //     const collision::Environment<FloatVector<rake>> &environment,
    //     const ReduceSettings &settings,
    //     const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
    // {
    //     if (path.size() < 3)
    //     {
    //         return false;
    //     }

    //     const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
    //     const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() :
    //     settings.max_empty_steps;

    //     bool result = false;
    //     for (auto i = 0U, no_change = 0U; i < max_steps or no_change < max_empty_steps; ++i, ++no_change)
    //     {
    //         int initial_size = path.size();
    //         int max_n = initial_size - 1;

    //         int range = 1 + static_cast<int>(
    //                             std::floor(0.5F + static_cast<float>(initial_size) *
    //                             settings.range_ratio));

    //         auto point_0 = rng->dist.uniform_integer(0, max_n);
    //         auto point_1 =
    //             rng->dist.uniform_integer(std::max(point_0 - range, 0), std::min(max_n, point_0 + range));

    //         if (std::abs(point_0 - point_1) < 2)
    //         {
    //             if (point_0 < max_n - 1)
    //             {
    //                 point_1 = point_0 + 2;
    //             }
    //             else if (point_0 > 1)
    //             {
    //                 point_1 = point_0 - 2;
    //             }
    //             else
    //             {
    //                 continue;
    //             }
    //         }

    //         if (point_0 > point_1)
    //         {
    //             std::swap(point_0, point_1);
    //         }

    //         if (validate_motion<Robot, rake, resolution>(path[point_0], path[point_1], environment))
    //         {
    //             path.erase(path.begin() + point_0 + 1, path.begin() + point_1);
    //             no_change = 0;
    //             result = true;
    //         }
    //     }

    //     return result;
    // }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline static auto shortcut_path(
        Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        vamp::planning::constraint::ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const ShortcutSettings & /*settings*/,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0F,
        int num_projection_iterations = 25,
        bool insert_all_to_tree = false) -> bool
    {
        if (path.size() < 3)
        {
            return false;
        }
        std::vector<typename Robot::Configuration> projected_vector;

        bool result = false;
        for (auto i = 0U; i < path.size() - 2; ++i)
        {
            for (auto j = path.size() - 1; j > i + 1; --j)
            {
                // if (project_constraint_motion<Robot, rake, resolution>(path[i], path[j], projected_vector,
                // constraint, environment) && ((projected_vector.back() - path[j]).squared_l2_norm() < 1e-6))
                if (project_constraint_motion<Robot, rake, resolution>(
                        path[i],
                        path[j],
                        projected_vector,
                        constraint,
                        environment,
                        projection_method,
                        projection_descent_rate,
                        num_projection_iterations,
                        0.001F,
                        insert_all_to_tree))
                {
                    if ((projected_vector.back() - path[j]).squared_l2_norm() < 1e-6)
                    {
                        path.erase(path.begin() + i + 1, path.begin() + j);
                        // std::cout << "Shortcutting path at " << i << " to " << j << std::endl;
                        result = true;
                        break;
                    }
                }
            }
        }

        return result;
    }

    // template <typename Robot, std::size_t rake, std::size_t resolution>
    // inline static auto perturb_path(
    //     Path<Robot> &path,
    //     const collision::Environment<FloatVector<rake>> &environment,
    //     const PerturbSettings &settings,
    //     const typename vamp::rng::RNG<Robot>::Ptr rng) -> bool
    // {
    //     if (path.size() < 3)
    //     {
    //         return false;
    //     }

    //     const auto max_steps = (not settings.max_steps) ? path.size() : settings.max_steps;
    //     const auto max_empty_steps = (not settings.max_empty_steps) ? path.size() :
    //     settings.max_empty_steps;

    //     bool changed = false;
    //     for (auto step = 0U, no_change = 0U; step < max_steps and no_change < max_empty_steps;
    //          ++step, ++no_change)
    //     {
    //         auto to_perturb_idx = rng->dist.uniform_integer(1UL, path.size() - 2);
    //         auto perturb_state = path[to_perturb_idx];
    //         auto before_state = path[to_perturb_idx - 1];
    //         auto after_state = path[to_perturb_idx + 1];

    //         float old_cost = perturb_state.distance(before_state) + perturb_state.distance(after_state);

    //         for (auto attempt = 0U; attempt < settings.perturbation_attempts; ++attempt)
    //         {
    //             auto perturbation = rng->next();
    //             Robot::scale_configuration(perturbation);

    //             const auto new_state = perturb_state.interpolate(perturbation, settings.range);
    //             float new_cost = new_state.distance(before_state) + new_state.distance(after_state);

    //             if (new_cost < old_cost and
    //                 validate_motion<Robot, rake, resolution>(before_state, new_state, environment) and
    //                 validate_motion<Robot, rake, resolution>(after_state, new_state, environment))
    //             {
    //                 no_change = 0;
    //                 changed = true;
    //                 path[to_perturb_idx] = new_state;
    //                 break;
    //             }
    //         }
    //     }

    //     return changed;
    // }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline auto simplify_with_constraints(
        const Path<Robot> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        vamp::planning::constraint::ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const SimplifySettings &settings,
        const typename vamp::rng::RNG<Robot>::Ptr rng,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0F,
        int num_projection_iterations = 25,
        bool insert_all_to_tree = false) -> PlanningResult<Robot>
    {
        auto start_time = std::chrono::steady_clock::now();

        PlanningResult<Robot> result;

        const auto bspline = [&result,
                              &environment,
                              settings,
                              &constraint,
                              projection_method,
                              projection_descent_rate,
                              num_projection_iterations,
                              insert_all_to_tree]()
        {
            return smooth_bspline<Robot, rake, resolution>(
                result.path,
                environment,
                constraint,
                settings.bspline,
                projection_method,
                projection_descent_rate,
                num_projection_iterations,
                insert_all_to_tree);
        };
        // const auto reduce = [&result, &environment, settings, rng]()
        // {
        //     return reduce_path_vertices<Robot, rake, resolution>(
        //         result.path, environment, settings.reduce, rng);
        // };
        const auto shortcut = [&result,
                               &environment,
                               settings,
                               &constraint,
                               projection_method,
                               projection_descent_rate,
                               num_projection_iterations,
                               insert_all_to_tree]()
        {
            return shortcut_path<Robot, rake, resolution>(
                result.path,
                environment,
                constraint,
                settings.shortcut,
                projection_method,
                projection_descent_rate,
                num_projection_iterations,
                insert_all_to_tree);
        };
        // const auto perturb = [&result, &environment, settings, rng]()
        // { return perturb_path<Robot, rake, resolution>(result.path, environment, settings.perturb, rng); };

        const std::map<SimplifyRoutine, std::function<bool()>> operations = {
            {BSPLINE, bspline},
            // {REDUCE, reduce},
            {SHORTCUT, shortcut},
            // {PERTURB, perturb},
        };

        std::vector<typename Robot::Configuration> projected_vector;
        // Check if straight line is valid
        if (path.size() == 2 or (path.size() > 2 and project_constraint_motion<Robot, rake, resolution>(
                                                         path.front(),
                                                         path.back(),
                                                         projected_vector,
                                                         constraint,
                                                         environment,
                                                         projection_method,
                                                         projection_descent_rate,
                                                         num_projection_iterations,
                                                         0.0001F,
                                                         true)))
        {
            // if the last projected configuration is not the same as the original end configuration
            if ((projected_vector.back() - path.back()).squared_l2_norm() < 1e-6)
            {
                result.path.emplace_back(path.front());
                for (auto i = 0U; i < projected_vector.size(); ++i)
                {
                    result.path.emplace_back(projected_vector[i]);
                }
                // result.path.emplace_back(path.back());
                result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                return result;
            }
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

        result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
        return result;
    }
}  // namespace vamp::planning::constraint
