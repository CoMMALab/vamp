#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <vamp/planning/rrtc.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace vamp::binding
{
    static constexpr const std::size_t batch_helper_rake = vamp::FloatVectorWidth;

    // Build a configuration from `Robot::dimension` scalars read through `at(d)`.
    // Goes through a padded, aligned buffer so we never read past the end of a
    // tightly-packed numpy row, and works for any stride.
    template <typename Robot, typename Configuration, typename AccessorT>
    inline auto load_configuration(const AccessorT &at) noexcept -> Configuration
    {
        std::array<float, Robot::dimension> buffer;
        for (std::size_t d = 0; d < Robot::dimension; ++d)
        {
            buffer[d] = at(d);
        }

        return Configuration(buffer);
    }

    template <typename Robot, typename Configuration>
    inline auto in_bounds(Configuration configuration) noexcept -> bool
    {
        Robot::descale_configuration(configuration);
        return (configuration <= 1.F).all() and (configuration >= 0.F).all();
    }

    template <typename Robot, typename HelperT>
    inline void bind_batch_planning_helpers(nanobind::module_ &submodule)
    {
        namespace nb = nanobind;
        using namespace nb::literals;

        using Configuration = typename HelperT::Configuration;
        using EnvironmentVector = typename HelperT::EnvironmentVector;
        using RowArray = nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu>;

        static constexpr auto validate = vamp::planning::validate_motion<Robot, batch_helper_rake, 1>;

        submodule.def(
            "validate_motion_batch",
            [](const RowArray &a,
               const RowArray &b,
               const typename HelperT::EnvironmentInput &environment,
               bool check_bounds) -> nb::ndarray<nb::numpy, bool, nb::shape<-1>, nb::device::cpu>
            {
                const std::size_t n = a.shape(0);
                if (b.shape(0) != n)
                {
                    throw std::runtime_error("validate_motion_batch expects same number of rows in a and b");
                }

                auto *results_data = new bool[n];
                nb::capsule owner(results_data, [](void *p) noexcept { delete[] reinterpret_cast<bool *>(p); });

                const auto a_view = a.view();
                const auto b_view = b.view();
                const EnvironmentVector env_v(environment);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 64)
#endif
                for (std::size_t i = 0; i < n; ++i)
                {
                    const auto config_a =
                        load_configuration<Robot, Configuration>([&](std::size_t d) { return a_view(i, d); });
                    const auto config_b =
                        load_configuration<Robot, Configuration>([&](std::size_t d) { return b_view(i, d); });

                    results_data[i] =
                        (not check_bounds or
                         (in_bounds<Robot>(config_a) and in_bounds<Robot>(config_b))) and
                        validate(config_a, config_b, env_v);
                }

                return nb::ndarray<nb::numpy, bool, nb::shape<-1>, nb::device::cpu>(results_data, {n}, owner);
            },
            "a"_a,
            "b"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "check_bounds"_a = true,
            "Validate a batch of motions a[i] -> b[i] and return boolean results. Matches "
            "validate_motion, including its check_bounds default.");

        submodule.def(
            "rrtc_batch",
            [](const RowArray &start,
               const RowArray &goal,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::RRTCSettings &settings) -> std::vector<typename HelperT::PlanningResult>
            {
                const std::size_t n = start.shape(0);
                if (goal.shape(0) != n)
                {
                    throw std::runtime_error("rrtc_batch expects same number of rows in start and goal");
                }

                std::vector<typename HelperT::PlanningResult> results(n);
                const auto start_view = start.view();
                const auto goal_view = goal.view();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
                for (std::size_t i = 0; i < n; ++i)
                {
                    typename HelperT::Type config_start{};
                    typename HelperT::Type config_goal{};
                    for (std::size_t d = 0; d < Robot::dimension; ++d)
                    {
                        config_start[d] = start_view(i, d);
                        config_goal[d] = goal_view(i, d);
                    }

                    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
                    results[i] = HelperT::RRTC::single(config_start, config_goal, environment, settings, rng);
                }

                return results;
            },
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "Run RRTC on a batch of start/goal configurations and return PlanningResult objects.");

        submodule.def(
            "aorrtc_batch",
            [](const RowArray &start,
               const RowArray &goal,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::AORRTCSettings &settings) -> std::vector<typename HelperT::PlanningResult>
            {
                const std::size_t n = start.shape(0);
                if (goal.shape(0) != n)
                {
                    throw std::runtime_error("aorrtc_batch expects same number of rows in start and goal");
                }

                std::vector<typename HelperT::PlanningResult> results(n);
                const auto start_view = start.view();
                const auto goal_view = goal.view();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1)
#endif
                for (std::size_t i = 0; i < n; ++i)
                {
                    typename HelperT::Type config_start{};
                    typename HelperT::Type config_goal{};
                    for (std::size_t d = 0; d < Robot::dimension; ++d)
                    {
                        config_start[d] = start_view(i, d);
                        config_goal[d] = goal_view(i, d);
                    }

                    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
                    results[i] = HelperT::AORRTC::single(config_start, config_goal, environment, settings, rng);
                }

                return results;
            },
            "start"_a,
            "goal"_a,
            "environment"_a,
            "settings"_a,
            "Run AORRTC on a batch of start/goal configurations and return PlanningResult objects.");

        submodule.def(
            "incremental_batch_validate_rrtc",
            [](const nb::ndarray<const float, nb::shape<Robot::dimension>, nb::device::cpu> &start_config,
               const nb::ndarray<const float, nb::shape<-1, -1, -1, Robot::dimension>, nb::device::cpu>
                   &batch_layers,
               const RowArray &goal_configs,
               nb::ndarray<bool, nb::shape<-1, -1, -1>, nb::device::cpu> &start_validate,
               nb::ndarray<bool, nb::shape<-1, -1, -1, -1>, nb::device::cpu> &layer_validate,
               nb::ndarray<bool, nb::shape<-1, -1, -1>, nb::device::cpu> &goal_validate,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::RRTCSettings &settings,
               std::size_t start_point_range,
               std::size_t end_point_range)
            {
                using RRTC = vamp::planning::RRTC<Robot, batch_helper_rake, Robot::resolution>;

                const std::size_t batch_size = batch_layers.shape(0);
                const std::size_t num_layers = batch_layers.shape(1);
                const std::size_t num_dreams = batch_layers.shape(2);
                const std::size_t num_goals = goal_configs.shape(0);

                const auto require = [](bool ok, const char *what)
                {
                    if (not ok)
                    {
                        throw std::runtime_error(std::string("incremental_batch_validate_rrtc: ") + what);
                    }
                };

                require(num_layers >= 1, "batch_layers must have at least one layer");
                require(
                    start_point_range <= end_point_range and end_point_range <= num_dreams,
                    "need start_point_range <= end_point_range <= num_dreams");
                require(
                    start_validate.shape(0) == batch_size and start_validate.shape(1) >= 1 and
                        start_validate.shape(2) >= end_point_range,
                    "start_validate must have shape (batch, 1, >= end_point_range)");
                require(
                    num_layers == 1 or
                        (layer_validate.shape(0) == batch_size and layer_validate.shape(1) >= num_layers - 1 and
                         layer_validate.shape(2) >= end_point_range and
                         layer_validate.shape(3) >= end_point_range),
                    "layer_validate must have shape (batch, layers - 1, >= end_point_range, >= end_point_range)");
                require(
                    goal_validate.shape(0) == batch_size and goal_validate.shape(1) >= end_point_range and
                        goal_validate.shape(2) >= num_goals,
                    "goal_validate must have shape (batch, >= end_point_range, num_goals)");

                const EnvironmentVector env_v(environment);
                const auto bl_view = batch_layers.view();
                const auto gc_view = goal_configs.view();
                const auto sc_view = start_config.view();
                auto cs_view = start_validate.view();
                auto cl_view = layer_validate.view();
                auto cg_view = goal_validate.view();

                const auto layer_point = [&bl_view](std::size_t b, std::size_t l, std::size_t i)
                {
                    return load_configuration<Robot, Configuration>([&](std::size_t d)
                                                                    { return bl_view(b, l, i, d); });
                };

                const auto c_s_v =
                    load_configuration<Robot, Configuration>([&](std::size_t d) { return sc_view(d); });

#ifdef _OPENMP
#pragma omp parallel
#endif
                {
                    typename RRTC::Workspace workspace(settings.max_samples);
                    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                    const auto check_connect = [&workspace, &rng, &settings, &env_v](const auto &a, const auto &b)
                    {
                        rng->reset();
                        auto result = RRTC::solve(a, b, env_v, settings, rng, workspace);
                        return result.path.size() > 1;
                    };

#ifdef _OPENMP
#pragma omp for collapse(2) schedule(dynamic, 16) nowait
#endif
                    for (std::size_t b = 0; b < batch_size; ++b)
                    {
                        for (std::size_t i = start_point_range; i < end_point_range; ++i)
                        {
                            cs_view(b, 0, i) = check_connect(c_s_v, layer_point(b, 0, i));
                        }
                    }

                    if (num_layers > 1)
                    {
                        // Check every edge that touches a new point: new -> any and old -> new.
#ifdef _OPENMP
#pragma omp for collapse(4) schedule(dynamic, 16) nowait
#endif
                        for (std::size_t b = 0; b < batch_size; ++b)
                        {
                            for (std::size_t l = 0; l < num_layers - 1; ++l)
                            {
                                for (std::size_t i = 0; i < end_point_range; ++i)
                                {
                                    for (std::size_t j = 0; j < end_point_range; ++j)
                                    {
                                        if (i < start_point_range and j < start_point_range)
                                        {
                                            continue;
                                        }

                                        cl_view(b, l, i, j) =
                                            check_connect(layer_point(b, l, i), layer_point(b, l + 1, j));
                                    }
                                }
                            }
                        }
                    }

#ifdef _OPENMP
#pragma omp for collapse(3) schedule(dynamic, 16) nowait
#endif
                    for (std::size_t b = 0; b < batch_size; ++b)
                    {
                        for (std::size_t i = start_point_range; i < end_point_range; ++i)
                        {
                            for (std::size_t g = 0; g < num_goals; ++g)
                            {
                                const auto c_g_v = load_configuration<Robot, Configuration>(
                                    [&](std::size_t d) { return gc_view(g, d); });
                                cg_view(b, i, g) = check_connect(layer_point(b, num_layers - 1, i), c_g_v);
                            }
                        }
                    }
                }
            },
            "start_config"_a,
            "batch_layers"_a,
            "goal_configs"_a,
            "start_validate"_a,
            "layer_validate"_a,
            "goal_validate"_a,
            "environment"_a,
            "settings"_a,
            "start_point_range"_a,
            "end_point_range"_a,
            "Incrementally fill connectivity arrays for points in [start_point_range, end_point_range) "
            "using RRTC solve checks. Raises on mismatched array shapes.");
    }
}  // namespace vamp::binding
