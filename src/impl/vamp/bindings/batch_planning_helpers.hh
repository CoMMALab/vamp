#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <vamp/planning/rrtc_prealloc.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace vamp::binding
{
    static constexpr const std::size_t batch_helper_rake = vamp::FloatVectorWidth;

    template <typename Robot, typename HelperT>
    inline auto logit_sample(typename HelperT::RNG::Ptr rng, float scale) noexcept ->
        typename HelperT::Configuration
    {
        const auto u = rng->next();
        return (u * (1.F - u).rcp()).log() * std::sqrt(scale);
    }

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

    // Repeatedly calls `try_sample(config)` until `batch_size` samples are accepted or
    // `max_attempts` total attempts are used (0 = unlimited). Only accepted samples are
    // returned, so the result has shape {num_accepted, dimension}.
    template <typename Robot, typename Configuration, typename SampleFnT>
    inline auto collect_samples(std::size_t batch_size, std::size_t max_attempts, const SampleFnT &try_sample)
        -> nanobind::ndarray<nanobind::numpy, float, nanobind::ndim<2>>
    {
        namespace nb = nanobind;

        auto *configs = new float[batch_size * Robot::dimension];
        nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });

        std::size_t accepted = 0;
        for (std::size_t attempts = 0;
             accepted < batch_size and (max_attempts == 0 or attempts < max_attempts);
             ++attempts)
        {
            Configuration config;
            if (try_sample(config))
            {
                config.to_array_unaligned(&configs[accepted * Robot::dimension]);
                ++accepted;
            }
        }

        return nb::ndarray<nb::numpy, float, nb::ndim<2>>(configs, {accepted, Robot::dimension}, owner);
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

        const auto is_valid = [](const Configuration &c, const EnvironmentVector &env_v) noexcept
        { return validate(c, c, env_v); };

        submodule.def(
            "batch_sample",
            [is_valid](
                std::size_t batch_size,
                std::size_t num_layers,
                std::size_t num_dreams,
                float scale,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env,
                std::size_t max_attempts)
            {
                const EnvironmentVector env_v(env);
                const std::size_t total = batch_size * num_layers * num_dreams;

                auto *configs = new float[total * Robot::dimension];
                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });

                std::size_t attempts = 0;
                for (std::size_t i = 0; i < total; ++i)
                {
                    Configuration config;
                    do
                    {
                        if (max_attempts != 0 and attempts++ >= max_attempts)
                        {
                            throw std::runtime_error(
                                "batch_sample: exhausted max_attempts after " + std::to_string(i) + " of " +
                                std::to_string(total) + " samples");
                        }

                        config = rng->next() * scale;
                        Robot::scale_configuration(config);
                    } while (not is_valid(config, env_v));

                    config.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                return nb::ndarray<nb::numpy, float, nb::ndim<4>>(
                    configs, {batch_size, num_layers, num_dreams, Robot::dimension}, owner);
            },
            "batch_size"_a,
            "num_layers"_a,
            "num_dreams"_a,
            "scale"_a,
            "rng"_a,
            "environment"_a,
            "max_attempts"_a = 0,
            "Sample batch_size x num_layers x num_dreams valid configurations. Raises if "
            "max_attempts (0 = unlimited) total attempts are used before all samples are found.");

        submodule.def(
            "batch_scale_sample",
            [is_valid](
                std::size_t batch_size,
                float scale,
                std::size_t max_attempts,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env)
            {
                const EnvironmentVector env_v(env);
                return collect_samples<Robot, Configuration>(
                    batch_size,
                    max_attempts,
                    [&](Configuration &config)
                    {
                        config = rng->next() * scale;
                        Robot::scale_configuration(config);
                        return is_valid(config, env_v);
                    });
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a,
            "Sample up to batch_size valid scaled configurations; returns only the samples found.");

        submodule.def(
            "batch_gaussian_sample",
            [is_valid](
                std::size_t batch_size,
                const nb::ndarray<const float, nb::shape<Robot::dimension>, nb::device::cpu> &mean,
                float stddev,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env,
                std::size_t max_attempts)
            {
                const auto mean_view = mean.view();
                auto mean_config =
                    load_configuration<Robot, Configuration>([&](std::size_t d) { return mean_view(d); });
                Robot::descale_configuration(mean_config);
                const EnvironmentVector env_v(env);

                return collect_samples<Robot, Configuration>(
                    batch_size,
                    max_attempts,
                    [&](Configuration &config)
                    {
                        config = mean_config + rng->next() * stddev;
                        Robot::scale_configuration(config);
                        return is_valid(config, env_v);
                    });
            },
            "batch_size"_a,
            "mean"_a,
            "variance"_a,
            "rng"_a,
            "environment"_a,
            "max_attempts"_a = 0,
            "Sample up to batch_size valid configurations around mean (in normalized joint space); "
            "returns only the samples found. `variance` scales the rng output (i.e., acts as a stddev).");

        const auto bridge_sample = [is_valid](
                                       typename HelperT::RNG::Ptr &rng,
                                       const EnvironmentVector &env_v,
                                       float scale,
                                       Configuration &out) -> bool
        {
            // Bridge test: both endpoints in collision, midpoint collision-free.
            Configuration ca = rng->next();
            Robot::scale_configuration(ca);
            if (is_valid(ca, env_v))
            {
                return false;
            }

            Configuration cb = ca + logit_sample<Robot, HelperT>(rng, scale).trim();
            Robot::descale_configuration(cb);
            cb = cb.clamp(0, 1);
            Robot::scale_configuration(cb);
            if (is_valid(cb, env_v))
            {
                return false;
            }

            out = ca.interpolate(cb, 0.5);
            return is_valid(out, env_v);
        };

        submodule.def(
            "batch_bridge_test_sample",
            [bridge_sample](
                std::size_t batch_size,
                float scale,
                std::size_t max_attempts,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env)
            {
                const EnvironmentVector env_v(env);
                return collect_samples<Robot, Configuration>(
                    batch_size,
                    max_attempts,
                    [&](Configuration &config) { return bridge_sample(rng, env_v, scale, config); });
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a,
            "Bridge-test sampling; returns only the samples found within max_attempts.");

        submodule.def(
            "batch_bridge_test_sample",
            [bridge_sample](
                std::size_t batch_size,
                float min_scale,
                float max_scale,
                std::size_t max_attempts,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env)
            {
                const EnvironmentVector env_v(env);
                std::default_random_engine generator;
                std::uniform_real_distribution<float> scale_dist(min_scale, max_scale);
                return collect_samples<Robot, Configuration>(
                    batch_size,
                    max_attempts,
                    [&](Configuration &config)
                    { return bridge_sample(rng, env_v, scale_dist(generator), config); });
            },
            "batch_size"_a,
            "min_scale"_a,
            "max_scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a,
            "Bridge-test sampling with scale drawn uniformly from [min_scale, max_scale].");

        submodule.def(
            "batch_surface_sample",
            [is_valid](
                std::size_t batch_size,
                float scale,
                std::size_t max_attempts,
                typename HelperT::RNG::Ptr rng,
                const typename HelperT::EnvironmentInput &env)
            {
                const EnvironmentVector env_v(env);
                return collect_samples<Robot, Configuration>(
                    batch_size,
                    max_attempts,
                    [&](Configuration &config)
                    {
                        // Surface sample: collision configuration perturbed into free space.
                        Configuration ca = rng->next();
                        Robot::scale_configuration(ca);
                        if (is_valid(ca, env_v))
                        {
                            return false;
                        }

                        config = ca + logit_sample<Robot, HelperT>(rng, scale).trim();
                        Robot::descale_configuration(config);
                        config = config.clamp(0, 1);
                        Robot::scale_configuration(config);
                        return is_valid(config, env_v);
                    });
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a,
            "Surface sampling; returns only the samples found within max_attempts.");

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
                using RRTCAlloc = vamp::planning::RRTC_Alloc<Robot, batch_helper_rake, Robot::resolution>;

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
                    RRTCAlloc rrtc(settings);
                    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                    const auto check_connect = [&rrtc, &rng, &settings, &env_v](const auto &a, const auto &b)
                    {
                        rng->reset();
                        auto result = rrtc.solve(a, b, env_v, settings, rng);
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
