#pragma once

#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
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
    inline auto logit_sample(typename HelperT::RNG::Ptr rng, float scale) noexcept
        -> typename HelperT::Configuration
    {
        const auto u = rng->next();
        return (u * (1.F - u).rcp()).log() * std::sqrt(scale);
    }

    template <typename Robot, typename HelperT>
    inline void bind_batch_planning_helpers(nanobind::module_ &submodule)
    {
        namespace nb = nanobind;
        using namespace nb::literals;

        using Configuration = typename HelperT::Configuration;
        static constexpr auto validate =
            vamp::planning::validate_motion<Robot, batch_helper_rake, 1>;

        submodule.def(
            "batch_sample",
            [](std::size_t batch_size,
               std::size_t num_layers,
               std::size_t num_dreams,
               float scale,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                const typename HelperT::EnvironmentVector env_v(env);
                const std::size_t total = batch_size * num_layers * num_dreams;

                auto *configs = new float[total * Robot::dimension];
                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                nb::ndarray<nb::numpy, float, nb::ndim<4>> config_nd(
                    configs, {batch_size, num_layers, num_dreams, Robot::dimension}, owner);

                for (std::size_t i = 0; i < total; ++i)
                {
                    Configuration config;
                    do
                    {
                        config = rng->next() * scale;
                        Robot::scale_configuration(config);
                    } while (not validate(config, config, env_v));

                    config.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                return config_nd;
            },
            "batch_size"_a,
            "num_layers"_a,
            "num_dreams"_a,
            "scale"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "batch_scale_sample",
            [](std::size_t batch_size,
               float scale,
               std::size_t max_attempts,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                const typename HelperT::EnvironmentVector env_v(env);

                auto *configs = new float[batch_size * Robot::dimension];
                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                nb::ndarray<nb::numpy, float, nb::ndim<2>> config_nd(
                    configs, {batch_size, Robot::dimension}, owner);

                std::size_t i = 0;
                std::size_t attempts = 0;
                for (; i < batch_size && attempts <= max_attempts; ++i)
                {
                    Configuration config;
                    do
                    {
                        if (attempts++ > max_attempts)
                        {
                            break;
                        }
                        config = rng->next() * scale;
                        Robot::scale_configuration(config);
                    } while (not validate(config, config, env_v));

                    config.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                return config_nd;
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "batch_gaussian_sample",
            [](std::size_t batch_size,
               const nb::ndarray<const float, nb::shape<Robot::dimension>, nb::device::cpu> &mean,
               float variance,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                Configuration mean_config(mean.data(), false);
                Robot::descale_configuration(mean_config);
                const typename HelperT::EnvironmentVector env_v(env);

                auto *configs = new float[batch_size * Robot::dimension];
                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                nb::ndarray<nb::numpy, float, nb::ndim<2>> config_nd(
                    configs, {batch_size, Robot::dimension}, owner);

                for (std::size_t i = 0; i < batch_size; ++i)
                {
                    Configuration config;
                    do
                    {
                        config = mean_config + rng->next() * variance;
                        Robot::scale_configuration(config);
                    } while (not validate(config, config, env_v));

                    config.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                return config_nd;
            },
            "batch_size"_a,
            "mean"_a,
            "variance"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "batch_bridge_test_sample",
            [](std::size_t batch_size,
               float scale,
               std::size_t max_attempts,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                const typename HelperT::EnvironmentVector env_v(env);
                auto *configs = new float[batch_size * Robot::dimension];

                std::size_t i = 0;
                std::size_t attempts = 0;
                for (; i < batch_size && attempts <= max_attempts; ++i)
                {
                    Configuration ca, cb, cc;
                    do
                    {
                        if (attempts++ > max_attempts)
                        {
                            break;
                        }

                        ca = rng->next();
                        Robot::scale_configuration(ca);
                        if (validate(ca, ca, env_v))
                        {
                            continue;
                        }

                        cb = ca + logit_sample<Robot, HelperT>(rng, scale).trim();
                        Robot::descale_configuration(cb);
                        cb = cb.clamp(0, 1);
                        Robot::scale_configuration(cb);
                        if (validate(cb, cb, env_v))
                        {
                            continue;
                        }

                        cc = ca.interpolate(cb, 0.5);
                    } while (not validate(cc, cc, env_v));

                    cc.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(configs, {i, Robot::dimension}, owner);
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "batch_bridge_test_sample",
            [](std::size_t batch_size,
               float min_scale,
               float max_scale,
               std::size_t max_attempts,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                const typename HelperT::EnvironmentVector env_v(env);
                std::default_random_engine generator;
                std::uniform_real_distribution<float> scalar_rng(min_scale, max_scale);
                auto *configs = new float[batch_size * Robot::dimension];

                std::size_t i = 0;
                std::size_t attempts = 0;
                for (; i < batch_size && attempts <= max_attempts; ++i)
                {
                    Configuration ca, cb, cc;
                    do
                    {
                        if (attempts++ > max_attempts)
                        {
                            break;
                        }

                        ca = rng->next();
                        Robot::scale_configuration(ca);
                        if (validate(ca, ca, env_v))
                        {
                            continue;
                        }

                        const float scale = scalar_rng(generator);
                        cb = ca + logit_sample<Robot, HelperT>(rng, scale).trim();
                        Robot::descale_configuration(cb);
                        cb = cb.clamp(0, 1);
                        Robot::scale_configuration(cb);
                        if (validate(cb, cb, env_v))
                        {
                            continue;
                        }

                        cc = ca.interpolate(cb, 0.5);
                    } while (not validate(cc, cc, env_v));

                    cc.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(configs, {i, Robot::dimension}, owner);
            },
            "batch_size"_a,
            "min_scale"_a,
            "max_scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "batch_surface_sample",
            [](std::size_t batch_size,
               float scale,
               std::size_t max_attempts,
               typename HelperT::RNG::Ptr rng,
               const typename HelperT::EnvironmentInput &env) noexcept
            {
                const typename HelperT::EnvironmentVector env_v(env);
                auto *configs = new float[batch_size * Robot::dimension];

                std::size_t i = 0;
                std::size_t attempts = 0;
                for (; i < batch_size && attempts <= max_attempts; ++i)
                {
                    Configuration ca, cc;
                    do
                    {
                        if (attempts++ > max_attempts)
                        {
                            break;
                        }

                        ca = rng->next();
                        Robot::scale_configuration(ca);
                        if (validate(ca, ca, env_v))
                        {
                            continue;
                        }

                        cc = ca + logit_sample<Robot, HelperT>(rng, scale).trim();
                        Robot::descale_configuration(cc);
                        cc = cc.clamp(0, 1);
                        Robot::scale_configuration(cc);
                    } while (not validate(cc, cc, env_v));

                    cc.to_array_unaligned(&configs[i * Robot::dimension]);
                }

                nb::capsule owner(configs, [](void *p) noexcept { delete[] reinterpret_cast<float *>(p); });
                return nb::ndarray<nb::numpy, float, nb::ndim<2>>(configs, {i, Robot::dimension}, owner);
            },
            "batch_size"_a,
            "scale"_a,
            "max_attempts"_a,
            "rng"_a,
            "environment"_a);

        submodule.def(
            "validate_motion_batch",
            [](const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &a,
               const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &b,
               const typename HelperT::EnvironmentInput &environment = vamp::collision::Environment<float>())
                -> nb::ndarray<nb::numpy, bool, nb::shape<-1>, nb::device::cpu>
            {
                const std::size_t n = a.shape(0);
                if (b.shape(0) != n)
                {
                    throw std::runtime_error("validate_motion_batch expects same number of rows in a and b");
                }

                auto *results_data = new bool[n];
                nb::capsule owner(results_data, [](void *p) noexcept { delete[] reinterpret_cast<bool *>(p); });

                auto a_view = a.view();
                auto b_view = b.view();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1000)
#endif
                for (std::size_t i = 0; i < n; ++i)
                {
                    typename HelperT::Type config_a{};
                    typename HelperT::Type config_b{};

                    for (std::size_t d = 0; d < Robot::dimension; ++d)
                    {
                        config_a[d] = a_view(i, d);
                        config_b[d] = b_view(i, d);
                    }

                    results_data[i] = HelperT::validate_motion(config_a, config_b, environment, false);
                }

                return nb::ndarray<nb::numpy, bool, nb::shape<-1>, nb::device::cpu>(results_data, {n}, owner);
            },
            "a"_a,
            "b"_a,
            "environment"_a = vamp::collision::Environment<float>(),
            "Validate a batch of motions and return boolean results.");

        submodule.def(
            "rrtc_batch",
            [](const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &start,
               const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &goal,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::RRTCSettings &settings) -> std::vector<typename HelperT::PlanningResult>
            {
                const std::size_t n = start.shape(0);
                if (goal.shape(0) != n)
                {
                    throw std::runtime_error("rrtc_batch expects same number of rows in start and goal");
                }

                std::vector<typename HelperT::PlanningResult> results(n);
                auto start_view = start.view();
                auto goal_view = goal.view();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1000)
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
            [](const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &start,
               const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu> &goal,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::AORRTCSettings &settings) -> std::vector<typename HelperT::PlanningResult>
            {
                const std::size_t n = start.shape(0);
                if (goal.shape(0) != n)
                {
                    throw std::runtime_error("aorrtc_batch expects same number of rows in start and goal");
                }

                std::vector<typename HelperT::PlanningResult> results(n);
                auto start_view = start.view();
                auto goal_view = goal.view();

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 1000)
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
               const nb::ndarray<const float, nb::shape<-1, Robot::dimension>, nb::device::cpu>
                   &goal_configs,
               nb::ndarray<bool, nb::shape<-1, -1, -1>, nb::device::cpu> &start_validate,
               nb::ndarray<bool, nb::shape<-1, -1, -1, -1>, nb::device::cpu> &layer_validate,
               nb::ndarray<bool, nb::shape<-1, -1, -1>, nb::device::cpu> &goal_validate,
               const typename HelperT::EnvironmentInput &environment,
               const vamp::planning::RRTCSettings &settings,
               std::size_t start_point_range,
               std::size_t end_point_range) noexcept
            {
                using RRTCAlloc = vamp::planning::RRTC_Alloc<Robot, batch_helper_rake, Robot::resolution>;
                const typename HelperT::EnvironmentVector env_v(environment);

                const std::size_t batch_size = batch_layers.shape(0);
                const std::size_t num_layers = batch_layers.shape(1);
                const std::size_t num_goals = goal_configs.shape(0);

                const auto bl_view = batch_layers.view();
                const auto gc_view = goal_configs.view();
                const auto cs_view = start_validate.view();
                const auto cl_view = layer_validate.view();
                const auto cg_view = goal_validate.view();

                const Configuration c_s_v(start_config.data(), false);

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
#pragma omp for collapse(2) schedule(dynamic, 1000) nowait
#endif
                    for (std::size_t b = 0; b < batch_size; ++b)
                    {
                        for (std::size_t i = start_point_range; i < end_point_range; ++i)
                        {
                            const Configuration c_b_v(&bl_view(b, 0, i, 0), false);
                            cs_view(b, 0, i) = check_connect(c_s_v, c_b_v);
                        }
                    }

                    if (num_layers > 1)
                    {
#ifdef _OPENMP
#pragma omp for collapse(4) schedule(dynamic, 1000) nowait
#endif
                        for (std::size_t b = 0; b < batch_size; ++b)
                        {
                            for (std::size_t l = 0; l < num_layers - 1; ++l)
                            {
                                for (std::size_t i = start_point_range; i < end_point_range; ++i)
                                {
                                    for (std::size_t j = 0; j < end_point_range; ++j)
                                    {
                                        const Configuration c_a_v(&bl_view(b, l, i, 0), false);
                                        const Configuration c_b_v(&bl_view(b, l + 1, j, 0), false);
                                        cl_view(b, l, i, j) = check_connect(c_a_v, c_b_v);
                                    }
                                }
                            }
                        }
                    }

#ifdef _OPENMP
#pragma omp for collapse(3) schedule(dynamic, 1000) nowait
#endif
                    for (std::size_t b = 0; b < batch_size; ++b)
                    {
                        for (std::size_t i = start_point_range; i < end_point_range; ++i)
                        {
                            for (std::size_t g = 0; g < num_goals; ++g)
                            {
                                const Configuration c_a_v(&bl_view(b, num_layers - 1, i, 0), false);
                                const Configuration c_g_v(&gc_view(g, 0), false);
                                cg_view(b, i, g) = check_connect(c_a_v, c_g_v);
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
            "Incrementally fill connectivity arrays using RRTC solve checks.");
    }
}  // namespace vamp::binding
