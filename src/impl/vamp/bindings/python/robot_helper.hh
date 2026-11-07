#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>

#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/planners/fcit.hh>
#include <vamp/planning/planners/grrtstar.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/prm.hh>
#include <vamp/planning/planners/roadmap.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#else
#include <stdexcept>
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <nanobind/eigen/dense.h>
#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

VAMP_DEFINE_HAS_METHOD(in_bounds)
VAMP_DEFINE_HAS_METHOD(set_lows)
VAMP_DEFINE_HAS_METHOD(set_highs)
VAMP_DEFINE_HAS_METHOD(set_radius)

// Robots generated with attachment link tables declare `exempt_link_pairs` and `link_names`.
template <typename T, typename = void>
struct robot_has_exempt_link_pairs : std::false_type
{
};

template <typename T>
struct robot_has_exempt_link_pairs<T, std::void_t<decltype(T::exempt_link_pairs)>> : std::true_type
{
};

template <typename T>
constexpr bool robot_has_exempt_link_pairs_v = robot_has_exempt_link_pairs<T>::value;

// Robots generated with several end-effectors declare `end_effectors` (frame names) and `eefk(x, i)`.
template <typename T, typename = void>
struct robot_has_end_effectors : std::false_type
{
};

template <typename T>
struct robot_has_end_effectors<T, std::void_t<decltype(T::end_effectors)>> : std::true_type
{
};

template <typename T>
constexpr bool robot_has_end_effectors_v = robot_has_end_effectors<T>::value;

namespace vamp::binding
{
    namespace nb = nanobind;
    using namespace nb::literals;

    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    // Whether a configuration lies within the robot's joint limits. Generated robots may provide
    // in_bounds themselves; otherwise it is checked in the normalized [0, 1] cube.
    template <typename Robot>
    inline auto in_bounds(const typename Robot::Configuration &configuration) -> bool
    {
        auto trimmed = configuration.trim();
        if constexpr (has_in_bounds_v<Robot>)
        {
            return Robot::in_bounds(trimmed);
        }
        else
        {
            Robot::descale_configuration(trimmed);
            return (trimmed <= 1.F).all() and (trimmed >= 0.F).all();
        }
    }

    // Conversions shared by the two Python input adapters, written against the derived
    // adapter's array().
    template <typename Robot, typename Derived>
    struct InputOps
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t r>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<r>;

        template <typename Type>
        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(Derived::array(a));
        }

        template <std::size_t r, typename Type>
        inline static auto block(const Type &a) -> ConfigurationBlock<r>
        {
            const auto &arr = Derived::array(a);
            ConfigurationBlock<r> out;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = arr[i];
            }
            return out;
        }
    };

    template <typename Robot>
    struct NDArrayInput : InputOps<Robot, NDArrayInput<Robot>>
    {
        using Type = nanobind::
            ndarray<FloatT, nanobind::numpy, nanobind::shape<Robot::dimension>, nanobind::device::cpu>;

        inline static auto from(const typename Robot::Configuration &c) -> Type
        {
            auto c_arr = c.to_array();
            return make_ndarray<Type, 1>(c_arr.data(), {Robot::dimension});
        }

        inline static auto array(const Type &a) -> typename Robot::ConfigurationArray
        {
            typename Robot::ConfigurationArray c;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Robot::dimension, scratch, "configuration");
            std::memcpy(c.data(), ptr, Robot::dimension * sizeof(float));
            return c;
        }
    };

    template <typename Robot>
    struct ArrayInput : InputOps<Robot, ArrayInput<Robot>>
    {
        // A plain std::array, not Robot::ConfigurationArray: the generated type is an aligned struct,
        // which nanobind's std::array caster does not accept, so Python lists would be rejected.
        using Type = std::array<FloatT, Robot::dimension>;

        inline static auto from(const typename Robot::Configuration &c) -> Type
        {
            Type a;
            auto c_arr = c.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                a[i] = c_arr[i];
            }
            return a;
        }

        inline static auto array(const Type &a) -> const Type &
        {
            return a;
        }
    };

    template <typename Robot, typename Input>
    struct StaticRobotTraits
    {
        using Cfg = typename Input::Type;
        using Pth = std::vector<Cfg>;
        using Pc = std::vector<vamp::collision::Point>;
        using Path = vamp::planning::Path<Robot>;
        using PlanningResult = vamp::planning::PlanningResult<Robot>;
        using Sampler = vamp::rng::RNG<Robot>;
        using Phs = vamp::planning::ProlateHyperspheroid<Robot>;
        using Env = vamp::collision::Environment<float>;
        using EnvVec = vamp::collision::Environment<vamp::FloatVector<rake>>;
        using PRMSettings = vamp::planning::RoadmapSettings<vamp::planning::PRMStarNeighborParams>;
        using FCITSettings = vamp::planning::RoadmapSettings<vamp::planning::FCITStarNeighborParams>;

        using Configuration = typename Robot::Configuration;
        using RNG = vamp::rng::RNG<Robot>;
        using Roadmap = vamp::planning::Roadmap<Robot>;

        template <vamp::planning::Planner P>
        using PlannerT = vamp::planning::PlannerClass<Robot, rake, Robot::resolution, P>;

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return PlannerT<P>::solve(Input::to(start), Input::to(goal), EnvVec(env), s, rng);
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
            }
            return PlannerT<P>::solve(Input::to(start), goals_v, EnvVec(env), s, rng);
        }

        static auto simplify(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return vamp::planning::simplify<Robot, rake, Robot::resolution>(p, EnvVec(env), settings, rng);
        }

        static auto roadmap(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const PRMSettings &s,
            std::shared_ptr<Sampler> rng) -> Roadmap
        {
            return PlannerT<vamp::planning::Planner::PRM>::build_roadmap(
                Input::to(start), Input::to(goal), EnvVec(env), s, rng);
        }

        static auto roadmap_vertex(const Roadmap &r, std::size_t i)
        {
            return Input::from(r.vertices[i]);
        }

        static auto fk(const Cfg &c) -> std::vector<vamp::collision::Sphere<float>>
        {
            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c), out);

            std::vector<vamp::collision::Sphere<float>> result(Robot::n_spheres);
            for (auto i = 0U; i < Robot::n_spheres; ++i)
            {
                result[i] = vamp::collision::Sphere<float>{
                    out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}]};
            }
            return result;
        }

        static auto eefk(const Cfg &c, std::size_t end_effector) -> Eigen::Matrix4f
        {
            if constexpr (robot_has_end_effectors_v<Robot>)
            {
                if (end_effector >= Robot::end_effectors.size())
                {
                    throw std::out_of_range("end_effector index out of range");
                }

                return Robot::eefk(Input::array(c), end_effector).matrix();
            }
            else
            {
                if (end_effector != 0)
                {
                    throw std::out_of_range("end_effector index out of range");
                }

                return Robot::eefk(Input::array(c)).matrix();
            }
        }

        static auto debug(const Cfg &c, const Env &env) -> typename Robot::Debug
        {
            return Robot::fkcc_debug(EnvVec(env), Input::template block<rake>(c));
        }

        static auto validate(const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            auto configuration = Input::to(c);
            return (not check_bounds or in_bounds<Robot>(configuration)) and
                   vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, EnvVec(env));
        }

        static auto validate_motion(const Cfg &c_in, const Cfg &c_out, const Env &env, bool check_bounds)
            -> bool
        {
            auto cfg_in = Input::to(c_in);
            auto cfg_out = Input::to(c_out);
            return (not check_bounds or (in_bounds<Robot>(cfg_in) and in_bounds<Robot>(cfg_out))) and
                   vamp::planning::validate_motion<Robot, rake, 1>(cfg_in, cfg_out, EnvVec(env));
        }

        static auto
        filter_self_from_pointcloud(const Pc &pc, float point_radius, const Cfg &c, const Env &env) -> Pc
        {
            // TODO: Do this smarter with SIMD CC
            EnvVec ev(env);

            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c), out);

            Pc filtered;
            filtered.reserve(pc.size());

            for (const auto &point : pc)
            {
                const float x = point[0], y = point[1], z = point[2], r = point_radius;

                bool valid = true;
                for (auto i = 0U; i < Robot::n_spheres; ++i)
                {
                    if (vamp::collision::sphere_sphere_sql2(
                            out.x[{i, 0}], out.y[{i, 0}], out.z[{i, 0}], out.r[{i, 0}], x, y, z, r) < 0 or
                        vamp::sphere_environment_in_collision<>(ev, x, y, z, r))
                    {
                        valid = false;
                        break;
                    }
                }

                if (valid)
                {
                    filtered.emplace_back(point);
                }
            }

            return filtered;
        }

        static auto make_halton() -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::rng::Halton<Robot>>();
        }

        static auto make_xorshift() -> std::shared_ptr<Sampler>
        {
#if defined(__x86_64__)
            return std::make_shared<vamp::rng::XORShift<Robot>>();
#else
            throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
        }

        static void phs_construct(Phs *self, const Cfg &focus_a, const Cfg &focus_b)
        {
            new (self) Phs(Input::to(focus_a), Input::to(focus_b));
        }

        static auto make_phs_sampler(const Phs &phs, std::shared_ptr<Sampler> inner)
            -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::planning::ProlateHyperspheroidRNG<Robot>>(phs, inner);
        }

        static auto path_get(const Path &p, std::size_t i)
        {
            return Input::from(p[i]);
        }

        static auto path_validate(Path &p, const Env &e) -> bool
        {
            return p.template validate<rake>(EnvVec(e));
        }

        static auto path_numpy(const Path &p)
        {
            using ND = nanobind::ndarray<nanobind::numpy, const FloatT, nanobind::device::cpu>;
            const std::size_t slop = Robot::Configuration::num_scalars_rounded - Robot::dimension;
            return make_ndarray_filled<ND, 2>(
                {p.size(), Robot::dimension},
                [&](FloatT *dst)
                {
                    for (auto i = 0U; i < p.size(); ++i)
                    {
                        p[i].to_array_unaligned(dst + i * Robot::dimension);
                    }
                },
                slop);
        }

        static void path_set(Path &p, std::size_t i, const Cfg &c)
        {
            p[i] = Input::to(c);
        }

        static void path_append(Path &p, const Cfg &c)
        {
            p.emplace_back(Input::to(c));
        }

        static void path_insert(Path &p, std::size_t i, const Cfg &c)
        {
            p.insert(p.cbegin() + i, Input::to(c));
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.solved;
        }

        static void sampler_skip(Sampler &s, std::size_t n)
        {
            for (auto i = 0U; i < n; ++i)
            {
                s.next();
            }
        }

        static auto sampler_next(Sampler &s)
        {
            return Input::from(s.next());
        }

        static void phs_set_transverse_diameter(Phs &p, float d)
        {
            p.set_transverse_diameter(d);
        }

        static auto phs_transform(Phs &p, const Cfg &x)
        {
            return Input::from(p.transform(Input::to(x)));
        }
    };

    template <typename Robot>
    inline auto init_robot(nanobind::module_ &pymodule) -> nanobind::module_
    {
        using NA = NDArrayInput<Robot>;
        using CA = ArrayInput<Robot>;
        using TA = StaticRobotTraits<Robot, NA>;
        using TC = StaticRobotTraits<Robot, CA>;

        auto submodule = pymodule.def_submodule(Robot::name, "Robot-specific submodule");

        submodule.def("dimension", []() { return Robot::dimension; });
        submodule.def("resolution", []() { return Robot::resolution; });
        submodule.def("n_spheres", []() { return Robot::n_spheres; });
        submodule.def("space_measure", []() { return Robot::space_measure(); });
        submodule.def(
            "min_max_radii",
            []() -> std::pair<float, float> { return {Robot::min_radius, Robot::max_radius}; });
        submodule.def("joint_names", []() { return Robot::joint_names; });
        submodule.def("end_effector", []() { return Robot::end_effector; });
        submodule.def(
            "end_effectors",
            []()
            {
                std::vector<std::string> names;
                if constexpr (robot_has_end_effectors_v<Robot>)
                {
                    for (const auto &name : Robot::end_effectors)
                    {
                        names.emplace_back(name);
                    }
                }
                else
                {
                    names.emplace_back(Robot::end_effector);
                }

                return names;
            },
            "Frame names of the robot's end-effectors; `eefk` and `Attachment` take indices into this list.");

        if constexpr (robot_has_exempt_link_pairs_v<Robot>)
        {
            submodule.def(
                "link_names",
                []()
                {
                    std::vector<std::string> names;
                    for (const auto &name : Robot::link_names)
                    {
                        names.emplace_back(name);
                    }

                    return names;
                },
                "Link (frame) names; their positions are the ids used by `Attachment.allowed_links`.");

            submodule.def(
                "exempt_links",
                [](const std::vector<std::string> &names)
                {
                    std::vector<std::size_t> links;
                    const auto add = [&links](std::size_t link)
                    {
                        if (std::find(links.begin(), links.end(), link) == links.end())
                        {
                            links.emplace_back(link);
                        }
                    };

                    for (const auto &name : names)
                    {
                        const auto it = std::find(Robot::link_names.begin(), Robot::link_names.end(), name);
                        if (it == Robot::link_names.end())
                        {
                            throw std::invalid_argument("Unknown link '" + name + "'");
                        }

                        const auto link = static_cast<std::size_t>(it - Robot::link_names.begin());
                        add(link);
                        for (const auto &[a, b] : Robot::exempt_link_pairs)
                        {
                            if (a == link)
                            {
                                add(b);
                            }
                            else if (b == link)
                            {
                                add(a);
                            }
                        }
                    }

                    return links;
                },
                "names"_a,
                "Link ids for `Attachment.allowed_links`: the named links plus every link they may touch "
                "(their disabled collision pairs).");
        }

        const auto bounds = [](float fill) -> typename NA::Type
        {
            std::array<float, Robot::dimension> v;
            v.fill(fill);
            auto cfg = typename Robot::Configuration(v);
            Robot::scale_configuration(cfg);
            return NA::from(cfg);
        };
        submodule.def("upper_bounds", [bounds]() { return bounds(1.0F); });
        submodule.def("lower_bounds", [bounds]() { return bounds(0.0F); });

        bind_sampler<TA>(submodule, "RNG");

        auto phs_k = bind_phs_class<TA>(submodule, "ProlateHyperspheroid");
        bind_phs_io<TA>(phs_k);
        bind_phs_io<TC>(phs_k);

        auto path_k = bind_path_class<TA>(submodule, "Path");
        bind_path_io<TA>(path_k);
        bind_path_io<TC>(path_k);

        bind_planning_result<TA>(submodule, "PlanningResult");
        bind_roadmap_class<TA>(submodule, "Roadmap");

        bind_robot_methods<TA>(submodule);
        bind_robot_methods<TC>(submodule);

        if constexpr (has_set_lows_v<Robot>)
        {
            submodule.def("set_lows", &Robot::set_lows, "Set lower bounds.");
        }
        if constexpr (has_set_highs_v<Robot>)
        {
            submodule.def("set_highs", &Robot::set_highs, "Set upper bounds.");
        }
        if constexpr (has_set_radius_v<Robot>)
        {
            submodule.def("set_radius", &Robot::set_radius, "Set radius.");
        }

        return submodule;
    }
}  // namespace vamp::binding
