#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>

#include <vamp/collision/filter.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/aorrtc.hh>
#include <vamp/planning/constraints/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/chart_local_planner.hh>
#include <vamp/planning/constraints/closed_loop_constraint.hh>
#include <vamp/planning/constraints/com_constraint.hh>
#include <vamp/planning/constraints/constraint.hh>
#include <vamp/planning/constraints/constraint_set.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/fcit.hh>
#include <vamp/planning/grrtstar.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/prm.hh>
#include <vamp/planning/roadmap.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/planning/rrtc_settings.hh>
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

VAMP_DEFINE_HAS_METHOD(set_lows)
VAMP_DEFINE_HAS_METHOD(set_highs)
VAMP_DEFINE_HAS_METHOD(set_radius)
VAMP_DEFINE_HAS_METHOD(flask)
VAMP_DEFINE_HAS_METHOD(n_eef)
VAMP_DEFINE_HAS_METHOD(n_closed_loops)

// Robots with generated center-of-mass kinematics declare `static constexpr bool has_com`.
template <typename T, typename = void>
struct robot_has_com : std::false_type
{
};

template <typename T>
struct robot_has_com<T, std::void_t<decltype(T::has_com)>> : std::true_type
{
};

template <typename T>
constexpr bool robot_has_com_v = robot_has_com<T>::value;

// Nested-type analogue of VAMP_DEFINE_HAS_METHOD: flask z-robots that declare an ambient
// position-space sibling (using Ambient = ...) get chart-based constrained planning bound.
template <typename T, typename = void>
struct has_ambient : std::false_type
{
};

template <typename T>
struct has_ambient<T, std::void_t<typename T::Ambient>> : std::true_type
{
};

template <typename T>
constexpr bool has_ambient_v = has_ambient<T>::value;

namespace vamp::binding
{
    namespace nb = nanobind;
    using namespace nb::literals;

    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    template <typename Robot>
    struct NDArrayInput
    {
        using Type = nanobind::
            ndarray<FloatT, nanobind::numpy, nanobind::shape<Robot::dimension>, nanobind::device::cpu>;

        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t r>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<r>;

        inline static auto from(const Configuration &c) -> Type
        {
            auto c_arr = c.to_array();
            return make_ndarray<Type, 1>(c_arr.data(), {Robot::dimension});
        }

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(array(a));
        }

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            ConfigurationArray c;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Robot::dimension, scratch, "configuration");
            std::memcpy(c.data(), ptr, Robot::dimension * sizeof(float));
            return c;
        }

        template <std::size_t r>
        inline static auto block(const Type &a) -> ConfigurationBlock<r>
        {
            ConfigurationBlock<r> out;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Robot::dimension, scratch, "configuration");
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = ptr[i];
            }
            return out;
        }
    };

    template <typename Robot>
    struct ArrayInput
    {
        using Type = typename Robot::ConfigurationArray;

        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        template <std::size_t r>
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<r>;

        inline static auto from(const Configuration &c) -> Type
        {
            Type a;
            auto c_arr = c.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                a[i] = c_arr[i];
            }
            return a;
        }

        inline static auto to(const Type &a) -> Configuration
        {
            return Configuration(a);
        }

        inline static auto array(const Type &a) -> ConfigurationArray
        {
            return a;
        }

        template <std::size_t r>
        inline static auto block(const Type &a) -> ConfigurationBlock<r>
        {
            ConfigurationBlock<r> out;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                out[i] = a[i];
            }
            return out;
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

        // Constraint-aware variants. These members are only instantiated when referenced,
        // so robots without generated constraint support never touch them.
        using ConstraintT = vamp::planning::constraint::Constraint<Robot, rake>;
        using ConstraintVec = std::vector<std::shared_ptr<const ConstraintT>>;
        using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
        using ConstrainedLP =
            vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
        using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;

        static void check_on_manifold(const ConstrainedLP &lp, const Configuration &q, const char *what)
        {
            if (not lp.satisfied(q))
            {
                throw std::invalid_argument(
                    std::string(what) +
                    " configuration violates the constraints; project it first with project()");
            }
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single_constrained(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            ConstraintVec constraints,
            const ConstraintSettings &cs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return solve_single<P, Settings>(start, goal, env, s, rng);
            }

            const ConstrainedLP lp(ConstraintSetT(std::move(constraints), cs));
            const auto start_c = Input::to(start);
            const auto goal_c = Input::to(goal);
            check_on_manifold(lp, start_c, "start");
            check_on_manifold(lp, goal_c, "goal");
            return PlannerT<P>::solve(start_c, goal_c, EnvVec(env), s, rng, lp);
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi_constrained(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            ConstraintVec constraints,
            const ConstraintSettings &cs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return solve_multi<P, Settings>(start, goals, env, s, rng);
            }

            const ConstrainedLP lp(ConstraintSetT(std::move(constraints), cs));
            const auto start_c = Input::to(start);
            check_on_manifold(lp, start_c, "start");

            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
                check_on_manifold(lp, goals_v.back(), "goal");
            }

            return PlannerT<P>::solve(start_c, goals_v, EnvVec(env), s, rng, lp);
        }

        static auto simplify_constrained(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng,
            ConstraintVec constraints,
            const ConstraintSettings &cs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return simplify(p, env, settings, rng);
            }

            const ConstrainedLP lp(ConstraintSetT(std::move(constraints), cs));
            for (std::size_t i = 0; i < p.size(); ++i)
            {
                if (not lp.satisfied(p[i]))
                {
                    throw std::invalid_argument(
                        "path state " + std::to_string(i) +
                        " violates the constraints; simplify requires an on-manifold path");
                }
            }

            return vamp::planning::simplify<Robot, rake, Robot::resolution>(
                p, EnvVec(env), settings, rng, lp);
        }

        static auto constraint_project(const Cfg &c, ConstraintVec constraints, const ConstraintSettings &cs)
            -> Cfg
        {
            const ConstraintSetT set(std::move(constraints), cs);
            auto q = Input::to(c);
            if (not set.project(q))
            {
                throw std::invalid_argument("projection onto the constraint manifold did not converge");
            }

            return Input::from(q);
        }

        static auto
        constraint_satisfied(const Cfg &c, ConstraintVec constraints, const ConstraintSettings &cs) -> bool
        {
            return ConstraintSetT(std::move(constraints), cs).satisfied(Input::to(c));
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

        static auto eefk(const Cfg &c) -> Eigen::Matrix4f
        {
            return Robot::eefk(Input::array(c)).matrix();
        }

        static auto debug(const Cfg &c, const Env &env) -> typename Robot::Debug
        {
            return Robot::fkcc_debug(EnvVec(env), Input::template block<rake>(c));
        }

        static auto validate(const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            auto configuration = Input::to(c);
            return (not check_bounds or Robot::in_bounds(configuration.trim())) and
                   vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, EnvVec(env));
        }

        static auto validate_motion(const Cfg &c_in, const Cfg &c_out, const Env &env, bool check_bounds)
            -> bool
        {
            auto cfg_in = Input::to(c_in);
            auto cfg_out = Input::to(c_out);
            return (not check_bounds
                    or (Robot::in_bounds(cfg_in.trim()) and Robot::in_bounds(cfg_out.trim()))) and
                   vamp::planning::validate_motion<Robot, rake, 1>(cfg_in, cfg_out, EnvVec(env));
        }

        static auto
        filter_self_from_pointcloud(const Pc &pc, float point_radius, const Cfg &c, const Env &env)
            -> std::vector<vamp::collision::Point>
        {
            std::vector<vamp::collision::Point> out;
            vamp::collision::filter_self_from_pointcloud<Robot, rake>(
                pc.empty() ? nullptr : pc.front().data(),
                pc.size(),
                point_radius,
                Input::template block<1>(c),
                EnvVec(env),
                out);
            return out;
        }

        static auto make_halton() -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::rng::Halton<Robot>>();
        }

        static auto make_xorshift(std::uint64_t seed) -> std::shared_ptr<Sampler>
        {
#if defined(__x86_64__)
            return (seed == 0) ? std::make_shared<vamp::rng::XORShift<Robot>>() :
                                 std::make_shared<vamp::rng::XORShift<Robot>>(seed, seed + 1);
#else
            throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
        }

        static auto make_phs(const Cfg &focus_a, const Cfg &focus_b) -> std::shared_ptr<Phs>
        {
            return std::make_shared<Phs>(Input::to(focus_a), Input::to(focus_b));
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
            return r.path.size() >= 2;
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

    // Chart-based constrained planning for flask z-robots with an ambient sibling.
    // Kept separate from StaticRobotTraits: member declarations here reference
    // typename Robot::Ambient, which would hard-error at class instantiation for
    // robots without one. Only referenced inside if constexpr (has_ambient_v<Robot>).
    template <typename Robot, typename Input>
    struct ChartRobotTraits
    {
        using Base = StaticRobotTraits<Robot, Input>;
        using Ambient = typename Robot::Ambient;

        using Cfg = typename Base::Cfg;
        using Pth = typename Base::Pth;
        using Path = typename Base::Path;
        using PlanningResult = typename Base::PlanningResult;
        using Sampler = typename Base::Sampler;
        using Env = typename Base::Env;
        using EnvVec = typename Base::EnvVec;
        using Configuration = typename Robot::Configuration;

        template <vamp::planning::Planner P>
        using PlannerT = typename Base::template PlannerT<P>;

        using ChartLP = vamp::planning::constraint::ChartLocalPlanner<Robot, rake, Robot::resolution>;
        using AmbientConstraintT = vamp::planning::constraint::Constraint<Ambient, rake>;
        using AmbientConstraintVec = std::vector<std::shared_ptr<const AmbientConstraintT>>;
        using AmbientConstraintSetT = vamp::planning::constraint::ConstraintSet<Ambient, rake>;
        using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;
        using ChartSettings = vamp::planning::constraint::ChartSettings;

        static auto make_lp(
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> ChartLP
        {
            return ChartLP(AmbientConstraintSetT(std::move(constraints), cs), chs);
        }

        static void check_on_manifold(const ChartLP &lp, const Configuration &z, const char *what)
        {
            if (not lp.satisfied(z))
            {
                throw std::invalid_argument(
                    std::string(what) +
                    " state violates the constraints; project it first with project()");
            }
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single_chart(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::template solve_single<P, Settings>(start, goal, env, s, rng);
            }

            const auto lp = make_lp(std::move(constraints), cs, chs);
            const auto start_c = Input::to(start);
            const auto goal_c = Input::to(goal);
            check_on_manifold(lp, start_c, "start");
            check_on_manifold(lp, goal_c, "goal");
            return PlannerT<P>::solve(start_c, goal_c, EnvVec(env), s, rng, lp);
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi_chart(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::template solve_multi<P, Settings>(start, goals, env, s, rng);
            }

            const auto lp = make_lp(std::move(constraints), cs, chs);
            const auto start_c = Input::to(start);
            check_on_manifold(lp, start_c, "start");

            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
                check_on_manifold(lp, goals_v.back(), "goal");
            }

            return PlannerT<P>::solve(start_c, goals_v, EnvVec(env), s, rng, lp);
        }

        static auto simplify_chart(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::simplify(p, env, settings, rng);
            }

            const auto lp = make_lp(std::move(constraints), cs, chs);
            for (std::size_t i = 0; i < p.size(); ++i)
            {
                if (not lp.satisfied(p[i]))
                {
                    throw std::invalid_argument(
                        "path state " + std::to_string(i) +
                        " violates the constraints; simplify requires an on-manifold path");
                }
            }

            return vamp::planning::simplify<Robot, rake, Robot::resolution>(
                p, EnvVec(env), settings, rng, lp);
        }

        static auto chart_project(
            const Cfg &c,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> Cfg
        {
            const auto lp = make_lp(std::move(constraints), cs, chs);
            auto z = Input::to(c);
            if (not lp.project(z))
            {
                throw std::invalid_argument("projection onto the constraint manifold did not converge");
            }

            return Input::from(z);
        }

        static auto
        chart_satisfied(const Cfg &c, AmbientConstraintVec constraints, const ConstraintSettings &cs)
            -> bool
        {
            return make_lp(std::move(constraints), cs, {}).satisfied(Input::to(c));
        }

        static auto debug_chart(
            const Cfg &c,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs) -> std::vector<std::array<float, Ambient::dimension>>
        {
            return make_lp(std::move(constraints), cs, chs).debug_chart(Input::to(c));
        }

        static auto lift_edge(
            const Cfg &from,
            const Cfg &target,
            AmbientConstraintVec constraints,
            bool forward,
            std::size_t n_samples,
            const ConstraintSettings &cs,
            const ChartSettings &chs)
            -> std::tuple<std::vector<Cfg>, std::vector<float>, float, float>
        {
            const auto lp = make_lp(std::move(constraints), cs, chs);

            std::vector<Configuration> states;
            std::vector<float> errors;
            float duration = 0.F, cost = 0.F;
            if (not lp.debug_lift_edge(
                    Input::to(from), Input::to(target), forward, n_samples, states, errors, duration, cost))
            {
                throw std::invalid_argument(
                    "edge lift failed: no chart at the from state, a lift diverged, or the "
                    "time-optimal solve failed");
            }

            std::vector<Cfg> out;
            out.reserve(states.size());
            for (const auto &s : states)
            {
                out.emplace_back(Input::from(s));
            }

            return {std::move(out), std::move(errors), duration, cost};
        }
    };

    template <typename Robot, typename Input>
    inline void bind_flask_methods(nanobind::module_ &submodule)
    {
        using Cfg = typename Input::Type;
        constexpr auto n_q = Robot::flat_dimension;

        submodule.def(
            "optimal_time",
            [](const Cfg &a, const Cfg &b) { return Robot::optimal_time(Input::to(a), Input::to(b)); },
            "a"_a,
            "b"_a,
            "Optimal duration T* of the local trajectory between flat states a and b.");

        submodule.def(
            "cost",
            [](const Cfg &a, const Cfg &b) { return Robot::cost(Input::to(a), Input::to(b)); },
            "a"_a,
            "b"_a,
            "LQMT edge cost C_loc(a -> b) = rho T* + integral |u|^2 of the local trajectory; "
            "asymmetric in (a, b).");

        submodule.def(
            "cost_grad",
            [](const Cfg &a, const Cfg &b)
            {
                const auto g = Robot::cost_grad(Input::to(a), Input::to(b));
                return std::make_tuple(g.cost, g.time, g.grad_a, g.grad_b);
            },
            "a"_a,
            "b"_a,
            "(cost, time, dC/da, dC/db) for the LQMT edge a -> b; gradients are laid out to "
            "match the Configuration = (q, v) memory ordering.");

        submodule.def(
            "eval",
            [](const Cfg &a, const Cfg &b, float T, float t)
            {
                const auto x = Robot::eval(Input::to(a), Input::to(b), T, t);
                std::array<float, n_q> y, yd, ydd;
                std::copy_n(x.begin(), n_q, y.begin());
                std::copy_n(x.begin() + n_q, n_q, yd.begin());
                std::copy_n(x.begin() + 2 * n_q, n_q, ydd.begin());
                return std::make_tuple(y, yd, ydd);
            },
            "a"_a,
            "b"_a,
            "T"_a,
            "t"_a,
            "Evaluate the trajectory from a to b of duration T at fraction t in [0, 1]; returns "
            "(position, velocity, acceleration).");
    }

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

        bind_robot_methods<TA>(submodule);
        bind_robot_methods<TC>(submodule);

        if constexpr (has_n_eef_v<Robot> or has_n_closed_loops_v<Robot> or robot_has_com_v<Robot>)
        {
            namespace vc = vamp::planning::constraint;
            using ConstraintT = vc::Constraint<Robot, rake>;

            nb::class_<ConstraintT>(
                submodule,
                "Constraint",
                "Manifold constraint over robot configurations. Constraints cache per-evaluation "
                "state and are not thread-safe: do not share one instance across concurrent "
                "planning calls.");

            if constexpr (has_n_eef_v<Robot>)
            {
                using TSC = vc::TaskSpaceConstraint<Robot, rake>;
                using Transform = typename TSC::Transform;
                using Bound = typename TSC::Bound;

                submodule.def("n_eef", []() { return Robot::n_eef; });

                auto tsc_k =
                    nb::class_<TSC, ConstraintT>(
                        submodule,
                        "TaskSpaceConstraint",
                        "Task Space Region constraint: for each end-effector, the pose of an "
                        "offset frame (eef_to_offset, in the end-effector frame) must lie within "
                        "[lower, upper] se(3) bounds of a reference frame (world_to_reference, in "
                        "the world frame). Transforms are (qw, qx, qy, qz, x, y, z).")
                        .def(
                            nb::init<
                                const std::array<Transform, Robot::n_eef> &,
                                const std::array<Transform, Robot::n_eef> &,
                                const std::array<Bound, Robot::n_eef> &,
                                const std::array<Bound, Robot::n_eef> &>(),
                            "eef_to_offset"_a,
                            "world_to_reference"_a,
                            "lower"_a,
                            "upper"_a);

                if constexpr (Robot::n_eef == 1)
                {
                    tsc_k.def(
                        "__init__",
                        [](TSC *t,
                           const Transform &eef_to_offset,
                           const Transform &world_to_reference,
                           const Bound &lower,
                           const Bound &upper) {
                            new (t) TSC(
                                std::array<Transform, 1>{eef_to_offset},
                                std::array<Transform, 1>{world_to_reference},
                                std::array<Bound, 1>{lower},
                                std::array<Bound, 1>{upper});
                        },
                        "eef_to_offset"_a,
                        "world_to_reference"_a,
                        "lower"_a,
                        "upper"_a,
                        "Single end-effector convenience constructor: unwrapped transforms and "
                        "bounds.");
                }

                if constexpr (Robot::n_eef >= 2)
                {
                    using BTSC = vc::BimanualTaskSpaceConstraint<Robot, rake>;
                    nb::class_<BTSC, ConstraintT>(
                        submodule,
                        "BimanualTaskSpaceConstraint",
                        "Relative pose constraint between two end-effectors: the pose of "
                        "end-effector 1 in the frame of end-effector 0 must lie within "
                        "[lower, upper] se(3) bounds of right_in_left. Transforms are "
                        "(qw, qx, qy, qz, x, y, z).")
                        .def(
                            nb::init<const Transform &, const Bound &, const Bound &>(),
                            "right_in_left"_a,
                            "lower"_a,
                            "upper"_a);
                }
            }

            if constexpr (has_n_closed_loops_v<Robot>)
            {
                using CLC = vc::ClosedLoopConstraint<Robot, rake>;

                submodule.def("n_closed_loops", []() { return Robot::n_closed_loops; });

                nb::class_<CLC, ConstraintT>(
                    submodule,
                    "ClosedLoopConstraint",
                    "Loop-closure constraint: each cut kinematic loop of the robot contributes "
                    "one equality row keeping the distance between its cut frames at the "
                    "loop's fixed length.")
                    .def(nb::init<>());
            }

            if constexpr (robot_has_com_v<Robot>)
            {
                using CMC = vc::CoMConstraint<Robot, rake>;

                nb::class_<CMC, ConstraintT>(
                    submodule,
                    "CoMConstraint",
                    "Support-polygon constraint on the center of mass: the xy projection of "
                    "the CoM (in the robot's CoM reference frame) must lie inside the convex "
                    "polygon given by its vertices in counterclockwise order.")
                    .def(
                        nb::init<const std::vector<typename CMC::Vertex> &>(),
                        "polygon"_a);
            }

            bind_constraint_methods<TA>(submodule);
            bind_constraint_methods<TC>(submodule);
        }

        if constexpr (has_flask_v<Robot>)
        {
            constexpr auto n_q = Robot::flat_dimension;

            submodule.def("flask", []() { return Robot::flask; });
            submodule.def("flat_dimension", []() { return Robot::flat_dimension; });
            submodule.def("rho", []() { return Robot::rho; });
            submodule.def("set_rho", [](float r) { Robot::rho = r; }, "rho"_a);
            submodule.def("velocity_limits", []() { return Robot::velocity_limits; });
            submodule.def("effort_limits", []() { return Robot::effort_limits; });

            bind_flask_methods<Robot, NA>(submodule);
            bind_flask_methods<Robot, CA>(submodule);

            if constexpr (has_ambient_v<Robot>)
            {
                submodule.def(
                    "ambient",
                    []() { return std::string(Robot::Ambient::name); },
                    "Name of the ambient position-space sibling robot whose constraints this "
                    "z-robot plans with.");

                bind_chart_methods<ChartRobotTraits<Robot, NA>>(submodule);
                bind_chart_methods<ChartRobotTraits<Robot, CA>>(submodule);
            }

            using XType = nanobind::
                ndarray<FloatT, nanobind::numpy, nanobind::shape<3 * n_q>, nanobind::device::cpu>;
            const char *torques_doc = "Joint torques for a flat state stack x = [q; qd; qdd].";
            submodule.def(
                "torques",
                [](const XType &x)
                {
                    std::array<float, 3 * n_q> xa;
                    std::vector<float> scratch;
                    const auto *ptr = as_flat_1d(x, 3 * n_q, scratch, "flat state");
                    std::memcpy(xa.data(), ptr, 3 * n_q * sizeof(float));
                    return Robot::torques(xa);
                },
                "x"_a,
                torques_doc);
            submodule.def(
                "torques",
                [](const std::array<float, 3 * n_q> &x) { return Robot::torques(x); },
                "x"_a,
                torques_doc);
        }

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
