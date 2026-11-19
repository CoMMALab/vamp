#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>

#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/manifold/closed_loop_constraint.hh>
#include <vamp/planning/constraints/manifold/com_constraint.hh>
#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/lead_screw_constraint.hh>
#include <vamp/planning/constraints/manifold/twist_constraint.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/planners/fcit.hh>
#include <vamp/planning/planners/grrtstar.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/prm.hh>
#include <vamp/planning/planners/roadmap.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
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
VAMP_DEFINE_HAS_METHOD(n_eef)
VAMP_DEFINE_HAS_METHOD(n_closed_loops)
VAMP_DEFINE_HAS_METHOD(n_cuboids)

// Robots with generated optional kinematics (center-of-mass, lead-screw, twist-Jacobian)
// declare `static constexpr bool has_<kernel>`.
#define VAMP_DEFINE_ROBOT_FLAG(flag)                                                                         \
    template <typename T, typename = void>                                                                   \
    struct robot_##flag : std::false_type                                                                    \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    struct robot_##flag<T, std::void_t<decltype(T::flag)>> : std::true_type                                  \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    constexpr bool robot_##flag##_v = robot_##flag<T>::value;

VAMP_DEFINE_ROBOT_FLAG(has_com)
VAMP_DEFINE_ROBOT_FLAG(has_lead_screw)
VAMP_DEFINE_ROBOT_FLAG(has_twist)

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

    // Number of cuboids in the robot's collision model. Robots generated before cuboid support do not
    // define `n_cuboids`; they have none.
    template <typename Robot>
    inline constexpr std::size_t n_cuboids_v = []() constexpr -> std::size_t
    {
        if constexpr (has_n_cuboids_v<Robot>)
        {
            return Robot::n_cuboids;
        }
        else
        {
            return 0;
        }
    }();

    // The one place a Python-side environment is turned into the SIMD environment used for collision
    // checking. Cuboids, whether on the robot or in an attachment, cannot be checked against pointclouds
    // (see vamp::check_cuboid_environment_supported), and the checking functions are noexcept, so the
    // unsupported case has to be caught here to surface as a Python RuntimeError instead of aborting the
    // interpreter.
    template <typename Robot>
    inline auto make_environment(const vamp::collision::Environment<float> &environment)
        -> vamp::collision::Environment<vamp::FloatVector<rake>>
    {
        bool attachment_has_cuboids = false;
        for (const auto &attachment : environment.attachments)
        {
            attachment_has_cuboids = attachment_has_cuboids or not attachment.cuboids.empty();
        }

        if (n_cuboids_v<Robot> > 0 or attachment_has_cuboids)
        {
            vamp::check_cuboid_environment_supported(environment);
        }

        return vamp::collision::Environment<vamp::FloatVector<rake>>(environment);
    }

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
            return PlannerT<P>::solve(Input::to(start), Input::to(goal), make_environment<Robot>(env), s, rng);
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
            return PlannerT<P>::solve(Input::to(start), goals_v, make_environment<Robot>(env), s, rng);
        }

        static auto simplify(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return vamp::planning::simplify<Robot, rake, Robot::resolution>(p, make_environment<Robot>(env), settings, rng);
        }

        // Shared skeleton of the constrained planner entry points: convert
        // inputs, enforce per-state feasibility with the family's remedy message, and
        // dispatch to the local-planner-aware solver. Callers handle their empty-set
        // fallbacks before building an LP.
        template <typename LP>
        static void
        check_feasible(const LP &lp, const Configuration &q, const char *what, const char *message)
        {
            if (not lp.satisfied(q))
            {
                throw std::invalid_argument(std::string(what) + message);
            }
        }

        template <vamp::planning::Planner P, typename Settings, typename LP>
        static auto solve_single_lp(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            const LP &lp,
            const char *message) -> PlanningResult
        {
            const auto start_c = Input::to(start);
            const auto goal_c = Input::to(goal);
            check_feasible(lp, start_c, "start", message);
            check_feasible(lp, goal_c, "goal", message);
            return PlannerT<P>::solve(start_c, goal_c, make_environment<Robot>(env), s, rng, lp);
        }

        template <vamp::planning::Planner P, typename Settings, typename LP>
        static auto solve_multi_lp(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            const LP &lp,
            const char *message) -> PlanningResult
        {
            const auto start_c = Input::to(start);
            check_feasible(lp, start_c, "start", message);

            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
                check_feasible(lp, goals_v.back(), "goal", message);
            }

            return PlannerT<P>::solve(start_c, goals_v, make_environment<Robot>(env), s, rng, lp);
        }

        template <typename LP>
        static auto simplify_lp(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng,
            const LP &lp,
            const char *message) -> PlanningResult
        {
            for (std::size_t i = 0; i < p.size(); ++i)
            {
                if (not lp.satisfied(p[i]))
                {
                    throw std::invalid_argument("path state " + std::to_string(i) + message);
                }
            }

            return vamp::planning::simplify<Robot, rake, Robot::resolution>(
                p, make_environment<Robot>(env), settings, rng, lp);
        }

        // Constraint-aware variants. These members are only instantiated when referenced,
        // so robots without generated constraint support never touch them.
        using ConstraintT = vamp::planning::constraint::Constraint<Robot, rake>;
        using ConstraintVec = std::vector<std::shared_ptr<const ConstraintT>>;
        using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
        using ConstrainedLP =
            vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
        using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;

        static constexpr const char *manifold_message = " configuration violates the constraints; project it "
                                                        "first with project()";
        static constexpr const char *manifold_simplify_message = " violates the constraints; simplify "
                                                                 "requires an on-manifold path";

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

            ConstraintSetT set(std::move(constraints), cs);
            return solve_single_lp<P>(
                start, goal, env, s, rng, ConstrainedLP(std::move(set)), manifold_message);
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
            ConstraintSetT set(std::move(constraints), cs);
            return solve_multi_lp<P>(
                start, goals, env, s, rng, ConstrainedLP(std::move(set)), manifold_message);
        }

        // Plan from `start` to a task-space region (the constraints, e.g. a TSR) with no goal
        // configuration: see TaskSpaceRRTC.
        static auto solve_task_space(
            const Cfg &start,
            ConstraintVec constraints,
            const Env &env,
            const vamp::planning::TaskSpaceRRTCSettings &s,
            std::shared_ptr<Sampler> rng,
            const ConstraintSettings &cs) -> PlanningResult
        {
            if (constraints.empty())
            {
                throw std::invalid_argument("task-space planning needs at least one goal constraint");
            }

            ConstraintSetT region(std::move(constraints), cs);
            return vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution>::solve(
                Input::to(start), region, make_environment<Robot>(env), s, rng);
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

            ConstraintSetT set(std::move(constraints), cs);
            return simplify_lp(
                p, env, settings, rng, ConstrainedLP(std::move(set)), manifold_simplify_message);
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

        static auto roadmap(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const PRMSettings &s,
            std::shared_ptr<Sampler> rng) -> Roadmap
        {
            return PlannerT<vamp::planning::Planner::PRM>::build_roadmap(
                Input::to(start), Input::to(goal), make_environment<Robot>(env), s, rng);
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

        // Robot cuboids posed at a configuration, as the same Cuboid type environments use (center, three
        // axes, and half-lengths), so they can be drawn like any other cuboid. Empty for robots without
        // cuboids.
        static auto cuboid_fk(const Cfg &c) -> std::vector<vamp::collision::Cuboid<float>>
        {
            std::vector<vamp::collision::Cuboid<float>> result;

            if constexpr (n_cuboids_v<Robot> > 0)
            {
                typename Robot::template Cuboids<1> out;
                Robot::template cuboid_fk<1>(Input::template block<1>(c), out);

                result.reserve(Robot::n_cuboids);
                for (auto i = 0U; i < Robot::n_cuboids; ++i)
                {
                    result.emplace_back(
                        out.x[{i, 0}],
                        out.y[{i, 0}],
                        out.z[{i, 0}],
                        out.axis_1_x[{i, 0}],
                        out.axis_1_y[{i, 0}],
                        out.axis_1_z[{i, 0}],
                        out.axis_2_x[{i, 0}],
                        out.axis_2_y[{i, 0}],
                        out.axis_2_z[{i, 0}],
                        out.axis_3_x[{i, 0}],
                        out.axis_3_y[{i, 0}],
                        out.axis_3_z[{i, 0}],
                        out.axis_1_r[{i, 0}],
                        out.axis_2_r[{i, 0}],
                        out.axis_3_r[{i, 0}]);
                }
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
            return Robot::fkcc_debug(make_environment<Robot>(env), Input::template block<rake>(c));
        }

        static auto validate(const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            auto configuration = Input::to(c);
            return (not check_bounds or in_bounds<Robot>(configuration)) and
                   vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, make_environment<Robot>(env));
        }

        static auto validate_motion(const Cfg &c_in, const Cfg &c_out, const Env &env, bool check_bounds)
            -> bool
        {
            auto cfg_in = Input::to(c_in);
            auto cfg_out = Input::to(c_out);
            return (not check_bounds or (in_bounds<Robot>(cfg_in) and in_bounds<Robot>(cfg_out))) and
                   vamp::planning::validate_motion<Robot, rake, 1>(cfg_in, cfg_out, make_environment<Robot>(env));
        }

        static auto
        filter_self_from_pointcloud(const Pc &pc, float point_radius, const Cfg &c, const Env &env) -> Pc
        {
            // TODO: Do this smarter with SIMD CC
            EnvVec ev(env);

            // NOTE: Only points are checked against the environment here (as spheres), never robot
            // cuboids, so unlike the other entry points this is fine for pointcloud environments and
            // does not go through make_environment.
            typename Robot::template Spheres<1> out;
            Robot::template sphere_fk<1>(Input::template block<1>(c), out);

            // Robot cuboids; empty for robots without any
            const auto robot_cuboids = cuboid_fk(c);

            Pc filtered;
            filtered.reserve(pc.size());

            for (const auto &point : pc)
            {
                const float x = point[0], y = point[1], z = point[2], r = point_radius;

                bool valid = true;
                for (auto k = 0U; k < Robot::n_spheres; ++k)
                {
                    if (vamp::collision::sphere_sphere_sql2(
                            out.x[{k, 0}], out.y[{k, 0}], out.z[{k, 0}], out.r[{k, 0}], x, y, z, r) < 0 or
                        vamp::sphere_environment_in_collision<>(ev, x, y, z, r))
                    {
                        valid = false;
                        break;
                    }
                }

                // Sphere (the point, inflated by point_radius) vs. each robot cuboid
                for (const auto &cuboid : robot_cuboids)
                {
                    if (not valid)
                    {
                        break;
                    }

                    const float xs = x - cuboid.x, ys = y - cuboid.y, zs = z - cuboid.z;
                    const float a1 = std::max(
                        std::abs(cuboid.axis_1_x * xs + cuboid.axis_1_y * ys + cuboid.axis_1_z * zs) -
                            cuboid.axis_1_r,
                        0.F);
                    const float a2 = std::max(
                        std::abs(cuboid.axis_2_x * xs + cuboid.axis_2_y * ys + cuboid.axis_2_z * zs) -
                            cuboid.axis_2_r,
                        0.F);
                    const float a3 = std::max(
                        std::abs(cuboid.axis_3_x * xs + cuboid.axis_3_y * ys + cuboid.axis_3_z * zs) -
                            cuboid.axis_3_r,
                        0.F);

                    if (a1 * a1 + a2 * a2 + a3 * a3 < r * r)
                    {
                        valid = false;
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
            return p.template validate<rake>(make_environment<Robot>(e));
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
    // `submodule_name` is the name the robot is exposed under in Python. The build uses the same name
    // (the robot's header name) to list robots and generate stubs, so it has to be what the module is
    // registered as. It defaults to the robot's own name; they differ for generated robots whose name has
    // no underscore but whose header does (e.g., KMRIiwaTool is named "kmriiwatool" but lives in
    // kmriiwa_tool.hh).
    inline auto init_robot(nanobind::module_ &pymodule, const char *submodule_name = Robot::name)
        -> nanobind::module_
    {
        using NA = NDArrayInput<Robot>;
        using CA = ArrayInput<Robot>;
        using TA = StaticRobotTraits<Robot, NA>;
        using TC = StaticRobotTraits<Robot, CA>;

        auto submodule = pymodule.def_submodule(submodule_name, "Robot-specific submodule");

        submodule.def("dimension", []() { return Robot::dimension; });
        submodule.def("resolution", []() { return Robot::resolution; });
        submodule.def("n_spheres", []() { return Robot::n_spheres; });
        submodule.def("n_cuboids", []() { return n_cuboids_v<Robot>; });
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

        if constexpr (
            has_n_eef_v<Robot> or has_n_closed_loops_v<Robot> or robot_has_com_v<Robot> or
            robot_has_lead_screw_v<Robot> or robot_has_twist_v<Robot>)
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

                auto tsc_k = nb::class_<TSC, ConstraintT>(
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
                           const Bound &upper)
                        {
                            new (t)
                                TSC(std::array<Transform, 1>{eef_to_offset},
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
                    .def(nb::init<const std::vector<typename CMC::Vertex> &>(), "polygon"_a);
            }

            if constexpr (robot_has_twist_v<Robot>)
            {
                using TWC = vc::TwistConstraint<Robot, rake, 1>;
                using LSC = vc::LeadScrewConstraint<Robot, rake>;
                using Transform = typename TWC::Transform;
                using CoefficientRow = std::array<float, 6>;

                nb::class_<TWC, ConstraintT>(
                    submodule,
                    "TwistConstraint",
                    "Constant-coefficient Pfaffian velocity constraint over the end-effector "
                    "twist: reference_coefficients . twist_ref + body_coefficients . "
                    "twist_loc = 0, where twist_ref and twist_loc are the [linear; angular] "
                    "twist of the offset end-effector frame (eef_to_offset, in the "
                    "end-effector frame) expressed in the reference frame's axes "
                    "(world_to_reference, in the world frame) and in the frame's own body "
                    "axes. Transforms are (qw, qx, qy, qz, x, y, z). Contributes no position "
                    "error, and is smooth for unbounded rotation.")
                    .def(
                        "__init__",
                        [](TWC *t,
                           const Transform &eef_to_offset,
                           const Transform &world_to_reference,
                           const CoefficientRow &reference_coefficients,
                           const CoefficientRow &body_coefficients)
                        {
                            new (t)
                                TWC(eef_to_offset,
                                    world_to_reference,
                                    typename TWC::Coefficients{reference_coefficients},
                                    typename TWC::Coefficients{body_coefficients});
                        },
                        "eef_to_offset"_a,
                        "world_to_reference"_a,
                        "reference_coefficients"_a,
                        "body_coefficients"_a);

                nb::class_<LSC, TWC>(
                    submodule,
                    "LeadScrewConstraint",
                    "Pfaffian lead-screw coupling: velocities of an offset end-effector frame "
                    "(eef_to_offset, in the end-effector frame) are restricted so translation "
                    "along the reference frame's z-axis (world_to_reference, in the world "
                    "frame) advances by pitch per full turn about it. Transforms are "
                    "(qw, qx, qy, qz, x, y, z). Contributes no position error.")
                    .def(
                        nb::init<const Transform &, const Transform &, float>(),
                        "eef_to_offset"_a,
                        "world_to_reference"_a,
                        "pitch"_a);
            }

            if constexpr (robot_has_lead_screw_v<Robot>)
            {
                using LSL = vc::LeadScrewLevelConstraint<Robot, rake>;
                using Transform = typename LSL::Transform;

                nb::class_<LSL, ConstraintT>(
                    submodule,
                    "LeadScrewLevelConstraint",
                    "Holonomic form of the lead-screw coupling: pins the screw invariant "
                    "h(q) = z-advance - (pitch / 2 pi) * rotation of the offset end-effector "
                    "frame in the reference frame to a target level. Transforms are "
                    "(qw, qx, qy, qz, x, y, z).")
                    .def(
                        nb::init<const Transform &, const Transform &, float, float>(),
                        "eef_to_offset"_a,
                        "world_to_reference"_a,
                        "pitch"_a,
                        "target"_a);
            }

            bind_constraint_methods<TA>(submodule);
            bind_constraint_methods<TC>(submodule);
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
