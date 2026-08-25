#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/bindings/python/parameterized_space_binder.hh>

#include <vamp/collision/filter.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/chart_local_planner.hh>
#include <vamp/planning/constraints/manifold/closed_loop_constraint.hh>
#include <vamp/planning/constraints/manifold/com_constraint.hh>
#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/lead_screw_constraint.hh>
#include <vamp/planning/constraints/manifold/twist_constraint.hh>
#include <vamp/planning/constraints/phase/eef_speed_constraint.hh>
#include <vamp/planning/constraints/phase/kinetic_energy_constraint.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/phase/phase_constraint.hh>
#include <vamp/planning/constraints/phase/phase_constraint_set.hh>
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
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/ke_shaped.hh>
#include <vamp/random/pinned.hh>
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
VAMP_DEFINE_HAS_METHOD(kinetic_energy)
VAMP_DEFINE_HAS_METHOD(n_end_effectors)

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

// Flask z-robots that declare an ambient position-space sibling (using Ambient = ...) get
// chart-based constrained planning bound; robots generated with a FLASK block carry their
// z-space sibling as a nested Flask struct.
VAMP_DEFINE_HAS_TYPE(Ambient)
VAMP_DEFINE_HAS_TYPE(Flask)
VAMP_DEFINE_HAS_TYPE(ParameterizedSpace)

// The is_same check rejects the injected-class-name: for the nested Flask struct itself,
// T::Flask names T, which would otherwise recurse init_robot forever.
template <typename T, bool = has_type_Flask_v<T>>
struct has_flask_robot : std::false_type
{
};

template <typename T>
struct has_flask_robot<T, true> : std::bool_constant<not std::is_same_v<typename T::Flask, T>>
{
};

template <typename T>
constexpr bool has_flask_robot_v = has_flask_robot<T>::value;

namespace vamp::binding
{
    namespace nb = nanobind;
    using namespace nb::literals;

    // `rake` is already defined in this namespace by parameterized_space_helper.hh
    // (included transitively via parameterized_space_binder.hh above); redefining it
    // here would be an ODR violation in any TU that includes both.

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
        using Type = typename Robot::ConfigurationArray;

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

        static void check_pins(
            const std::shared_ptr<Sampler> &rng,
            const Configuration &c,
            const char *which)
        {
            if (auto pinned = std::dynamic_pointer_cast<vamp::rng::PinnedRNG<Robot>>(rng))
            {
                if (not pinned->matches(c))
                {
                    throw std::invalid_argument(
                        std::string(which) +
                        " configuration does not match the sampler's pinned joint values!");
                }
            }
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            check_pins(rng, Input::to(start), "Start");
            check_pins(rng, Input::to(goal), "Goal");
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
            check_pins(rng, Input::to(start), "Start");
            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
                check_pins(rng, goals_v.back(), "Goal");
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

        // Shared skeleton of the constrained/phase/chart planner entry points: convert
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
            return PlannerT<P>::solve(start_c, goal_c, EnvVec(env), s, rng, lp);
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

            return PlannerT<P>::solve(start_c, goals_v, EnvVec(env), s, rng, lp);
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
                p, EnvVec(env), settings, rng, lp);
        }

        // Constraint-aware variants. These members are only instantiated when referenced,
        // so robots without generated constraint support never touch them.
        using ConstraintT = vamp::planning::constraint::Constraint<Robot, rake>;
        using ConstraintVec = std::vector<std::shared_ptr<const ConstraintT>>;
        using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
        using ConstrainedLP =
            vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
        using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;

        static constexpr const char *manifold_message =
            " configuration violates the constraints; project it first with project()";
        static constexpr const char *manifold_simplify_message =
            " violates the constraints; simplify requires an on-manifold path";

        // Forward a PinnedRNG's pinned dimensions into a constraint set so projection
        // holds them exactly (their Jacobian columns are zeroed before every descent step).
        static void apply_pins(ConstraintSetT &set, const std::shared_ptr<Sampler> &rng)
        {
            if (auto pinned = std::dynamic_pointer_cast<vamp::rng::PinnedRNG<Robot>>(rng))
            {
                set.set_pinned(pinned->pinned_dims());
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

            check_pins(rng, Input::to(start), "Start");
            check_pins(rng, Input::to(goal), "Goal");

            ConstraintSetT set(std::move(constraints), cs);
            apply_pins(set, rng);
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

            check_pins(rng, Input::to(start), "Start");
            for (const auto &g : goals)
            {
                check_pins(rng, Input::to(g), "Goal");
            }

            ConstraintSetT set(std::move(constraints), cs);
            apply_pins(set, rng);
            return solve_multi_lp<P>(
                start, goals, env, s, rng, ConstrainedLP(std::move(set)), manifold_message);
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
            apply_pins(set, rng);
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

        // Projection within a pinned sampler's slice: snap the pinned joints to their
        // pinned values, then hold them exactly while the active joints descend.
        static auto constraint_project_pinned(
            const Cfg &c,
            ConstraintVec constraints,
            std::shared_ptr<Sampler> rng,
            const ConstraintSettings &cs) -> Cfg
        {
            ConstraintSetT set(std::move(constraints), cs);
            auto q = Input::to(c);
            if (auto pinned = std::dynamic_pointer_cast<vamp::rng::PinnedRNG<Robot>>(rng))
            {
                q = q * pinned->unmask + pinned->pinned;
                set.set_pinned(pinned->pinned_dims());
            }

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

        // Phase-constraint variants (flask robots): like the constraint-aware variants
        // above, these members are only instantiated when referenced.
        using PhaseConstraintT = vamp::planning::constraint::PhaseConstraint<Robot, rake>;
        using PhaseConstraintVec = std::vector<std::shared_ptr<const PhaseConstraintT>>;
        using PhaseSetT = vamp::planning::constraint::PhaseConstraintSet<Robot, rake>;
        using PhaseLP = vamp::planning::UnconstrainedLocalPlanner<Robot, rake, Robot::resolution>;

        static constexpr const char *phase_message =
            " state violates the phase constraints; scale its velocity by velocity_scale() first";
        static constexpr const char *phase_simplify_message =
            " violates the phase constraints; simplify requires a phase-feasible path";

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single_phase(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (phase_constraints.empty())
            {
                return solve_single<P, Settings>(start, goal, env, s, rng);
            }

            return solve_single_lp<P>(
                start, goal, env, s, rng, PhaseLP(PhaseSetT(std::move(phase_constraints))), phase_message);
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi_phase(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (phase_constraints.empty())
            {
                return solve_multi<P, Settings>(start, goals, env, s, rng);
            }

            return solve_multi_lp<P>(
                start, goals, env, s, rng, PhaseLP(PhaseSetT(std::move(phase_constraints))), phase_message);
        }

        static auto simplify_phase(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (phase_constraints.empty())
            {
                return simplify(p, env, settings, rng);
            }

            return simplify_lp(
                p, env, settings, rng, PhaseLP(PhaseSetT(std::move(phase_constraints))), phase_simplify_message);
        }

        static auto phase_satisfied(const Cfg &c, PhaseConstraintVec phase_constraints) -> bool
        {
            return PhaseSetT(std::move(phase_constraints)).satisfied(Input::to(c));
        }

        static auto phase_velocity_scale(const Cfg &c, PhaseConstraintVec phase_constraints) -> float
        {
            return PhaseSetT(std::move(phase_constraints)).velocity_scale(Input::to(c));
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

        static auto make_pinned_sampler(
            std::shared_ptr<Sampler> inner,
            const std::vector<std::string> &active_joints,
            const Cfg &default_configuration) -> std::shared_ptr<Sampler>
        {
            for (const auto &name : active_joints)
            {
                if (std::find(Robot::joint_names.begin(), Robot::joint_names.end(), name) ==
                    Robot::joint_names.end())
                {
                    throw std::invalid_argument(
                        "Active joint `" + name + "` is not a joint of this robot!");
                }
            }

            alignas(FloatVectorAlignment)
                std::array<float, Configuration::num_scalars_rounded> mask{};
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                const bool active =
                    std::find(active_joints.begin(), active_joints.end(), Robot::joint_names[i]) !=
                    active_joints.end();
                mask[i] = active ? 0.F : 1.F;
            }

            return std::make_shared<vamp::rng::PinnedRNG<Robot>>(
                std::move(inner), Configuration(mask.data()), Input::to(default_configuration));
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

    // Chart-based constrained planning for flask z-robots with an ambient sibling.
    // Kept separate from StaticRobotTraits: member declarations here reference
    // typename Robot::Ambient, which would hard-error at class instantiation for
    // robots without one. Only referenced inside if constexpr (has_type_Ambient_v<Robot>).
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
        using PhaseConstraintVec = typename Base::PhaseConstraintVec;
        using PhaseSetT = typename Base::PhaseSetT;

        static auto make_lp(
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints = {}) -> ChartLP
        {
            return ChartLP(
                AmbientConstraintSetT(std::move(constraints), cs),
                chs,
                PhaseSetT(std::move(phase_constraints)));
        }

        static constexpr const char *chart_message =
            " state violates the constraints; project it first with project()";

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single_chart(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::template solve_single_phase<P, Settings>(
                    start, goal, env, s, rng, std::move(phase_constraints));
            }

            return Base::template solve_single_lp<P>(
                start,
                goal,
                env,
                s,
                rng,
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints)),
                chart_message);
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
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::template solve_multi_phase<P, Settings>(
                    start, goals, env, s, rng, std::move(phase_constraints));
            }

            return Base::template solve_multi_lp<P>(
                start,
                goals,
                env,
                s,
                rng,
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints)),
                chart_message);
        }

        static auto simplify_chart(
            const Path &p,
            const Env &env,
            const vamp::planning::SimplifySettings &settings,
            std::shared_ptr<Sampler> rng,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints) -> PlanningResult
        {
            if (constraints.empty())
            {
                return Base::simplify_phase(p, env, settings, rng, std::move(phase_constraints));
            }

            return Base::simplify_lp(
                p,
                env,
                settings,
                rng,
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints)),
                Base::manifold_simplify_message);
        }

        static auto chart_project(
            const Cfg &c,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints) -> Cfg
        {
            const auto lp = make_lp(std::move(constraints), cs, chs, std::move(phase_constraints));
            auto z = Input::to(c);
            if (not lp.project(z))
            {
                throw std::invalid_argument("projection onto the constraint manifold did not converge");
            }

            return Input::from(z);
        }

        static auto chart_satisfied(
            const Cfg &c,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            PhaseConstraintVec phase_constraints) -> bool
        {
            return make_lp(std::move(constraints), cs, {}, std::move(phase_constraints))
                .satisfied(Input::to(c));
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
            if (not lp.lift_edge(
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
                           const CoefficientRow &body_coefficients) {
                            new (t) TWC(
                                eef_to_offset,
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

            // Phase constraints need the robot's generated dynamics kernels; only
            // regenerated flask robots have them.
            if constexpr (has_kinetic_energy_v<Robot> or has_n_end_effectors_v<Robot>)
            {
                namespace vc = vamp::planning::constraint;
                using PhaseConstraintT = vc::PhaseConstraint<Robot, rake>;

                nb::class_<PhaseConstraintT>(
                    submodule,
                    "PhaseConstraint",
                    "Phase-space inequality constraint g(q, qdot) <= 0 over flat states. Constraints "
                    "cache per-evaluation state and are not thread-safe: do not share one "
                    "instance across concurrent planning calls.");

                if constexpr (has_kinetic_energy_v<Robot>)
                {
                    using KEC = vc::KineticEnergyConstraint<Robot, rake>;
                    nb::class_<KEC, PhaseConstraintT>(
                        submodule,
                        "KineticEnergyConstraint",
                        "Bounds the robot's total kinetic energy: "
                        "(1/2) qdot^T M(q) qdot <= max_energy.")
                        .def(nb::init<float>(), "max_energy"_a);

                    submodule.def(
                        "kinetic_energy",
                        [](const std::array<float, Robot::dimension> &x)
                        { return Robot::kinetic_energy(x); },
                        "state"_a,
                        "Total kinetic energy (1/2) qdot^T M(q) qdot of a flat state (q, qdot).");

                    submodule.def(
                        "ke_shaped",
                        [](std::shared_ptr<vamp::rng::RNG<Robot>> sampler, float max_energy)
                            -> std::shared_ptr<vamp::rng::RNG<Robot>>
                        {
                            return std::make_shared<vamp::rng::KineticEnergyShaped<Robot>>(
                                std::move(sampler), max_energy);
                        },
                        "sampler"_a,
                        "max_energy"_a,
                        "Wrap a sampler so sample kinetic energy is uniform on [0, max_energy] "
                        "instead of concentrated near the velocity-box maximum; rescales each "
                        "sample's velocity block by the scalar kinetic-energy kernel.");
                }

                if constexpr (has_n_end_effectors_v<Robot>)
                {
                    using ESC = vc::EEFSpeedConstraint<Robot, rake>;
                    nb::class_<ESC, PhaseConstraintT>(
                        submodule,
                        "EEFSpeedConstraint",
                        "Bounds the workspace speed of every end-effector origin: "
                        "||v_eef|| <= max_speed (linear velocity only).")
                        .def(nb::init<float>(), "max_speed"_a);

                    submodule.def("n_end_effectors", []() { return Robot::n_end_effectors; });
                    submodule.def(
                        "eef_velocity",
                        [](const std::array<float, Robot::dimension> &x)
                        { return Robot::eef_velocity(x); },
                        "state"_a,
                        "Linear workspace velocity (x, y, z per end effector) of a flat state "
                        "(q, qdot).");
                }

                bind_phase_methods<TA>(submodule);
                bind_phase_methods<TC>(submodule);
            }

            if constexpr (has_type_Ambient_v<Robot>)
            {
                using Ambient = typename Robot::Ambient;

                submodule.def(
                    "ambient",
                    []() { return std::string(Ambient::name); },
                    "Name of the ambient position-space sibling robot whose constraints this "
                    "z-robot plans with.");

                // Chart-based constrained planning projects through the ambient robot's
                // generated constraint kernels; skip binding when it has none.
                if constexpr (
                    has_n_eef_v<Ambient> or has_n_closed_loops_v<Ambient> or
                    robot_has_com_v<Ambient>)
                {
                    bind_chart_methods<ChartRobotTraits<Robot, NA>>(submodule);
                    bind_chart_methods<ChartRobotTraits<Robot, CA>>(submodule);
                }
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

        // Robots with a ParameterizedSpace (a task-space parameterization that IK-resolves
        // into this robot's own ambient Configuration space, e.g. RBY1) get task-space
        // planning bound as a nested submodule (e.g. vamp.rby1.parameterized_space).
        if constexpr (has_type_ParameterizedSpace_v<Robot>)
        {
            using PTraits = ParameterizedSpaceTraits<Robot>;

            auto param_submodule =
                submodule.def_submodule("parameterized_space", "Task-space planning submodule");

            bind_sampler<PTraits>(param_submodule, "RNG");
            bind_task_space_informed_sampler<PTraits>(param_submodule);

            auto param_path_k = bind_path_class<PTraits>(param_submodule, "Path");
            bind_path_io<PTraits>(param_path_k);

            bind_planning_result<PTraits>(param_submodule, "PlanningResult");

            bind_parameterized_space_methods<PTraits>(param_submodule);
        }

        // FLASK z-space sibling generated as a nested struct: bind it as a nested
        // submodule (e.g. vamp.panda.flask).
        if constexpr (has_flask_robot_v<Robot>)
        {
            init_robot<typename Robot::Flask>(submodule);
        }

        return submodule;
    }
}  // namespace vamp::binding
