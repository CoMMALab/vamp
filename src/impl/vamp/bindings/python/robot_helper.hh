#pragma once

#include <algorithm>
#include <stdexcept>
#include <string>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>

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
#include <vamp/planning/constraints/goal_region.hh>
#include <vamp/planning/constraints/task_space_sampler.hh>
#include <vamp/planning/planners/avp.hh>
#include <vamp/planning/trajectory.hh>
#include <vamp/planning/planners/rrt.hh>
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

#include <nanobind/stl/optional.h>
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

    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

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

        // End-effector payloads (Robot::Payload) carried by an environment's attachments: the
        // torque check reads them from the environment directly; phase constraints and
        // samplers get them here.
        template <typename R = Robot>
        static auto payload_of(const Env &env)
        {
            return vamp::collision::attachment_payload<R::payload_dimension / 10>(env);
        }

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
                start, goal, env, s, rng, PhaseLP(PhaseSetT(std::move(phase_constraints), payload_of(env))), phase_message);
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
                start, goals, env, s, rng, PhaseLP(PhaseSetT(std::move(phase_constraints), payload_of(env))), phase_message);
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
                p, env, settings, rng, PhaseLP(PhaseSetT(std::move(phase_constraints), payload_of(env))), phase_simplify_message);
        }

        static auto phase_satisfied(const Cfg &c, PhaseConstraintVec phase_constraints, const Env &env) -> bool
        {
            return PhaseSetT(std::move(phase_constraints), payload_of(env)).satisfied(Input::to(c));
        }

        static auto phase_velocity_scale(const Cfg &c, PhaseConstraintVec phase_constraints, const Env &env)
            -> float
        {
            return PhaseSetT(std::move(phase_constraints), payload_of(env)).velocity_scale(Input::to(c));
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
            PhaseConstraintVec phase_constraints = {},
            const typename Robot::Payload &payload = {}) -> ChartLP
        {
            AmbientConstraintSetT set(std::move(constraints), cs);
            if (chs.exact_chart and not set.exact_form())
            {
                throw std::invalid_argument(
                    "exact_chart requires every constraint to provide an equality form (e.g. a "
                    "bimanual relative pose with all rows tight) and no Pfaffian rows");
            }

            return ChartLP(std::move(set), chs, PhaseSetT(std::move(phase_constraints), payload));
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
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints), Base::payload_of(env)),
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
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints), Base::payload_of(env)),
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
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints), Base::payload_of(env)),
                Base::manifold_simplify_message);
        }

        static auto chart_project(
            const Cfg &c,
            AmbientConstraintVec constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            PhaseConstraintVec phase_constraints,
            const Env &env) -> Cfg
        {
            const auto lp =
                make_lp(std::move(constraints), cs, chs, std::move(phase_constraints), Base::payload_of(env));
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
            PhaseConstraintVec phase_constraints,
            const Env &env) -> bool
        {
            return make_lp(std::move(constraints), cs, {}, std::move(phase_constraints), Base::payload_of(env))
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
            const ChartSettings &chs,
            bool with_accelerations) -> nanobind::tuple
        {
            const auto lp = make_lp(std::move(constraints), cs, chs);

            std::vector<Configuration> states;
            std::vector<float> errors;
            std::vector<std::array<float, Ambient::dimension>> accelerations;
            float duration = 0.F, cost = 0.F;
            if (not lp.lift_edge(
                    Input::to(from),
                    Input::to(target),
                    forward,
                    n_samples,
                    states,
                    errors,
                    duration,
                    cost,
                    with_accelerations ? &accelerations : nullptr))
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

            if (with_accelerations)
            {
                return nanobind::make_tuple(
                    std::move(out), std::move(errors), duration, cost, std::move(accelerations));
            }

            return nanobind::make_tuple(std::move(out), std::move(errors), duration, cost);
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
            submodule.def("effort_scale", []() { return Robot::effort_scale; });
            submodule.def(
                "set_effort_scale",
                [](float s) { Robot::effort_scale = s; },
                "scale"_a,
                "Uniformly derate (scale < 1) the effort limits the torque check enforces.");

            using Env = typename TA::Env;

            bind_flask_methods<Robot, NA>(submodule);
            bind_flask_methods<Robot, CA>(submodule);

            // Time-parameterized trajectories: from flask paths (their own LQMT motion), or retimed
            // time-optimally along any joint path (a trajectory's knots or raw positions, e.g. a
            // geometric path) under the torque and joint-velocity limits.
            {
                namespace vp_ = vamp::planning;
                using Traj = vp_::Trajectory<Robot>;
                using Joints = typename Traj::Joints;
                nb::class_<Traj>(
                    submodule,
                    "Trajectory",
                    "Time-parameterized joint trajectory: knots (t, q, qd, qdd), queried at any time by "
                    "quintic Hermite interpolation; torques from the payload-aware RNEA.")
                    .def_prop_ro("duration", &Traj::duration, "Duration in seconds.")
                    .def("__len__", &Traj::size)
                    .def_prop_ro("times", [](const Traj &t) { return t.times; }, "Knot times.")
                    .def_prop_ro(
                        "knots",
                        [](const Traj &t) { return nb::make_tuple(t.q, t.qd, t.qdd); },
                        "Knot (q, qd, qdd) lists.")
                    .def(
                        "__call__",
                        [](const Traj &t, float time)
                        {
                            Joints q, qd, qdd;
                            t.state(time, q, qd, qdd);
                            return nb::make_tuple(q, qd, qdd);
                        },
                        "time"_a,
                        "(q, qd, qdd) at `time` (clamped to the trajectory's span).")
                    .def(
                        "sample",
                        [](const Traj &t, const std::vector<float> &times)
                        {
                            std::vector<Joints> qs(times.size()), qds(times.size()), qdds(times.size());
                            for (std::size_t k = 0; k < times.size(); ++k)
                            {
                                t.state(times[k], qs[k], qds[k], qdds[k]);
                            }

                            return nb::make_tuple(qs, qds, qdds);
                        },
                        "times"_a,
                        "(q, qd, qdd) lists at each of `times`.")
                    .def(
                        "torques",
                        [](const Traj &t, float time, const Env &env) { return t.torques(time, TA::payload_of(env)); },
                        "time"_a,
                        "environment"_a = Env{},
                        "Joint torques at `time`, including the environment's attachment payloads.");

                submodule.def(
                    "trajectory",
                    [](const vp_::Path<Robot> &path, std::size_t samples)
                    {
                        const auto traj = vp_::trajectory_from_path<Robot>(
                            path, vp_::UnconstrainedLocalPlanner<Robot, rake, Robot::resolution>{},
                            typename Robot::Payload{}, samples);
                        if (not traj)
                        {
                            throw std::invalid_argument("empty path");
                        }

                        return *traj;
                    },
                    "path"_a,
                    "samples"_a = 64,
                    "The executed motion of an (unconstrained) flask path: each edge's LQMT cubic at its "
                    "optimal duration, sampled at `samples` knots.");

                const auto retime = [](const std::vector<Joints> &positions,
                                       const Env &env,
                                       std::size_t grid,
                                       float effort_fraction,
                                       float end_speed,
                                       std::size_t substeps,
                                       std::size_t repair_iterations)
                {
                    vp_::RetimeSettings settings;
                    settings.repair_iterations = repair_iterations;
                    settings.grid = grid;
                    settings.effort_fraction = effort_fraction;
                    settings.end_speed = end_speed;
                    settings.substeps = substeps;
                    std::string reason;
                    const auto traj = vp_::topp_retime<Robot>(positions, TA::payload_of(env), settings, &reason);
                    if (not traj)
                    {
                        throw std::invalid_argument(reason);
                    }

                    return *traj;
                };
                const char *retime_doc =
                    "Time-optimal retiming (TOPP-RA, interpolation discretization) along a joint path -- a "
                    "trajectory's knots, or positions such as a densified geometric path -- under the "
                    "torque limits (effort_fraction of the scaled limits, with the environment's payloads) "
                    "and joint-velocity limits; starts at rest, ends with path speed at most end_speed "
                    "(chord-length parameter in [0, 1] per second). Constraints hold at the grid points; up "
                    "to repair_iterations rounds tighten intervals whose executed knots overshoot the torque "
                    "bound. Raises ValueError if no feasible timing exists.";
                submodule.def(
                    "retime",
                    [retime](const Traj &t, const Env &env, std::size_t grid, float effort_fraction, float end_speed,
                             std::size_t substeps, std::size_t repair_iterations)
                    { return retime(t.q, env, grid, effort_fraction, end_speed, substeps, repair_iterations); },
                    "trajectory"_a,
                    "environment"_a = Env{},
                    "grid"_a = 800,
                    "effort_fraction"_a = 0.98F,
                    "end_speed"_a = 0.F,
                    "substeps"_a = 4,
                    "repair_iterations"_a = 6,
                    retime_doc);
                submodule.def(
                    "retime",
                    retime,
                    "positions"_a,
                    "environment"_a = Env{},
                    "grid"_a = 800,
                    "effort_fraction"_a = 0.98F,
                    "end_speed"_a = 0.F,
                    "substeps"_a = 4,
                    "repair_iterations"_a = 6,
                    retime_doc);
            }

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
                        [](const std::array<float, Robot::dimension> &x, const Env &env)
                        { return Robot::kinetic_energy(x, TA::payload_of(env)); },
                        "state"_a,
                        "environment"_a = Env{},
                        "Total kinetic energy (1/2) qdot^T M(q) qdot of a flat state (q, qdot), "
                        "including the payloads of the environment's attachments.");

                    submodule.def(
                        "ke_shaped",
                        [](std::shared_ptr<vamp::rng::RNG<Robot>> sampler, float max_energy, const Env &env)
                            -> std::shared_ptr<vamp::rng::RNG<Robot>>
                        {
                            return std::make_shared<vamp::rng::KineticEnergyShaped<Robot>>(
                                std::move(sampler), max_energy, TA::payload_of(env));
                        },
                        "sampler"_a,
                        "max_energy"_a,
                        "environment"_a = Env{},
                        "Wrap a sampler so sample kinetic energy is uniform on [0, max_energy] "
                        "instead of concentrated near the velocity-box maximum; rescales each "
                        "sample's velocity block by the scalar kinetic-energy kernel. The energy "
                        "includes the payloads of the given environment's attachments.");
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

                    namespace vc = vamp::planning::constraint;
                    using CT = ChartRobotTraits<Robot, NA>;
                    using GR = vc::ImpactGoalRegion<Robot, rake>;
                    using Q = nanobind::ndarray<
                        FloatT,
                        nanobind::numpy,
                        nanobind::shape<Ambient::dimension>,
                        nanobind::device::cpu>;

                    nb::class_<GR>(
                        submodule,
                        "ImpactGoalRegion",
                        "Goal region for constrained flask planning: positions on the manifold of "
                        "`constraints` (every constraint needs an equality form, e.g. all rows "
                        "tight), velocities realizing equality-row rates speed * rate, speed in "
                        "[min_speed, max_speed], at minimum kinetic energy under the environment's "
                        "payloads. Such a velocity is M-orthogonal to the manifold's self-motion, so "
                        "an inelastic impact enforcing the constraints brings the state exactly to "
                        "rest: e.g. the planning constraints plus a pinned object pose, with the "
                        "pin rows' rate the object's approach twist (TSR rows change at the rate of "
                        "the offset frame's twist in the reference frame, linear first). Pass "
                        "samples as goals to the chart planners.")
                        .def(
                            "__init__",
                            [](GR *self,
                               typename CT::AmbientConstraintVec constraints,
                               std::vector<float> rate,
                               float min_speed,
                               float max_speed,
                               const vc::ConstraintSettings &cs,
                               const vc::ChartSettings &chs,
                               const typename CT::Env &env,
                               typename CT::PhaseConstraintVec phase,
                               std::optional<Q> seed,
                               float spread,
                               std::size_t refine_steps,
                               float refine_step)
                            {
                                typename CT::AmbientConstraintSetT set(std::move(constraints), cs);
                                if (not set.exact_form())
                                {
                                    throw std::invalid_argument(
                                        "ImpactGoalRegion requires every constraint to provide an "
                                        "equality form (all manifold rows tight) and no Pfaffian rows");
                                }

                                if (rate.size() != set.equality_rows())
                                {
                                    throw std::invalid_argument(
                                        "rate needs one entry per equality row (" +
                                        std::to_string(set.equality_rows()) + ")");
                                }

                                if (not (0.F <= min_speed and min_speed <= max_speed))
                                {
                                    throw std::invalid_argument("need 0 <= min_speed <= max_speed");
                                }

                                new (self) GR(
                                    std::move(set), std::move(rate), min_speed, max_speed, chs, env, std::move(phase));
                                if (seed)
                                {
                                    std::array<float, Ambient::dimension> sa;
                                    std::vector<float> scratch;
                                    const auto *ptr = as_flat_1d(*seed, Ambient::dimension, scratch, "seed");
                                    std::copy_n(ptr, Ambient::dimension, sa.begin());
                                    self->set_seed(sa, spread);
                                }

                                self->set_refinement(refine_steps, refine_step);
                            },
                            "constraints"_a,
                            "rate"_a,
                            "min_speed"_a,
                            "max_speed"_a,
                            "constraint_settings"_a = vc::ConstraintSettings{},
                            "chart_settings"_a = vc::ChartSettings{},
                            "environment"_a = typename CT::Env{},
                            "phase_constraints"_a = typename CT::PhaseConstraintVec{},
                            "seed"_a = nb::none(),
                            "spread"_a = 0.1F,
                            "refine_steps"_a = 0,
                            "refine_step"_a = 0.1F,
                            "refine_steps > 0 refines each drawn posture by up to that many null-space steps "
                            "(refine_step rad, the manifold's self-motion) lowering its static torque under the "
                            "payloads. seed (optional configuration): draw positions around it, each joint within "
                            "+- spread / 2 of its range, keeping goals on the seed's inverse-kinematics "
                            "branch; without it positions cover the whole joint box.")
                        .def_prop_ro("rows", &GR::rows, "Stacked equality rows (the length of rate).")
                        .def(
                            "sample",
                            [](const GR &self, std::shared_ptr<vamp::rng::RNG<Robot>> rng, std::size_t n, std::size_t max_attempts)
                            {
                                std::vector<typename NA::Type> out;
                                for (std::size_t a = 0; a < max_attempts and out.size() < n; ++a)
                                {
                                    if (const auto z = self.sample(*rng))
                                    {
                                        out.emplace_back(NA::from(*z));
                                    }
                                }

                                return out;
                            },
                            "sampler"_a,
                            "n"_a = 1,
                            "max_attempts"_a = 1000,
                            "Draw up to n members (fewer if max_attempts draws are exhausted): each "
                            "draw projects a sampler position onto the manifold, rejects bound, "
                            "collision, and phase-constraint violations, and takes a speed uniformly "
                            "from the same sample.")
                        .def(
                            "velocity",
                            [](const GR &self, const Q &q, float speed)
                            {
                                std::array<float, Ambient::dimension> qa;
                                std::vector<float> scratch;
                                const auto *ptr = as_flat_1d(q, Ambient::dimension, scratch, "configuration");
                                std::copy_n(ptr, Ambient::dimension, qa.begin());
                                const auto v = self.velocity(qa, speed);
                                if (not v)
                                {
                                    throw std::invalid_argument("the equality rows are singular at this configuration");
                                }

                                return *v;
                            },
                            "configuration"_a,
                            "speed"_a,
                            "Minimum-kinetic-energy velocity realizing equality-row rates speed * rate.")
                        .def(
                            "impulse",
                            [](const GR &self, const typename NA::Type &state) { return self.impulse(NA::to(state)); },
                            "state"_a,
                            "Constraint impulse (J M^-1 J^T)^-1 J qd of an inelastic impact stopping the "
                            "equality rows at `state`.");

                    submodule.def(
                        "trajectory",
                        [](const vamp::planning::Path<Robot> &path,
                           typename CT::AmbientConstraintVec constraints,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs,
                           const typename CT::Env &env,
                           std::size_t samples)
                        {
                            const auto payload = CT::Base::payload_of(env);
                            const auto lp = CT::make_lp(
                                std::move(constraints), cs, chs, typename CT::PhaseConstraintVec{}, payload);
                            const auto traj = vamp::planning::trajectory_from_path<Robot>(path, lp, payload, samples);
                            if (not traj)
                            {
                                throw std::invalid_argument("an edge of the path could not be lifted");
                            }

                            return *traj;
                        },
                        "path"_a,
                        "constraints"_a,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "environment"_a = typename CT::Env{},
                        "samples"_a = 64,
                        "The executed motion of a constrained flask path: each edge's chart lift (of the two "
                        "directions a planner may have built it in, the one with the lower peak torque ratio "
                        "under the environment's payloads), sampled at `samples` knots.");

                    // Forward-only goal-biased RRT on the constraint manifold: one tree from the
                    // start, a goal_bias fraction of iterations steering toward a goal draw.
                    using RRTP = vamp::planning::RRT<Robot, rake, Robot::resolution>;
                    const auto rrt_doc =
                        "Forward-only RRT on the constraint manifold (chart-local time-optimal "
                        "steering, every edge in execution direction): a settings.goal_bias fraction "
                        "of iterations steers toward a goal draw -- fresh members of `goal` (an "
                        "ImpactGoalRegion, drawn with `goal_sampler`) or entries of a goal list, in "
                        "turn -- from the best of the nearest nodes; reaching a goal solves. Raises "
                        "ValueError if the start violates the constraints.";
                    submodule.def(
                        "rrt",
                        [](const typename NA::Type &start,
                           const GR &goal,
                           std::shared_ptr<vamp::rng::RNG<Robot>> goal_sampler,
                           const typename CT::Env &env,
                           const vamp::planning::RRTSettings &settings,
                           std::shared_ptr<vamp::rng::RNG<Robot>> sampler,
                           typename CT::AmbientConstraintVec constraints,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs,
                           typename CT::PhaseConstraintVec phase)
                        {
                            const auto lp = CT::make_lp(std::move(constraints), cs, chs, std::move(phase), CT::Base::payload_of(env));
                            const auto s = NA::to(start);
                            CT::Base::check_feasible(lp, s, "start", CT::chart_message);
                            const auto draw = [&]() -> std::optional<typename Robot::Configuration>
                            {
                                for (auto attempt = 0; attempt < 32; ++attempt)
                                {
                                    if (auto z = goal.sample(*goal_sampler))
                                    {
                                        return z;
                                    }
                                }

                                return std::nullopt;
                            };
                            return RRTP::solve(s, draw, typename CT::EnvVec(env), settings, sampler, lp);
                        },
                        "start"_a,
                        "goal"_a,
                        "goal_sampler"_a,
                        "environment"_a,
                        "settings"_a,
                        "sampler"_a,
                        "constraints"_a,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "phase_constraints"_a = typename CT::PhaseConstraintVec{},
                        rrt_doc);
                    submodule.def(
                        "rrt",
                        [](const typename NA::Type &start,
                           const std::vector<typename NA::Type> &goals,
                           const typename CT::Env &env,
                           const vamp::planning::RRTSettings &settings,
                           std::shared_ptr<vamp::rng::RNG<Robot>> sampler,
                           typename CT::AmbientConstraintVec constraints,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs,
                           typename CT::PhaseConstraintVec phase)
                        {
                            const auto lp = CT::make_lp(std::move(constraints), cs, chs, std::move(phase), CT::Base::payload_of(env));
                            const auto s = NA::to(start);
                            CT::Base::check_feasible(lp, s, "start", CT::chart_message);
                            std::vector<typename Robot::Configuration> gs;
                            for (const auto &g : goals)
                            {
                                gs.emplace_back(NA::to(g));
                                CT::Base::check_feasible(lp, gs.back(), "goal", CT::chart_message);
                            }

                            if (gs.empty())
                            {
                                throw std::invalid_argument("rrt needs at least one goal");
                            }

                            std::size_t next = 0;
                            const auto draw = [&]() -> std::optional<typename Robot::Configuration>
                            { return gs[next++ % gs.size()]; };
                            return RRTP::solve(s, draw, typename CT::EnvVec(env), settings, sampler, lp);
                        },
                        "start"_a,
                        "goals"_a,
                        "environment"_a,
                        "settings"_a,
                        "sampler"_a,
                        "constraints"_a,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "phase_constraints"_a = typename CT::PhaseConstraintVec{},
                        rrt_doc);

                    submodule.def(
                        "avp",
                        [](const typename NA::Type &start,
                           const std::vector<typename NA::Type> &goals,
                           const typename CT::Env &env,
                           const vamp::planning::AVPSettings &settings,
                           std::shared_ptr<vamp::rng::RNG<Robot>> sampler,
                           typename CT::AmbientConstraintVec constraints,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs)
                        {
                            // Edges check geometry only (attachments keep their spheres but carry no
                            // load); speeds propagate under the payload.
                            const auto lp = CT::make_lp(std::move(constraints), cs, chs);
                            auto geometry = env;
                            for (auto &a : geometry.attachments)
                            {
                                a.inertia.fill(0.F);
                            }

                            const auto s = NA::to(start);
                            CT::Base::check_feasible(lp, s, "start", CT::chart_message);
                            std::vector<typename Robot::Configuration> gs;
                            for (const auto &g : goals)
                            {
                                gs.emplace_back(NA::to(g));
                                CT::Base::check_feasible(lp, gs.back(), "goal", CT::chart_message);
                            }

                            vamp::planning::AVPStats stats;
                            std::optional<vamp::planning::Trajectory<Robot>> timed;
                            auto result = vamp::planning::AVP<Robot, rake, Robot::resolution>::solve(
                                s, gs, typename CT::EnvVec(geometry), CT::Base::payload_of(env), settings, sampler, lp,
                                &stats, &timed);
                            nb::dict st;
                            st["propagations"] = stats.propagations;
                            st["empty"] = stats.empty;
                            return nb::make_tuple(std::move(result), std::move(timed), st);
                        },
                        "start"_a,
                        "goals"_a,
                        "environment"_a,
                        "settings"_a,
                        "sampler"_a,
                        "constraints"_a,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "Admissible velocity propagation (Pham et al., RSS 2013) in BKPIECE's bidirectional "
                        "grid search: edges check geometry only, each node carries an interval of path-speed "
                        "scales sd^2 propagated TOPP-style along its edges under the environment's payloads "
                        "(forward reachability on the start tree, backward controllability on the goal tree); "
                        "trees connect where the intervals meet. Returns (result, trajectory, stats): the "
                        "trajectory is the time-optimal timing of the solution along the edges as propagated "
                        "(None if unsolved).");

                    submodule.def(
                        "avp_retime",
                        [](const std::vector<typename NA::Type> &path,
                           const typename CT::Env &env,
                           typename CT::AmbientConstraintVec constraints,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs,
                           std::size_t edge_samples,
                           float effort_fraction)
                        {
                            using AVPT = vamp::planning::AVP<Robot, rake, Robot::resolution>;
                            const auto lp = CT::make_lp(std::move(constraints), cs, chs);
                            if (path.size() < 2)
                            {
                                throw std::invalid_argument("avp_retime needs at least two states");
                            }

                            std::vector<typename AVPT::Segment> segments;
                            for (std::size_t i = 0; i + 1 < path.size(); ++i)
                            {
                                segments.push_back({NA::to(path[i]), NA::to(path[i + 1]), true});
                            }

                            return AVPT::time(
                                segments, AVPT::root_interval(NA::to(path.front()), false),
                                AVPT::root_interval(NA::to(path.back()), true), lp, CT::Base::payload_of(env),
                                edge_samples, effort_fraction);
                        },
                        "path"_a,
                        "environment"_a,
                        "constraints"_a,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "edge_samples"_a = 16,
                        "effort_fraction"_a = 1.F,
                        "Time-optimal timing (the avp planner's) of a flask path along its forward exact edges under "
                        "the environment's payloads: the start at its velocity (any scale at rest), the end at "
                        "most at its velocity. None if no timing exists.");

                    submodule.def(
                        "payload_shaped",
                        [](std::shared_ptr<vamp::rng::RNG<Robot>> sampler,
                           typename CT::AmbientConstraintVec constraints,
                           const typename CT::Env &env,
                           float slack,
                           std::size_t max_attempts,
                           bool project,
                           const vc::ConstraintSettings &cs,
                           const vc::ChartSettings &chs) -> std::shared_ptr<vamp::rng::RNG<Robot>>
                        {
                            typename CT::AmbientConstraintSetT set(std::move(constraints), cs);
                            if (not set.exact_form())
                            {
                                throw std::invalid_argument(
                                    "payload_shaped projects with the exact chart: every constraint "
                                    "needs an equality form (all manifold rows tight)");
                            }

                            return std::make_shared<vc::PayloadShaped<Robot, rake>>(
                                std::move(sampler), std::move(set), chs, env, slack, max_attempts, project);
                        },
                        "sampler"_a,
                        "constraints"_a,
                        "environment"_a,
                        "slack"_a = 0.2F,
                        "max_attempts"_a = 50,
                        "project"_a = false,
                        "constraint_settings"_a = vc::ConstraintSettings{},
                        "chart_settings"_a = vc::ChartSettings{},
                        "Wrap a sampler with payload awareness: keep a draw only if the robot could "
                        "hold the environment's attachment payloads at its position at rest within "
                        "(1 + slack) of its (scaled) effort limits (scored on the raw draw, or after "
                        "projecting it onto the constraint manifold with project=True); after "
                        "max_attempts rejections the last draw is returned. Velocities come from "
                        "the inner sampler.");

                    if constexpr (has_n_eef_v<Ambient>)
                    {
                        using TSS = vc::TaskSpaceSampler<Robot, rake>;
                        nb::class_<TSS, vamp::rng::RNG<Robot>>(
                            submodule, "TaskSpaceSampler", "Sampler returned by task_space_sampler.")
                            .def_prop_ro(
                                "stats",
                                [](const TSS &self)
                                {
                                    const auto &st = self.stats();
                                    nb::dict out;
                                    out["draws"] = st.draws;
                                    out["task_draws"] = st.task_draws;
                                    out["attempts"] = st.attempts;
                                    out["pose_collisions"] = st.pose_collisions;
                                    out["projections"] = st.projections;
                                    out["projected"] = st.projected;
                                    out["refined"] = st.refined;
                                    out["held"] = st.held;
                                    out["fallbacks"] = st.fallbacks;
                                    out["local_draws"] = st.local_draws;
                                    out["local_held"] = st.local_held;
                                    return out;
                                },
                                "Draw counters: next() calls, task-space draws, pose/twist attempts, candidate "
                                "projections tried and succeeded, attempts whose best posture held the payload, "
                                "and fallbacks to the inner sampler.");
                        submodule.def(
                            "task_space_sampler",
                            [](std::shared_ptr<vamp::rng::RNG<Robot>> sampler,
                               typename CT::AmbientConstraintVec constraints,
                               const typename TSS::Transform &object_in_eef,
                               const typename CT::Env &env,
                               const std::array<float, 3> &lower,
                               const std::array<float, 3> &upper,
                               const std::array<float, 4> &orientation,
                               float tilt,
                               float yaw,
                               float max_speed,
                               float max_angular_speed,
                               std::vector<std::array<float, 4>> directions,
                               float direction_jitter,
                               std::vector<std::vector<float>> seeds,
                               float spread,
                               std::size_t candidates,
                               float slack,
                               std::size_t max_attempts,
                               float fraction,
                               std::shared_ptr<vamp::rng::RNG<Robot>> mixture,
                               std::size_t memory,
                               float memory_spread,
                               std::size_t refine_steps,
                               float refine_step,
                               int local_mode,
                               float local_step,
                               float local_angle,
                               float local_speed,
                               float rollout_min,
                               float rollout_max,
                               float max_acceleration,
                               std::size_t eef,
                               const vc::ConstraintSettings &cs,
                               const vc::ChartSettings &chs) -> std::shared_ptr<TSS>
                            {
                                if (eef >= Ambient::n_eef)
                                {
                                    throw std::invalid_argument("eef out of range");
                                }

                                for (const auto &seed : seeds)
                                {
                                    if (seed.size() < Ambient::dimension)
                                    {
                                        throw std::invalid_argument("each seed needs one entry per joint");
                                    }
                                }

                                if (not typename CT::AmbientConstraintSetT(constraints, cs).exact_form())
                                {
                                    throw std::invalid_argument(
                                        "task_space_sampler projects with the exact chart: every constraint "
                                        "needs an equality form (all manifold rows tight)");
                                }

                                vc::TaskSpaceSamplerSettings settings;
                                settings.lower = lower;
                                settings.upper = upper;
                                settings.orientation = orientation;
                                settings.tilt = tilt;
                                settings.yaw = yaw;
                                settings.max_speed = max_speed;
                                settings.max_angular_speed = max_angular_speed;
                                settings.directions = std::move(directions);
                                settings.direction_jitter = direction_jitter;
                                settings.seeds = std::move(seeds);
                                settings.spread = spread;
                                settings.candidates = std::max<std::size_t>(candidates, 1);
                                settings.slack = slack;
                                settings.max_attempts = std::max<std::size_t>(max_attempts, 1);
                                settings.fraction = std::clamp(fraction, 0.F, 1.F);
                                settings.memory = memory;
                                settings.memory_spread = memory_spread;
                                settings.refine_steps = refine_steps;
                                settings.refine_step = refine_step;
                                settings.local_mode = local_mode;
                                settings.local_step = local_step;
                                settings.local_angle = local_angle;
                                settings.local_speed = local_speed;
                                settings.rollout_min = rollout_min;
                                settings.rollout_max = std::max(rollout_max, rollout_min);
                                settings.max_acceleration = max_acceleration;
                                auto out = std::make_shared<TSS>(
                                    std::move(sampler), std::move(constraints), cs, eef, object_in_eef,
                                    std::move(settings), chs, env, std::move(mixture));
                                if (not out->valid())
                                {
                                    throw std::invalid_argument(
                                        "this robot's task-space constraint has no equality kernel");
                                }

                                return out;
                            },
                            "sampler"_a,
                            "constraints"_a,
                            "object_in_eef"_a,
                            "environment"_a,
                            "lower"_a,
                            "upper"_a,
                            "orientation"_a,
                            "tilt"_a = 0.2F,
                            "yaw"_a = 0.5F,
                            "max_speed"_a = 0.8F,
                            "max_angular_speed"_a = 0.3F,
                            "directions"_a = std::vector<std::array<float, 4>>{},
                            "direction_jitter"_a = 0.3F,
                            "seeds"_a = std::vector<std::vector<float>>{},
                            "spread"_a = 0.3F,
                            "candidates"_a = 4,
                            "slack"_a = 0.2F,
                            "max_attempts"_a = 20,
                            "fraction"_a = 1.F,
                            "mixture"_a = nb::none(),
                            "memory"_a = 512,
                            "memory_spread"_a = 0.05F,
                            "refine_steps"_a = 3,
                            "refine_step"_a = 0.1F,
                            "local_mode"_a = 0,
                            "local_step"_a = 0.05F,
                            "local_angle"_a = 0.1F,
                            "local_speed"_a = 0.2F,
                            "rollout_min"_a = 0.1F,
                            "rollout_max"_a = 0.3F,
                            "max_acceleration"_a = 4.F,
                            "eef"_a = 0,
                            "constraint_settings"_a = vc::ConstraintSettings{},
                            "chart_settings"_a = vc::ChartSettings{},
                            "Task-space state sampler for a held object (end effector `eef`, object "
                            "frame `object_in_eef`, wxyz + xyz): draws the object's pose (position in "
                            "[lower, upper], orientation within +-yaw about its z and +-tilt about its x/y "
                            "of `orientation`) and twist (linear speed up to max_speed along a weighted "
                            "`directions` entry (x, y, z, weight) or a uniform direction; angular "
                            "components up to max_angular_speed), projects `candidates` postures onto "
                            "`constraints` plus the object pin (around random `seeds`, +-spread/2 of each "
                            "joint range, or anywhere), keeps the lowest static torque ratio under the "
                            "environment's payloads, and realizes the twist at minimum kinetic energy "
                            "with the constraints' rows still. Velocities are scaled into the joint limits "
                            "and halved until the zero-acceleration torque ratio is within 1 + slack; "
                            "draws that cannot hold the payload at rest within it are redrawn, falling "
                            "back to `sampler` after max_attempts. `sampler` supplies the uniform draws and "
                            "should be uniform; with fraction < 1 the remaining draws come from `mixture` "
                            "(default: `sampler`). Accepted postures are remembered by object position (up to "
                            "`memory`); half the candidates start from the nearest remembered postures, jittered "
                            "by +-memory_spread/2 of each joint range. The chosen posture is refined by up to "
                            "refine_steps null-space steps (refine_step rad) that lower its static torque ratio. "
                            "local_mode 1/2 enables local draws for planners that expand locally (BKPIECE): "
                            "the object steps from the selected state's pose and twist (1: within local_step m, "
                            "local_angle rad, local_speed m/s; 2: a rollout of rollout_min..rollout_max s under "
                            "an acceleration within the torque limits, at most max_acceleration m/s^2) and the "
                            "posture continues from the state's (object on end effector 0).");
                    }
                }
            }

            using XType = nanobind::
                ndarray<FloatT, nanobind::numpy, nanobind::shape<3 * n_q>, nanobind::device::cpu>;
            const char *torques_doc =
                "Joint torques for a flat state stack x = [q; qd; qdd], including the payloads of "
                "the environment's attachments.";
            submodule.def(
                "torques",
                [](const XType &x, const Env &env)
                {
                    std::array<float, 3 * n_q> xa;
                    std::vector<float> scratch;
                    const auto *ptr = as_flat_1d(x, 3 * n_q, scratch, "flat state");
                    std::memcpy(xa.data(), ptr, 3 * n_q * sizeof(float));
                    return Robot::torques(xa, TA::payload_of(env));
                },
                "x"_a,
                "environment"_a = Env{},
                torques_doc);
            submodule.def(
                "torques",
                [](const std::array<float, 3 * n_q> &x, const Env &env) { return Robot::torques(x, TA::payload_of(env)); },
                "x"_a,
                "environment"_a = Env{},
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

        // FLASK z-space sibling generated as a nested struct: bind it as a nested
        // submodule (e.g. vamp.panda.flask).
        if constexpr (has_flask_robot_v<Robot>)
        {
            init_robot<typename Robot::Flask>(submodule);
        }

        return submodule;
    }
}  // namespace vamp::binding
