#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/bindings/python/space_helper.hh>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/planners/aorrtc_settings.hh>
#include <vamp/planning/planners/grrtstar.hh>
#include <vamp/planning/planners/grrtstar_settings.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#endif

#include <nanobind/nanobind.h>

// Not every ParameterizedSpace needs a "mid pose" concept: it exists for bimanual/
// mobile-base spaces (e.g. RBY1, whose t_mid_left/t_mid_right hand offsets are derived
// from a reference whole-body configuration) but not single-arm ones (e.g. IiwaMarker,
// whose eef pose IS the task-space pose already). Detected via SFINAE so
// ParameterizedSpaceTraits::compute_mid_pose and its binding can each stay a no-op for
// robots without it, rather than hard-erroring at compile time.
VAMP_DEFINE_HAS_METHOD(compute_mid_pose)

// GCP (self-motion-manifold branch selector) is RBY1-specific: each rainbow arm has its
// own (elbow_sel, shoulder_sel, wrist_sel) triple, set independently per arm via
// Space::set_gcp(left, right). IiwaMarker has an analogous but differently-shaped
// concept (a single triple, set via set_smm) -- not the same signature, so not unified
// with this one; add a separate has_set_smm_v/binding if that needs exposing too.
VAMP_DEFINE_HAS_METHOD(set_gcp)

namespace vamp::binding
{
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    // Traits for planning over Robot::ParameterizedSpace: a task-space parameterization
    // that IK-resolves into Robot's own ambient Configuration space. Mirrors the shape of
    // StaticRobotTraits (robot_helper.hh) closely enough to reuse bind_path_class,
    // bind_path_io, bind_planning_result, and bind_sampler from api_binder.hh unmodified,
    // but only supports the local-planner-aware planners (rrtc/aorrtc/grrtstar) -- PRM/FCIT
    // don't take a local planner and so can't resolve task-space states into the ambient
    // robot.
    template <typename Robot, typename Input = SpaceNDArrayInput<Robot, typename Robot::ParameterizedSpace>>
    struct ParameterizedSpaceTraits
    {
        using RobotType = Robot;
        using Space = typename Robot::ParameterizedSpace;
        using Ambient = typename Space::Ambient;

        using Cfg = typename Input::Type;
        using Pth = std::vector<Cfg>;
        using Path = vamp::planning::Path<Robot, Space>;
        using PlanningResult = vamp::planning::PlanningResult<Robot, Space>;
        using Sampler = vamp::rng::RNG<Robot, Space>;
        using Env = vamp::collision::Environment<float>;
        using EnvVec = vamp::collision::Environment<vamp::FloatVector<rake>>;

        using Configuration = typename Space::State;
        using RNG = vamp::rng::RNG<Robot, Space>;
        using LocalPlanner =
            vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution, Space>;

        // Explicit specialization of a nested member template isn't legal while the
        // enclosing ParameterizedSpaceTraits is still a template, so pick the planner type
        // via std::conditional_t instead of a specialized helper struct.
        template <vamp::planning::Planner P>
        using TaskPlannerT = std::conditional_t<
            P == vamp::planning::Planner::RRTC,
            vamp::planning::RRTC<Robot, rake, Robot::resolution, Space>,
            std::conditional_t<
                P == vamp::planning::Planner::AORRTC,
                vamp::planning::AORRTC<Robot, rake, Robot::resolution, Space>,
                vamp::planning::GRRTStar<Robot, rake, Robot::resolution, Space>>>;

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return TaskPlannerT<P>::template solve<LocalPlanner>(
                Input::to(start), Input::to(goal), EnvVec(env), s, rng, LocalPlanner());
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

            return TaskPlannerT<P>::template solve<LocalPlanner>(
                Input::to(start), goals_v, EnvVec(env), s, rng, LocalPlanner());
        }

        // Distinct from the ambient robot's simplify() (which takes SimplifySettings and runs
        // the full bspline/reduce/perturb pipeline): this wraps the raw shortcut_path the
        // task-space demo uses, taking ShortcutSettings. Mutates path in place.
        static auto shortcut(
            Path &path,
            const Env &env,
            const vamp::planning::ShortcutSettings &settings) -> bool
        {
            return vamp::planning::shortcut_path<Robot, rake, Robot::resolution, LocalPlanner, Space>(
                path, EnvVec(env), settings, LocalPlanner());
        }

        // IK-resolve a single task-space state into the ambient robot's Configuration.
        // Returns (valid, ambient_configuration); does not collision-check the result --
        // callers that need that should follow up with the ambient robot's validate().
        //
        // Returns a plain std::array<float, Ambient::dimension> rather than
        // Ambient::ConfigurationArray: nanobind's std::array caster (nanobind/stl/array.h)
        // is specialized on std::array<Type, Size> exactly, and ConfigurationArray -- a
        // struct that derives from std::array<FloatT, dimension> to carry alignas -- is a
        // distinct type nanobind has no caster for, so returning it directly makes this
        // uncallable from python ("Unable to convert function return value to a Python
        // type!") despite compiling and generating a signature. Both are the same bytes,
        // so this is a lossless conversion.
        static auto resolve(const Cfg &state) -> std::pair<bool, std::array<float, Ambient::dimension>>
        {
            const auto s = Input::to(state);
            typename Space::template StateBlock<rake> block;
            for (std::size_t i = 0; i < Space::dimension; ++i)
            {
                block[i] = s.broadcast(i);
            }

            auto [valid, ambient_block] = Space::template resolve_block<rake>(block);

            std::array<float, Ambient::dimension> ambient;
            for (std::size_t i = 0; i < Ambient::dimension; ++i)
            {
                ambient[i] = ambient_block[{i, 0}];
            }

            return {valid, ambient};
        }

        // IK-free prefilter: hand geometry (+ any attachments) vs. environment/each other.
        // Not a substitute for a full collision check on the resolved ambient configuration.
        static auto eefs_collision_free(const Cfg &state, const Env &env) -> bool
        {
            const auto s = Input::to(state);
            typename Space::template StateBlock<rake> block;
            for (std::size_t i = 0; i < Space::dimension; ++i)
            {
                block[i] = s.broadcast(i);
            }

            return Space::template eefs_collision_free<rake>(EnvVec(env), block);
        }

        // Derive t_mid_left/t_mid_right (thread-local to Space) from a reference whole-body
        // ambient configuration; the normal way to set them before planning. Only present
        // on spaces with a mid-pose concept (e.g. RBY1); see has_compute_mid_pose_v above.
        // The binder (parameterized_space_binder.hh) only exposes this to python when the
        // if constexpr branch below is taken, so the else branch is unreachable in
        // practice -- it exists only so this method stays well-formed to reference (e.g.
        // take the address of) for spaces without compute_mid_pose.
        //
        // Takes a plain std::array<float, Ambient::dimension> rather than
        // Ambient::ConfigurationArray for the same reason resolve() returns one instead
        // of the latter -- see the comment there.
        static void compute_mid_pose(const std::array<float, Ambient::dimension> &ambient)
        {
            if constexpr (has_compute_mid_pose_v<Space>)
            {
                typename Ambient::ConfigurationArray q;
                for (std::size_t i = 0; i < Ambient::dimension; ++i)
                {
                    q[i] = ambient[i];
                }
                Space::compute_mid_pose(q);
            }
            else
            {
                (void) ambient;
                throw std::runtime_error("this task space has no mid-pose concept");
            }
        }

        // Overwrite the static-stability support polygon LocalPlanner checks the resolved
        // ambient center of mass against (see ParameterizedLocalPlanner::set_support_polygon).
        // Only meaningful on spaces with a compute_com utility (e.g. RBY1); on spaces without
        // one, com_within_support_polygon is a no-op (always true) regardless of what the
        // polygon is set to, so the binder (parameterized_space_binder.hh) doesn't expose this
        // at all rather than expose a setter with no effect. Same has_compute_com<Space, rake>
        // gate ParameterizedLocalPlanner::com_within_support_polygon itself uses.
        static void set_support_polygon(const std::vector<std::array<float, 2>> &polygon_xy)
        {
            if constexpr (vamp::planning::constraint::detail::has_compute_com<Space, rake>::value)
            {
                LocalPlanner::set_support_polygon(polygon_xy);
            }
            else
            {
                (void) polygon_xy;
                throw std::runtime_error("this task space has no center-of-mass/support-polygon concept");
            }
        }

        // Set the GCP (elbow/shoulder/wrist self-motion-manifold branch) selector shared by
        // every lane -- the "many task-space states, one shared branch" mode
        // ParameterizedLocalPlanner/resolve_and_check use while extending the tree, where
        // the whole planning problem is meant to stay on a single branch. `left`/`right`
        // are each (elbow_sel, shoulder_sel, wrist_sel), matching Space::set_gcp. Only
        // present on spaces with a GCP concept (currently RBY1); see has_set_gcp_v above.
        static void set_gcp(const std::array<float, 3> &left, const std::array<float, 3> &right)
        {
            if constexpr (has_set_gcp_v<Space>)
            {
                Space::set_gcp(left, right);
            }
            else
            {
                (void) left;
                (void) right;
                throw std::runtime_error("this task space has no GCP (self-motion-manifold branch) concept");
            }
        }

        static auto make_halton() -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::rng::Halton<Robot, Space>>();
        }

        static auto make_xorshift(std::uint64_t seed) -> std::shared_ptr<Sampler>
        {
#if defined(__x86_64__)
            return (seed == 0) ? std::make_shared<vamp::rng::XORShift<Robot, Space>>() :
                                 std::make_shared<vamp::rng::XORShift<Robot, Space>>(seed, seed + 1);
#else
            throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
        }

        // Path element access (path_get/set/append/insert/numpy), for bind_path_class /
        // bind_path_io.
        static auto path_get(const Path &p, std::size_t i) -> Cfg
        {
            return Input::from(p[i]);
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
            p.insert(p.begin() + static_cast<std::ptrdiff_t>(i), Input::to(c));
        }

        static auto path_numpy(const Path &p)
        {
            return make_ndarray_filled<2>(
                {p.size(), Space::dimension},
                [&](float *buf)
                {
                    for (std::size_t i = 0; i < p.size(); ++i)
                    {
                        auto arr = p[i].to_array();
                        for (std::size_t j = 0; j < Space::dimension; ++j)
                        {
                            buf[(i * Space::dimension) + j] = arr[j];
                        }
                    }
                });
        }

        static auto path_validate(Path &, const Env &) -> bool
        {
            // Path::validate only compiles for Space == Robot (see plan.hh); task-space path
            // validity goes through LocalPlanner::validate / resolve() + the ambient robot's
            // validate() instead.
            throw std::runtime_error(
                "path.validate() is not supported for a task-space Path; resolve() each "
                "waypoint and validate the ambient robot's configuration instead");
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.solved;
        }

        static void sampler_skip(Sampler &s, std::size_t n)
        {
            for (std::size_t i = 0; i < n; ++i)
            {
                s.next();
            }
        }

        static auto sampler_next(Sampler &s) -> Cfg
        {
            return Input::from(s.next());
        }
    };
}  // namespace vamp::binding
