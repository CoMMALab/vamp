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
#include <vamp/vector.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#endif

#include <nanobind/nanobind.h>

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
        static auto resolve(const Cfg &state) -> std::pair<bool, typename Ambient::ConfigurationArray>
        {
            const auto s = Input::to(state);
            typename Space::template StateBlock<rake> block;
            for (std::size_t i = 0; i < Space::dimension; ++i)
            {
                block[i] = s.broadcast(i);
            }

            auto [valid, ambient_block] = Space::template resolve_block<rake>(block);

            typename Ambient::ConfigurationArray ambient;
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
        // ambient configuration; the normal way to set them before planning.
        static void compute_mid_pose(const typename Ambient::ConfigurationArray &ambient)
        {
            Space::compute_mid_pose(ambient);
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
