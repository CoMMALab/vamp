#pragma once

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/simplify_settings.hh>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace vamp::jit
{
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;

    struct DynamicPath
    {
        std::shared_ptr<DynamicRobot> robot;
        std::vector<std::vector<float>> waypoints;
        std::size_t dim{0};

        DynamicPath() = default;

        explicit DynamicPath(std::shared_ptr<DynamicRobot> r)
          : robot(std::move(r)), dim(robot ? robot->dimension() : 0)
        {
        }

        auto size() const -> std::size_t
        {
            return waypoints.size();
        }

    private:
        // The path_helpers in plan.hh use ADL on std::vector<float>, which
        // gives Euclidean L2 / linear blend. That's wrong for non-Euclidean
        // JIT robots (e.g. PR2's planar base — L2 of cos/sin is not the SO(2)
        // geodesic). Route through the JIT FFI so the metric and interpolation
        // are the same ones the planner uses internally.

        auto _distance(const std::vector<float> &a, const std::vector<float> &b) const -> float
        {
            return robot->ops().cfg_distance(a.data(), b.data());
        }

        auto _interpolate(const std::vector<float> &a, const std::vector<float> &b, float t) const
            -> std::vector<float>
        {
            std::vector<float> out(dim);
            robot->ops().cfg_interpolate(a.data(), b.data(), t, out.data());
            return out;
        }

    public:
        auto cost() const -> float
        {
            const auto n = waypoints.size();
            if (n < 2)
            {
                return std::numeric_limits<float>::infinity();
            }
            // Match static Path<Robot>::cost(): robot-aware edge cost (LQMT C_loc on flask
            // siblings), falling back to the distance metric on ambient robots.
            const ffi::FlaskCostFn edge_cost = robot ? robot->ops().flask.cost : nullptr;
            float total = 0;
            for (std::size_t i = 0; i + 1 < n; ++i)
            {
                total += (edge_cost != nullptr) ?
                             edge_cost(waypoints[i].data(), waypoints[i + 1].data()) :
                             _distance(waypoints[i], waypoints[i + 1]);
            }
            return total;
        }

        auto subdivide() -> void
        {
            const auto n = waypoints.size();
            if (n < 2)
            {
                return;
            }
            std::vector<std::vector<float>> next;
            next.reserve(n * 2);
            for (std::size_t i = 0; i + 1 < n; ++i)
            {
                next.emplace_back(waypoints[i]);
                next.emplace_back(_interpolate(waypoints[i], waypoints[i + 1], 0.5F));
            }
            next.emplace_back(waypoints.back());
            waypoints = std::move(next);
        }

        auto interpolate_to_n_states(std::size_t n) -> void
        {
            const auto n_p = waypoints.size();
            if (n_p < 2 or n < n_p)
            {
                return;
            }
            std::vector<float> seg_lengths(n_p - 1);
            float remaining_length = 0.;
            for (std::size_t i = 0; i + 1 < n_p; ++i)
            {
                seg_lengths[i] = _distance(waypoints[i], waypoints[i + 1]);
                remaining_length += seg_lengths[i];
            }
            if (remaining_length < std::numeric_limits<float>::epsilon())
            {
                return;
            }
            std::vector<std::vector<float>> next;
            next.reserve(n);
            const auto n1 = n_p - 1;
            for (std::size_t i = 0; i < n1; ++i)
            {
                const auto &a = waypoints[i];
                const auto &b = waypoints[i + 1];
                next.emplace_back(a);
                const auto max_n_states = n + i - n_p;
                if (max_n_states > 0)
                {
                    auto ns = (i + 1 == n1) ? (max_n_states + 2) :
                                              (static_cast<std::size_t>(
                                                   std::floor(0.5 + n * seg_lengths[i] / remaining_length)) +
                                               1);
                    ns = (ns > 2) ? std::min(ns - 2, max_n_states) : 0;
                    for (std::size_t k = 1; k <= ns; ++k)
                    {
                        next.emplace_back(
                            _interpolate(a, b, static_cast<float>(k) / static_cast<float>(ns)));
                    }
                    n -= ns + 1;
                    remaining_length -= seg_lengths[i];
                }
                else
                {
                    n -= 1;
                }
            }
            next.emplace_back(waypoints.back());
            waypoints = std::move(next);
        }

        auto interpolate_to_resolution(std::size_t resolution) -> void
        {
            const auto n_p = waypoints.size();
            if (n_p < 2)
            {
                return;
            }
            std::vector<std::vector<float>> next;
            for (std::size_t i = 0; i + 1 < n_p; ++i)
            {
                const auto &a = waypoints[i];
                const auto &b = waypoints[i + 1];
                const float segment_cost = _distance(a, b);
                const auto segment_states =
                    static_cast<std::size_t>(segment_cost * static_cast<float>(resolution));
                next.emplace_back(a);
                if (segment_cost < 1.F / static_cast<float>(resolution))
                {
                    continue;
                }
                for (std::size_t k = 1; k < segment_states; ++k)
                {
                    next.emplace_back(
                        _interpolate(
                            a, b, static_cast<float>(k) / static_cast<float>(segment_states)));
                }
            }
            next.emplace_back(waypoints.back());
            waypoints = std::move(next);
        }

        auto validate(const vamp::collision::Environment<float> &env) -> bool
        {
            if (not robot)
            {
                throw std::runtime_error("DynamicPath has no associated robot");
            }
            const auto &ops = robot->ops();
            for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
            {
                if (not ops.validate_motion(
                        waypoints[i].data(), waypoints[i + 1].data(), static_cast<const void *>(&env), 0))
                {
                    return false;
                }
            }
            return true;
        }
    };

    struct DynamicSampler
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::SamplerHandle *handle{nullptr};

        DynamicSampler() = default;

        DynamicSampler(std::shared_ptr<DynamicRobot> r, ffi::SamplerHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicSampler()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().sampler_destroy(handle);
            }
        }

        DynamicSampler(const DynamicSampler &) = delete;
        DynamicSampler &operator=(const DynamicSampler &) = delete;

        auto reset() -> void
        {
            robot->ops().sampler_reset(handle);
        }

        auto skip(std::uint64_t n) -> void
        {
            robot->ops().sampler_skip(handle, n);
        }

        auto next(float *out) -> void
        {
            robot->ops().sampler_next(handle, out);
        }
    };

    struct DynamicPhs
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::PhsHandle *handle{nullptr};

        DynamicPhs() = default;

        DynamicPhs(std::shared_ptr<DynamicRobot> r, ffi::PhsHandle *h) : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicPhs()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().phs_destroy(handle);
            }
        }

        DynamicPhs(const DynamicPhs &) = delete;
        DynamicPhs &operator=(const DynamicPhs &) = delete;

        auto set_transverse_diameter(float diameter) -> void
        {
            robot->ops().phs_set_diameter(handle, diameter);
        }

        auto transform(const float *in, float *out) const -> void
        {
            robot->ops().phs_transform(handle, in, out);
        }
    };

    struct DynamicPlanResult
    {
        std::shared_ptr<DynamicPath> path;
        bool success{false};
        std::size_t iterations{0};
        std::uint64_t nanoseconds{0};
        std::vector<std::size_t> size;

        auto solved() const -> bool
        {
            return success;
        }
    };

    struct DynamicConstraint
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::ConstraintHandle *handle{nullptr};

        DynamicConstraint() = default;

        DynamicConstraint(std::shared_ptr<DynamicRobot> r, ffi::ConstraintHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicConstraint()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().constraint.destroy(handle);
            }
        }

        DynamicConstraint(const DynamicConstraint &) = delete;
        DynamicConstraint &operator=(const DynamicConstraint &) = delete;
    };

    using DynamicConstraintVec = std::vector<std::shared_ptr<DynamicConstraint>>;
    using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;

    // Match the remedy messages of the static constrained bindings so scripts can be
    // shared between static modules and JIT robots.
    inline constexpr const char *manifold_message =
        " configuration violates the constraints; project it first with project()";
    inline constexpr const char *manifold_simplify_message =
        " violates the constraints; simplify requires an on-manifold path";

    inline auto constraint_ops_checked(const DynamicRobot &robot) -> const RobotOps::ConstraintOps &
    {
        const auto &cops = robot.ops().constraint;
        if (cops.satisfied == nullptr)
        {
            throw std::runtime_error(
                "this robot was generated without constraint kernels; pass constraint recipe "
                "options (e.g., constraints=True) to load_robot");
        }
        return cops;
    }

    inline auto constraint_handles(const DynamicRobot &robot, const DynamicConstraintVec &constraints)
        -> std::vector<ffi::ConstraintHandle *>
    {
        std::vector<ffi::ConstraintHandle *> handles;
        handles.reserve(constraints.size());
        for (const auto &c : constraints)
        {
            if (not c or c->handle == nullptr)
            {
                throw std::invalid_argument("null constraint");
            }

            if (c->robot.get() != &robot)
            {
                throw std::invalid_argument("constraint was created for a different robot");
            }

            handles.emplace_back(c->handle);
        }
        return handles;
    }

    namespace detail
    {
        template <typename F, typename... Args>
        inline auto make_constraint(
            std::shared_ptr<DynamicRobot> robot,
            F factory,
            const char *missing_message,
            Args &&...args) -> std::shared_ptr<DynamicConstraint>
        {
            if (factory == nullptr)
            {
                throw std::runtime_error(missing_message);
            }

            auto *h = factory(std::forward<Args>(args)...);
            return std::make_shared<DynamicConstraint>(std::move(robot), h);
        }
    }  // namespace detail

    inline auto make_task_space_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *eef_to_offset,
        const float *world_to_reference,
        const float *lower,
        const float *upper) -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.task_space_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without task-space constraint kernels; pass "
            "constraints=True to load_robot",
            eef_to_offset,
            world_to_reference,
            lower,
            upper);
    }

    inline auto make_bimanual_task_space_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *right_in_left,
        const float *lower,
        const float *upper) -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.bimanual_task_space_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without bimanual constraint kernels; pass "
            "constraints=True and at least two end_effectors to load_robot",
            right_in_left,
            lower,
            upper);
    }

    inline auto make_closed_loop_constraint(std::shared_ptr<DynamicRobot> robot)
        -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.closed_loop_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without closed-loop constraint kernels; pass "
            "closed_loops=[...] to load_robot");
    }

    inline auto make_com_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *vertices_xy,
        std::uint64_t n_vertices) -> std::shared_ptr<DynamicConstraint>
    {
        // Host-side mirror of the CoMConstraint constructor check: exceptions must not
        // cross JIT-compiled frames.
        if (n_vertices < 3)
        {
            throw std::invalid_argument("CoM support polygon needs at least three vertices");
        }

        auto factory = robot->ops().constraint.com_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without center-of-mass constraint kernels; pass "
            "com=True (or com={\"reference_frames\": [...]}) to load_robot",
            vertices_xy,
            n_vertices);
    }

    inline auto make_twist_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *eef_to_offset,
        const float *world_to_reference,
        const float *reference_coefficients,
        const float *body_coefficients) -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.twist_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without twist constraint kernels; pass twist=True "
            "to load_robot",
            eef_to_offset,
            world_to_reference,
            reference_coefficients,
            body_coefficients);
    }

    inline auto make_lead_screw_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *eef_to_offset,
        const float *world_to_reference,
        float pitch) -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.lead_screw_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without lead-screw constraint kernels; pass "
            "lead_screw=True to load_robot",
            eef_to_offset,
            world_to_reference,
            pitch);
    }

    inline auto make_lead_screw_level_constraint(
        std::shared_ptr<DynamicRobot> robot,
        const float *eef_to_offset,
        const float *world_to_reference,
        float pitch,
        float target) -> std::shared_ptr<DynamicConstraint>
    {
        auto factory = robot->ops().constraint.lead_screw_level_new;
        return detail::make_constraint(
            std::move(robot),
            factory,
            "this robot was generated without lead-screw constraint kernels; pass "
            "lead_screw=True to load_robot",
            eef_to_offset,
            world_to_reference,
            pitch,
            target);
    }

    inline auto make_halton_sampler(std::shared_ptr<DynamicRobot> robot) -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_halton();
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_xorshift_sampler(std::shared_ptr<DynamicRobot> robot, std::uint64_t seed = 0)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_xorshift(seed);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto make_phs(std::shared_ptr<DynamicRobot> robot, const float *focus_a, const float *focus_b)
        -> std::shared_ptr<DynamicPhs>
    {
        auto *h = robot->ops().phs_new(focus_a, focus_b);
        return std::make_shared<DynamicPhs>(std::move(robot), h);
    }

    inline auto
    make_phs_sampler(std::shared_ptr<DynamicRobot> robot, const DynamicPhs &phs, DynamicSampler &inner)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = robot->ops().sampler_phs(phs.handle, inner.handle);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    inline auto drain_handle(std::shared_ptr<DynamicRobot> robot, ffi::PlanResultHandle *handle)
        -> DynamicPlanResult
    {
        const auto &ops = robot->ops();
        auto meta = ops.result_meta(handle);

        DynamicPlanResult result;
        result.success = meta.success != 0;
        result.iterations = meta.iterations;
        result.nanoseconds = meta.nanoseconds;
        result.path = std::make_shared<DynamicPath>(robot);
        result.path->waypoints.reserve(meta.waypoints);

        std::vector<float> wp(meta.dimension);
        for (auto i = 0U; i < meta.waypoints; ++i)
        {
            ops.result_copy_waypoint(handle, i, wp.data());
            result.path->waypoints.emplace_back(wp);
        }

        ops.result_sizes(handle, &result.size);
        ops.result_destroy(handle);
        return result;
    }

    template <typename SettingsT>
    inline auto solve(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto &p = robot->ops().planners[static_cast<std::size_t>(planner)];
        if (p.solve == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *handle = p.solve(
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        return drain_handle(std::move(robot), handle);
    }

    template <typename SettingsT>
    inline auto solve_multi(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        const auto &p = robot->ops().planners[static_cast<std::size_t>(planner)];
        if (p.solve_multi == nullptr)
        {
            throw std::runtime_error("planner not loaded on this DynamicRobot");
        }

        auto *handle = p.solve_multi(
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);

        return drain_handle(std::move(robot), handle);
    }

    inline auto simplify(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler) -> DynamicPlanResult
    {
        auto *handle = robot->ops().simplify(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle);
        return drain_handle(std::move(robot), handle);
    }

    inline auto constraint_satisfied(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs) -> bool
    {
        const auto &cops = constraint_ops_checked(*robot);
        auto handles = constraint_handles(*robot, constraints);
        return cops.satisfied(config, handles.data(), handles.size(), static_cast<const void *>(&cs)) != 0;
    }

    inline auto constraint_project(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs) -> std::vector<float>
    {
        const auto &cops = constraint_ops_checked(*robot);
        auto handles = constraint_handles(*robot, constraints);
        std::vector<float> out(robot->dimension());
        if (cops.project(
                config, handles.data(), handles.size(), static_cast<const void *>(&cs), out.data()) == 0)
        {
            throw std::invalid_argument("projection onto the constraint manifold did not converge");
        }

        return out;
    }

    namespace detail
    {
        // Feasibility checks live host-side (rather than on the JIT side of the FFI like
        // the static bindings' check_feasible) so that no exception unwinds through a
        // JIT-compiled frame.
        inline auto check_feasible(
            const RobotOps::ConstraintOps &cops,
            const std::vector<ffi::ConstraintHandle *> &handles,
            const ConstraintSettings &cs,
            const float *q,
            const char *what,
            const char *message) -> void
        {
            if (cops.satisfied(q, handles.data(), handles.size(), static_cast<const void *>(&cs)) == 0)
            {
                throw std::invalid_argument(std::string(what) + message);
            }
        }

        inline auto constrained_planner_entry(const DynamicRobot &robot, vamp::planning::Planner planner)
            -> const RobotOps::PlannerEntry &
        {
            const auto &p = robot.ops().planners[static_cast<std::size_t>(planner)];
            if (p.solve == nullptr)
            {
                throw std::runtime_error("planner not loaded on this DynamicRobot");
            }

            if (p.solve_constrained == nullptr)
            {
                throw std::runtime_error(
                    "planner '" + std::string(vamp::planning::planner_name(planner)) +
                    "' does not support constrained planning");
            }

            return p;
        }
    }  // namespace detail

    template <typename SettingsT>
    inline auto solve_constrained(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return solve(std::move(robot), planner, start, goal, env, settings, sampler);
        }

        const auto &cops = constraint_ops_checked(*robot);
        const auto &p = detail::constrained_planner_entry(*robot, planner);
        auto handles = constraint_handles(*robot, constraints);

        detail::check_feasible(cops, handles, cs, start, "start", manifold_message);
        detail::check_feasible(cops, handles, cs, goal, "goal", manifold_message);

        auto *handle = p.solve_constrained(
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs));

        return drain_handle(std::move(robot), handle);
    }

    template <typename SettingsT>
    inline auto solve_multi_constrained(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return solve_multi(std::move(robot), planner, start, goals, n_goals, env, settings, sampler);
        }

        const auto &cops = constraint_ops_checked(*robot);
        const auto &p = detail::constrained_planner_entry(*robot, planner);
        auto handles = constraint_handles(*robot, constraints);

        detail::check_feasible(cops, handles, cs, start, "start", manifold_message);
        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_goals; ++i)
        {
            detail::check_feasible(cops, handles, cs, goals + i * dim, "goal", manifold_message);
        }

        auto *handle = p.solve_multi_constrained(
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs));

        return drain_handle(std::move(robot), handle);
    }

    inline auto simplify_constrained(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return simplify(std::move(robot), path, n_waypoints, env, settings, sampler);
        }

        const auto &cops = constraint_ops_checked(*robot);
        auto handles = constraint_handles(*robot, constraints);

        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_waypoints; ++i)
        {
            if (cops.satisfied(
                    path + i * dim, handles.data(), handles.size(), static_cast<const void *>(&cs)) == 0)
            {
                throw std::invalid_argument(
                    "path state " + std::to_string(i) + manifold_simplify_message);
            }
        }

        auto *handle = cops.simplify_constrained(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs));

        return drain_handle(std::move(robot), handle);
    }

    // ------------------------------------------------------------------
    // Flask (flat-system) siblings: phase constraints, LQMT kernels, and
    // chart-constrained planning. All entries below expect the flask sibling
    // robot (robot.flask), not the ambient robot.

    struct DynamicPhaseConstraint
    {
        std::shared_ptr<DynamicRobot> robot;
        ffi::PhaseConstraintHandle *handle{nullptr};

        DynamicPhaseConstraint() = default;

        DynamicPhaseConstraint(std::shared_ptr<DynamicRobot> r, ffi::PhaseConstraintHandle *h)
          : robot(std::move(r)), handle(h)
        {
        }

        ~DynamicPhaseConstraint()
        {
            if (handle != nullptr and robot)
            {
                robot->ops().flask.phase_destroy(handle);
            }
        }

        DynamicPhaseConstraint(const DynamicPhaseConstraint &) = delete;
        DynamicPhaseConstraint &operator=(const DynamicPhaseConstraint &) = delete;
    };

    using DynamicPhaseConstraintVec = std::vector<std::shared_ptr<DynamicPhaseConstraint>>;
    using ChartSettings = vamp::planning::constraint::ChartSettings;

    // Match the remedy messages of the static flask bindings.
    inline constexpr const char *phase_message =
        " state violates the phase constraints; scale its velocity by velocity_scale() first";
    inline constexpr const char *phase_simplify_message =
        " violates the phase constraints; simplify requires a phase-feasible path";
    inline constexpr const char *chart_message =
        " state violates the constraints; project it first with project()";

    inline auto flask_ops_checked(const DynamicRobot &robot) -> const RobotOps::FlaskOps &
    {
        const auto &fops = robot.ops().flask;
        if (fops.optimal_time == nullptr)
        {
            throw std::runtime_error(
                "this robot has no flask kernels; pass flask={...} to load_robot and call "
                "these methods on robot.flask");
        }
        return fops;
    }

    inline auto chart_ops_checked(const DynamicRobot &robot) -> const RobotOps::FlaskOps &
    {
        const auto &fops = flask_ops_checked(robot);
        if (fops.chart_project == nullptr)
        {
            throw std::runtime_error(
                "this robot was generated without chart kernels; chart-constrained planning "
                "requires constraint options (e.g., constraints=True) on the ambient robot");
        }
        return fops;
    }

    inline auto phase_handles(const DynamicRobot &robot, const DynamicPhaseConstraintVec &phase)
        -> std::vector<ffi::PhaseConstraintHandle *>
    {
        std::vector<ffi::PhaseConstraintHandle *> handles;
        handles.reserve(phase.size());
        for (const auto &c : phase)
        {
            if (not c or c->handle == nullptr)
            {
                throw std::invalid_argument("null phase constraint");
            }

            if (c->robot.get() != &robot)
            {
                throw std::invalid_argument("phase constraint was created for a different robot");
            }

            handles.emplace_back(c->handle);
        }
        return handles;
    }

    inline auto make_kinetic_energy_constraint(std::shared_ptr<DynamicRobot> robot, float max_energy)
        -> std::shared_ptr<DynamicPhaseConstraint>
    {
        auto *h = flask_ops_checked(*robot).phase_kinetic_energy_new(max_energy);
        return std::make_shared<DynamicPhaseConstraint>(std::move(robot), h);
    }

    inline auto make_eef_speed_constraint(std::shared_ptr<DynamicRobot> robot, float max_speed)
        -> std::shared_ptr<DynamicPhaseConstraint>
    {
        auto *h = flask_ops_checked(*robot).phase_eef_speed_new(max_speed);
        return std::make_shared<DynamicPhaseConstraint>(std::move(robot), h);
    }

    inline auto
    make_ke_shaped_sampler(std::shared_ptr<DynamicRobot> robot, DynamicSampler &inner, float max_energy)
        -> std::shared_ptr<DynamicSampler>
    {
        auto *h = flask_ops_checked(*robot).sampler_ke_shaped(inner.handle, max_energy);
        return std::make_shared<DynamicSampler>(std::move(robot), h);
    }

    struct DynamicCostGrad
    {
        float cost{0.0F};
        float time{0.0F};
        std::vector<float> grad_a;
        std::vector<float> grad_b;
    };

    inline auto flask_optimal_time(DynamicRobot &robot, const float *a, const float *b) -> float
    {
        return flask_ops_checked(robot).optimal_time(a, b);
    }

    inline auto flask_cost(DynamicRobot &robot, const float *a, const float *b) -> float
    {
        return flask_ops_checked(robot).cost(a, b);
    }

    inline auto flask_cost_grad(DynamicRobot &robot, const float *a, const float *b) -> DynamicCostGrad
    {
        const auto &fops = flask_ops_checked(robot);
        DynamicCostGrad out;
        out.grad_a.resize(robot.dimension());
        out.grad_b.resize(robot.dimension());
        fops.cost_grad(a, b, &out.cost, &out.time, out.grad_a.data(), out.grad_b.data());
        return out;
    }

    inline auto flask_eval(DynamicRobot &robot, const float *a, const float *b, float T, float t)
        -> std::vector<float>
    {
        const auto &fops = flask_ops_checked(robot);
        std::vector<float> out(3 * fops.flat_dimension());
        fops.eval(a, b, T, t, out.data());
        return out;
    }

    inline auto flask_torques(DynamicRobot &robot, const float *x) -> std::vector<float>
    {
        const auto &fops = flask_ops_checked(robot);
        std::vector<float> out(fops.flat_dimension());
        fops.torques(x, out.data());
        return out;
    }

    inline auto flask_kinetic_energy(DynamicRobot &robot, const float *x) -> float
    {
        return flask_ops_checked(robot).kinetic_energy(x);
    }

    inline auto flask_eef_velocity(DynamicRobot &robot, const float *x) -> std::vector<float>
    {
        const auto &fops = flask_ops_checked(robot);
        std::vector<float> out(3 * fops.n_end_effectors());
        fops.eef_velocity(x, out.data());
        return out;
    }

    inline auto flask_n_end_effectors(DynamicRobot &robot) -> std::uint64_t
    {
        return flask_ops_checked(robot).n_end_effectors();
    }

    inline auto flask_flat_dimension(DynamicRobot &robot) -> std::uint64_t
    {
        return flask_ops_checked(robot).flat_dimension();
    }

    inline auto flask_rho(DynamicRobot &robot) -> float
    {
        return flask_ops_checked(robot).rho_get();
    }

    inline auto flask_set_rho(DynamicRobot &robot, float rho) -> void
    {
        flask_ops_checked(robot).rho_set(rho);
    }

    inline auto flask_velocity_limits(DynamicRobot &robot) -> std::vector<float>
    {
        const auto &fops = flask_ops_checked(robot);
        std::vector<float> out(fops.flat_dimension());
        fops.velocity_limits(out.data());
        return out;
    }

    inline auto flask_effort_limits(DynamicRobot &robot) -> std::vector<float>
    {
        const auto &fops = flask_ops_checked(robot);
        std::vector<float> out(fops.flat_dimension());
        fops.effort_limits(out.data());
        return out;
    }

    inline auto phase_satisfied(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicPhaseConstraintVec &phase) -> bool
    {
        const auto &fops = flask_ops_checked(*robot);
        auto handles = phase_handles(*robot, phase);
        return fops.phase_satisfied(config, handles.data(), handles.size()) != 0;
    }

    inline auto phase_velocity_scale(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicPhaseConstraintVec &phase) -> float
    {
        const auto &fops = flask_ops_checked(*robot);
        auto handles = phase_handles(*robot, phase);
        return fops.phase_velocity_scale(config, handles.data(), handles.size());
    }

    namespace detail
    {
        inline auto phase_planner_entry(const DynamicRobot &robot, vamp::planning::Planner planner)
            -> const RobotOps::PlannerEntry &
        {
            const auto &p = robot.ops().planners[static_cast<std::size_t>(planner)];
            if (p.solve == nullptr)
            {
                throw std::runtime_error("planner not loaded on this DynamicRobot");
            }

            if (p.solve_phase == nullptr)
            {
                throw std::runtime_error(
                    "planner '" + std::string(vamp::planning::planner_name(planner)) +
                    "' does not support constrained planning");
            }

            return p;
        }

        inline auto chart_planner_entry(const DynamicRobot &robot, vamp::planning::Planner planner)
            -> const RobotOps::PlannerEntry &
        {
            const auto &p = robot.ops().planners[static_cast<std::size_t>(planner)];
            if (p.solve == nullptr)
            {
                throw std::runtime_error("planner not loaded on this DynamicRobot");
            }

            if (p.solve_chart == nullptr)
            {
                throw std::runtime_error(
                    "planner '" + std::string(vamp::planning::planner_name(planner)) +
                    "' does not support constrained planning");
            }

            return p;
        }

        inline auto check_phase_feasible(
            const RobotOps::FlaskOps &fops,
            const std::vector<ffi::PhaseConstraintHandle *> &handles,
            const float *q,
            const char *what,
            const char *message) -> void
        {
            if (fops.phase_satisfied(q, handles.data(), handles.size()) == 0)
            {
                throw std::invalid_argument(std::string(what) + message);
            }
        }

        // Mirrors the static bindings, which check start/goal feasibility against the
        // same LP the planner uses (real constraint + chart settings + phase set).
        inline auto check_chart_feasible(
            const RobotOps::FlaskOps &fops,
            const float *q,
            const std::vector<ffi::ConstraintHandle *> &constraints,
            const ConstraintSettings &cs,
            const ChartSettings &chs,
            const std::vector<ffi::PhaseConstraintHandle *> &phase,
            const char *what,
            const char *message) -> void
        {
            if (fops.chart_satisfied(
                    q,
                    constraints.data(),
                    constraints.size(),
                    static_cast<const void *>(&cs),
                    static_cast<const void *>(&chs),
                    phase.data(),
                    phase.size()) == 0)
            {
                throw std::invalid_argument(std::string(what) + message);
            }
        }
    }  // namespace detail

    template <typename SettingsT>
    inline auto solve_phase(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (phase.empty())
        {
            return solve(std::move(robot), planner, start, goal, env, settings, sampler);
        }

        const auto &fops = flask_ops_checked(*robot);
        const auto &p = detail::phase_planner_entry(*robot, planner);
        auto handles = phase_handles(*robot, phase);

        detail::check_phase_feasible(fops, handles, start, "start", phase_message);
        detail::check_phase_feasible(fops, handles, goal, "goal", phase_message);

        auto *handle = p.solve_phase(
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size());

        return drain_handle(std::move(robot), handle);
    }

    template <typename SettingsT>
    inline auto solve_multi_phase(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (phase.empty())
        {
            return solve_multi(std::move(robot), planner, start, goals, n_goals, env, settings, sampler);
        }

        const auto &fops = flask_ops_checked(*robot);
        const auto &p = detail::phase_planner_entry(*robot, planner);
        auto handles = phase_handles(*robot, phase);

        detail::check_phase_feasible(fops, handles, start, "start", phase_message);
        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_goals; ++i)
        {
            detail::check_phase_feasible(fops, handles, goals + i * dim, "goal", phase_message);
        }

        auto *handle = p.solve_multi_phase(
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size());

        return drain_handle(std::move(robot), handle);
    }

    inline auto simplify_phase(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (phase.empty())
        {
            return simplify(std::move(robot), path, n_waypoints, env, settings, sampler);
        }

        const auto &fops = flask_ops_checked(*robot);
        auto handles = phase_handles(*robot, phase);

        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_waypoints; ++i)
        {
            if (fops.phase_satisfied(path + i * dim, handles.data(), handles.size()) == 0)
            {
                throw std::invalid_argument("path state " + std::to_string(i) + phase_simplify_message);
            }
        }

        auto *handle = fops.simplify_phase(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size());

        return drain_handle(std::move(robot), handle);
    }

    // Chart-constrained entries: manifold constraints are ambient-robot constraints, so
    // handle identity is validated against this flask robot's ambient.

    inline auto chart_project(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const ChartSettings &chs,
        const DynamicPhaseConstraintVec &phase) -> std::vector<float>
    {
        const auto &fops = chart_ops_checked(*robot);
        auto handles = constraint_handles(*robot->ambient(), constraints);
        auto ph = phase_handles(*robot, phase);

        std::vector<float> out(robot->dimension());
        if (fops.chart_project(
                config,
                handles.data(),
                handles.size(),
                static_cast<const void *>(&cs),
                static_cast<const void *>(&chs),
                ph.data(),
                ph.size(),
                out.data()) == 0)
        {
            throw std::invalid_argument("projection onto the constraint manifold did not converge");
        }

        return out;
    }

    inline auto chart_satisfied(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const DynamicPhaseConstraintVec &phase) -> bool
    {
        const auto &fops = chart_ops_checked(*robot);
        auto handles = constraint_handles(*robot->ambient(), constraints);
        auto ph = phase_handles(*robot, phase);

        // The static binding builds its local planner with default chart settings here.
        const ChartSettings chs{};
        return fops.chart_satisfied(
                   config,
                   handles.data(),
                   handles.size(),
                   static_cast<const void *>(&cs),
                   static_cast<const void *>(&chs),
                   ph.data(),
                   ph.size()) != 0;
    }

    inline auto chart_debug(
        std::shared_ptr<DynamicRobot> robot,
        const float *config,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const ChartSettings &chs) -> std::vector<std::vector<float>>
    {
        const auto &fops = chart_ops_checked(*robot);
        auto handles = constraint_handles(*robot->ambient(), constraints);

        std::vector<float> flat;
        fops.chart_debug(
            config,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs),
            static_cast<const void *>(&chs),
            &flat);

        const auto width = static_cast<std::size_t>(fops.flat_dimension());
        std::vector<std::vector<float>> rows;
        rows.reserve(flat.size() / width);
        for (std::size_t i = 0; i + width <= flat.size(); i += width)
        {
            rows.emplace_back(flat.begin() + i, flat.begin() + i + width);
        }
        return rows;
    }

    struct DynamicLiftedEdge
    {
        std::vector<std::vector<float>> states;
        std::vector<float> errors;
        float duration{0.0F};
        float cost{0.0F};
    };

    inline auto chart_lift_edge(
        std::shared_ptr<DynamicRobot> robot,
        const float *from,
        const float *target,
        const DynamicConstraintVec &constraints,
        bool forward,
        std::uint64_t n_samples,
        const ConstraintSettings &cs,
        const ChartSettings &chs) -> DynamicLiftedEdge
    {
        const auto &fops = chart_ops_checked(*robot);
        auto handles = constraint_handles(*robot->ambient(), constraints);

        std::vector<float> flat_states;
        DynamicLiftedEdge out;
        if (fops.chart_lift_edge(
                from,
                target,
                handles.data(),
                handles.size(),
                forward ? 1 : 0,
                n_samples,
                static_cast<const void *>(&cs),
                static_cast<const void *>(&chs),
                &flat_states,
                &out.errors,
                &out.duration,
                &out.cost) == 0)
        {
            throw std::invalid_argument(
                "edge lift failed: no chart at the from state, a lift diverged, or the "
                "time-optimal solve failed");
        }

        const auto width = robot->dimension();
        out.states.reserve(flat_states.size() / width);
        for (std::size_t i = 0; i + width <= flat_states.size(); i += width)
        {
            out.states.emplace_back(flat_states.begin() + i, flat_states.begin() + i + width);
        }
        return out;
    }

    template <typename SettingsT>
    inline auto solve_chart(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goal,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const ChartSettings &chs,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return solve_phase(std::move(robot), planner, start, goal, env, settings, sampler, phase);
        }

        const auto &fops = chart_ops_checked(*robot);
        const auto &p = detail::chart_planner_entry(*robot, planner);
        auto handles = constraint_handles(*robot->ambient(), constraints);
        auto ph = phase_handles(*robot, phase);

        detail::check_chart_feasible(fops, start, handles, cs, chs, ph, "start", chart_message);
        detail::check_chart_feasible(fops, goal, handles, cs, chs, ph, "goal", chart_message);

        auto *handle = p.solve_chart(
            start,
            goal,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs),
            static_cast<const void *>(&chs),
            ph.data(),
            ph.size());

        return drain_handle(std::move(robot), handle);
    }

    template <typename SettingsT>
    inline auto solve_multi_chart(
        std::shared_ptr<DynamicRobot> robot,
        vamp::planning::Planner planner,
        const float *start,
        const float *goals,
        std::uint64_t n_goals,
        const vamp::collision::Environment<float> &env,
        const SettingsT &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const ChartSettings &chs,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return solve_multi_phase(
                std::move(robot), planner, start, goals, n_goals, env, settings, sampler, phase);
        }

        const auto &fops = chart_ops_checked(*robot);
        const auto &p = detail::chart_planner_entry(*robot, planner);
        auto handles = constraint_handles(*robot->ambient(), constraints);
        auto ph = phase_handles(*robot, phase);

        detail::check_chart_feasible(fops, start, handles, cs, chs, ph, "start", chart_message);
        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_goals; ++i)
        {
            detail::check_chart_feasible(
                fops, goals + i * dim, handles, cs, chs, ph, "goal", chart_message);
        }

        auto *handle = p.solve_multi_chart(
            start,
            goals,
            n_goals,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs),
            static_cast<const void *>(&chs),
            ph.data(),
            ph.size());

        return drain_handle(std::move(robot), handle);
    }

    inline auto simplify_chart(
        std::shared_ptr<DynamicRobot> robot,
        const float *path,
        std::uint64_t n_waypoints,
        const vamp::collision::Environment<float> &env,
        const vamp::planning::SimplifySettings &settings,
        DynamicSampler &sampler,
        const DynamicConstraintVec &constraints,
        const ConstraintSettings &cs,
        const ChartSettings &chs,
        const DynamicPhaseConstraintVec &phase) -> DynamicPlanResult
    {
        if (constraints.empty())
        {
            return simplify_phase(std::move(robot), path, n_waypoints, env, settings, sampler, phase);
        }

        const auto &fops = chart_ops_checked(*robot);
        auto handles = constraint_handles(*robot->ambient(), constraints);
        auto ph = phase_handles(*robot, phase);

        const auto dim = robot->dimension();
        for (std::uint64_t i = 0; i < n_waypoints; ++i)
        {
            if (fops.chart_satisfied(
                    path + i * dim,
                    handles.data(),
                    handles.size(),
                    static_cast<const void *>(&cs),
                    static_cast<const void *>(&chs),
                    ph.data(),
                    ph.size()) == 0)
            {
                throw std::invalid_argument(
                    "path state " + std::to_string(i) + manifold_simplify_message);
            }
        }

        auto *handle = fops.simplify_chart(
            path,
            n_waypoints,
            static_cast<const void *>(&env),
            static_cast<const void *>(&settings),
            sampler.handle,
            handles.data(),
            handles.size(),
            static_cast<const void *>(&cs),
            static_cast<const void *>(&chs),
            ph.data(),
            ph.size());

        return drain_handle(std::move(robot), handle);
    }

    inline auto
    debug(DynamicRobot &robot, const float *config, const vamp::collision::Environment<float> &env)
        -> DebugType
    {
        const auto &ops = robot.ops();
        auto *handle = ops.debug(config, static_cast<const void *>(&env));
        DebugType copy = *reinterpret_cast<DebugType *>(handle);
        ops.debug_destroy(handle);
        return copy;
    }

    inline auto eefk(DynamicRobot &robot, const float *config) -> Eigen::Matrix4f
    {
        Eigen::Matrix4f result;
        robot.ops().eefk(config, result.data());
        return result;
    }

    inline auto fk(DynamicRobot &robot, const float *config) -> std::vector<vamp::collision::Sphere<float>>
    {
        const auto n = robot.n_spheres();
        std::vector<float> buf(n * 4);
        robot.ops().fk(config, buf.data());

        std::vector<vamp::collision::Sphere<float>> out;
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            out.emplace_back(buf[i * 4 + 0], buf[i * 4 + 1], buf[i * 4 + 2], buf[i * 4 + 3]);
        }
        return out;
    }

    inline auto validate(
        DynamicRobot &robot,
        const float *config,
        const vamp::collision::Environment<float> &env,
        bool check_bounds) -> bool
    {
        return robot.ops().validate(config, static_cast<const void *>(&env), check_bounds ? 1 : 0) != 0;
    }

    inline auto validate_motion(
        DynamicRobot &robot,
        const float *c_in,
        const float *c_out,
        const vamp::collision::Environment<float> &env,
        bool check_bounds) -> bool
    {
        return robot.ops().validate_motion(
                   c_in, c_out, static_cast<const void *>(&env), check_bounds ? 1 : 0) != 0;
    }

    inline auto filter_self_from_pointcloud(
        DynamicRobot &robot,
        const float *points,
        std::uint64_t n_points,
        float point_radius,
        const float *config,
        const vamp::collision::Environment<float> &env) -> std::vector<vamp::collision::Point>
    {
        std::vector<vamp::collision::Point> out;
        robot.ops().filter_pointcloud(
            points, n_points, point_radius, config, static_cast<const void *>(&env), &out);
        return out;
    }
}  // namespace vamp::jit
