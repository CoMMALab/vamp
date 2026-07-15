#include <vamp_python_init.hh>

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/jit/api.hh>
#include <vamp/jit/build_paths.hh>
#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>

#include <vamp/collision/environment.hh>
#include <vamp/collision/math.hh>
#include <vamp/collision/shapes.hh>
#include <vamp/planning/planners/aorrtc_settings.hh>
#include <vamp/planning/planners/grrtstar_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/roadmap.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

#include <cricket/codegen.hh>
#include <cricket/jit/object_cache.hh>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <dlfcn.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace
{
    namespace vj = vamp::jit;
    namespace vp = vamp::planning;

    using ConfigNd = nb::ndarray<const float, nb::ndim<1>, nb::device::cpu>;
    using PathNd = nb::ndarray<const float, nb::ndim<2>, nb::device::cpu>;
}  // namespace

namespace vamp::binding
{
    struct VectorConfig
    {
        using Type = std::vector<float>;

        static auto as_ptr(const Type &v, std::size_t dim, std::vector<float> & /*scratch*/, const char *what)
            -> const float *
        {
            if (v.size() != dim)
            {
                throw std::runtime_error(std::string(what) + " has wrong dimension");
            }
            return v.data();
        }
    };

    struct NDArrayConfig
    {
        using Type = ConfigNd;

        static auto as_ptr(const Type &a, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> const float *
        {
            return as_flat_1d(a, dim, scratch, what);
        }
    };

    struct VectorPath
    {
        using Type = std::vector<std::vector<float>>;

        static auto as_ptr(const Type &v, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            scratch.clear();
            scratch.reserve(dim * v.size());
            for (const auto &wp : v)
            {
                if (wp.size() != dim)
                {
                    throw std::runtime_error(std::string(what) + " has wrong waypoint dimension");
                }
                scratch.insert(scratch.end(), wp.begin(), wp.end());
            }
            return {scratch.data(), v.size()};
        }
    };

    struct NDArrayPath
    {
        using Type = PathNd;

        static auto as_ptr(const Type &a, std::size_t dim, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, dim, scratch, what);
        }
    };

    struct VectorPointcloud
    {
        using Type = std::vector<vamp::collision::Point>;

        static auto as_ptr(const Type &v, std::vector<float> & /*scratch*/, const char * /*what*/)
            -> std::pair<const float *, std::uint64_t>
        {
            return {v.empty() ? nullptr : v.front().data(), v.size()};
        }
    };

    struct NDArrayPointcloud
    {
        using Type = PathNd;

        static auto as_ptr(const Type &a, std::vector<float> &scratch, const char *what)
            -> std::pair<const float *, std::uint64_t>
        {
            return as_flat_2d(a, 3, scratch, what);
        }
    };

    inline auto to_waypoint_raw(vj::DynamicPath &p, std::size_t sz, const float *src, bool allow_init)
        -> std::vector<float>
    {
        if (p.dim == 0 and allow_init)
        {
            p.dim = sz;
        }
        else if (sz != p.dim)
        {
            throw std::runtime_error("waypoint has wrong dimension");
        }
        return std::vector<float>(src, src + p.dim);
    }

    inline auto take_waypoint(vj::DynamicPath &p, const std::vector<float> &c, bool allow_init)
        -> std::vector<float>
    {
        return to_waypoint_raw(p, c.size(), c.data(), allow_init);
    }

    inline auto take_waypoint(vj::DynamicPath &p, const ConfigNd &c, bool allow_init) -> std::vector<float>
    {
        std::vector<float> scratch;
        const auto sz = c.shape(0);
        const auto *ptr = as_flat_1d(c, p.dim == 0 ? sz : p.dim, scratch, "waypoint");
        return to_waypoint_raw(p, sz, ptr, allow_init);
    }

    template <std::size_t N>
    inline auto flatten(const std::vector<std::array<float, N>> &v) -> std::vector<float>
    {
        std::vector<float> out;
        out.reserve(v.size() * N);
        for (const auto &e : v)
        {
            out.insert(out.end(), e.begin(), e.end());
        }
        return out;
    }

    inline void check_idx(const vj::DynamicPath &p, std::size_t i)
    {
        if (i >= p.waypoints.size())
        {
            throw nb::index_error();
        }
    }

    template <typename ConfigInput, typename PathInput, typename PcInput>
    struct DynamicRobotTraits
    {
        using Cfg = typename ConfigInput::Type;
        using Pth = typename PathInput::Type;
        using Pc = typename PcInput::Type;
        using Path = vj::DynamicPath;
        using PlanningResult = vj::DynamicPlanResult;
        using Sampler = vj::DynamicSampler;
        using Phs = vj::DynamicPhs;
        using Env = vamp::collision::Environment<float>;
        using PRMSettings = vp::RoadmapSettings<vp::PRMStarNeighborParams>;
        using FCITSettings = vp::RoadmapSettings<vp::FCITStarNeighborParams>;
        using PhaseConstraintVec = vj::DynamicPhaseConstraintVec;

        template <vp::Planner P, typename Settings>
        static auto solve_single(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            return vj::solve(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                ConfigInput::as_ptr(goal, d, gs, "goal"),
                env,
                settings,
                *sampler);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi(
                self, P, ConfigInput::as_ptr(start, d, ss, "start"), gptr, n, env, settings, *sampler);
        }

        static auto simplify(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            std::shared_ptr<Sampler> sampler) -> PlanningResult
        {
            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify(self, pptr, n, env, settings, *sampler);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_single_constrained(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings) -> PlanningResult
        {
            // On flask siblings, `constraints` selects chart-constrained planning over
            // ambient-robot constraints, matching the static flask submodule surface.
            if (self->ambient() != nullptr)
            {
                return solve_single_chart<P, Settings>(
                    self, start, goal, env, settings, sampler, constraints, constraint_settings, {}, {});
            }

            const auto d = self->dimension();
            std::vector<float> ss, gs;
            return vj::solve_constrained(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                ConfigInput::as_ptr(goal, d, gs, "goal"),
                env,
                settings,
                *sampler,
                constraints,
                constraint_settings);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi_constrained(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings) -> PlanningResult
        {
            if (self->ambient() != nullptr)
            {
                return solve_multi_chart<P, Settings>(
                    self, start, goals, env, settings, sampler, constraints, constraint_settings, {}, {});
            }

            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi_constrained(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                gptr,
                n,
                env,
                settings,
                *sampler,
                constraints,
                constraint_settings);
        }

        static auto simplify_constrained(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings) -> PlanningResult
        {
            if (self->ambient() != nullptr)
            {
                return simplify_chart(
                    self, path, env, settings, sampler, constraints, constraint_settings, {}, {});
            }

            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify_constrained(
                self, pptr, n, env, settings, *sampler, constraints, constraint_settings);
        }

        static auto constraint_satisfied(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &c,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings) -> bool
        {
            std::vector<float> scratch;
            const auto *ptr = ConfigInput::as_ptr(c, self->dimension(), scratch, "configuration");
            if (self->ambient() != nullptr)
            {
                return vj::chart_satisfied(self, ptr, constraints, constraint_settings, {});
            }

            return vj::constraint_satisfied(self, ptr, constraints, constraint_settings);
        }

        static auto constraint_project(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &c,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings)
        {
            std::vector<float> scratch;
            const auto *ptr = ConfigInput::as_ptr(c, self->dimension(), scratch, "configuration");
            auto out = (self->ambient() != nullptr) ?
                           vj::chart_project(
                               self, ptr, constraints, constraint_settings, vj::ChartSettings{}, {}) :
                           vj::constraint_project(self, ptr, constraints, constraint_settings);
            return make_ndarray<1>(out.data(), {out.size()});
        }

        template <vp::Planner P, typename Settings>
        static auto solve_single_phase(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            return vj::solve_phase(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                ConfigInput::as_ptr(goal, d, gs, "goal"),
                env,
                settings,
                *sampler,
                phase_constraints);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi_phase(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi_phase(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                gptr,
                n,
                env,
                settings,
                *sampler,
                phase_constraints);
        }

        static auto simplify_phase(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            std::shared_ptr<Sampler> sampler,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify_phase(self, pptr, n, env, settings, *sampler, phase_constraints);
        }

        static auto phase_satisfied(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &state,
            const PhaseConstraintVec &phase_constraints) -> bool
        {
            std::vector<float> scratch;
            return vj::phase_satisfied(
                self, ConfigInput::as_ptr(state, self->dimension(), scratch, "state"), phase_constraints);
        }

        static auto phase_velocity_scale(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &state,
            const PhaseConstraintVec &phase_constraints) -> float
        {
            std::vector<float> scratch;
            return vj::phase_velocity_scale(
                self, ConfigInput::as_ptr(state, self->dimension(), scratch, "state"), phase_constraints);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_single_chart(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            return vj::solve_chart(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                ConfigInput::as_ptr(goal, d, gs, "goal"),
                env,
                settings,
                *sampler,
                constraints,
                constraint_settings,
                chart_settings,
                phase_constraints);
        }

        template <vp::Planner P, typename Settings>
        static auto solve_multi_chart(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            const auto d = self->dimension();
            std::vector<float> ss, gs;
            auto [gptr, n] = PathInput::as_ptr(goals, d, gs, "goals");
            return vj::solve_multi_chart(
                self,
                P,
                ConfigInput::as_ptr(start, d, ss, "start"),
                gptr,
                n,
                env,
                settings,
                *sampler,
                constraints,
                constraint_settings,
                chart_settings,
                phase_constraints);
        }

        static auto simplify_chart(
            std::shared_ptr<vj::DynamicRobot> self,
            const Pth &path,
            const Env &env,
            const vp::SimplifySettings &settings,
            std::shared_ptr<Sampler> sampler,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings,
            const PhaseConstraintVec &phase_constraints) -> PlanningResult
        {
            std::vector<float> scratch;
            auto [pptr, n] = PathInput::as_ptr(path, self->dimension(), scratch, "path");
            return vj::simplify_chart(
                self,
                pptr,
                n,
                env,
                settings,
                *sampler,
                constraints,
                constraint_settings,
                chart_settings,
                phase_constraints);
        }

        static auto chart_project(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &state,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings,
            const PhaseConstraintVec &phase_constraints)
        {
            std::vector<float> scratch;
            auto out = vj::chart_project(
                self,
                ConfigInput::as_ptr(state, self->dimension(), scratch, "state"),
                constraints,
                constraint_settings,
                chart_settings,
                phase_constraints);
            return make_ndarray<1>(out.data(), {out.size()});
        }

        static auto chart_satisfied(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &state,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const PhaseConstraintVec &phase_constraints) -> bool
        {
            std::vector<float> scratch;
            return vj::chart_satisfied(
                self,
                ConfigInput::as_ptr(state, self->dimension(), scratch, "state"),
                constraints,
                constraint_settings,
                phase_constraints);
        }

        static auto debug_chart(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &state,
            const vj::DynamicConstraintVec &constraints,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings) -> std::vector<std::vector<float>>
        {
            std::vector<float> scratch;
            return vj::chart_debug(
                self,
                ConfigInput::as_ptr(state, self->dimension(), scratch, "state"),
                constraints,
                constraint_settings,
                chart_settings);
        }

        static auto lift_edge(
            std::shared_ptr<vj::DynamicRobot> self,
            const Cfg &from_state,
            const Cfg &target,
            const vj::DynamicConstraintVec &constraints,
            bool forward,
            std::size_t n_samples,
            const vj::ConstraintSettings &constraint_settings,
            const vj::ChartSettings &chart_settings)
            -> std::tuple<std::vector<std::vector<float>>, std::vector<float>, float, float>
        {
            const auto d = self->dimension();
            std::vector<float> sf, st;
            auto out = vj::chart_lift_edge(
                self,
                ConfigInput::as_ptr(from_state, d, sf, "from_state"),
                ConfigInput::as_ptr(target, d, st, "target"),
                constraints,
                forward,
                n_samples,
                constraint_settings,
                chart_settings);
            return {std::move(out.states), std::move(out.errors), out.duration, out.cost};
        }

        // The gating check precedes input validation so that ambient robots (whose
        // dimension differs from the sibling's flat states) raise the flask error, not
        // a misleading dimension mismatch.
        static auto flask_optimal_time(vj::DynamicRobot &self, const Cfg &a, const Cfg &b) -> float
        {
            vj::flask_ops_checked(self);
            const auto d = self.dimension();
            std::vector<float> sa, sb;
            return vj::flask_optimal_time(
                self, ConfigInput::as_ptr(a, d, sa, "a"), ConfigInput::as_ptr(b, d, sb, "b"));
        }

        static auto flask_cost(vj::DynamicRobot &self, const Cfg &a, const Cfg &b) -> float
        {
            vj::flask_ops_checked(self);
            const auto d = self.dimension();
            std::vector<float> sa, sb;
            return vj::flask_cost(
                self, ConfigInput::as_ptr(a, d, sa, "a"), ConfigInput::as_ptr(b, d, sb, "b"));
        }

        static auto flask_cost_grad(vj::DynamicRobot &self, const Cfg &a, const Cfg &b)
        {
            vj::flask_ops_checked(self);
            const auto d = self.dimension();
            std::vector<float> sa, sb;
            auto g = vj::flask_cost_grad(
                self, ConfigInput::as_ptr(a, d, sa, "a"), ConfigInput::as_ptr(b, d, sb, "b"));
            return std::make_tuple(g.cost, g.time, std::move(g.grad_a), std::move(g.grad_b));
        }

        static auto flask_eval(vj::DynamicRobot &self, const Cfg &a, const Cfg &b, float T, float t)
        {
            vj::flask_ops_checked(self);
            const auto d = self.dimension();
            std::vector<float> sa, sb;
            auto x = vj::flask_eval(
                self, ConfigInput::as_ptr(a, d, sa, "a"), ConfigInput::as_ptr(b, d, sb, "b"), T, t);
            const auto n_q = x.size() / 3;
            return std::make_tuple(
                std::vector<float>(x.begin(), x.begin() + n_q),
                std::vector<float>(x.begin() + n_q, x.begin() + 2 * n_q),
                std::vector<float>(x.begin() + 2 * n_q, x.end()));
        }

        static auto flask_torques(vj::DynamicRobot &self, const Cfg &x) -> std::vector<float>
        {
            const auto dx = 3 * static_cast<std::size_t>(vj::flask_flat_dimension(self));
            std::vector<float> scratch;
            return vj::flask_torques(self, ConfigInput::as_ptr(x, dx, scratch, "flat state"));
        }

        static auto flask_kinetic_energy(vj::DynamicRobot &self, const Cfg &state) -> float
        {
            vj::flask_ops_checked(self);
            std::vector<float> scratch;
            return vj::flask_kinetic_energy(
                self, ConfigInput::as_ptr(state, self.dimension(), scratch, "state"));
        }

        static auto flask_eef_velocity(vj::DynamicRobot &self, const Cfg &state) -> std::vector<float>
        {
            vj::flask_ops_checked(self);
            std::vector<float> scratch;
            return vj::flask_eef_velocity(
                self, ConfigInput::as_ptr(state, self.dimension(), scratch, "state"));
        }

        static auto fk(vj::DynamicRobot &self, const Cfg &c) -> std::vector<vamp::collision::Sphere<float>>
        {
            std::vector<float> scratch;
            return vj::fk(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"));
        }

        static auto eefk(vj::DynamicRobot &self, const Cfg &c) -> Eigen::Matrix4f
        {
            std::vector<float> scratch;
            return vj::eefk(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"));
        }

        static auto debug(vj::DynamicRobot &self, const Cfg &c, const Env &env) -> vj::DebugType
        {
            std::vector<float> scratch;
            return vj::debug(self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"), env);
        }

        static auto validate(vj::DynamicRobot &self, const Cfg &c, const Env &env, bool check_bounds) -> bool
        {
            std::vector<float> scratch;
            return vj::validate(
                self, ConfigInput::as_ptr(c, self.dimension(), scratch, "configuration"), env, check_bounds);
        }

        static auto validate_motion(
            vj::DynamicRobot &self,
            const Cfg &c_in,
            const Cfg &c_out,
            const Env &env,
            bool check_bounds) -> bool
        {
            std::vector<float> s_in, s_out;
            return vj::validate_motion(
                self,
                ConfigInput::as_ptr(c_in, self.dimension(), s_in, "configuration_in"),
                ConfigInput::as_ptr(c_out, self.dimension(), s_out, "configuration_out"),
                env,
                check_bounds);
        }

        static auto filter_self_from_pointcloud(
            vj::DynamicRobot &self,
            const Pc &pc,
            float point_radius,
            const Cfg &c,
            const Env &env) -> std::vector<vamp::collision::Point>
        {
            std::vector<float> cfg_scratch, pc_scratch;
            auto [pptr, n] = PcInput::as_ptr(pc, pc_scratch, "pc");
            return vj::filter_self_from_pointcloud(
                self,
                pptr,
                n,
                point_radius,
                ConfigInput::as_ptr(c, self.dimension(), cfg_scratch, "config"),
                env);
        }

        static auto make_halton(std::shared_ptr<vj::DynamicRobot> self) -> std::shared_ptr<Sampler>
        {
            return vj::make_halton_sampler(std::move(self));
        }

        static auto make_xorshift(std::shared_ptr<vj::DynamicRobot> self, std::uint64_t seed)
            -> std::shared_ptr<Sampler>
        {
            return vj::make_xorshift_sampler(std::move(self), seed);
        }

        static auto make_phs(std::shared_ptr<vj::DynamicRobot> self, const Cfg &focus_a, const Cfg &focus_b)
            -> std::shared_ptr<Phs>
        {
            const auto d = self->dimension();
            std::vector<float> sa, sb;
            return vj::make_phs(
                self,
                ConfigInput::as_ptr(focus_a, d, sa, "focus_a"),
                ConfigInput::as_ptr(focus_b, d, sb, "focus_b"));
        }

        static auto make_phs_sampler(
            std::shared_ptr<vj::DynamicRobot> self,
            const Phs &phs,
            std::shared_ptr<Sampler> inner) -> std::shared_ptr<Sampler>
        {
            return vj::make_phs_sampler(std::move(self), phs, *inner);
        }

        static auto path_get(const Path &p, std::size_t i)
        {
            check_idx(p, i);
            return make_ndarray<1>(p.waypoints[i].data(), {p.dim});
        }

        static auto path_validate(Path &p, const Env &e) -> bool
        {
            return p.validate(e);
        }

        static auto path_numpy(const Path &p)
        {
            const auto n = p.waypoints.size();
            return make_ndarray_filled<2>(
                {n, p.dim},
                [&](float *dst)
                {
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        std::memcpy(dst + i * p.dim, p.waypoints[i].data(), p.dim * sizeof(float));
                    }
                });
        }

        static void path_set(Path &p, std::size_t i, const Cfg &c)
        {
            check_idx(p, i);
            p.waypoints[i] = take_waypoint(p, c, false);
        }

        static void path_append(Path &p, const Cfg &c)
        {
            p.waypoints.emplace_back(take_waypoint(p, c, true));
        }

        static void path_insert(Path &p, std::size_t i, const Cfg &c)
        {
            p.waypoints.insert(p.waypoints.cbegin() + i, take_waypoint(p, c, true));
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.solved();
        }

        static void sampler_skip(Sampler &s, std::size_t n)
        {
            s.skip(n);
        }

        static auto sampler_next(Sampler &s)
        {
            const auto dim = s.robot->dimension();
            std::vector<float> out(dim);
            s.next(out.data());
            return make_ndarray<1>(out.data(), {dim});
        }

        static void phs_set_transverse_diameter(Phs &p, float d)
        {
            p.set_transverse_diameter(d);
        }

        static auto phs_transform(Phs &phs, const Cfg &x) -> nb::ndarray<nb::numpy, float, nb::device::cpu>
        {
            const auto dim = phs.robot->dimension();
            std::vector<float> scratch;
            const auto *xptr = ConfigInput::as_ptr(x, dim, scratch, "x");
            std::vector<float> out(dim);
            phs.transform(xptr, out.data());
            return make_ndarray<1>(out.data(), {dim});
        }
    };

    using DTV = DynamicRobotTraits<VectorConfig, VectorPath, VectorPointcloud>;
    using DTN = DynamicRobotTraits<NDArrayConfig, NDArrayPath, NDArrayPointcloud>;

    template <typename Traits, typename Klass>
    void bind_dynamic_flask_kernels(Klass &klass)
    {
        klass.def(
            "optimal_time",
            &Traits::flask_optimal_time,
            "a"_a,
            "b"_a,
            "Optimal duration T* of the local trajectory between flat states a and b.");
        klass.def(
            "cost",
            &Traits::flask_cost,
            "a"_a,
            "b"_a,
            "LQMT edge cost C_loc(a -> b) = rho T* + integral |u|^2 of the local trajectory; "
            "asymmetric in (a, b).");
        klass.def(
            "cost_grad",
            &Traits::flask_cost_grad,
            "a"_a,
            "b"_a,
            "(cost, time, dC/da, dC/db) for the LQMT edge a -> b; gradients are laid out to "
            "match the Configuration = (q, v) memory ordering.");
        klass.def(
            "eval",
            &Traits::flask_eval,
            "a"_a,
            "b"_a,
            "T"_a,
            "t"_a,
            "Evaluate the trajectory from a to b of duration T at fraction t in [0, 1]; "
            "returns (position, velocity, acceleration).");
        klass.def(
            "kinetic_energy",
            &Traits::flask_kinetic_energy,
            "state"_a,
            "Total kinetic energy (1/2) qdot^T M(q) qdot of a flat state (q, qdot).");
        klass.def(
            "eef_velocity",
            &Traits::flask_eef_velocity,
            "state"_a,
            "Linear workspace velocity (x, y, z per end effector) of a flat state (q, qdot).");
        klass.def(
            "torques",
            &Traits::flask_torques,
            "x"_a,
            "Joint torques for a flat state stack x = [q; qd; qdd].");
    }

    void init_dynamic(nb::module_ &pymodule)
    {
        // Re-dlopen _core_ext.so with RTLD_GLOBAL so template instantiations
        // become visible to the JIT's process-symbol search — Python loads
        // extension modules with RTLD_LOCAL by default.
        Dl_info info{};
        if (dladdr(reinterpret_cast<void *>(&init_dynamic), &info) == 0 or info.dli_fname == nullptr)
        {
            throw std::runtime_error("Failed to promote RTLD to global.");
        }
        dlopen(info.dli_fname, RTLD_NOW | RTLD_GLOBAL | RTLD_NOLOAD);

        auto path_k = bind_path_class<DTV>(pymodule, "DynamicPath");
        bind_path_io<DTV>(path_k);
        bind_path_io<DTN>(path_k);

        bind_planning_result<DTV>(pymodule, "DynamicPlanResult");
        bind_sampler<DTV>(pymodule, "DynamicSampler");

        auto phs_k = bind_phs_class<DTV>(pymodule, "DynamicPhs");
        bind_phs_io<DTV>(phs_k);
        bind_phs_io<DTN>(phs_k);

        nb::class_<vj::DynamicConstraint>(
            pymodule,
            "DynamicConstraint",
            "Manifold constraint over robot configurations. Constraints cache per-evaluation "
            "state and are not thread-safe: do not share one instance across concurrent "
            "planning calls.");

        nb::class_<vj::DynamicPhaseConstraint>(
            pymodule,
            "DynamicPhaseConstraint",
            "Phase-space inequality constraint g(q, qdot) <= 0 over flat states. Constraints "
            "cache per-evaluation state and are not thread-safe: do not share one instance "
            "across concurrent planning calls.");

        auto klass =
            nb::class_<vj::DynamicRobot>(pymodule, "DynamicRobot")
                .def_prop_ro("dimension", &vj::DynamicRobot::dimension)
                .def_prop_ro("rake", &vj::DynamicRobot::rake)
                .def_prop_ro("n_spheres", &vj::DynamicRobot::n_spheres)
                .def_prop_ro("space_measure", &vj::DynamicRobot::space_measure, "Measure of robot's C-space.")
                .def_prop_ro(
                    "joint_names",
                    &vj::DynamicRobot::joint_names,
                    "Joint names for the robot in order of DoF.")
                .def(
                    "min_max_radii",
                    [](const vj::DynamicRobot &self) -> std::pair<float, float>
                    { return {self.min_radius(), self.max_radius()}; },
                    "Minimum and maximum radii sizes of robot spheres.")
                .def(
                    "upper_bounds",
                    [](const vj::DynamicRobot &self)
                    {
                        const auto &b = self.upper_bounds();
                        return make_ndarray<1>(b.data(), {b.size()});
                    })
                .def(
                    "lower_bounds",
                    [](const vj::DynamicRobot &self)
                    {
                        const auto &b = self.lower_bounds();
                        return make_ndarray<1>(b.data(), {b.size()});
                    });

        bind_robot_methods<DTV>(klass);
        bind_robot_methods<DTN>(klass);
        bind_constraint_methods<DTV>(klass);
        bind_constraint_methods<DTN>(klass);

        // Registration order matters for shared overload names: the constrained entries
        // dispatch to the chart path on flask siblings, and chart-specific kwargs reach
        // the later chart overloads via nanobind kwarg-name fallthrough.
        bind_phase_methods<DTV>(klass);
        bind_phase_methods<DTN>(klass);
        bind_chart_methods<DTV>(klass);
        bind_chart_methods<DTN>(klass);

        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               std::shared_ptr<vj::DynamicSampler> sampler)
            { return DTV::simplify(self, path.waypoints, env, settings, sampler); },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "Path simplification.");

        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               std::shared_ptr<vj::DynamicSampler> sampler,
               const vj::DynamicConstraintVec &constraints,
               const vj::ConstraintSettings &constraint_settings)
            {
                return DTV::simplify_constrained(
                    self, path.waypoints, env, settings, sampler, constraints, constraint_settings);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "constraints"_a,
            "constraint_settings"_a = vj::ConstraintSettings{},
            "Simplification heuristics restricted to the constraint manifold. Raises ValueError "
            "if any path state violates the constraints.");

        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               std::shared_ptr<vj::DynamicSampler> sampler,
               const vj::DynamicPhaseConstraintVec &phase_constraints)
            {
                return DTV::simplify_phase(self, path.waypoints, env, settings, sampler, phase_constraints);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "phase_constraints"_a,
            "Simplification heuristics with every validated motion checked against the phase "
            "constraints. Raises ValueError if any path state violates them.");

        klass.def(
            "simplify",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const vj::DynamicPath &path,
               const vamp::collision::Environment<float> &env,
               const vp::SimplifySettings &settings,
               std::shared_ptr<vj::DynamicSampler> sampler,
               const vj::DynamicConstraintVec &constraints,
               const vj::ConstraintSettings &constraint_settings,
               const vj::ChartSettings &chart_settings,
               const vj::DynamicPhaseConstraintVec &phase_constraints)
            {
                return DTV::simplify_chart(
                    self,
                    path.waypoints,
                    env,
                    settings,
                    sampler,
                    constraints,
                    constraint_settings,
                    chart_settings,
                    phase_constraints);
            },
            "path"_a,
            "environment"_a,
            "settings"_a,
            "sampler"_a,
            "constraints"_a,
            "constraint_settings"_a = vj::ConstraintSettings{},
            "chart_settings"_a = vj::ChartSettings{},
            "phase_constraints"_a = vj::DynamicPhaseConstraintVec{},
            "Simplification heuristics restricted to the constraint manifold. Raises ValueError "
            "if any path state violates the constraints.");

        klass.def(
            "n_eef",
            [](const vj::DynamicRobot &self)
            {
                if (not self.capabilities().constraints)
                {
                    throw std::runtime_error(
                        "this robot was generated without task-space constraint kernels; pass "
                        "constraints=True to load_robot");
                }
                return self.capabilities().n_eef;
            },
            "Number of end-effectors with task-space constraint kernels.");

        klass.def(
            "n_closed_loops",
            [](const vj::DynamicRobot &self)
            {
                if (not self.capabilities().closed_loops)
                {
                    throw std::runtime_error(
                        "this robot was generated without closed-loop constraint kernels; pass "
                        "closed_loops=[...] to load_robot");
                }
                return self.capabilities().n_closed_loops;
            },
            "Number of cut kinematic loops with closed-loop constraint kernels.");

        klass.def(
            "TaskSpaceConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::vector<std::array<float, 7>> &eef_to_offset,
               const std::vector<std::array<float, 7>> &world_to_reference,
               const std::vector<std::array<float, 6>> &lower,
               const std::vector<std::array<float, 6>> &upper)
            {
                const auto n = self->capabilities().n_eef;
                if (eef_to_offset.size() != n or world_to_reference.size() != n or lower.size() != n or
                    upper.size() != n)
                {
                    throw std::invalid_argument(
                        "expected one transform and one bound per end-effector (n_eef = " +
                        std::to_string(n) + ")");
                }

                auto rte = flatten<7>(eef_to_offset);
                auto wtr = flatten<7>(world_to_reference);
                auto lo = flatten<6>(lower);
                auto hi = flatten<6>(upper);
                return vj::make_task_space_constraint(
                    std::move(self), rte.data(), wtr.data(), lo.data(), hi.data());
            },
            "eef_to_offset"_a,
            "world_to_reference"_a,
            "lower"_a,
            "upper"_a,
            "Task Space Region constraint: for each end-effector, the pose of an "
            "offset frame (eef_to_offset, in the end-effector frame) must lie within "
            "[lower, upper] se(3) bounds of a reference frame (world_to_reference, in "
            "the world frame). Transforms are (qw, qx, qy, qz, x, y, z).");

        klass.def(
            "TaskSpaceConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::array<float, 7> &eef_to_offset,
               const std::array<float, 7> &world_to_reference,
               const std::array<float, 6> &lower,
               const std::array<float, 6> &upper)
            {
                if (self->capabilities().n_eef != 1)
                {
                    throw std::invalid_argument(
                        "unwrapped transforms and bounds are only valid for single "
                        "end-effector robots; pass per-end-effector lists");
                }

                return vj::make_task_space_constraint(
                    std::move(self),
                    eef_to_offset.data(),
                    world_to_reference.data(),
                    lower.data(),
                    upper.data());
            },
            "eef_to_offset"_a,
            "world_to_reference"_a,
            "lower"_a,
            "upper"_a,
            "Single end-effector convenience constructor: unwrapped transforms and bounds.");

        klass.def(
            "BimanualTaskSpaceConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::array<float, 7> &right_in_left,
               const std::array<float, 6> &lower,
               const std::array<float, 6> &upper)
            {
                return vj::make_bimanual_task_space_constraint(
                    std::move(self), right_in_left.data(), lower.data(), upper.data());
            },
            "right_in_left"_a,
            "lower"_a,
            "upper"_a,
            "Relative pose constraint between two end-effectors: the pose of "
            "end-effector 1 in the frame of end-effector 0 must lie within "
            "[lower, upper] se(3) bounds of right_in_left. Transforms are "
            "(qw, qx, qy, qz, x, y, z).");

        klass.def(
            "ClosedLoopConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self)
            { return vj::make_closed_loop_constraint(std::move(self)); },
            "Loop-closure constraint: each cut kinematic loop of the robot contributes "
            "one equality row keeping the distance between its cut frames at the "
            "loop's fixed length.");

        klass.def(
            "CoMConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self, const std::vector<std::array<float, 2>> &polygon)
            {
                auto flat = flatten<2>(polygon);
                return vj::make_com_constraint(std::move(self), flat.data(), polygon.size());
            },
            "polygon"_a,
            "Support-polygon constraint on the center of mass: the xy projection of "
            "the CoM (in the robot's CoM reference frame) must lie inside the convex "
            "polygon given by its vertices in counterclockwise order.");

        klass.def(
            "TwistConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::array<float, 7> &eef_to_offset,
               const std::array<float, 7> &world_to_reference,
               const std::array<float, 6> &reference_coefficients,
               const std::array<float, 6> &body_coefficients)
            {
                return vj::make_twist_constraint(
                    std::move(self),
                    eef_to_offset.data(),
                    world_to_reference.data(),
                    reference_coefficients.data(),
                    body_coefficients.data());
            },
            "eef_to_offset"_a,
            "world_to_reference"_a,
            "reference_coefficients"_a,
            "body_coefficients"_a,
            "Constant-coefficient Pfaffian velocity constraint over the end-effector "
            "twist: reference_coefficients . twist_ref + body_coefficients . "
            "twist_loc = 0, where twist_ref and twist_loc are the [linear; angular] "
            "twist of the offset end-effector frame (eef_to_offset, in the "
            "end-effector frame) expressed in the reference frame's axes "
            "(world_to_reference, in the world frame) and in the frame's own body "
            "axes. Transforms are (qw, qx, qy, qz, x, y, z). Contributes no position "
            "error, and is smooth for unbounded rotation.");

        klass.def(
            "LeadScrewConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::array<float, 7> &eef_to_offset,
               const std::array<float, 7> &world_to_reference,
               float pitch)
            {
                return vj::make_lead_screw_constraint(
                    std::move(self), eef_to_offset.data(), world_to_reference.data(), pitch);
            },
            "eef_to_offset"_a,
            "world_to_reference"_a,
            "pitch"_a,
            "Pfaffian lead-screw coupling: velocities of an offset end-effector frame "
            "(eef_to_offset, in the end-effector frame) are restricted so translation "
            "along the reference frame's z-axis (world_to_reference, in the world "
            "frame) advances by pitch per full turn about it. Transforms are "
            "(qw, qx, qy, qz, x, y, z). Contributes no position error.");

        klass.def(
            "LeadScrewLevelConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self,
               const std::array<float, 7> &eef_to_offset,
               const std::array<float, 7> &world_to_reference,
               float pitch,
               float target)
            {
                return vj::make_lead_screw_level_constraint(
                    std::move(self), eef_to_offset.data(), world_to_reference.data(), pitch, target);
            },
            "eef_to_offset"_a,
            "world_to_reference"_a,
            "pitch"_a,
            "target"_a,
            "Holonomic form of the lead-screw coupling: pins the screw invariant "
            "h(q) = z-advance - (pitch / 2 pi) * rotation of the offset end-effector "
            "frame in the reference frame to a target level. Transforms are "
            "(qw, qx, qy, qz, x, y, z).");

        klass.def_prop_ro(
            "flask",
            [](const vj::DynamicRobot &self) { return self.flask_sibling(); },
            "The flask (flat-system) sibling robot generated from the recipe's flask block; "
            "None when the robot was loaded without one.");

        bind_dynamic_flask_kernels<DTV>(klass);
        bind_dynamic_flask_kernels<DTN>(klass);

        klass.def(
            "flat_dimension",
            [](vj::DynamicRobot &self) { return vj::flask_flat_dimension(self); });
        klass.def("rho", [](vj::DynamicRobot &self) { return vj::flask_rho(self); });
        klass.def(
            "set_rho",
            [](vj::DynamicRobot &self, float rho) { vj::flask_set_rho(self, rho); },
            "rho"_a);
        klass.def(
            "velocity_limits",
            [](vj::DynamicRobot &self) { return vj::flask_velocity_limits(self); });
        klass.def(
            "effort_limits", [](vj::DynamicRobot &self) { return vj::flask_effort_limits(self); });
        klass.def(
            "n_end_effectors",
            [](vj::DynamicRobot &self) { return vj::flask_n_end_effectors(self); });

        klass.def(
            "ke_shaped",
            [](std::shared_ptr<vj::DynamicRobot> self,
               std::shared_ptr<vj::DynamicSampler> sampler,
               float max_energy)
            { return vj::make_ke_shaped_sampler(std::move(self), *sampler, max_energy); },
            "sampler"_a,
            "max_energy"_a,
            "Wrap a sampler so sample kinetic energy is uniform on [0, max_energy] instead of "
            "concentrated near the velocity-box maximum; rescales each sample's velocity block "
            "by the scalar kinetic-energy kernel.");

        klass.def(
            "KineticEnergyConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self, float max_energy)
            { return vj::make_kinetic_energy_constraint(std::move(self), max_energy); },
            "max_energy"_a,
            "Bounds the robot's total kinetic energy: (1/2) qdot^T M(q) qdot <= max_energy.");

        klass.def(
            "EEFSpeedConstraint",
            [](std::shared_ptr<vj::DynamicRobot> self, float max_speed)
            { return vj::make_eef_speed_constraint(std::move(self), max_speed); },
            "max_speed"_a,
            "Bounds the workspace speed of every end-effector origin: ||v_eef|| <= max_speed "
            "(linear velocity only).");

        pymodule.def(
            "load_robot",
            [](const std::string &urdf,
               const std::optional<std::string> &srdf,
               nb::object end_effector,
               const std::vector<std::string> &planners,
               std::size_t rake,
               std::size_t resolution,
               const std::string &name,
               const std::optional<std::array<double, 3>> &bounds_lower,
               const std::optional<std::array<double, 3>> &bounds_upper,
               bool compact_collisions,
               bool constraints,
               nb::object com,
               nb::object closed_loops,
               bool lead_screw,
               bool twist,
               nb::object flask) -> std::shared_ptr<vj::DynamicRobot>
            {
                cricket::GenOptions g;
                g.urdf = urdf;
                if (srdf)
                {
                    g.srdf = std::filesystem::path(*srdf);
                }

                if (not end_effector.is_none())
                {
                    if (nb::isinstance<nb::str>(end_effector))
                    {
                        auto s = nb::cast<std::string>(end_effector);
                        if (not s.empty())
                        {
                            g.end_effectors.push_back(std::move(s));
                        }
                    }
                    else
                    {
                        g.end_effectors = nb::cast<std::vector<std::string>>(end_effector);
                    }
                }

                if (bounds_lower and bounds_upper)
                {
                    cricket::Bounds b;
                    b.lower = Eigen::Vector3d(
                        (*bounds_lower)[0], (*bounds_lower)[1], (*bounds_lower)[2]);
                    b.upper = Eigen::Vector3d(
                        (*bounds_upper)[0], (*bounds_upper)[1], (*bounds_upper)[2]);
                    g.bounds = b;
                }

                g.data = {
                    {"name", name},
                    {"resolution", resolution},
                    {"compact_collisions", compact_collisions},
                    {"constraints", constraints},
                    {"lead_screw", lead_screw},
                    {"twist", twist},
                };

                if (not com.is_none())
                {
                    if (nb::isinstance<nb::bool_>(com))
                    {
                        g.data["com"] = nb::cast<bool>(com);
                    }
                    else if (nb::isinstance<nb::dict>(com))
                    {
                        auto d = nb::cast<nb::dict>(com);
                        nlohmann::json j = nlohmann::json::object();
                        if (d.contains("reference_frames"))
                        {
                            j["reference_frames"] =
                                nb::cast<std::vector<std::string>>(d["reference_frames"]);
                        }
                        g.data["com"] = std::move(j);
                    }
                    else
                    {
                        g.data["com"] = nlohmann::json{
                            {"reference_frames", nb::cast<std::vector<std::string>>(com)}};
                    }
                }

                // Presence of the key is what enables loop-closure codegen, so only set
                // it for a non-empty loop list.
                if (not closed_loops.is_none())
                {
                    nlohmann::json loops = nlohmann::json::array();
                    for (nb::handle item : nb::cast<nb::iterable>(closed_loops))
                    {
                        nlohmann::json loop;
                        if (nb::isinstance<nb::dict>(item))
                        {
                            auto d = nb::cast<nb::dict>(item);
                            loop["start_frame"] = nb::cast<std::string>(d["start_frame"]);
                            loop["end_frame"] = nb::cast<std::string>(d["end_frame"]);
                            loop["length"] = nb::cast<double>(d["length"]);
                        }
                        else
                        {
                            auto [sf, ef, len] =
                                nb::cast<std::tuple<std::string, std::string, double>>(item);
                            loop["start_frame"] = std::move(sf);
                            loop["end_frame"] = std::move(ef);
                            loop["length"] = len;
                        }
                        loops.push_back(std::move(loop));
                    }

                    if (not loops.empty())
                    {
                        g.data["closed_loops"] = std::move(loops);
                    }
                }

                if (not flask.is_none())
                {
                    auto d = nb::cast<nb::dict>(flask);
                    nlohmann::json j = nlohmann::json::object();
                    if (d.contains("rho"))
                    {
                        j["rho"] = nb::cast<double>(d["rho"]);
                    }
                    if (d.contains("resolution"))
                    {
                        j["resolution"] = nb::cast<std::size_t>(d["resolution"]);
                    }
                    if (d.contains("velocity_limits"))
                    {
                        j["velocity_limits"] = nb::cast<std::vector<double>>(d["velocity_limits"]);
                    }
                    if (d.contains("effort_limits"))
                    {
                        j["effort_limits"] = nb::cast<std::vector<double>>(d["effort_limits"]);
                    }
                    g.data["flask"] = std::move(j);
                }

                auto gen = cricket::generate_robot_source(g);

                vj::LoadOptions opts = vj::default_load_options();
                opts.robot_source = gen.source;
                opts.robot_name = gen.robot_name.empty() ? name : gen.robot_name;
                opts.dimension = gen.dimension;
                opts.rake = rake;
                opts.resolution = resolution;
                for (const auto &p : planners)
                {
                    opts.planners.push_back(vp::planner_from_name(p));
                }

                opts.capabilities.constraints = gen.data.value("has_constraints", false);
                opts.capabilities.com = gen.data.value("has_com", false);
                opts.capabilities.closed_loops = gen.data.value("has_closed_loops", false);
                opts.capabilities.lead_screw = gen.data.value("has_lead_screw", false);
                opts.capabilities.twist = gen.data.value("has_twist", false);
                opts.capabilities.n_eef = gen.data.value("num_end_effectors", std::size_t{0});
                opts.capabilities.n_closed_loops = gen.data.value("num_closed_loops", std::size_t{0});
                opts.capabilities.flask = gen.data.value("has_flask", false);
                opts.capabilities.flask_resolution = gen.data.value("flask_resolution", resolution);

                static const auto cache =
                    std::make_shared<cricket::jit::DiskObjectCache>(cricket::jit::default_cache_dir());
                return std::make_shared<vj::DynamicRobot>(opts, cache);
            },
            "urdf"_a,
            "srdf"_a = nb::none(),
            "end_effector"_a = nb::none(),
            "planners"_a = std::vector<std::string>{"rrtc"},
            "rake"_a = 8,
            "resolution"_a = 32,
            "name"_a = std::string("DynamicRobot"),
            "bounds_lower"_a = nb::none(),
            "bounds_upper"_a = nb::none(),
            "compact_collisions"_a = false,
            "constraints"_a = false,
            "com"_a = nb::none(),
            "closed_loops"_a = nb::none(),
            "lead_screw"_a = false,
            "twist"_a = false,
            "flask"_a = nb::none(),
            "JIT-compile a robot from a URDF. end_effector accepts a frame name or a list of "
            "frame names; constraints/com/closed_loops/lead_screw/twist mirror the cricket "
            "recipe keys and select which constraint kernels are generated. flask takes a dict "
            "mirroring the recipe's flask block (rho required; resolution, velocity_limits, "
            "effort_limits optional) and generates a flat-system sibling robot on .flask.");
    }
}  // namespace vamp::binding