#include <vamp/collision/sphere_sphere.hh>
#include <vamp/planning/phs.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/planning/validate.hh>
{% if has_any_constraint %}
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/settings.hh>
{% endif %}
{% if has_constraints %}
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
{% endif %}
{% if has_constraints and num_end_effectors > 1 %}
#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
{% endif %}
{% if has_closed_loops %}
#include <vamp/planning/constraints/manifold/closed_loop_constraint.hh>
{% endif %}
{% if has_com %}
#include <vamp/planning/constraints/manifold/com_constraint.hh>
{% endif %}
{% if has_twist %}
#include <vamp/planning/constraints/manifold/twist_constraint.hh>
{% endif %}
{% if has_lead_screw %}
#include <vamp/planning/constraints/manifold/lead_screw_constraint.hh>
{% endif %}

// clang-format off
#define VAMP_JIT_ROBOT_TYPE                 {{robot_type}}
#define VAMP_JIT_RAKE                       {{rake}}
#define VAMP_JIT_RESOLUTION                 {{resolution}}

#define VAMP_JIT_FN_RESULT_META             vamp_jit_{{sym}}_result_meta
#define VAMP_JIT_FN_RESULT_COPY_WAYPOINT    vamp_jit_{{sym}}_result_copy_waypoint
#define VAMP_JIT_FN_RESULT_DESTROY          vamp_jit_{{sym}}_result_destroy
#define VAMP_JIT_FN_RESULT_SIZES            vamp_jit_{{sym}}_result_sizes

#define VAMP_JIT_FN_SAMPLER_HALTON          vamp_jit_{{sym}}_sampler_halton
#define VAMP_JIT_FN_SAMPLER_XORSHIFT        vamp_jit_{{sym}}_sampler_xorshift
#define VAMP_JIT_FN_SAMPLER_RESET           vamp_jit_{{sym}}_sampler_reset
#define VAMP_JIT_FN_SAMPLER_SKIP            vamp_jit_{{sym}}_sampler_skip
#define VAMP_JIT_FN_SAMPLER_NEXT            vamp_jit_{{sym}}_sampler_next
#define VAMP_JIT_FN_SAMPLER_DESTROY         vamp_jit_{{sym}}_sampler_destroy

#define VAMP_JIT_FN_SIMPLIFY                vamp_jit_{{sym}}_simplify

#define VAMP_JIT_FN_DEBUG                   vamp_jit_{{sym}}_debug
#define VAMP_JIT_FN_DEBUG_DESTROY           vamp_jit_{{sym}}_debug_destroy
#define VAMP_JIT_FN_EEFK                    vamp_jit_{{sym}}_eefk
#define VAMP_JIT_FN_FK                      vamp_jit_{{sym}}_fk
#define VAMP_JIT_FN_VALIDATE                vamp_jit_{{sym}}_validate
#define VAMP_JIT_FN_VALIDATE_MOTION         vamp_jit_{{sym}}_validate_motion
#define VAMP_JIT_FN_CFG_DISTANCE            vamp_jit_{{sym}}_cfg_distance
#define VAMP_JIT_FN_CFG_INTERPOLATE         vamp_jit_{{sym}}_cfg_interpolate
#define VAMP_JIT_FN_FILTER_PC               vamp_jit_{{sym}}_filter_self_from_pointcloud
#define VAMP_JIT_FN_SPACE_MEASURE           vamp_jit_{{sym}}_space_measure
#define VAMP_JIT_FN_MIN_MAX_RADII           vamp_jit_{{sym}}_min_max_radii
#define VAMP_JIT_FN_N_SPHERES               vamp_jit_{{sym}}_n_spheres
#define VAMP_JIT_FN_JOINT_NAMES             vamp_jit_{{sym}}_joint_names
#define VAMP_JIT_FN_UPPER_BOUNDS            vamp_jit_{{sym}}_upper_bounds
#define VAMP_JIT_FN_LOWER_BOUNDS            vamp_jit_{{sym}}_lower_bounds

#define VAMP_JIT_FN_PHS_NEW                 vamp_jit_{{sym}}_phs_new
#define VAMP_JIT_FN_PHS_DESTROY             vamp_jit_{{sym}}_phs_destroy
#define VAMP_JIT_FN_PHS_SET_DIAMETER        vamp_jit_{{sym}}_phs_set_transverse_diameter
#define VAMP_JIT_FN_PHS_TRANSFORM           vamp_jit_{{sym}}_phs_transform
#define VAMP_JIT_FN_SAMPLER_PHS             vamp_jit_{{sym}}_sampler_phs
// clang-format on

namespace {{robot_ns}}
{
    using R = VAMP_JIT_ROBOT_TYPE;
    using SamplerPtr = typename vamp::rng::RNG<R>::Ptr;

    struct WrappedResult
    {
        vamp::planning::PlanningResult<R> inner;
    };

    inline auto load_config(const float *data) -> typename R::Configuration
    {
        typename R::ConfigurationArray arr;
        std::memcpy(arr.data(), data, R::dimension * sizeof(float));
        return typename R::Configuration(arr);
    }

    inline auto load_block_1(const float *data) -> typename R::template ConfigurationBlock<1>
    {
        typename R::template ConfigurationBlock<1> out;
        for (std::size_t i = 0; i < R::dimension; ++i)
        {
            out[i] = data[i];
        }
        return out;
    }

    inline auto raked_env(const void *env_ptr)
    {
        return vamp::collision::Environment<vamp::FloatVector<VAMP_JIT_RAKE>>(
            *static_cast<const vamp::collision::Environment<float> *>(env_ptr));
    }

    inline auto deref_sampler(vamp::jit::ffi::SamplerHandle *h) -> SamplerPtr &
    {
        return *reinterpret_cast<SamplerPtr *>(h);
    }
}  // namespace {{robot_ns}}

extern "C" vamp::jit::ffi::PlanResultMeta VAMP_JIT_FN_RESULT_META(const vamp::jit::ffi::PlanResultHandle *h)
{
    const auto *w = reinterpret_cast<const {{robot_ns}}::WrappedResult *>(h);
    vamp::jit::ffi::PlanResultMeta m{};
    m.success = w->inner.solved ? 1 : 0;
    m.dimension = {{robot_ns}}::R::dimension;
    m.waypoints = w->inner.path.size();
    m.nanoseconds = w->inner.nanoseconds;
    m.iterations = w->inner.iterations;
    m.cost = w->inner.path.cost();
    return m;
}

extern "C" void
VAMP_JIT_FN_RESULT_COPY_WAYPOINT(const vamp::jit::ffi::PlanResultHandle *h, std::uint64_t idx, float *out)
{
    const auto *w = reinterpret_cast<const {{robot_ns}}::WrappedResult *>(h);
    auto arr = w->inner.path[idx].to_array();
    std::memcpy(out, arr.data(), {{robot_ns}}::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_RESULT_DESTROY(vamp::jit::ffi::PlanResultHandle *h)
{
    delete reinterpret_cast<{{robot_ns}}::WrappedResult *>(h);
}

extern "C" void VAMP_JIT_FN_RESULT_SIZES(const vamp::jit::ffi::PlanResultHandle *h, void *out_sizes_vec)
{
    const auto *w = reinterpret_cast<const {{robot_ns}}::WrappedResult *>(h);
    auto *out = static_cast<std::vector<std::size_t> *>(out_sizes_vec);
    *out = w->inner.size;
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_FN_SAMPLER_HALTON()
{
    auto *p = new {{robot_ns}}::SamplerPtr(std::make_shared<vamp::rng::Halton<{{robot_ns}}::R>>());
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
}

extern "C" vamp::jit::ffi::SamplerHandle *VAMP_JIT_FN_SAMPLER_XORSHIFT(std::uint64_t seed)
{
#if defined(__x86_64__)
    auto *p = (seed == 0) ?
                  new {{robot_ns}}::SamplerPtr(std::make_shared<vamp::rng::XORShift<{{robot_ns}}::R>>()) :
                  new {{robot_ns}}::SamplerPtr(
                      std::make_shared<vamp::rng::XORShift<{{robot_ns}}::R>>(seed, seed + 1));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(p);
#else
    throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
}

extern "C" void VAMP_JIT_FN_SAMPLER_RESET(vamp::jit::ffi::SamplerHandle *h)
{
    {{robot_ns}}::deref_sampler(h)->reset();
}

extern "C" void VAMP_JIT_FN_SAMPLER_SKIP(vamp::jit::ffi::SamplerHandle *h, std::uint64_t n)
{
    auto &rng = {{robot_ns}}::deref_sampler(h);
    for (std::uint64_t i = 0; i < n; ++i)
    {
        rng->next();
    }
}

extern "C" void VAMP_JIT_FN_SAMPLER_NEXT(vamp::jit::ffi::SamplerHandle *h, float *out)
{
    auto arr = {{robot_ns}}::deref_sampler(h)->next().to_array();
    std::memcpy(out, arr.data(), {{robot_ns}}::R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_SAMPLER_DESTROY(vamp::jit::ffi::SamplerHandle *h)
{
    delete reinterpret_cast<{{robot_ns}}::SamplerPtr *>(h);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SIMPLIFY(
    const float *path_ptr,
    std::uint64_t n_waypoints,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using R = {{robot_ns}}::R;
    vamp::planning::Path<R> path;
    path.reserve(n_waypoints);
    for (std::uint64_t i = 0; i < n_waypoints; ++i)
    {
        path.emplace_back({{robot_ns}}::load_config(path_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        vamp::planning::simplify<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>(path, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::DebugHandle *VAMP_JIT_FN_DEBUG(const float *config, const void *env_ptr)
{
    using R = {{robot_ns}}::R;
    typename R::template ConfigurationBlock<VAMP_JIT_RAKE> block;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        block[i] = config[i];
    }
    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    auto *result = new typename R::Debug(R::template fkcc_debug<VAMP_JIT_RAKE>(env_rake, block));
    return reinterpret_cast<vamp::jit::ffi::DebugHandle *>(result);
}

extern "C" void VAMP_JIT_FN_DEBUG_DESTROY(vamp::jit::ffi::DebugHandle *h)
{
    using DebugType =
        std::pair<std::vector<std::vector<std::string>>, std::vector<std::pair<std::size_t, std::size_t>>>;
    delete reinterpret_cast<DebugType *>(h);
}

extern "C" void VAMP_JIT_FN_EEFK(const float *config, float *out_matrix)
{
    using R = {{robot_ns}}::R;
    typename R::ConfigurationArray cfg;
    for (std::size_t i = 0; i < R::dimension; ++i)
    {
        cfg[i] = config[i];
    }

    Eigen::Matrix4f mat = R::eefk(cfg).matrix();
    std::memcpy(out_matrix, mat.data(), 16 * sizeof(float));
}

extern "C" void VAMP_JIT_FN_FK(const float *config, float *out_spheres)
{
    using R = {{robot_ns}}::R;
    auto block = {{robot_ns}}::load_block_1(config);

    typename R::template Spheres<1> out;
    R::template sphere_fk<1>(block, out);

    for (std::size_t i = 0; i < R::n_spheres; ++i)
    {
        out_spheres[i * 4 + 0] = out.x[{i, 0}];
        out_spheres[i * 4 + 1] = out.y[{i, 0}];
        out_spheres[i * 4 + 2] = out.z[{i, 0}];
        out_spheres[i * 4 + 3] = out.r[{i, 0}];
    }
}

extern "C" std::int32_t
VAMP_JIT_FN_VALIDATE(const float *config_ptr, const void *env_ptr, std::int32_t check_bounds)
{
    using R = {{robot_ns}}::R;
    auto configuration = {{robot_ns}}::load_config(config_ptr);
    return (not check_bounds or R::in_bounds(configuration.trim())) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(
               configuration, configuration, {{robot_ns}}::raked_env(env_ptr));
}

extern "C" std::int32_t VAMP_JIT_FN_VALIDATE_MOTION(
    const float *c_in_ptr,
    const float *c_out_ptr,
    const void *env_ptr,
    std::int32_t check_bounds)
{
    using R = {{robot_ns}}::R;
    auto c_in = {{robot_ns}}::load_config(c_in_ptr);
    auto c_out = {{robot_ns}}::load_config(c_out_ptr);
    return (not check_bounds or (R::in_bounds(c_in.trim()) and R::in_bounds(c_out.trim()))) and
           vamp::planning::validate_motion<R, VAMP_JIT_RAKE, 1>(
               c_in, c_out, {{robot_ns}}::raked_env(env_ptr));
}

extern "C" float
VAMP_JIT_FN_CFG_DISTANCE(const float *a_ptr, const float *b_ptr)
{
    using R = {{robot_ns}}::R;
    return R::distance({{robot_ns}}::load_config(a_ptr), {{robot_ns}}::load_config(b_ptr));
}

extern "C" void
VAMP_JIT_FN_CFG_INTERPOLATE(const float *a_ptr, const float *b_ptr, float t, float *out_ptr)
{
    using R = {{robot_ns}}::R;
    auto c = R::interpolate(
        {{robot_ns}}::load_config(a_ptr), {{robot_ns}}::load_config(b_ptr), t);
    auto arr = c.to_array();
    std::memcpy(out_ptr, arr.data(), R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_FILTER_PC(
    const float *points_in,
    std::uint64_t n_points,
    float point_radius,
    const float *config,
    const void *env_ptr,
    void *out_filtered_vec)
{
    vamp::collision::filter_self_from_pointcloud<{{robot_ns}}::R, VAMP_JIT_RAKE>(
        points_in,
        n_points,
        point_radius,
        {{robot_ns}}::load_block_1(config),
        {{robot_ns}}::raked_env(env_ptr),
        *static_cast<std::vector<vamp::collision::Point> *>(out_filtered_vec));
}

extern "C" float VAMP_JIT_FN_SPACE_MEASURE()
{
    return {{robot_ns}}::R::space_measure();
}

extern "C" void VAMP_JIT_FN_MIN_MAX_RADII(float *out_min, float *out_max)
{
    *out_min = {{robot_ns}}::R::min_radius;
    *out_max = {{robot_ns}}::R::max_radius;
}

extern "C" std::uint64_t VAMP_JIT_FN_N_SPHERES()
{
    return {{robot_ns}}::R::n_spheres;
}

extern "C" void VAMP_JIT_FN_JOINT_NAMES(void *out_strings)
{
    auto *v = static_cast<std::vector<std::string> *>(out_strings);
    v->clear();
    for (auto sv : {{robot_ns}}::R::joint_names)
    {
        v->emplace_back(sv);
    }
}

extern "C" void VAMP_JIT_FN_UPPER_BOUNDS(float *out)
{
    using R = {{robot_ns}}::R;
    std::array<float, R::dimension> ones;
    ones.fill(1.0F);
    typename R::Configuration v(ones);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_LOWER_BOUNDS(float *out)
{
    using R = {{robot_ns}}::R;
    std::array<float, R::dimension> zeros;
    zeros.fill(0.0F);
    typename R::Configuration v(zeros);
    R::scale_configuration(v);
    auto arr = v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::PhsHandle *VAMP_JIT_FN_PHS_NEW(const float *focus_a, const float *focus_b)
{
    using R = {{robot_ns}}::R;
    auto fa = {{robot_ns}}::load_config(focus_a);
    auto fb = {{robot_ns}}::load_config(focus_b);
    auto *phs = new vamp::planning::ProlateHyperspheroid<R>(fa, fb);
    return reinterpret_cast<vamp::jit::ffi::PhsHandle *>(phs);
}

extern "C" void VAMP_JIT_FN_PHS_DESTROY(vamp::jit::ffi::PhsHandle *h)
{
    delete reinterpret_cast<vamp::planning::ProlateHyperspheroid<{{robot_ns}}::R> *>(h);
}

extern "C" void VAMP_JIT_FN_PHS_SET_DIAMETER(vamp::jit::ffi::PhsHandle *h, float diameter)
{
    reinterpret_cast<vamp::planning::ProlateHyperspheroid<{{robot_ns}}::R> *>(h)->set_transverse_diameter(
        diameter);
}

extern "C" void VAMP_JIT_FN_PHS_TRANSFORM(const vamp::jit::ffi::PhsHandle *h, const float *in, float *out)
{
    using R = {{robot_ns}}::R;
    auto in_v = {{robot_ns}}::load_config(in);
    auto out_v = reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(h)->transform(in_v);
    auto arr = out_v.to_array();
    std::memcpy(out, arr.data(), R::dimension * sizeof(float));
}

extern "C" vamp::jit::ffi::SamplerHandle *
VAMP_JIT_FN_SAMPLER_PHS(const vamp::jit::ffi::PhsHandle *phs_h, vamp::jit::ffi::SamplerHandle *inner_h)
{
    using R = {{robot_ns}}::R;
    auto phs = *reinterpret_cast<const vamp::planning::ProlateHyperspheroid<R> *>(phs_h);
    auto inner_rng = {{robot_ns}}::deref_sampler(inner_h);
    auto *holder = new {{robot_ns}}::SamplerPtr(
        std::make_shared<vamp::planning::ProlateHyperspheroidRNG<R>>(phs, inner_rng));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(holder);
}
{% if has_any_constraint %}

// clang-format off
#define VAMP_JIT_FN_CONSTRAINT_DESTROY      vamp_jit_{{sym}}_constraint_destroy
#define VAMP_JIT_FN_CONSTRAINT_PROJECT      vamp_jit_{{sym}}_constraint_project
#define VAMP_JIT_FN_CONSTRAINT_SATISFIED    vamp_jit_{{sym}}_constraint_satisfied
#define VAMP_JIT_FN_SIMPLIFY_CONSTRAINED    vamp_jit_{{sym}}_simplify_constrained
#define VAMP_JIT_FN_C_TASK_SPACE_NEW        vamp_jit_{{sym}}_constraint_task_space_new
#define VAMP_JIT_FN_C_BIMANUAL_NEW          vamp_jit_{{sym}}_constraint_bimanual_task_space_new
#define VAMP_JIT_FN_C_CLOSED_LOOP_NEW       vamp_jit_{{sym}}_constraint_closed_loop_new
#define VAMP_JIT_FN_C_COM_NEW               vamp_jit_{{sym}}_constraint_com_new
#define VAMP_JIT_FN_C_TWIST_NEW             vamp_jit_{{sym}}_constraint_twist_new
#define VAMP_JIT_FN_C_LEAD_SCREW_NEW        vamp_jit_{{sym}}_constraint_lead_screw_new
#define VAMP_JIT_FN_C_LEAD_SCREW_LEVEL_NEW  vamp_jit_{{sym}}_constraint_lead_screw_level_new
// clang-format on

namespace {{robot_ns}}
{
    using ConstraintT = vamp::planning::constraint::Constraint<R, VAMP_JIT_RAKE>;
    using ConstraintPtr = std::shared_ptr<const ConstraintT>;
    using ConstraintSetT = vamp::planning::constraint::ConstraintSet<R, VAMP_JIT_RAKE>;
    using ConstrainedLP =
        vamp::planning::constraint::ConstrainedLocalPlanner<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>;

    inline auto wrap_constraint(ConstraintPtr p) -> vamp::jit::ffi::ConstraintHandle *
    {
        return reinterpret_cast<vamp::jit::ffi::ConstraintHandle *>(new ConstraintPtr(std::move(p)));
    }

    inline auto load_constraints(vamp::jit::ffi::ConstraintHandle *const *handles, std::uint64_t n)
        -> std::vector<ConstraintPtr>
    {
        std::vector<ConstraintPtr> out;
        out.reserve(n);
        for (std::uint64_t i = 0; i < n; ++i)
        {
            out.emplace_back(*reinterpret_cast<const ConstraintPtr *>(handles[i]));
        }
        return out;
    }

    inline auto load_constraint_set(
        vamp::jit::ffi::ConstraintHandle *const *handles,
        std::uint64_t n,
        const void *cs_ptr) -> ConstraintSetT
    {
        const auto &cs = *static_cast<const vamp::planning::constraint::ConstraintSettings *>(cs_ptr);
        return ConstraintSetT(load_constraints(handles, n), cs);
    }

    template <std::size_t N>
    inline auto load_array(const float *data) -> std::array<float, N>
    {
        std::array<float, N> out;
        std::memcpy(out.data(), data, N * sizeof(float));
        return out;
    }
}  // namespace {{robot_ns}}

extern "C" void VAMP_JIT_FN_CONSTRAINT_DESTROY(vamp::jit::ffi::ConstraintHandle *h)
{
    delete reinterpret_cast<{{robot_ns}}::ConstraintPtr *>(h);
}

extern "C" std::int32_t VAMP_JIT_FN_CONSTRAINT_PROJECT(
    const float *config,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr,
    float *out)
{
    auto set = {{robot_ns}}::load_constraint_set(constraints, n_constraints, constraint_settings_ptr);
    auto q = {{robot_ns}}::load_config(config);
    if (not set.project(q))
    {
        return 0;
    }

    auto arr = q.to_array();
    std::memcpy(out, arr.data(), {{robot_ns}}::R::dimension * sizeof(float));
    return 1;
}

extern "C" std::int32_t VAMP_JIT_FN_CONSTRAINT_SATISFIED(
    const float *config,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr)
{
    auto set = {{robot_ns}}::load_constraint_set(constraints, n_constraints, constraint_settings_ptr);
    return set.satisfied({{robot_ns}}::load_config(config)) ? 1 : 0;
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SIMPLIFY_CONSTRAINED(
    const float *path_ptr,
    std::uint64_t n_waypoints,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr)
{
    using R = {{robot_ns}}::R;
    vamp::planning::Path<R> path;
    path.reserve(n_waypoints);
    for (std::uint64_t i = 0; i < n_waypoints; ++i)
    {
        path.emplace_back({{robot_ns}}::load_config(path_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = {{robot_ns}}::ConstrainedLP(
        {{robot_ns}}::load_constraint_set(constraints, n_constraints, constraint_settings_ptr));

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        vamp::planning::simplify<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>(
            path, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% endif %}
{% if has_constraints %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_TASK_SPACE_NEW(
    const float *eef_to_offset,
    const float *world_to_reference,
    const float *lower,
    const float *upper)
{
    using R = {{robot_ns}}::R;
    using TSC = vamp::planning::constraint::TaskSpaceConstraint<R, VAMP_JIT_RAKE>;

    std::array<typename TSC::Transform, R::n_eef> rTe;
    std::array<typename TSC::Transform, R::n_eef> wTr;
    std::array<typename TSC::Bound, R::n_eef> lo;
    std::array<typename TSC::Bound, R::n_eef> hi;
    for (std::size_t i = 0; i < R::n_eef; ++i)
    {
        rTe[i] = {{robot_ns}}::load_array<7>(eef_to_offset + i * 7);
        wTr[i] = {{robot_ns}}::load_array<7>(world_to_reference + i * 7);
        lo[i] = {{robot_ns}}::load_array<6>(lower + i * 6);
        hi[i] = {{robot_ns}}::load_array<6>(upper + i * 6);
    }

    return {{robot_ns}}::wrap_constraint(std::make_shared<const TSC>(rTe, wTr, lo, hi));
}
{% endif %}
{% if has_constraints and num_end_effectors > 1 %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_BIMANUAL_NEW(
    const float *right_in_left,
    const float *lower,
    const float *upper)
{
    using BTSC = vamp::planning::constraint::BimanualTaskSpaceConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE>;
    return {{robot_ns}}::wrap_constraint(std::make_shared<const BTSC>(
        {{robot_ns}}::load_array<7>(right_in_left),
        {{robot_ns}}::load_array<6>(lower),
        {{robot_ns}}::load_array<6>(upper)));
}
{% endif %}
{% if has_closed_loops %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_CLOSED_LOOP_NEW()
{
    using CLC = vamp::planning::constraint::ClosedLoopConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE>;
    return {{robot_ns}}::wrap_constraint(std::make_shared<const CLC>());
}
{% endif %}
{% if has_com %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_COM_NEW(
    const float *vertices_xy,
    std::uint64_t n_vertices)
{
    using CMC = vamp::planning::constraint::CoMConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE>;
    std::vector<typename CMC::Vertex> polygon;
    polygon.reserve(n_vertices);
    for (std::uint64_t i = 0; i < n_vertices; ++i)
    {
        polygon.push_back(typename CMC::Vertex{vertices_xy[i * 2], vertices_xy[i * 2 + 1]});
    }

    return {{robot_ns}}::wrap_constraint(std::make_shared<const CMC>(polygon));
}
{% endif %}
{% if has_twist %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_TWIST_NEW(
    const float *eef_to_offset,
    const float *world_to_reference,
    const float *reference_coefficients,
    const float *local_coefficients)
{
    using TWC = vamp::planning::constraint::TwistConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE, 1>;
    return {{robot_ns}}::wrap_constraint(std::make_shared<const TWC>(
        {{robot_ns}}::load_array<7>(eef_to_offset),
        {{robot_ns}}::load_array<7>(world_to_reference),
        typename TWC::Coefficients{ {{robot_ns}}::load_array<6>(reference_coefficients)},
        typename TWC::Coefficients{ {{robot_ns}}::load_array<6>(local_coefficients)}));
}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_LEAD_SCREW_NEW(
    const float *eef_to_offset,
    const float *world_to_reference,
    float pitch)
{
    using LSC = vamp::planning::constraint::LeadScrewConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE>;
    return {{robot_ns}}::wrap_constraint(std::make_shared<const LSC>(
        {{robot_ns}}::load_array<7>(eef_to_offset),
        {{robot_ns}}::load_array<7>(world_to_reference),
        pitch));
}
{% endif %}
{% if has_lead_screw %}

extern "C" vamp::jit::ffi::ConstraintHandle *VAMP_JIT_FN_C_LEAD_SCREW_LEVEL_NEW(
    const float *eef_to_offset,
    const float *world_to_reference,
    float pitch,
    float target)
{
    using LSL = vamp::planning::constraint::LeadScrewLevelConstraint<{{robot_ns}}::R, VAMP_JIT_RAKE>;
    return {{robot_ns}}::wrap_constraint(std::make_shared<const LSL>(
        {{robot_ns}}::load_array<7>(eef_to_offset),
        {{robot_ns}}::load_array<7>(world_to_reference),
        pitch,
        target));
}
{% endif %}