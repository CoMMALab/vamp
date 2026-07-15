#include <vamp/planning/constraints/phase/phase_constraint.hh>
#include <vamp/planning/constraints/phase/phase_constraint_set.hh>
#include <vamp/planning/constraints/phase/kinetic_energy_constraint.hh>
#include <vamp/planning/constraints/phase/eef_speed_constraint.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/random/ke_shaped.hh>
{% if has_chart %}
#include <vamp/planning/constraints/chart_local_planner.hh>
#include <vamp/planning/constraints/settings.hh>
{% endif %}

// clang-format off
#define VAMP_JIT_FN_PHASE_DESTROY           vamp_jit_{{sym}}_phase_destroy
#define VAMP_JIT_FN_PHASE_KE_NEW            vamp_jit_{{sym}}_phase_kinetic_energy_new
#define VAMP_JIT_FN_PHASE_EEF_SPEED_NEW     vamp_jit_{{sym}}_phase_eef_speed_new
#define VAMP_JIT_FN_PHASE_SATISFIED         vamp_jit_{{sym}}_phase_satisfied
#define VAMP_JIT_FN_PHASE_VELOCITY_SCALE    vamp_jit_{{sym}}_phase_velocity_scale
#define VAMP_JIT_FN_SAMPLER_KE_SHAPED       vamp_jit_{{sym}}_sampler_ke_shaped
#define VAMP_JIT_FN_SIMPLIFY_PHASE          vamp_jit_{{sym}}_simplify_phase

#define VAMP_JIT_FN_FLASK_OPTIMAL_TIME      vamp_jit_{{sym}}_optimal_time
#define VAMP_JIT_FN_FLASK_COST              vamp_jit_{{sym}}_cost
#define VAMP_JIT_FN_FLASK_COST_GRAD         vamp_jit_{{sym}}_cost_grad
#define VAMP_JIT_FN_FLASK_EVAL              vamp_jit_{{sym}}_eval
#define VAMP_JIT_FN_FLASK_TORQUES           vamp_jit_{{sym}}_torques
#define VAMP_JIT_FN_FLASK_KINETIC_ENERGY    vamp_jit_{{sym}}_kinetic_energy
#define VAMP_JIT_FN_FLASK_EEF_VELOCITY      vamp_jit_{{sym}}_eef_velocity
#define VAMP_JIT_FN_FLASK_N_EEF             vamp_jit_{{sym}}_n_end_effectors
#define VAMP_JIT_FN_FLASK_FLAT_DIMENSION    vamp_jit_{{sym}}_flat_dimension
#define VAMP_JIT_FN_FLASK_RHO_GET           vamp_jit_{{sym}}_rho_get
#define VAMP_JIT_FN_FLASK_RHO_SET           vamp_jit_{{sym}}_rho_set
#define VAMP_JIT_FN_FLASK_VELOCITY_LIMITS   vamp_jit_{{sym}}_velocity_limits
#define VAMP_JIT_FN_FLASK_EFFORT_LIMITS     vamp_jit_{{sym}}_effort_limits
{% if has_chart %}
#define VAMP_JIT_FN_CHART_PROJECT           vamp_jit_{{sym}}_chart_project
#define VAMP_JIT_FN_CHART_SATISFIED         vamp_jit_{{sym}}_chart_satisfied
#define VAMP_JIT_FN_CHART_DEBUG             vamp_jit_{{sym}}_chart_debug
#define VAMP_JIT_FN_CHART_LIFT_EDGE         vamp_jit_{{sym}}_chart_lift_edge
#define VAMP_JIT_FN_SIMPLIFY_CHART          vamp_jit_{{sym}}_simplify_chart
{% endif %}
// clang-format on

namespace vamp_jit_flask
{
    using A = vamp_jit_robot::R;
    using F = vamp_jit_flask_robot::R;

    using PhaseConstraintT = vamp::planning::constraint::PhaseConstraint<F, {{rake}}>;
    using PhaseConstraintPtr = std::shared_ptr<const PhaseConstraintT>;
    using PhaseSetT = vamp::planning::constraint::PhaseConstraintSet<F, {{rake}}>;
    using PhaseLP = vamp::planning::UnconstrainedLocalPlanner<F, {{rake}}, F::resolution>;

    inline auto wrap_phase(PhaseConstraintPtr p) -> vamp::jit::ffi::PhaseConstraintHandle *
    {
        return reinterpret_cast<vamp::jit::ffi::PhaseConstraintHandle *>(
            new PhaseConstraintPtr(std::move(p)));
    }

    inline auto load_phase(vamp::jit::ffi::PhaseConstraintHandle *const *handles, std::uint64_t n)
        -> std::vector<PhaseConstraintPtr>
    {
        std::vector<PhaseConstraintPtr> out;
        out.reserve(n);
        for (std::uint64_t i = 0; i < n; ++i)
        {
            out.emplace_back(*reinterpret_cast<const PhaseConstraintPtr *>(handles[i]));
        }
        return out;
    }

    inline auto make_phase_lp(vamp::jit::ffi::PhaseConstraintHandle *const *handles, std::uint64_t n)
        -> PhaseLP
    {
        return PhaseLP(PhaseSetT(load_phase(handles, n)));
    }
{% if has_chart %}

    using ChartLP = vamp::planning::constraint::ChartLocalPlanner<F, {{rake}}, F::resolution>;

    inline auto make_chart_lp(
        vamp::jit::ffi::ConstraintHandle *const *constraints,
        std::uint64_t n_constraints,
        const void *constraint_settings_ptr,
        const void *chart_settings_ptr,
        vamp::jit::ffi::PhaseConstraintHandle *const *phase,
        std::uint64_t n_phase) -> ChartLP
    {
        const auto &chs =
            *static_cast<const vamp::planning::constraint::ChartSettings *>(chart_settings_ptr);
        return ChartLP(
            vamp_jit_robot::load_constraint_set(constraints, n_constraints, constraint_settings_ptr),
            chs,
            PhaseSetT(load_phase(phase, n_phase)));
    }
{% endif %}
}  // namespace vamp_jit_flask

extern "C" void VAMP_JIT_FN_PHASE_DESTROY(vamp::jit::ffi::PhaseConstraintHandle *h)
{
    delete reinterpret_cast<vamp_jit_flask::PhaseConstraintPtr *>(h);
}

extern "C" vamp::jit::ffi::PhaseConstraintHandle *VAMP_JIT_FN_PHASE_KE_NEW(float max_energy)
{
    using KE = vamp::planning::constraint::KineticEnergyConstraint<vamp_jit_flask::F, {{rake}}>;
    return vamp_jit_flask::wrap_phase(std::make_shared<const KE>(max_energy));
}

extern "C" vamp::jit::ffi::PhaseConstraintHandle *VAMP_JIT_FN_PHASE_EEF_SPEED_NEW(float max_speed)
{
    using ES = vamp::planning::constraint::EEFSpeedConstraint<vamp_jit_flask::F, {{rake}}>;
    return vamp_jit_flask::wrap_phase(std::make_shared<const ES>(max_speed));
}

extern "C" std::int32_t VAMP_JIT_FN_PHASE_SATISFIED(
    const float *config,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    auto set = vamp_jit_flask::PhaseSetT(vamp_jit_flask::load_phase(phase, n_phase));
    return set.satisfied(vamp_jit_flask_robot::load_config(config)) ? 1 : 0;
}

extern "C" float VAMP_JIT_FN_PHASE_VELOCITY_SCALE(
    const float *config,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    auto set = vamp_jit_flask::PhaseSetT(vamp_jit_flask::load_phase(phase, n_phase));
    return set.velocity_scale(vamp_jit_flask_robot::load_config(config));
}

extern "C" vamp::jit::ffi::SamplerHandle *
VAMP_JIT_FN_SAMPLER_KE_SHAPED(vamp::jit::ffi::SamplerHandle *inner_h, float max_energy)
{
    auto inner = vamp_jit_flask_robot::deref_sampler(inner_h);
    auto *holder = new vamp_jit_flask_robot::SamplerPtr(
        std::make_shared<vamp::rng::KineticEnergyShaped<vamp_jit_flask::F>>(inner, max_energy));
    return reinterpret_cast<vamp::jit::ffi::SamplerHandle *>(holder);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SIMPLIFY_PHASE(
    const float *path_ptr,
    std::uint64_t n_waypoints,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    using F = vamp_jit_flask::F;
    vamp::planning::Path<F> path;
    path.reserve(n_waypoints);
    for (std::uint64_t i = 0; i < n_waypoints; ++i)
    {
        path.emplace_back(vamp_jit_flask_robot::load_config(path_ptr + i * F::dimension));
    }

    auto env_rake = vamp_jit_flask_robot::raked_env(env_ptr);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = vamp_jit_flask_robot::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_phase_lp(phase, n_phase);

    auto *wrapped = new vamp_jit_flask_robot::WrappedResult{
        vamp::planning::simplify<F, {{rake}}, F::resolution>(path, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" float VAMP_JIT_FN_FLASK_OPTIMAL_TIME(const float *a, const float *b)
{
    return vamp_jit_flask::F::optimal_time(
        vamp_jit_flask_robot::load_config(a), vamp_jit_flask_robot::load_config(b));
}

extern "C" float VAMP_JIT_FN_FLASK_COST(const float *a, const float *b)
{
    return vamp_jit_flask::F::cost(
        vamp_jit_flask_robot::load_config(a), vamp_jit_flask_robot::load_config(b));
}

extern "C" void VAMP_JIT_FN_FLASK_COST_GRAD(
    const float *a,
    const float *b,
    float *out_cost,
    float *out_time,
    float *out_grad_a,
    float *out_grad_b)
{
    using F = vamp_jit_flask::F;
    const auto cg = F::cost_grad(
        vamp_jit_flask_robot::load_config(a), vamp_jit_flask_robot::load_config(b));
    *out_cost = cg.cost;
    *out_time = cg.time;
    std::memcpy(out_grad_a, cg.grad_a.data(), F::dimension * sizeof(float));
    std::memcpy(out_grad_b, cg.grad_b.data(), F::dimension * sizeof(float));
}

extern "C" void
VAMP_JIT_FN_FLASK_EVAL(const float *a, const float *b, float T, float t, float *out)
{
    using F = vamp_jit_flask::F;
    const auto x = F::eval(
        vamp_jit_flask_robot::load_config(a), vamp_jit_flask_robot::load_config(b), T, t);
    std::memcpy(out, x.data(), 3 * F::flat_dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_FLASK_TORQUES(const float *x, float *out)
{
    using F = vamp_jit_flask::F;
    std::array<float, 3 * F::flat_dimension> in;
    std::memcpy(in.data(), x, in.size() * sizeof(float));
    const auto tau = F::torques(in);
    std::memcpy(out, tau.data(), F::flat_dimension * sizeof(float));
}

extern "C" float VAMP_JIT_FN_FLASK_KINETIC_ENERGY(const float *x)
{
    using F = vamp_jit_flask::F;
    std::array<float, F::dimension> in;
    std::memcpy(in.data(), x, in.size() * sizeof(float));
    return F::kinetic_energy(in);
}

extern "C" void VAMP_JIT_FN_FLASK_EEF_VELOCITY(const float *x, float *out)
{
    using F = vamp_jit_flask::F;
    std::array<float, F::dimension> in;
    std::memcpy(in.data(), x, in.size() * sizeof(float));
    const auto v = F::eef_velocity(in);
    std::memcpy(out, v.data(), 3 * F::n_end_effectors * sizeof(float));
}

extern "C" std::uint64_t VAMP_JIT_FN_FLASK_N_EEF()
{
    return vamp_jit_flask::F::n_end_effectors;
}

extern "C" std::uint64_t VAMP_JIT_FN_FLASK_FLAT_DIMENSION()
{
    return vamp_jit_flask::F::flat_dimension;
}

extern "C" float VAMP_JIT_FN_FLASK_RHO_GET()
{
    return vamp_jit_flask::F::rho;
}

extern "C" void VAMP_JIT_FN_FLASK_RHO_SET(float rho)
{
    vamp_jit_flask::F::rho = rho;
}

extern "C" void VAMP_JIT_FN_FLASK_VELOCITY_LIMITS(float *out)
{
    using F = vamp_jit_flask::F;
    std::memcpy(out, F::velocity_limits.data(), F::flat_dimension * sizeof(float));
}

extern "C" void VAMP_JIT_FN_FLASK_EFFORT_LIMITS(float *out)
{
    using F = vamp_jit_flask::F;
    std::memcpy(out, F::effort_limits.data(), F::flat_dimension * sizeof(float));
}
{% if has_chart %}

extern "C" std::int32_t VAMP_JIT_FN_CHART_PROJECT(
    const float *config,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr,
    const void *chart_settings_ptr,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase,
    float *out)
{
    const auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, phase, n_phase);
    auto z = vamp_jit_flask_robot::load_config(config);
    if (not lp.project(z))
    {
        return 0;
    }

    auto arr = z.to_array();
    std::memcpy(out, arr.data(), vamp_jit_flask::F::dimension * sizeof(float));
    return 1;
}

extern "C" std::int32_t VAMP_JIT_FN_CHART_SATISFIED(
    const float *config,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr,
    const void *chart_settings_ptr,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    const auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, phase, n_phase);
    return lp.satisfied(vamp_jit_flask_robot::load_config(config)) ? 1 : 0;
}

extern "C" void VAMP_JIT_FN_CHART_DEBUG(
    const float *config,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr,
    const void *chart_settings_ptr,
    void *out_rows_vec)
{
    using A = vamp_jit_flask::A;
    const auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, nullptr, 0);
    const auto rows = lp.debug_chart(vamp_jit_flask_robot::load_config(config));

    auto *out = static_cast<std::vector<float> *>(out_rows_vec);
    out->clear();
    out->reserve(rows.size() * A::dimension);
    for (const auto &row : rows)
    {
        out->insert(out->end(), row.begin(), row.end());
    }
}

extern "C" std::int32_t VAMP_JIT_FN_CHART_LIFT_EDGE(
    const float *from,
    const float *target,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    std::int32_t forward,
    std::uint64_t n_samples,
    const void *constraint_settings_ptr,
    const void *chart_settings_ptr,
    void *out_states_vec,
    void *out_errors_vec,
    float *out_duration,
    float *out_cost)
{
    using F = vamp_jit_flask::F;
    const auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, nullptr, 0);

    std::vector<typename F::Configuration> states;
    auto *errors = static_cast<std::vector<float> *>(out_errors_vec);
    errors->clear();
    float duration = 0.F, cost = 0.F;
    if (not lp.lift_edge(
            vamp_jit_flask_robot::load_config(from),
            vamp_jit_flask_robot::load_config(target),
            forward != 0,
            n_samples,
            states,
            *errors,
            duration,
            cost))
    {
        return 0;
    }

    auto *out_states = static_cast<std::vector<float> *>(out_states_vec);
    out_states->clear();
    out_states->reserve(states.size() * F::dimension);
    for (const auto &s : states)
    {
        auto arr = s.to_array();
        out_states->insert(out_states->end(), arr.begin(), arr.begin() + F::dimension);
    }

    *out_duration = duration;
    *out_cost = cost;
    return 1;
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SIMPLIFY_CHART(
    const float *path_ptr,
    std::uint64_t n_waypoints,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr,
    const void *chart_settings_ptr,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    using F = vamp_jit_flask::F;
    vamp::planning::Path<F> path;
    path.reserve(n_waypoints);
    for (std::uint64_t i = 0; i < n_waypoints; ++i)
    {
        path.emplace_back(vamp_jit_flask_robot::load_config(path_ptr + i * F::dimension));
    }

    auto env_rake = vamp_jit_flask_robot::raked_env(env_ptr);
    const auto &settings = *static_cast<const vamp::planning::SimplifySettings *>(settings_ptr);
    auto rng = vamp_jit_flask_robot::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, phase, n_phase);

    auto *wrapped = new vamp_jit_flask_robot::WrappedResult{
        vamp::planning::simplify<F, {{rake}}, F::resolution>(path, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% endif %}
