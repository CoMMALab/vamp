// clang-format off
#include <vamp/planning/planners/{{planner_header}}>
#include <vamp/planning/planners/{{settings_header}}>

#define VAMP_JIT_PLANNER_NS         vamp_jit_{{planner_name}}
#define VAMP_JIT_PLANNER_CLASS      vamp::planning::{{planner_class}}
#define VAMP_JIT_SETTINGS_CLASS     vamp::planning::{{settings_class}}
#define VAMP_JIT_RAKE               {{rake}}
#define VAMP_JIT_RESOLUTION         {{resolution}}

#define VAMP_JIT_FN_SOLVE           vamp_jit_{{planner_name}}_solve
#define VAMP_JIT_FN_SOLVE_MULTI     vamp_jit_{{planner_name}}_solve_multi
{% if has_any_constraint and planner_local_planner %}
#define VAMP_JIT_FN_SOLVE_CONSTRAINED       vamp_jit_{{planner_name}}_solve_constrained
#define VAMP_JIT_FN_SOLVE_MULTI_CONSTRAINED vamp_jit_{{planner_name}}_solve_multi_constrained
{% endif %}
{% if is_flask and planner_local_planner %}
#define VAMP_JIT_FN_SOLVE_PHASE             vamp_jit_{{planner_name}}_solve_phase
#define VAMP_JIT_FN_SOLVE_MULTI_PHASE       vamp_jit_{{planner_name}}_solve_multi_phase
{% endif %}
{% if is_flask and has_chart and planner_local_planner %}
#define VAMP_JIT_FN_SOLVE_CHART             vamp_jit_{{planner_name}}_solve_chart
#define VAMP_JIT_FN_SOLVE_MULTI_CHART       vamp_jit_{{planner_name}}_solve_multi_chart
{% endif %}
// clang-format on

#include <vector>

namespace VAMP_JIT_PLANNER_NS
{
    using R = {{robot_ns}}::R;
    using PlannerT = VAMP_JIT_PLANNER_CLASS<R, VAMP_JIT_RAKE, VAMP_JIT_RESOLUTION>;
    using SettingsT = VAMP_JIT_SETTINGS_CLASS;
}  // namespace VAMP_JIT_PLANNER_NS

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE(
    const float *start_ptr,
    const float *goal_ptr,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    auto goal = {{robot_ns}}::load_config(goal_ptr);
    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);

    auto *wrapped = new {{robot_ns}}::WrappedResult{PlannerT::solve(start, goal, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_MULTI(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back({{robot_ns}}::load_config(goals_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);

    auto *wrapped = new {{robot_ns}}::WrappedResult{PlannerT::solve(start, goals, env_rake, settings, rng)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% if has_any_constraint and planner_local_planner %}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_CONSTRAINED(
    const float *start_ptr,
    const float *goal_ptr,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    auto goal = {{robot_ns}}::load_config(goal_ptr);
    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = {{robot_ns}}::ConstrainedLP(
        {{robot_ns}}::load_constraint_set(constraints, n_constraints, constraint_settings_ptr));

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goal, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_MULTI_CONSTRAINED(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::ConstraintHandle *const *constraints,
    std::uint64_t n_constraints,
    const void *constraint_settings_ptr)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back({{robot_ns}}::load_config(goals_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = {{robot_ns}}::ConstrainedLP(
        {{robot_ns}}::load_constraint_set(constraints, n_constraints, constraint_settings_ptr));

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goals, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% endif %}
{% if is_flask and planner_local_planner %}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_PHASE(
    const float *start_ptr,
    const float *goal_ptr,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    auto goal = {{robot_ns}}::load_config(goal_ptr);
    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_phase_lp(phase, n_phase);

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goal, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_MULTI_PHASE(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
    const void *env_ptr,
    const void *settings_ptr,
    vamp::jit::ffi::SamplerHandle *sampler,
    vamp::jit::ffi::PhaseConstraintHandle *const *phase,
    std::uint64_t n_phase)
{
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back({{robot_ns}}::load_config(goals_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_phase_lp(phase, n_phase);

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goals, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% endif %}
{% if is_flask and has_chart and planner_local_planner %}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_CHART(
    const float *start_ptr,
    const float *goal_ptr,
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
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    auto goal = {{robot_ns}}::load_config(goal_ptr);
    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, phase, n_phase);

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goal, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}

extern "C" vamp::jit::ffi::PlanResultHandle *VAMP_JIT_FN_SOLVE_MULTI_CHART(
    const float *start_ptr,
    const float *goals_ptr,
    std::uint64_t n_goals,
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
    using namespace VAMP_JIT_PLANNER_NS;

    auto start = {{robot_ns}}::load_config(start_ptr);
    std::vector<typename R::Configuration> goals;
    goals.reserve(n_goals);
    for (std::uint64_t i = 0; i < n_goals; ++i)
    {
        goals.emplace_back({{robot_ns}}::load_config(goals_ptr + i * R::dimension));
    }

    auto env_rake = {{robot_ns}}::raked_env(env_ptr);
    const auto &settings = *static_cast<const SettingsT *>(settings_ptr);
    auto rng = {{robot_ns}}::deref_sampler(sampler);
    auto lp = vamp_jit_flask::make_chart_lp(
        constraints, n_constraints, constraint_settings_ptr, chart_settings_ptr, phase, n_phase);

    auto *wrapped = new {{robot_ns}}::WrappedResult{
        PlannerT::solve(start, goals, env_rake, settings, rng, lp)};
    return reinterpret_cast<vamp::jit::ffi::PlanResultHandle *>(wrapped);
}
{% endif %}