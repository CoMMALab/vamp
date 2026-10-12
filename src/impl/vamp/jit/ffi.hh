#pragma once

#include <cstddef>
#include <cstdint>

namespace vamp::jit::ffi
{
    struct PlanResultHandle;
    struct SamplerHandle;
    struct DebugHandle;
    struct PhsHandle;
    struct ConstraintHandle;
    struct PhaseConstraintHandle;
    struct GoalRegionHandle;

    struct PlanResultMeta
    {
        std::int32_t success;
        std::uint32_t dimension;
        std::uint64_t waypoints;
        std::uint64_t nanoseconds;
        std::uint64_t iterations;
        float cost;
        std::uint64_t samples_drawn;
        std::uint64_t nn_queries;
        std::uint64_t motions_attempted;
        std::uint64_t motions_valid;
        std::uint64_t rake_calls;
    };

    using SolveFn = PlanResultHandle *(*)(const float *start,
                                          const float *goal,
                                          const void *env_ptr,
                                          const void *settings_ptr,
                                          SamplerHandle *sampler);

    using SolveMultiFn = PlanResultHandle *(*)(const float *start,
                                               const float *goals_ptr,
                                               std::uint64_t n_goals,
                                               const void *env_ptr,
                                               const void *settings_ptr,
                                               SamplerHandle *sampler);

    using SimplifyFn = PlanResultHandle *(*)(const float *path_ptr,
                                             std::uint64_t n_waypoints,
                                             const void *env_ptr,
                                             const void *settings_ptr,
                                             SamplerHandle *sampler);

    using ResultMetaFn = PlanResultMeta (*)(const PlanResultHandle *);
    using ResultCopyWaypointFn = void (*)(const PlanResultHandle *, std::uint64_t idx, float *out);
    using ResultDestroyFn = void (*)(PlanResultHandle *);
    using ResultSizesFn = void (*)(const PlanResultHandle *, void *out_sizes_vec);

    using SamplerHaltonFn = SamplerHandle *(*)();
    using SamplerXorshiftFn = SamplerHandle *(*)(std::uint64_t seed);
    using SamplerResetFn = void (*)(SamplerHandle *);
    using SamplerSkipFn = void (*)(SamplerHandle *, std::uint64_t n);
    using SamplerNextFn = void (*)(SamplerHandle *, float *out);
    using SamplerDestroyFn = void (*)(SamplerHandle *);

    using DebugFn = DebugHandle *(*)(const float *config, const void *env_ptr);
    using DebugDestroyFn = void (*)(DebugHandle *);

    using EefkFn = void (*)(const float *config, float *out_matrix);
    using FkFn = void (*)(const float *config, float *out_spheres);
    using ValidateFn = std::int32_t (*)(const float *config, const void *env_ptr, std::int32_t check_bounds);
    using ValidateMotionFn = std::int32_t (
            *)(const float *c_in, const float *c_out, const void *env_ptr, std::int32_t check_bounds);
    // Joint-aware configuration metric / interpolation on the robot's manifold.
    // For Euclidean robots the trace lowers to L2 / linear blend; for SE(2)/
    // SO(3)/SE(3) it uses pinocchio's geodesic distance / exponential.
    using CfgDistanceFn = float (*)(const float *a, const float *b);
    using CfgInterpolateFn = void (*)(const float *a, const float *b, float t, float *out);

    using FilterPointcloudFn = void (*)(
        const float *points_in,
        std::uint64_t n_points,
        float point_radius,
        const float *config,
        const void *env_ptr,
        void *out_filtered);

    using SpaceMeasureFn = float (*)();
    using MinMaxRadiiFn = void (*)(float *out_min, float *out_max);
    using NSpheresFn = std::uint64_t (*)();
    using BoundsFn = void (*)(float *out);
    using JointNamesFn = void (*)(void *out_strings);

    using PhsNewFn = PhsHandle *(*)(const float *focus_a, const float *focus_b);
    using PhsDestroyFn = void (*)(PhsHandle *);
    using PhsSetDiameterFn = void (*)(PhsHandle *, float diameter);
    using PhsTransformFn = void (*)(const PhsHandle *, const float *in, float *out);
    using SamplerPhsFn = SamplerHandle *(*)(const PhsHandle *, SamplerHandle *inner);

    // Constraint factories. Transforms are 7 floats (wxyz quaternion + xyz translation),
    // bounds are 6 floats; per-end-effector arguments are packed contiguously (n_eef x 7,
    // n_eef x 6). The settings pointer in project/satisfied/solve is a host-side
    // vamp::planning::constraint::ConstraintSettings, type-punned like planner settings.
    using TaskSpaceConstraintNewFn = ConstraintHandle *(*)(const float *eef_to_offset,
                                                           const float *world_to_reference,
                                                           const float *lower,
                                                           const float *upper);
    using BimanualTaskSpaceConstraintNewFn =
        ConstraintHandle *(*)(const float *reference, const float *lower, const float *upper);
    using ClosedLoopConstraintNewFn = ConstraintHandle *(*)();
    using CoMConstraintNewFn = ConstraintHandle *(*)(const float *vertices_xy, std::uint64_t n_vertices);
    using TwistConstraintNewFn = ConstraintHandle *(*)(const float *eef_to_offset,
                                                       const float *world_to_reference,
                                                       const float *reference_coefficients,
                                                       const float *local_coefficients);
    using LeadScrewConstraintNewFn = ConstraintHandle *(*)(const float *eef_to_offset,
                                                           const float *world_to_reference,
                                                           float pitch);
    using LeadScrewLevelConstraintNewFn = ConstraintHandle *(*)(const float *eef_to_offset,
                                                                const float *world_to_reference,
                                                                float pitch,
                                                                float target);
    using ConstraintDestroyFn = void (*)(ConstraintHandle *);

    using ConstraintProjectFn = std::int32_t (*)(const float *config,
                                                 ConstraintHandle *const *constraints,
                                                 std::uint64_t n_constraints,
                                                 const void *constraint_settings_ptr,
                                                 float *out);
    using ConstraintSatisfiedFn = std::int32_t (*)(const float *config,
                                                   ConstraintHandle *const *constraints,
                                                   std::uint64_t n_constraints,
                                                   const void *constraint_settings_ptr);

    using SolveConstrainedFn = PlanResultHandle *(*)(const float *start,
                                                     const float *goal,
                                                     const void *env_ptr,
                                                     const void *settings_ptr,
                                                     SamplerHandle *sampler,
                                                     ConstraintHandle *const *constraints,
                                                     std::uint64_t n_constraints,
                                                     const void *constraint_settings_ptr);

    using SolveMultiConstrainedFn = PlanResultHandle *(*)(const float *start,
                                                          const float *goals_ptr,
                                                          std::uint64_t n_goals,
                                                          const void *env_ptr,
                                                          const void *settings_ptr,
                                                          SamplerHandle *sampler,
                                                          ConstraintHandle *const *constraints,
                                                          std::uint64_t n_constraints,
                                                          const void *constraint_settings_ptr);

    using SimplifyConstrainedFn = PlanResultHandle *(*)(const float *path_ptr,
                                                        std::uint64_t n_waypoints,
                                                        const void *env_ptr,
                                                        const void *settings_ptr,
                                                        SamplerHandle *sampler,
                                                        ConstraintHandle *const *constraints,
                                                        std::uint64_t n_constraints,
                                                        const void *constraint_settings_ptr);

    // Phase constraints (flask z-robots): configurations are flat states of the flask
    // sibling (dimension = 2 * flat_dimension).
    using PhaseKineticEnergyNewFn = PhaseConstraintHandle *(*)(float max_energy);
    using PhaseEEFSpeedNewFn = PhaseConstraintHandle *(*)(float max_speed);
    using PhaseDestroyFn = void (*)(PhaseConstraintHandle *);
    // env_ptr (host-side Environment<float>, or nullptr) supplies end-effector payloads
    // from its attachments to dynamics-dependent kernels.
    using PhaseSatisfiedFn = std::int32_t (*)(const float *config,
                                              PhaseConstraintHandle *const *phase,
                                              std::uint64_t n_phase,
                                              const void *env_ptr);
    using PhaseVelocityScaleFn = float (*)(const float *config,
                                           PhaseConstraintHandle *const *phase,
                                           std::uint64_t n_phase,
                                           const void *env_ptr);
    using SamplerKeShapedFn = SamplerHandle *(*)(SamplerHandle *inner, float max_energy, const void *env_ptr);

    using SolvePhaseFn = PlanResultHandle *(*)(const float *start,
                                               const float *goal,
                                               const void *env_ptr,
                                               const void *settings_ptr,
                                               SamplerHandle *sampler,
                                               PhaseConstraintHandle *const *phase,
                                               std::uint64_t n_phase);

    using SolveMultiPhaseFn = PlanResultHandle *(*)(const float *start,
                                                    const float *goals_ptr,
                                                    std::uint64_t n_goals,
                                                    const void *env_ptr,
                                                    const void *settings_ptr,
                                                    SamplerHandle *sampler,
                                                    PhaseConstraintHandle *const *phase,
                                                    std::uint64_t n_phase);

    using SimplifyPhaseFn = PlanResultHandle *(*)(const float *path_ptr,
                                                  std::uint64_t n_waypoints,
                                                  const void *env_ptr,
                                                  const void *settings_ptr,
                                                  SamplerHandle *sampler,
                                                  PhaseConstraintHandle *const *phase,
                                                  std::uint64_t n_phase);

    // Flask LQMT kernels. States a/b are flat states (2 * flat_dimension); eval outputs
    // the cubic state row-stack [y; yd; ydd] (3 * flat_dimension).
    using FlaskOptimalTimeFn = float (*)(const float *a, const float *b);
    using FlaskCostFn = float (*)(const float *a, const float *b);
    using FlaskCostGradFn = void (*)(const float *a,
                                     const float *b,
                                     float *out_cost,
                                     float *out_time,
                                     float *out_grad_a,
                                     float *out_grad_b);
    using FlaskEvalFn = void (*)(const float *a, const float *b, float T, float t, float *out);
    using FlaskTorquesFn = void (*)(const float *x, float *out, const void *env_ptr);
    using FlaskKineticEnergyFn = float (*)(const float *x, const void *env_ptr);
    using FlaskEEFVelocityFn = void (*)(const float *x, float *out);
    using FlaskNEndEffectorsFn = std::uint64_t (*)();
    using FlaskFlatDimensionFn = std::uint64_t (*)();
    using FlaskRhoGetFn = float (*)();
    using FlaskRhoSetFn = void (*)(float rho);
    using FlaskEffortScaleGetFn = float (*)();
    using FlaskEffortScaleSetFn = void (*)(float scale);
    using FlaskLimitsFn = void (*)(float *out);

    // Chart-based constrained planning on the flask sibling: manifold constraints are
    // ambient-robot handles; the chart settings pointer is a host-side
    // vamp::planning::constraint::ChartSettings, type-punned like planner settings.
    using ChartProjectFn = std::int32_t (*)(const float *config,
                                            ConstraintHandle *const *constraints,
                                            std::uint64_t n_constraints,
                                            const void *constraint_settings_ptr,
                                            const void *chart_settings_ptr,
                                            PhaseConstraintHandle *const *phase,
                                            std::uint64_t n_phase,
                                            const void *env_ptr,
                                            float *out);
    using ChartSatisfiedFn = std::int32_t (*)(const float *config,
                                              ConstraintHandle *const *constraints,
                                              std::uint64_t n_constraints,
                                              const void *constraint_settings_ptr,
                                              const void *chart_settings_ptr,
                                              PhaseConstraintHandle *const *phase,
                                              std::uint64_t n_phase,
                                              const void *env_ptr);
    using ChartDebugFn = void (*)(const float *config,
                                  ConstraintHandle *const *constraints,
                                  std::uint64_t n_constraints,
                                  const void *constraint_settings_ptr,
                                  const void *chart_settings_ptr,
                                  void *out_rows_vec);
    using ChartLiftEdgeFn = std::int32_t (*)(const float *from,
                                             const float *target,
                                             ConstraintHandle *const *constraints,
                                             std::uint64_t n_constraints,
                                             std::int32_t forward,
                                             std::uint64_t n_samples,
                                             const void *constraint_settings_ptr,
                                             const void *chart_settings_ptr,
                                             void *out_states_vec,
                                             void *out_errors_vec,
                                             float *out_duration,
                                             float *out_cost,
                                             void *out_accelerations_vec);

    using SolveChartFn = PlanResultHandle *(*)(const float *start,
                                               const float *goal,
                                               const void *env_ptr,
                                               const void *settings_ptr,
                                               SamplerHandle *sampler,
                                               ConstraintHandle *const *constraints,
                                               std::uint64_t n_constraints,
                                               const void *constraint_settings_ptr,
                                               const void *chart_settings_ptr,
                                               PhaseConstraintHandle *const *phase,
                                               std::uint64_t n_phase);

    using SolveMultiChartFn = PlanResultHandle *(*)(const float *start,
                                                    const float *goals_ptr,
                                                    std::uint64_t n_goals,
                                                    const void *env_ptr,
                                                    const void *settings_ptr,
                                                    SamplerHandle *sampler,
                                                    ConstraintHandle *const *constraints,
                                                    std::uint64_t n_constraints,
                                                    const void *constraint_settings_ptr,
                                                    const void *chart_settings_ptr,
                                                    PhaseConstraintHandle *const *phase,
                                                    std::uint64_t n_phase);

    using SimplifyChartFn = PlanResultHandle *(*)(const float *path_ptr,
                                                  std::uint64_t n_waypoints,
                                                  const void *env_ptr,
                                                  const void *settings_ptr,
                                                  SamplerHandle *sampler,
                                                  ConstraintHandle *const *constraints,
                                                  std::uint64_t n_constraints,
                                                  const void *constraint_settings_ptr,
                                                  const void *chart_settings_ptr,
                                                  PhaseConstraintHandle *const *phase,
                                                  std::uint64_t n_phase);

    // Impact goal regions on the flask sibling (vamp::planning::constraint::ImpactGoalRegion).
    // New returns nullptr when the constraint set has no equality form (out_rows 0) or rate does
    // not have one entry per equality row (out_rows the row count).
    using GoalRegionNewFn = GoalRegionHandle *(*)(ConstraintHandle *const *constraints,
                                                  std::uint64_t n_constraints,
                                                  const float *rate,
                                                  std::uint64_t n_rate,
                                                  float min_speed,
                                                  float max_speed,
                                                  const void *constraint_settings_ptr,
                                                  const void *chart_settings_ptr,
                                                  const void *env_ptr,
                                                  PhaseConstraintHandle *const *phase,
                                                  std::uint64_t n_phase,
                                                  std::uint64_t *out_rows);
    using GoalRegionDestroyFn = void (*)(GoalRegionHandle *);
    using GoalRegionSetSeedFn = void (*)(GoalRegionHandle *, const float *seed, float spread);
    using GoalRegionSampleFn = std::int32_t (*)(GoalRegionHandle *, SamplerHandle *sampler, float *out_state);
    using GoalRegionVelocityFn = std::int32_t (*)(GoalRegionHandle *, const float *q, float speed, float *out);
    using GoalRegionImpulseFn = void (*)(GoalRegionHandle *, const float *state, float *out);
}  // namespace vamp::jit::ffi
