#pragma once

#include <vamp/jit/ffi.hh>
#include <vamp/jit/stub_gen.hh>
#include <vamp/planning/planner.hh>

#include <cricket/jit/compiler.hh>
#include <cricket/jit/session.hh>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace vamp::jit
{
    struct LoadOptions
    {
        std::string robot_source;
        std::string robot_name;
        std::size_t dimension;
        std::size_t rake = 8;
        std::size_t resolution = 32;

        std::vector<vamp::planning::Planner> planners;

        RobotCapabilities capabilities;

        cricket::jit::CompileOptions compile_options;
    };

    auto default_load_options() -> LoadOptions;

    struct RobotOps
    {
        struct PlannerEntry
        {
            ffi::SolveFn solve{nullptr};
            ffi::SolveMultiFn solve_multi{nullptr};
            ffi::SolveConstrainedFn solve_constrained{nullptr};
            ffi::SolveMultiConstrainedFn solve_multi_constrained{nullptr};

            // Flask sibling only: phase-constrained and chart-constrained solves.
            ffi::SolvePhaseFn solve_phase{nullptr};
            ffi::SolveMultiPhaseFn solve_multi_phase{nullptr};
            ffi::SolveChartFn solve_chart{nullptr};
            ffi::SolveMultiChartFn solve_multi_chart{nullptr};
        };

        std::array<PlannerEntry, vamp::planning::N_PLANNERS> planners{};

        // Constraint entry points; factories are only non-null for capabilities present
        // in the generated robot source.
        struct ConstraintOps
        {
            ffi::ConstraintDestroyFn destroy{nullptr};
            ffi::ConstraintProjectFn project{nullptr};
            ffi::ConstraintSatisfiedFn satisfied{nullptr};
            ffi::SimplifyConstrainedFn simplify_constrained{nullptr};

            ffi::TaskSpaceConstraintNewFn task_space_new{nullptr};
            ffi::BimanualTaskSpaceConstraintNewFn bimanual_task_space_new{nullptr};
            ffi::ClosedLoopConstraintNewFn closed_loop_new{nullptr};
            ffi::CoMConstraintNewFn com_new{nullptr};
            ffi::TwistConstraintNewFn twist_new{nullptr};
            ffi::LeadScrewConstraintNewFn lead_screw_new{nullptr};
            ffi::LeadScrewLevelConstraintNewFn lead_screw_level_new{nullptr};
        };

        ConstraintOps constraint{};

        // Flask entry points, populated only on the flask sibling robot; chart entries
        // additionally require a manifold-defining kernel on the ambient robot.
        struct FlaskOps
        {
            ffi::PhaseDestroyFn phase_destroy{nullptr};
            ffi::PhaseKineticEnergyNewFn phase_kinetic_energy_new{nullptr};
            ffi::PhaseEEFSpeedNewFn phase_eef_speed_new{nullptr};
            ffi::PhaseSatisfiedFn phase_satisfied{nullptr};
            ffi::PhaseVelocityScaleFn phase_velocity_scale{nullptr};
            ffi::SamplerKeShapedFn sampler_ke_shaped{nullptr};
            ffi::SimplifyPhaseFn simplify_phase{nullptr};

            ffi::FlaskOptimalTimeFn optimal_time{nullptr};
            ffi::FlaskCostFn cost{nullptr};
            ffi::FlaskCostGradFn cost_grad{nullptr};
            ffi::FlaskEvalFn eval{nullptr};
            ffi::FlaskTorquesFn torques{nullptr};
            ffi::FlaskKineticEnergyFn kinetic_energy{nullptr};
            ffi::FlaskEEFVelocityFn eef_velocity{nullptr};
            ffi::FlaskNEndEffectorsFn n_end_effectors{nullptr};
            ffi::FlaskFlatDimensionFn flat_dimension{nullptr};
            ffi::FlaskRhoGetFn rho_get{nullptr};
            ffi::FlaskRhoSetFn rho_set{nullptr};
            ffi::FlaskLimitsFn velocity_limits{nullptr};
            ffi::FlaskLimitsFn effort_limits{nullptr};

            ffi::ChartProjectFn chart_project{nullptr};
            ffi::ChartSatisfiedFn chart_satisfied{nullptr};
            ffi::ChartDebugFn chart_debug{nullptr};
            ffi::ChartLiftEdgeFn chart_lift_edge{nullptr};
            ffi::SimplifyChartFn simplify_chart{nullptr};
        };

        FlaskOps flask{};

        ffi::ResultMetaFn result_meta{nullptr};
        ffi::ResultCopyWaypointFn result_copy_waypoint{nullptr};
        ffi::ResultDestroyFn result_destroy{nullptr};
        ffi::ResultSizesFn result_sizes{nullptr};

        ffi::SimplifyFn simplify{nullptr};

        ffi::SamplerHaltonFn sampler_halton{nullptr};
        ffi::SamplerXorshiftFn sampler_xorshift{nullptr};
        ffi::SamplerResetFn sampler_reset{nullptr};
        ffi::SamplerSkipFn sampler_skip{nullptr};
        ffi::SamplerNextFn sampler_next{nullptr};
        ffi::SamplerDestroyFn sampler_destroy{nullptr};

        ffi::DebugFn debug{nullptr};
        ffi::DebugDestroyFn debug_destroy{nullptr};
        ffi::EefkFn eefk{nullptr};
        ffi::FkFn fk{nullptr};
        ffi::ValidateFn validate{nullptr};
        ffi::ValidateMotionFn validate_motion{nullptr};
        ffi::CfgDistanceFn cfg_distance{nullptr};
        ffi::CfgInterpolateFn cfg_interpolate{nullptr};
        ffi::FilterPointcloudFn filter_pointcloud{nullptr};

        ffi::PhsNewFn phs_new{nullptr};
        ffi::PhsDestroyFn phs_destroy{nullptr};
        ffi::PhsSetDiameterFn phs_set_diameter{nullptr};
        ffi::PhsTransformFn phs_transform{nullptr};
        ffi::SamplerPhsFn sampler_phs{nullptr};
    };

    class DynamicRobot
    {
    public:
        explicit DynamicRobot(
            const LoadOptions &opts,
            std::shared_ptr<cricket::jit::DiskObjectCache> cache = nullptr);
        ~DynamicRobot();

        DynamicRobot(const DynamicRobot &) = delete;
        DynamicRobot &operator=(const DynamicRobot &) = delete;

        auto ops() const -> const RobotOps &
        {
            return ops_;
        }

        auto has_planner(vamp::planning::Planner p) const -> bool
        {
            return ops_.planners[static_cast<std::size_t>(p)].solve != nullptr;
        }

        auto capabilities() const -> const RobotCapabilities &
        {
            return capabilities_;
        }

        auto dimension() const -> std::size_t
        {
            return dimension_;
        }

        auto rake() const -> std::size_t
        {
            return rake_;
        }

        auto n_spheres() const -> std::size_t
        {
            return n_spheres_;
        }

        auto space_measure() const -> float
        {
            return space_measure_;
        }

        auto min_radius() const -> float
        {
            return min_radius_;
        }

        auto max_radius() const -> float
        {
            return max_radius_;
        }

        auto joint_names() const -> const std::vector<std::string> &
        {
            return joint_names_;
        }

        auto upper_bounds() const -> const std::vector<float> &
        {
            return upper_bounds_;
        }

        auto lower_bounds() const -> const std::vector<float> &
        {
            return lower_bounds_;
        }

        // The flask (flat-system) sibling robot, sharing this robot's JIT session; null
        // unless the recipe had a flask block. The sibling itself has no sibling.
        auto flask_sibling() const -> const std::shared_ptr<DynamicRobot> &
        {
            return flask_;
        }

        // Identity of the ambient robot this flask sibling was derived from (null on
        // ambient robots). Used only for pointer comparison when validating that chart
        // constraints were created on the ambient robot; any live constraint keeps the
        // ambient robot alive through its own shared_ptr.
        auto ambient() const -> const DynamicRobot *
        {
            return ambient_;
        }

    private:
        struct FlaskSiblingTag
        {
        };

        DynamicRobot(FlaskSiblingTag, const DynamicRobot &ambient, const LoadOptions &opts);

        // Entry points and metadata common to both the ambient and flask stubs,
        // resolved under the given symbol name (robot_name or robot_name + "_flask").
        void resolve_base(const std::string &r);

        std::size_t dimension_;
        std::size_t rake_;
        RobotCapabilities capabilities_;
        std::shared_ptr<cricket::jit::JitSession> session_;
        RobotOps ops_{};
        std::shared_ptr<DynamicRobot> flask_;
        const DynamicRobot *ambient_{nullptr};

        std::size_t n_spheres_{0};
        float space_measure_{0.0F};
        float min_radius_{0.0F};
        float max_radius_{0.0F};
        std::vector<std::string> joint_names_;
        std::vector<float> upper_bounds_;
        std::vector<float> lower_bounds_;
    };
}  // namespace vamp::jit
