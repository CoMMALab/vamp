#include <vamp/jit/dynamic_robot.hh>
#include <vamp/jit/build_paths.hh>

#include <cricket/jit/compiler.hh>

#include <llvm/Support/Error.h>

#include <stdexcept>
#include <utility>

namespace vamp::jit
{
    namespace
    {
        auto lookup(cricket::jit::JitSession &session, const std::string &symbol) -> void *
        {
            auto addr = session.lookup(symbol);
            if (not addr)
            {
                throw std::runtime_error(
                    "vamp::jit: cannot resolve symbol '" + symbol + "': " + llvm::toString(addr.takeError()));
            }
            return addr->toPtr<void *>();
        }

        template <typename F>
        auto resolve(cricket::jit::JitSession &s, vamp::planning::Planner p, std::string_view suffix) -> F
        {
            return reinterpret_cast<F>(lookup(s, planner_symbol(p, suffix)));
        }

        template <typename F>
        auto resolve(cricket::jit::JitSession &s, const std::string &robot, std::string_view suffix) -> F
        {
            return reinterpret_cast<F>(lookup(s, robot_symbol(robot, suffix)));
        }

        // Flask planner renders use planner_name = "flask_" + name, so their symbols are
        // vamp_jit_flask_<planner>_<suffix>.
        template <typename F>
        auto
        resolve_flask(cricket::jit::JitSession &s, vamp::planning::Planner p, std::string_view suffix) -> F
        {
            return reinterpret_cast<F>(lookup(
                s,
                "vamp_jit_flask_" + std::string(vamp::planning::planner_name(p)) + "_" +
                    std::string(suffix)));
        }
    }  // namespace

    auto default_load_options() -> LoadOptions
    {
        LoadOptions opts;
        opts.compile_options.include_dirs.assign(paths::include_dirs.begin(), paths::include_dirs.end());
        opts.compile_options.system_include_dirs.assign(
            paths::system_include_dirs.begin(), paths::system_include_dirs.end());
        opts.compile_options.defines.assign(paths::defines.begin(), paths::defines.end());
        opts.compile_options.extra_flags.assign(paths::extra_flags.begin(), paths::extra_flags.end());
        return opts;
    }

    DynamicRobot::DynamicRobot(const LoadOptions &opts, std::shared_ptr<cricket::jit::DiskObjectCache> cache)
      : dimension_(opts.dimension), rake_(opts.rake), capabilities_(opts.capabilities)
    {
        auto source = generate_stub_source(
            opts.robot_source, opts.robot_name, opts.rake, opts.resolution, opts.planners, opts.capabilities);

        session_ = std::make_shared<cricket::jit::JitSession>(cache);

        std::unique_ptr<llvm::MemoryBuffer> cached_obj;
        if (cache)
        {
            cached_obj = cache->load_object(cricket::jit::hash_source(source, opts.compile_options));
        }

        if (cached_obj)
        {
            if (auto err = session_->add_object_file(std::move(cached_obj)))
            {
                throw std::runtime_error(
                    "vamp::jit: add_object_file failed: " + llvm::toString(std::move(err)));
            }
        }
        else
        {
            cricket::jit::ClangCompiler compiler;
            auto module = compiler.compile(source, opts.compile_options);
            if (auto err = session_->add_module(std::move(module)))
            {
                throw std::runtime_error("vamp::jit: add_module failed: " + llvm::toString(std::move(err)));
            }
        }

        for (auto p : opts.planners)
        {
            auto &e = ops_.planners[static_cast<std::size_t>(p)];
            e.solve = resolve<ffi::SolveFn>(*session_, p, "solve");
            e.solve_multi = resolve<ffi::SolveMultiFn>(*session_, p, "solve_multi");

            if (opts.capabilities.any() and vamp::planning::planner_descriptor(p).local_planner)
            {
                e.solve_constrained = resolve<ffi::SolveConstrainedFn>(*session_, p, "solve_constrained");
                e.solve_multi_constrained =
                    resolve<ffi::SolveMultiConstrainedFn>(*session_, p, "solve_multi_constrained");
            }
        }

        const auto &r = opts.robot_name;

        resolve_base(r);

        const auto &caps = opts.capabilities;
        if (caps.any())
        {
            ops_.constraint.destroy = resolve<ffi::ConstraintDestroyFn>(*session_, r, "constraint_destroy");
            ops_.constraint.project = resolve<ffi::ConstraintProjectFn>(*session_, r, "constraint_project");
            ops_.constraint.satisfied =
                resolve<ffi::ConstraintSatisfiedFn>(*session_, r, "constraint_satisfied");
            ops_.constraint.simplify_constrained =
                resolve<ffi::SimplifyConstrainedFn>(*session_, r, "simplify_constrained");
        }

        if (caps.constraints)
        {
            ops_.constraint.task_space_new =
                resolve<ffi::TaskSpaceConstraintNewFn>(*session_, r, "constraint_task_space_new");
            if (caps.n_eef > 1)
            {
                ops_.constraint.bimanual_task_space_new = resolve<ffi::BimanualTaskSpaceConstraintNewFn>(
                    *session_, r, "constraint_bimanual_task_space_new");
            }
        }

        if (caps.closed_loops)
        {
            ops_.constraint.closed_loop_new =
                resolve<ffi::ClosedLoopConstraintNewFn>(*session_, r, "constraint_closed_loop_new");
        }

        if (caps.com)
        {
            ops_.constraint.com_new = resolve<ffi::CoMConstraintNewFn>(*session_, r, "constraint_com_new");
        }

        if (caps.twist)
        {
            ops_.constraint.twist_new =
                resolve<ffi::TwistConstraintNewFn>(*session_, r, "constraint_twist_new");
            ops_.constraint.lead_screw_new =
                resolve<ffi::LeadScrewConstraintNewFn>(*session_, r, "constraint_lead_screw_new");
        }

        if (caps.lead_screw)
        {
            ops_.constraint.lead_screw_level_new =
                resolve<ffi::LeadScrewLevelConstraintNewFn>(*session_, r, "constraint_lead_screw_level_new");
        }

        if (caps.flask)
        {
            flask_ = std::shared_ptr<DynamicRobot>(new DynamicRobot(FlaskSiblingTag{}, *this, opts));
        }
    }

    DynamicRobot::DynamicRobot(FlaskSiblingTag, const DynamicRobot &ambient, const LoadOptions &opts)
      : dimension_(2 * opts.dimension),
        rake_(opts.rake),
        capabilities_(opts.capabilities),
        session_(ambient.session_),
        ambient_(&ambient)
    {
        const auto &caps = opts.capabilities;
        const auto r = opts.robot_name + "_flask";

        for (auto p : opts.planners)
        {
            auto &e = ops_.planners[static_cast<std::size_t>(p)];
            e.solve = resolve_flask<ffi::SolveFn>(*session_, p, "solve");
            e.solve_multi = resolve_flask<ffi::SolveMultiFn>(*session_, p, "solve_multi");

            if (vamp::planning::planner_descriptor(p).local_planner)
            {
                e.solve_phase = resolve_flask<ffi::SolvePhaseFn>(*session_, p, "solve_phase");
                e.solve_multi_phase =
                    resolve_flask<ffi::SolveMultiPhaseFn>(*session_, p, "solve_multi_phase");

                if (caps.chart())
                {
                    e.solve_chart = resolve_flask<ffi::SolveChartFn>(*session_, p, "solve_chart");
                    e.solve_multi_chart =
                        resolve_flask<ffi::SolveMultiChartFn>(*session_, p, "solve_multi_chart");
                }
            }
        }

        resolve_base(r);

        auto &f = ops_.flask;
        f.phase_destroy = resolve<ffi::PhaseDestroyFn>(*session_, r, "phase_destroy");
        f.phase_kinetic_energy_new =
            resolve<ffi::PhaseKineticEnergyNewFn>(*session_, r, "phase_kinetic_energy_new");
        f.phase_eef_speed_new = resolve<ffi::PhaseEEFSpeedNewFn>(*session_, r, "phase_eef_speed_new");
        f.phase_satisfied = resolve<ffi::PhaseSatisfiedFn>(*session_, r, "phase_satisfied");
        f.phase_velocity_scale = resolve<ffi::PhaseVelocityScaleFn>(*session_, r, "phase_velocity_scale");
        f.sampler_ke_shaped = resolve<ffi::SamplerKeShapedFn>(*session_, r, "sampler_ke_shaped");
        f.simplify_phase = resolve<ffi::SimplifyPhaseFn>(*session_, r, "simplify_phase");

        f.optimal_time = resolve<ffi::FlaskOptimalTimeFn>(*session_, r, "optimal_time");
        f.cost = resolve<ffi::FlaskCostFn>(*session_, r, "cost");
        f.cost_grad = resolve<ffi::FlaskCostGradFn>(*session_, r, "cost_grad");
        f.eval = resolve<ffi::FlaskEvalFn>(*session_, r, "eval");
        f.torques = resolve<ffi::FlaskTorquesFn>(*session_, r, "torques");
        f.kinetic_energy = resolve<ffi::FlaskKineticEnergyFn>(*session_, r, "kinetic_energy");
        f.eef_velocity = resolve<ffi::FlaskEEFVelocityFn>(*session_, r, "eef_velocity");
        f.n_end_effectors = resolve<ffi::FlaskNEndEffectorsFn>(*session_, r, "n_end_effectors");
        f.flat_dimension = resolve<ffi::FlaskFlatDimensionFn>(*session_, r, "flat_dimension");
        f.rho_get = resolve<ffi::FlaskRhoGetFn>(*session_, r, "rho_get");
        f.rho_set = resolve<ffi::FlaskRhoSetFn>(*session_, r, "rho_set");
        f.velocity_limits = resolve<ffi::FlaskLimitsFn>(*session_, r, "velocity_limits");
        f.effort_limits = resolve<ffi::FlaskLimitsFn>(*session_, r, "effort_limits");

        if (caps.chart())
        {
            f.chart_project = resolve<ffi::ChartProjectFn>(*session_, r, "chart_project");
            f.chart_satisfied = resolve<ffi::ChartSatisfiedFn>(*session_, r, "chart_satisfied");
            f.chart_debug = resolve<ffi::ChartDebugFn>(*session_, r, "chart_debug");
            f.chart_lift_edge = resolve<ffi::ChartLiftEdgeFn>(*session_, r, "chart_lift_edge");
            f.simplify_chart = resolve<ffi::SimplifyChartFn>(*session_, r, "simplify_chart");
        }
    }

    void DynamicRobot::resolve_base(const std::string &r)
    {
        ops_.result_meta = resolve<ffi::ResultMetaFn>(*session_, r, "result_meta");
        ops_.result_copy_waypoint = resolve<ffi::ResultCopyWaypointFn>(*session_, r, "result_copy_waypoint");
        ops_.result_destroy = resolve<ffi::ResultDestroyFn>(*session_, r, "result_destroy");
        ops_.result_sizes = resolve<ffi::ResultSizesFn>(*session_, r, "result_sizes");

        ops_.simplify = resolve<ffi::SimplifyFn>(*session_, r, "simplify");

        ops_.sampler_halton = resolve<ffi::SamplerHaltonFn>(*session_, r, "sampler_halton");
        ops_.sampler_xorshift = resolve<ffi::SamplerXorshiftFn>(*session_, r, "sampler_xorshift");
        ops_.sampler_reset = resolve<ffi::SamplerResetFn>(*session_, r, "sampler_reset");
        ops_.sampler_skip = resolve<ffi::SamplerSkipFn>(*session_, r, "sampler_skip");
        ops_.sampler_next = resolve<ffi::SamplerNextFn>(*session_, r, "sampler_next");
        ops_.sampler_destroy = resolve<ffi::SamplerDestroyFn>(*session_, r, "sampler_destroy");

        ops_.debug = resolve<ffi::DebugFn>(*session_, r, "debug");
        ops_.debug_destroy = resolve<ffi::DebugDestroyFn>(*session_, r, "debug_destroy");
        ops_.eefk = resolve<ffi::EefkFn>(*session_, r, "eefk");
        ops_.fk = resolve<ffi::FkFn>(*session_, r, "fk");
        ops_.validate = resolve<ffi::ValidateFn>(*session_, r, "validate");
        ops_.validate_motion = resolve<ffi::ValidateMotionFn>(*session_, r, "validate_motion");
        ops_.cfg_distance = resolve<ffi::CfgDistanceFn>(*session_, r, "cfg_distance");
        ops_.cfg_interpolate = resolve<ffi::CfgInterpolateFn>(*session_, r, "cfg_interpolate");
        ops_.filter_pointcloud =
            resolve<ffi::FilterPointcloudFn>(*session_, r, "filter_self_from_pointcloud");

        ops_.phs_new = resolve<ffi::PhsNewFn>(*session_, r, "phs_new");
        ops_.phs_destroy = resolve<ffi::PhsDestroyFn>(*session_, r, "phs_destroy");
        ops_.phs_set_diameter = resolve<ffi::PhsSetDiameterFn>(*session_, r, "phs_set_transverse_diameter");
        ops_.phs_transform = resolve<ffi::PhsTransformFn>(*session_, r, "phs_transform");
        ops_.sampler_phs = resolve<ffi::SamplerPhsFn>(*session_, r, "sampler_phs");

        auto space_measure_fn = resolve<ffi::SpaceMeasureFn>(*session_, r, "space_measure");
        auto min_max_fn = resolve<ffi::MinMaxRadiiFn>(*session_, r, "min_max_radii");
        auto n_spheres_fn = resolve<ffi::NSpheresFn>(*session_, r, "n_spheres");
        auto joint_names_fn = resolve<ffi::JointNamesFn>(*session_, r, "joint_names");
        auto upper_fn = resolve<ffi::BoundsFn>(*session_, r, "upper_bounds");
        auto lower_fn = resolve<ffi::BoundsFn>(*session_, r, "lower_bounds");

        space_measure_ = space_measure_fn();
        min_max_fn(&min_radius_, &max_radius_);
        n_spheres_ = n_spheres_fn();
        joint_names_fn(&joint_names_);

        upper_bounds_.resize(dimension_);
        lower_bounds_.resize(dimension_);
        upper_fn(upper_bounds_.data());
        lower_fn(lower_bounds_.data());
    }

    DynamicRobot::~DynamicRobot() = default;
}  // namespace vamp::jit
