#pragma once

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/parameterized_space_helper.hh>

#include <vamp/planning/constraints/task_space_informed_sampler.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/random/rng.hh>

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

namespace vamp::binding
{
    namespace nb_ = nanobind;
    namespace vp_ = vamp::planning;
    using namespace nb_::literals;

    // Binds the task-space planning entry points for Robot::ParameterizedSpace: rrtc
    // (the only local-planner-aware planner that's actually solvable here -- see below),
    // plus resolve/eefs_collision_free/compute_mid_pose/shortcut and the samplers.
    // register_planner (api_binder.hh) already dispatches to Traits::solve_single/
    // solve_multi generically, with no assumption that Space == Robot, so it's reused
    // unmodified here.
    //
    // aorrtc/grrtstar are deliberately NOT registered: both pull in PHS-ellipsoid
    // informed sampling once a first solution is found, which needs Space::space_measure()
    // (planners/grrtstar.hh, aorrtc.hh) -- a well-defined quantity for a joint/configuration
    // space (product of joint ranges) but not a natural one for a task-space region, whose
    // bounds can be one-sided or unbounded and which isn't Euclidean in the same sense.
    // Neither existing task-space demo (scripts/cpp/iiwa_maze_solver.cc,
    // rby1_task_space_planner.cc) uses anything but rrtc; inventing a measure just to
    // satisfy the template isn't worth it unless a real use case needs the anytime/optimal
    // behavior AORRTC/GRRTSTAR provide. prm/fcit are excluded for a different reason: they
    // don't take a local planner at all, so they can't resolve task-space states into the
    // ambient robot to collision-check them.
    template <typename Traits, typename Target>
    inline void bind_parameterized_space_methods(Target &t)
    {
        t.def(
            "resolve",
            &Traits::resolve,
            "state"_a,
            "IK-resolve a task-space state into the ambient robot's configuration. Returns "
            "(valid, ambient_configuration).");
        t.def(
            "eefs_collision_free",
            &Traits::eefs_collision_free,
            "state"_a,
            "environment"_a,
            "IK-free prefilter: whether the end-effector geometry (and any attachments) at "
            "this state is free of environment/self collision. Not a substitute for a full "
            "check on the resolved ambient configuration.");
        // Only present on task spaces with a mid-pose concept (e.g. RBY1's bimanual
        // t_mid_left/t_mid_right hand offsets); a single-arm space like IiwaMarker's eef
        // pose IS the task-space pose already, so it has nothing to derive here.
        if constexpr (has_compute_mid_pose_v<typename Traits::Space>)
        {
            t.def(
                "compute_mid_pose",
                &Traits::compute_mid_pose,
                "ambient_configuration"_a,
                "Derive the fixed hand offsets used by resolve() from a reference whole-body "
                "ambient configuration. Call this once before planning.");
        }
        // Only present on task spaces with a center-of-mass/support-polygon stability
        // concept (e.g. RBY1's mobile-base ground-contact polygon); see the same gate on
        // Traits::set_support_polygon (parameterized_space_helper.hh).
        if constexpr (vp_::constraint::detail::has_compute_com<typename Traits::Space, rake>::value)
        {
            t.def(
                "set_support_polygon",
                &Traits::set_support_polygon,
                "polygon_xy"_a,
                "Overwrite the static-stability support polygon the local planner checks the "
                "resolved ambient center of mass against, as a list of (x, y) vertices in the "
                "mobile base's local frame. Affects every subsequent planning call on this "
                "thread (thread-local, like compute_mid_pose's t_mid_left/t_mid_right) until "
                "set again.");
        }
        // Only present on task spaces with a GCP (self-motion-manifold branch selector)
        // concept (currently RBY1); see the same gate on Traits::set_gcp
        // (parameterized_space_helper.hh).
        if constexpr (has_set_gcp_v<typename Traits::Space>)
        {
            t.def(
                "set_gcp",
                &Traits::set_gcp,
                "left"_a,
                "right"_a,
                "Set the (elbow_sel, shoulder_sel, wrist_sel) self-motion-manifold branch "
                "each rainbow arm's IK resolves onto, one triple per arm. Affects every "
                "subsequent resolve()/planning call on this thread (thread-local) until set "
                "again.");
        }
        t.def(
            "shortcut",
            &Traits::shortcut,
            "path"_a,
            "environment"_a,
            "settings"_a = vp_::ShortcutSettings{},
            "Shortcut a solved task-space path in place. Every collapsed edge is "
            "revalidated (IK-resolve, eef-collision prefilter, fkcc/fkcc_attach), so this "
            "can only remove waypoints, never bypass validity. Returns whether anything "
            "changed.");

        t.def("halton", &Traits::make_halton, "Create a Halton sampler for this task space.");
        t.def(
            "xorshift",
            &Traits::make_xorshift,
            "seed"_a = std::uint64_t{0},
            "Create an XORShift sampler for this task space.");

        register_planner<Traits, vp_::Planner::RRTC, vp_::RRTCSettings>(t, "rrtc");
    }

    // Task-Space-Region sampler for single-end-effector task spaces whose State places the
    // eef pose (x, y, z, qx, qy, qz, qw) at indices [0, 7) -- i.e. Space::sample_dimension ==
    // 7, matching TaskSpaceInformedSampler's own layout assumption (see
    // task_space_informed_sampler.hh). Multi-end-effector / non-pose-first spaces (e.g.
    // RBY1::ParameterizedSpace, whose sample_dimension is 17) don't fit this layout and get
    // their own robot-specific sampler instead (e.g. rby1_sampler_binder.hh's
    // FixedBaseSampler). Must run after bind_sampler<Traits> has registered Traits::Sampler
    // (nanobind requires a subclass's base already bound).
    template <typename Traits>
    inline void bind_task_space_informed_sampler(nb_::module_ &m)
    {
        using Space = typename Traits::Space;
        if constexpr (Space::sample_dimension == 7)
        {
            using Robot = typename Traits::RobotType;
            using TSS = vp_::TaskSpaceInformedSampler<Robot, Space>;
            using Base = typename Traits::Sampler;
            using Transform = typename TSS::Transform;
            using Bound = typename TSS::Bound;
            using Env = typename Traits::Env;

            nb_::class_<TSS, Base>(
                m,
                "TaskSpaceInformedSampler",
                "Samples task-space states directly inside a Task Space Region (an "
                "end-effector pose bound relative to a reference frame) instead of rejecting "
                "samples on a downstream IK/collision check. See TaskSpaceConstraint for the "
                "shared (eef_to_offset, world_to_reference, lower, upper) bound convention.")
                .def(
                    nb_::init<
                        const Transform &,
                        const Transform &,
                        const Bound &,
                        const Bound &,
                        Env,
                        std::shared_ptr<Base>>(),
                    "eef_to_offset"_a,
                    "world_to_reference"_a,
                    "lower"_a,
                    "upper"_a,
                    "environment"_a,
                    "inner"_a,
                    "lower/upper are (dx, dy, dz, rx, ry, rz) translation + so(3) log-map "
                    "rotation bounds of the offset frame relative to the reference frame. "
                    "`inner` supplies the Halton/XORShift stream this resamples on top of.");
        }
    }
}  // namespace vamp::binding
