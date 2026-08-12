#pragma once

#include <vamp/bindings/python/api_binder.hh>
#include <vamp/bindings/python/parameterized_space_helper.hh>

#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/aorrtc_settings.hh>
#include <vamp/planning/planners/grrtstar_settings.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

#include <nanobind/nanobind.h>

namespace vamp::binding
{
    namespace nb_ = nanobind;
    namespace vp_ = vamp::planning;
    using namespace nb_::literals;

    // Binds the task-space planning entry points for Robot::ParameterizedSpace: rrtc/
    // aorrtc/grrtstar (the local-planner-aware planners; prm/fcit don't take a local
    // planner and so can't resolve task-space states into the ambient robot), plus
    // resolve/eefs_collision_free/compute_mid_pose/shortcut and the samplers.
    // register_planner (api_binder.hh) already dispatches to Traits::solve_single/
    // solve_multi generically, with no assumption that Space == Robot, so it's reused
    // unmodified here.
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
        t.def(
            "compute_mid_pose",
            &Traits::compute_mid_pose,
            "ambient_configuration"_a,
            "Derive the fixed hand offsets used by resolve() from a reference whole-body "
            "ambient configuration. Call this once before planning.");
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
        register_planner<Traits, vp_::Planner::AORRTC, vp_::AORRTCSettings>(t, "aorrtc");
        register_planner<Traits, vp_::Planner::GRRTSTAR, vp_::GRRTStarSettings>(t, "grrtstar");
    }
}  // namespace vamp::binding
