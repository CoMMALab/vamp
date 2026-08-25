#pragma once

#include <array>
#include <memory>

#include <vamp/random/rby1_fixed_base_sampler.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/rby1.hh>

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>

// RBY1-specific: RBY1FixedBaseSampler hardcodes RBY1::ParameterizedSpace's State layout
// (base at indices [0, 4), t_mid_pose position at [12, 15)), so unlike
// TaskSpaceInformedSampler (parameterized_space_binder.hh) it cannot be bound generically
// from robot_helper.hh. Wired in only for the rby1 module via cmake/Python.cmake's
// VAMP_ROBOT_EXTRA_HEADER/VAMP_ROBOT_EXTRA_CALL, so this header is never included from a
// build lacking rby1.hh.
namespace vamp::binding
{
    namespace nb_ = nanobind;
    using namespace nb_::literals;

    // Called from the generated rby1.cc (see robot.cc.in) with the rby1 submodule that
    // init_robot<RBY1>() already populated with a `parameterized_space` submodule
    // (including its "RNG" == RNG<RBY1, RBY1::ParameterizedSpace> base class); this attaches
    // FixedBaseSampler onto that same submodule as a subclass of it.
    inline void bind_rby1_task_sampler(nb_::module_ &submodule)
    {
        using Robot = vamp::robots::RBY1;
        using Space = Robot::ParameterizedSpace;
        using Base = vamp::rng::RNG<Robot, Space>;
        using FBS = vamp::rng::RBY1FixedBaseSampler;

        auto param_submodule = nb_::cast<nb_::module_>(submodule.attr("parameterized_space"));

        nb_::class_<FBS, Base>(
            param_submodule,
            "FixedBaseSampler",
            "Wraps an inner ParameterizedSpace sampler and, on every draw, pins the mobile "
            "base at the origin and remaps the t_mid_pose position into a box of "
            "+/-position_half_range around start_position. Torso, psi_left/psi_right, and "
            "the t_mid_pose orientation are left exactly as the inner sampler draws them.")
            .def(
                nb_::init<std::shared_ptr<Base>, std::array<float, 3>, float>(),
                "inner"_a,
                "start_position"_a,
                "position_half_range"_a = 0.4F);
    }
}  // namespace vamp::binding
