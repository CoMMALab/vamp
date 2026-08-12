#pragma once

#include <cstring>
#include <vector>

#include <vamp/bindings/python/array_helpers.hh>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

namespace vamp::binding
{
    namespace nb = nanobind;

    // Mirrors InputOps<Robot, Derived> (robot_helper.hh), but against a task-space `Space`
    // (e.g. Robot::ParameterizedSpace) rather than the robot's own ambient Configuration.
    template <typename Robot, typename Space, typename Derived>
    struct SpaceInputOps
    {
        using State = typename Space::State;
        using StateArray = typename Space::StateArray;

        template <typename Type>
        inline static auto to(const Type &a) -> State
        {
            return State(Derived::array(a));
        }
    };

    template <typename Robot, typename Space>
    struct SpaceNDArrayInput : SpaceInputOps<Robot, Space, SpaceNDArrayInput<Robot, Space>>
    {
        using Type =
            nanobind::ndarray<FloatT, nanobind::numpy, nanobind::shape<Space::dimension>, nanobind::device::cpu>;

        inline static auto from(const typename Space::State &s) -> Type
        {
            auto s_arr = s.to_array();
            return make_ndarray<Type, 1>(s_arr.data(), {Space::dimension});
        }

        inline static auto array(const Type &a) -> typename Space::StateArray
        {
            typename Space::StateArray s;
            std::vector<float> scratch;
            const auto *ptr = as_flat_1d(a, Space::dimension, scratch, "state");
            std::memcpy(s.data(), ptr, Space::dimension * sizeof(float));
            return s;
        }
    };

    template <typename Robot, typename Space>
    struct SpaceArrayInput : SpaceInputOps<Robot, Space, SpaceArrayInput<Robot, Space>>
    {
        using Type = typename Space::StateArray;

        inline static auto from(const typename Space::State &s) -> Type
        {
            Type a;
            auto s_arr = s.to_array();
            for (auto i = 0U; i < Space::dimension; ++i)
            {
                a[i] = s_arr[i];
            }
            return a;
        }

        inline static auto array(const Type &a) -> const Type &
        {
            return a;
        }
    };
}  // namespace vamp::binding
