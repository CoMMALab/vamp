#pragma once

#if defined(VAMP_JAX_ROBOT) && defined(VAMP_JAX_ROBOT_NAME)

#include <nanobind/nanobind.h>
#include <xla/ffi/api/c_api.h>
#include <xla/ffi/api/ffi.h>

#include <vamp/bindings/jax/jax.hh>

#include <vamp/robots/panda.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/validate.hh>

namespace nb = nanobind;
namespace ffi = xla::ffi;

namespace vamp::binding::jax
{
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

    template <typename Robot>
    inline auto validate_motion_pairwise_impl(
        ffi::Buffer<ffi::F32> a,
        ffi::Buffer<ffi::F32> b,
        ffi::ResultBuffer<ffi::PRED> r) noexcept -> ffi::Error
    {
        const auto a_d = a.dimensions();
        const auto b_d = b.dimensions();

        const auto *a_data = a.typed_data();
        const auto *b_data = b.typed_data();
        auto *r_data = r->typed_data();

        EnvironmentVector env;

        for (auto i = 0U; i < a_d[0]; ++i)
        {
            const typename Robot::Configuration a_c(&a_data[i * 7], false);
            for (auto j = 0U; j < b_d[0]; ++j)
            {
                const typename Robot::Configuration b_c(&b_data[j * 7], false);
                r_data[i * b_d[0] + j] = vamp::planning::validate_motion<Robot, rake, 2>(a_c, b_c, env);
            }
        }

        return ffi::Error::Success();
    }

    template <typename T>
    nb::capsule encapsulate_ffi(T *fn)
    {
        static_assert(
            std::is_invocable_r_v<XLA_FFI_Error *, T, XLA_FFI_CallFrame *>,
            "Encapsulated function must be and XLA FFI handler");
        return nb::capsule(reinterpret_cast<void *>(fn));
    }
}  // namespace vamp::binding::jax

#define VAMP_PASTE(A, B) A##B
#define VAMP_XSTRING(A) VAMP_STRING(A)
#define VAMP_STRING(A) #A
#define XLA_SYMBOL_NAME(robot_name) VAMP_PASTE(validate_motion_pairwise_, robot_name)

XLA_FFI_DEFINE_HANDLER_SYMBOL(
    XLA_SYMBOL_NAME(VAMP_JAX_ROBOT_NAME),
    vamp::binding::jax::validate_motion_pairwise_impl<VAMP_JAX_ROBOT>,
    ffi::Ffi::Bind()
        .Arg<ffi::Buffer<ffi::F32>>()   // config buffer a
        .Arg<ffi::Buffer<ffi::F32>>()   // config buffer b
        .Ret<ffi::Buffer<ffi::PRED>>()  // pairwise validity
);

template <typename Robot>
void init_robot_jax(nanobind::module_ &pymodule)
{
    auto submodule = pymodule.def_submodule(Robot::name, "Robot-specific JAX submodule");

    submodule.def(
        "registrations",
        []()
        {
            nb::dict registrations;
            registrations[VAMP_XSTRING(XLA_SYMBOL_NAME(VAMP_JAX_ROBOT_NAME))] =
                vamp::binding::jax::encapsulate_ffi(XLA_SYMBOL_NAME(VAMP_JAX_ROBOT_NAME));
            return registrations;
        });
}

#endif
