// M2 scene-specialized collision kernel (design doc Component 1, levels 1-2:
// const-fold obstacle immediates + precomputed early-exit constants, loop unroll,
// type-pruned to spheres-only).
//
// The body mirrors sphere_environment_in_collision()'s spheres loop
// (collision/validity.hh:46-73) exactly -- same signature, same expressions --
// but over the constexpr scene arrays in scene_gen.hh. At -O3 clang const-folds
// every obstacle field into an immediate and unrolls the fixed-count loop. Being
// the same expressions over constant data, it is bit-exact with the generic
// kernel by construction (spheres-only scenes; other obstacle types pruned away).
#pragma once

#include <cstddef>

#include <vamp/collision/math.hh>
#include <vamp/collision/sphere_sphere.hh>

#include "scene_gen.hh"

namespace vamp::m2
{
    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline auto scene_in_collision_spec(ArgT1 sx_, ArgT2 sy_, ArgT3 sz_, ArgT4 sr_) noexcept -> bool
    {
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

#pragma clang loop unroll(full)
        for (std::size_t i = 0; i < scene_gen::N; ++i)
        {
            const auto diff = static_cast<DataT>(scene_gen::OBS_MIND[i]) - max_extent;
            if (diff.test_zero())
            {
                break;  // sorted ascending: all remaining obstacles are farther
            }

            if (not collision::sphere_sphere_sql2(
                        static_cast<DataT>(scene_gen::OBS_X[i]),
                        static_cast<DataT>(scene_gen::OBS_Y[i]),
                        static_cast<DataT>(scene_gen::OBS_Z[i]),
                        static_cast<DataT>(scene_gen::OBS_R[i]),
                        sx,
                        sy,
                        sz,
                        sr)
                        .test_zero())
            {
                return true;
            }
        }

        return false;
    }
}  // namespace vamp::m2
