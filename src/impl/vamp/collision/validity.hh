#pragma once

#include <vamp/collision/shapes.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/sphere_capsule.hh>
#include <vamp/collision/sphere_cuboid.hh>
#include <vamp/collision/sphere_heightfield.hh>
#include <vamp/collision/math.hh>

#include <limits>
#include <optional>

namespace vamp
{
    template <
        typename VectorDataT,
        typename ArgT1,
        typename ArgT2,
        typename ArgT3,
        typename ArgT4,
        typename ArgT5,
        typename ArgT6,
        typename ArgT7,
        typename ArgT8>
    inline constexpr auto sphere_sphere_self_collision(
        ArgT1 ax_,
        ArgT2 ay_,
        ArgT3 az_,
        ArgT4 ar_,
        ArgT5 bx_,
        ArgT6 by_,
        ArgT7 bz_,
        ArgT8 br_) noexcept -> bool
    {
        auto ax = static_cast<VectorDataT>(ax_);
        auto ay = static_cast<VectorDataT>(ay_);
        auto az = static_cast<VectorDataT>(az_);
        auto ar = static_cast<VectorDataT>(ar_);
        auto bx = static_cast<VectorDataT>(bx_);
        auto by = static_cast<VectorDataT>(by_);
        auto bz = static_cast<VectorDataT>(bz_);
        auto br = static_cast<VectorDataT>(br_);

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        return not collision::sphere_sphere_sql2(ax, ay, az, ar, bx, by, bz, br).test_zero();
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto sphere_environment_in_collision(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                return true;
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                return true;
            }
        }

        const std::array<DataT, 3> positions = {sx, sy, sz};
        for (const auto &pc : e.pointclouds)
        {
            if (pc.collides_simd(positions, sr))
            {
                return true;
            }
        }

        return false;
    }

    // Signed distance from the query sphere (sx, sy, sz, sr) to its nearest primitive in
    // `e`, plus the (unit) gradient of that distance w.r.t. the query's center -- the
    // differentiable counterpart of sphere_environment_in_collision above, used to push a
    // representative link sphere out of collision (see
    // vamp::planning::constraint::EnvironmentCollisionConstraint). Point clouds and
    // attachments aren't included (no gradient-producing kernel exists for them yet); a
    // query with no primitives at all, or only point clouds/attachments, returns a distance
    // of +infinity with an arbitrary (straight up) gradient.
    //
    // Unlike sphere_environment_in_collision, this does not early-exit via each shape's
    // sorted `min_distance` broad-phase: pruning would need to be sound per-SIMD-lane
    // (different lanes can have different nearest primitives), which the existing
    // broad-phase early-`break` was never designed to guarantee, so every primitive is
    // checked to keep this exact.
    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline auto sphere_environment_min_distance_gradient(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_,
        DataT eps = DataT(1e-6F)) noexcept -> collision::DistanceGradient<DataT>
    {
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);

        DataT best_distance(std::numeric_limits<float>::max());
        DataT best_gx(0.F);
        DataT best_gy(0.F);
        DataT best_gz(1.F);

        const auto consider = [&](const collision::DistanceGradient<DataT> &cand)
        {
            const auto better = cand.distance < best_distance;
            best_gx = best_gx.blend(cand.gx, better);
            best_gy = best_gy.blend(cand.gy, better);
            best_gz = best_gz.blend(cand.gz, better);
            best_distance = best_distance.blend(cand.distance, better);
        };

        for (const auto &es : e.spheres)
        {
            consider(collision::sphere_sphere_gradient(es, sx, sy, sz, sr, eps));
        }

        for (const auto &ec : e.capsules)
        {
            consider(collision::sphere_capsule_gradient(ec, sx, sy, sz, sr, eps));
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            consider(collision::sphere_z_aligned_capsule_gradient(ec, sx, sy, sz, sr, eps));
        }

        for (const auto &ec : e.cuboids)
        {
            consider(collision::sphere_cuboid_gradient(ec, sx, sy, sz, sr, eps));
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            consider(collision::sphere_z_aligned_cuboid_gradient(ec, sx, sy, sz, sr, eps));
        }

        for (const auto &eh : e.heightfields)
        {
            consider(collision::sphere_heightfield_gradient(eh, sx, sy, sz, sr));
        }

        return collision::DistanceGradient<DataT>{best_distance, best_gx, best_gy, best_gz};
    }

    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline auto sphere_environment_get_collisions(
        const collision::Environment<DataT> &e,  //
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> std::vector<std::string>
    {
        std::vector<std::string> objects;

        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        const auto max_extent = collision::sqrt(collision::dot_3(sx, sy, sz, sx, sy, sz)) + sr;

        for (const auto &es : e.spheres)
        {
            const auto diff = es.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_sphere_sql2(es, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(es.name);
            }
        }

        for (const auto &ec : e.capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_capsules)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_capsule(ec, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        const auto rsq = sr * sr;
        for (const auto &ec : e.cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &ec : e.z_aligned_cuboids)
        {
            const auto diff = ec.min_distance - max_extent;
            if (diff.test_zero())
            {
                break;
            }

            if (not collision::sphere_z_aligned_cuboid(ec, sx, sy, sz, rsq).test_zero())
            {
                objects.emplace_back(ec.name);
            }
        }

        for (const auto &eh : e.heightfields)
        {
            if (not collision::sphere_heightfield(eh, sx, sy, sz, sr).test_zero())
            {
                objects.emplace_back(eh.name);
            }
        }

        return objects;
    }

    template <typename DataT>
    inline constexpr auto attachment_environment_collision(const collision::Environment<DataT> &e) noexcept
        -> bool
    {
        for (const auto &attachment : e.attachments)
        {
            for (const auto &s : attachment.posed_spheres)
            {
                // HACK: The radius needs to be a float, and unfortunately the spheres assume
                // homogeneous DataT for storage
                // TODO: Fix the sphere representation to allow to store float radii even with vector
                // centers
                if (sphere_environment_in_collision(e, s.x, s.y, s.z, s.r[{0, 0}]))
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Checks a robot sphere against every attachment riding on end-effector `end_effector`.
    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto attachment_sphere_collision(
        const collision::Environment<DataT> &e,
        std::size_t end_effector,
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        // TODO: Figure out a way to avoid needing to upcast floats to vectors
        auto sx = static_cast<DataT>(sx_);
        auto sy = static_cast<DataT>(sy_);
        auto sz = static_cast<DataT>(sz_);
        auto sr = static_cast<DataT>(sr_);
        for (const auto &attachment : e.attachments)
        {
            if (attachment.end_effector != end_effector)
            {
                continue;
            }

            for (const auto &att_s : attachment.posed_spheres)
            {
                if (not collision::sphere_sphere_sql2(sx, sy, sz, sr, att_s.x, att_s.y, att_s.z, att_s.r)
                            .test_zero())
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Single-attachment form of the previous generator: checks against the attachments of end-effector 0.
    template <typename DataT, typename ArgT1, typename ArgT2, typename ArgT3, typename ArgT4>
    inline constexpr auto attachment_sphere_collision(
        const collision::Environment<DataT> &e,
        ArgT1 sx_,
        ArgT2 sy_,
        ArgT3 sz_,
        ArgT4 sr_) noexcept -> bool
    {
        return attachment_sphere_collision<DataT>(e, std::size_t{0}, sx_, sy_, sz_, sr_);
    }

    // Checks attachments on different end-effectors against each other; attachments riding
    // on the same end-effector move rigidly together and are never tested.
    template <typename DataT>
    inline constexpr auto attachment_attachment_collision(const collision::Environment<DataT> &e) noexcept
        -> bool
    {
        for (auto i = 0U; i < e.attachments.size(); ++i)
        {
            const auto &a = e.attachments[i];
            for (auto j = i + 1; j < e.attachments.size(); ++j)
            {
                const auto &b = e.attachments[j];
                if (a.end_effector == b.end_effector)
                {
                    continue;
                }

                for (const auto &sa : a.posed_spheres)
                {
                    for (const auto &sb : b.posed_spheres)
                    {
                        if (not collision::sphere_sphere_sql2(
                                    sa.x, sa.y, sa.z, sa.r, sb.x, sb.y, sb.z, sb.r)
                                    .test_zero())
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }
}  // namespace vamp
