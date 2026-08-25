#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vector>
#include <vamp/collision/shapes.hh>
#include <vamp/collision/capt.hh>
#include <vamp/collision/mvt.hh>
#include <vamp/collision/attachments.hh>

namespace vamp::collision
{
    template <typename DataT>
    struct Environment
    {
        std::vector<Sphere<DataT>> spheres;
        std::vector<Capsule<DataT>> capsules;
        std::vector<Capsule<DataT>> z_aligned_capsules;
        std::vector<Cylinder<DataT>> cylinders;
        std::vector<Cuboid<DataT>> cuboids;
        std::vector<Cuboid<DataT>> z_aligned_cuboids;
        std::vector<HeightField<DataT>> heightfields;
        std::vector<CAPT> pointclouds;
        std::vector<MVT> pointclouds_mvt;
        std::vector<Attachment<DataT>> attachments;

        // Per-query self-collision pair partition (m71): indices into the robot's static
        // cc_self_pairs that survive query-scoped pruning. Empty => check all pairs (default,
        // backward-compatible). Only the compact-collision kernels (digit, r2c6) consult this.
        std::vector<unsigned int> active_self_pairs;

        Environment() = default;

        template <typename OtherDataT>
        explicit Environment(const Environment<OtherDataT> &other)
          : spheres(other.spheres.begin(), other.spheres.end())
          , capsules(other.capsules.begin(), other.capsules.end())
          , z_aligned_capsules(other.z_aligned_capsules.begin(), other.z_aligned_capsules.end())
          , cylinders(other.cylinders.begin(), other.cylinders.end())
          , cuboids(other.cuboids.begin(), other.cuboids.end())
          , z_aligned_cuboids(other.z_aligned_cuboids.begin(), other.z_aligned_cuboids.end())
          , heightfields(other.heightfields.begin(), other.heightfields.end())
          , pointclouds(other.pointclouds.begin(), other.pointclouds.end())
          , pointclouds_mvt(other.pointclouds_mvt.begin(), other.pointclouds_mvt.end())
          , attachments(other.template clone_attachments<DataT>())
          , active_self_pairs(other.active_self_pairs.begin(), other.active_self_pairs.end())
        {
        }

        inline auto add_sphere(const Sphere<DataT> &sphere)
        {
            spheres.emplace_back(sphere);
            sort();
        }

        // Z-aligned shapes are classified here so they dispatch to the specialized
        // collision routines.
        inline auto add_cuboid(const Cuboid<DataT> &cuboid)
        {
            if (cuboid.axis_3_z == 1.)
            {
                z_aligned_cuboids.emplace_back(cuboid);
            }
            else
            {
                cuboids.emplace_back(cuboid);
            }

            sort();
        }

        inline auto add_capsule(const Capsule<DataT> &capsule)
        {
            if (capsule.xv == 0. and capsule.yv == 0.)
            {
                z_aligned_capsules.emplace_back(capsule);
            }
            else
            {
                capsules.emplace_back(capsule);
            }

            sort();
        }

        inline auto add_heightfield(const HeightField<DataT> &heightfield)
        {
            heightfields.emplace_back(heightfield);
        }

        inline auto sort()
        {
            std::sort(
                spheres.begin(),
                spheres.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                capsules.begin(),
                capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_capsules.begin(),
                z_aligned_capsules.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cylinders.begin(),
                cylinders.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                cuboids.begin(),
                cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
            std::sort(
                z_aligned_cuboids.begin(),
                z_aligned_cuboids.end(),
                [](const auto &a, const auto &b) { return a.min_distance < b.min_distance; });
        }

    private:
        template <typename OtherDataT>
        friend struct Environment;

        template <typename OtherDataT>
        inline auto clone_attachments() const noexcept -> std::vector<Attachment<OtherDataT>>
        {
            std::vector<Attachment<OtherDataT>> result;
            result.reserve(attachments.size());
            for (const auto &attachment : attachments)
            {
                result.emplace_back(attachment);
            }

            return result;
        }
    };

    // Poses every attachment riding on end-effector `end_effector`.
    template <typename DataT>
    inline auto set_attachment_pose(
        const Environment<DataT> &e,
        std::size_t end_effector,
        const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) noexcept
    {
        for (const auto &attachment : e.attachments)
        {
            if (attachment.end_effector == end_effector)
            {
                attachment.pose(p_tf);
            }
        }
    }

}  // namespace vamp::collision
