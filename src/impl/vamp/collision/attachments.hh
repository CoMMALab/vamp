#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <type_traits>
#include <vamp/collision/shapes.hh>
#include <vector>

namespace vamp::collision
{
    template <typename DataT>
    struct Attachment
    {
        Attachment(const Eigen::Transform<DataT, 3, Eigen::Isometry> &tf) noexcept : tf(std::move(tf))
        {
        }

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Eigen::Transform<float, 3, Eigen::Isometry> &tf) noexcept
          : Attachment(tf.cast<DataT>())
        {
        }

        Attachment(const Attachment &) = default;

        template <typename DT = DataT, typename = std::enable_if_t<not std::is_same_v<DT, float>>>
        Attachment(const Attachment<float> &o) noexcept : Attachment(o.tf)
        {
            end_effector = o.end_effector;
            allowed_links = o.allowed_links;
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }

            cuboids.reserve(o.cuboids.size());
            for (const auto &cuboid : o.cuboids)
            {
                cuboids.emplace_back(cuboid);
            }
        }

        // Index into the robot's end-effector list this attachment rides on.
        std::size_t end_effector = 0;
        // Links (the robot's link ids, see Robot::link_names) this attachment may touch without being
        // flagged. Empty: the attachment is checked against every link its end-effector is checked against.
        std::vector<std::size_t> allowed_links;

        inline auto allows(std::size_t link) const noexcept -> bool
        {
            for (const auto allowed : allowed_links)
            {
                if (allowed == link)
                {
                    return true;
                }
            }

            return false;
        }

        // The attached geometry, in the attachment's own frame (`tf` relative to the end-effector)
        std::vector<Sphere<DataT>> spheres;
        std::vector<Cuboid<DataT>> cuboids;

        // The attached geometry in the world frame, as of the last call to pose.
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        // Posed cuboids are OrientedBoxes (rather than Cuboids) since they are recomputed at every
        // configuration, so they should not pay to precompute a min_distance.
        mutable std::vector<OrientedBox<DataT>> posed_cuboids;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        inline void pose(const Eigen::Transform<DataT, 3, Eigen::Isometry> &p_tf) const noexcept
        {
            const auto &n_tf = p_tf * tf;

            posed_spheres.resize(spheres.size());
            for (auto i = 0U; i < spheres.size(); ++i)
            {
                const auto &s = spheres[i];
                Eigen::Matrix<DataT, 3, 1> sp(s.x, s.y, s.z);
                auto tfs = n_tf * sp;
                posed_spheres[i] = Sphere<DataT>(tfs[0], tfs[1], tfs[2], s.r);
            }

            // Cuboids move rigidly with the attachment: the center is transformed like a sphere's, and
            // the axes (directions) are only rotated. The half-lengths are unchanged.
            posed_cuboids.resize(cuboids.size());
            const auto &rotation = n_tf.linear();
            for (auto i = 0U; i < cuboids.size(); ++i)
            {
                const auto &c = cuboids[i];
                Eigen::Matrix<DataT, 3, 1> center(c.x, c.y, c.z);
                Eigen::Matrix<DataT, 3, 1> axis_1(c.axis_1_x, c.axis_1_y, c.axis_1_z);
                Eigen::Matrix<DataT, 3, 1> axis_2(c.axis_2_x, c.axis_2_y, c.axis_2_z);
                Eigen::Matrix<DataT, 3, 1> axis_3(c.axis_3_x, c.axis_3_y, c.axis_3_z);

                const Eigen::Matrix<DataT, 3, 1> tc = n_tf * center;
                const Eigen::Matrix<DataT, 3, 1> t1 = rotation * axis_1;
                const Eigen::Matrix<DataT, 3, 1> t2 = rotation * axis_2;
                const Eigen::Matrix<DataT, 3, 1> t3 = rotation * axis_3;

                posed_cuboids[i] = OrientedBox<DataT>(
                    tc[0],
                    tc[1],
                    tc[2],
                    t1[0],
                    t1[1],
                    t1[2],
                    t2[0],
                    t2[1],
                    t2[2],
                    t3[0],
                    t3[1],
                    t3[2],
                    c.axis_1_r,
                    c.axis_2_r,
                    c.axis_3_r);
            }
        }
    };
}  // namespace vamp::collision
