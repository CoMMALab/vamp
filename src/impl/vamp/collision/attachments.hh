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

        std::vector<Sphere<DataT>> approximateCuboidWithSpheres(const Cuboid<DataT>& cub) {
            std::vector<Sphere<DataT>> spheres;

            // 1. Determine the radius of the spheres (using the smallest axis)
            DataT s_r = std::min({cub.axis_1_r, cub.axis_2_r, cub.axis_3_r});
            DataT diameter = static_cast<DataT>(2.0) * s_r;

            // 2. Calculate how many spheres fit along each axis
            // We use ceil to ensure coverage, or round for a centered fit
            int n1 = std::max(1, (int)std::round(cub.axis_1_r / s_r));
            int n2 = std::max(1, (int)std::round(cub.axis_2_r / s_r));
            int n3 = std::max(1, (int)std::round(cub.axis_3_r / s_r));

            // 3. Calculate step sizes along each basis vector
            // A half-length of axis_1_r means the total length is 2 * axis_1_r
            for (int i = 0; i < n1; ++i) {
                for (int j = 0; j < n2; ++j) {
                    for (int k = 0; k < n3; ++k) {
                        // Calculate offset from the center of the cuboid
                        // We map indices to the range [-axis_r + s_r, axis_r - s_r]
                        DataT f1 = (n1 > 1) ? (DataT(2 * i) / (n1 - 1) - 1.0) * (cub.axis_1_r - s_r) : 0;
                        DataT f2 = (n2 > 1) ? (DataT(2 * j) / (n2 - 1) - 1.0) * (cub.axis_2_r - s_r) : 0;
                        DataT f3 = (n3 > 1) ? (DataT(2 * k) / (n3 - 1) - 1.0) * (cub.axis_3_r - s_r) : 0;

                        Sphere<DataT> s;
                        s.x = cub.x + f1 * cub.axis_1_x + f2 * cub.axis_2_x + f3 * cub.axis_3_x;
                        s.y = cub.y + f1 * cub.axis_1_y + f2 * cub.axis_2_y + f3 * cub.axis_3_y;
                        s.z = cub.z + f1 * cub.axis_1_z + f2 * cub.axis_2_z + f3 * cub.axis_3_z;
                        s.r = s_r;

                        spheres.push_back(s);
                    }
                }
            }

            return spheres;
        }

        // convert cuboids to a list of spheres for easier collision checking
        void convert_cuboids_to_spheres() noexcept
        {
            std::vector<Sphere<DataT>> new_spheres;
            for (const auto &cuboid : cuboids)
            {
                auto cuboid_spheres = approximateCuboidWithSpheres(cuboid);
                new_spheres.insert(new_spheres.end(), cuboid_spheres.begin(), cuboid_spheres.end());
            }

            spheres.reserve(spheres.size() + new_spheres.size());
            spheres.insert(spheres.end(), new_spheres.begin(), new_spheres.end());

            cuboids.clear();
        }

        std::vector<Sphere<DataT>> spheres;
        std::vector<Cuboid<DataT>> cuboids;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
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
        }
    };
}  // namespace vamp::collision
