#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
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
            excluded_end_effectors = o.excluded_end_effectors;
            spheres.reserve(o.spheres.size());
            for (const auto &sphere : o.spheres)
            {
                spheres.emplace_back(sphere);
            }
        }

        // Index into the robot's end-effector list this attachment rides on.
        std::size_t end_effector = 0;

        // Additional end-effector indices this attachment should never be collision-checked
        // against (e.g. the other end-effector(s) jointly grasping the same object as this
        // attachment). The relative pose between those end-effectors is not enforced here —
        // that is the planner's responsibility.
        std::vector<std::size_t> excluded_end_effectors;

        std::vector<Sphere<DataT>> spheres;
        // HACK: To get around passing the environment as const but needing to re-pose the
        // attachments
        mutable std::vector<Sphere<DataT>> posed_spheres;
        Eigen::Transform<DataT, 3, Eigen::Isometry> tf;

        // Whether this attachment should be excluded from collision checks against attachments
        // riding on the given end-effector index (either because it's the same index, or
        // because it was explicitly excluded).
        [[nodiscard]] inline auto excludes(std::size_t other_end_effector) const noexcept -> bool
        {
            return other_end_effector == end_effector or
                   std::find(
                       excluded_end_effectors.cbegin(), excluded_end_effectors.cend(), other_end_effector) !=
                       excluded_end_effectors.cend();
        }

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
