#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/constants.hh>
#include <vamp/random/rng.hh>
#include <vamp/vector.hh>

// Informed sampler for a robot's task-space parameterization (Robot::ParameterizedSpace, e.g.
// Iiwa::ParameterizedSpace): Space::State already *is* an end-effector pose (+ redundancy
// parameter(s)) at indices [0, 7) -- (x, y, z, qx, qy, qz, qw) -- with any further indices
// (psi, ...) left as whatever Space::sample() produces for them. So instead of sampling the
// whole space and rejecting on a downstream IK/collision check, this samples end-effector poses
// directly inside a Task Space Region (Berenson et al., "Task Space Regions", IJRR 2011).
//
// Region parameterization (matches vamp::planning::constraint::TaskSpaceConstraint): an offset
// frame's pose, eef_to_offset (= eef_T_offset: maps offset-frame coordinates into the end-
// effector frame), is bounded relative to a reference frame, world_to_reference (=
// world_T_reference: maps reference-frame coordinates into the world frame), by [lower, upper]
// on [dx, dy, dz, rx, ry, rz] -- the translation and so(3) log-map rotation vector of the offset
// frame *as seen from the reference frame* (reference_T_offset).
//
// Instead of drawing fresh uniforms for translation/rotation and hand-rolling the redundancy
// (psi, ...) tail, this reuses Space::sample() wholesale via `inner` (a Halton<Robot, Space>)
// and only overrides the fields that need to land inside the TSR:
//   - translation: Space::sample() maps its raw Halton digit x in [0, 1] through a fixed affine
//     map (the generated sample()'s default +/-2 box -- see default_position_lower/range below)
//     into its own default box; that map is invertible, so the original low-discrepancy digit is
//     recovered exactly and re-mapped into [lower, upper]'s translation half instead -- same
//     Halton stratification, different box.
//   - rotation: Space::sample()'s quaternion is already Shoemake-uniform over all of SO(3). If
//     the TSR leaves rotation unconstrained (bound width >= a full turn on all three log-map
//     axes), that quaternion is used as reference_to_offset's rotation directly -- free reuse,
//     ideal even. Only when rotation is genuinely bounded does this fall back to fresh uniform
//     draws over the rotation-vector box + exp map (Halton stratification is not preserved for
//     that case, since Shoemake's 3 inputs don't invert into an axis-angle box).
//   - psi / any further redundancy parameters (index 7+): left exactly as Space::sample()
//     produced them -- whatever that generated tail formula is, we never need to know it.
//
// world_to_offset = world_to_reference * reference_to_offset, then
// world_to_eef = world_to_offset * eef_to_offset^{-1}, written directly into indices [0, 7).

namespace vamp::planning
{
    namespace detail
    {
        // so(3) exponential map: rotation vector (axis * angle) -> unit quaternion.
        inline auto exp_so3(const Eigen::Vector3f &w) noexcept -> Eigen::Quaternionf
        {
            const float theta = w.norm();
            if (theta < 1e-8F)
            {
                return Eigen::Quaternionf::Identity();
            }

            return Eigen::Quaternionf(Eigen::AngleAxisf(theta, w / theta));
        }

        // A bound this narrow (< ~1e-6 m or rad) is treated as fixed: sampling it would just
        // reproduce the midpoint with extra float error, so skip the draw and use it directly.
        static constexpr float fixed_bound_eps = 1e-6F;
    }  // namespace detail

    // Samples Space::State directly inside a task space region. Single end-effector only, with
    // the pose occupying State indices [0, 7) -- (x, y, z, qx, qy, qz, qw) -- matching Space's
    // sample() layout (see Iiwa::ParameterizedSpace).
    template <typename Robot, typename Space = Robot>
    struct TaskSpaceInformedSampler : public rng::RNG<Robot, Space>
    {
        using Configuration = typename Space::State;

        using Transform = std::array<float, 7>;
        // (dx, dy, dz, rx, ry, rz): translation + so(3) log-map rotation bound.
        using Bound = std::array<float, 6>;

        // Space::sample()'s default box for eef position (see e.g. rby1.hh / iiwa_marker.hh's
        // generated sample(): y[i] = -2 + 4*u). Used to invert back to the underlying uniform
        // draw before remapping into [lower, upper]'s translation half -- same convention as
        // RBY1FixedBaseSampler::default_position_lower/range.
        static constexpr float default_position_lower = -2.0F;
        static constexpr float default_position_range = 4.0F;

        TaskSpaceInformedSampler(
            const Transform &eef_to_offset,
            const Transform &world_to_reference,
            const Bound &lower,
            const Bound &upper,
            vamp::collision::Environment<float> environment_in,
            typename rng::RNG<Robot, Space>::Ptr inner_in) noexcept
          : lower_(lower)
          , upper_(upper)
          , environment_(std::move(environment_in))
          , inner(std::move(inner_in))
        {
            // Precompute the two fixed transforms as quaternion + translation once, instead of
            // rebuilding Isometry3f (3x3 rotation matrices) for them on every next() call.
            const auto [eef_to_offset_q, eef_to_offset_t] = to_quat_trans(eef_to_offset);
            // offset_to_eef = eef_to_offset^{-1}: for a unit quaternion, inverse == conjugate.
            offset_to_eef_q_ = eef_to_offset_q.conjugate();
            offset_to_eef_t_ = -(offset_to_eef_q_ * eef_to_offset_t);

            std::tie(world_to_reference_q_, world_to_reference_t_) = to_quat_trans(world_to_reference);

            // Precompute per-dimension range/midpoint and whether a dimension is narrow enough
            // to just use its midpoint instead of sampling it, plus the inverse position scale
            // (turns a per-call division into a multiply below).
            for (std::size_t i = 0; i < 6; ++i)
            {
                range_[i] = upper_[i] - lower_[i];
                mid_[i] = 0.5F * (lower_[i] + upper_[i]);
            }

            all_pos_fixed_ = true;
            for (std::size_t i = 0; i < 3; ++i)
            {
                pos_fixed_[i] = range_[i] < detail::fixed_bound_eps;
                all_pos_fixed_ = all_pos_fixed_ and pos_fixed_[i];
            }

            if (all_pos_fixed_)
            {
                fixed_translation_ = Eigen::Vector3f(mid_[0], mid_[1], mid_[2]);
            }

            constexpr float full_turn = 2.F * static_cast<float>(vamp::utils::constants::pi);
            rotation_free_ = true;
            all_rot_fixed_ = true;
            for (std::size_t i = 0; i < 3; ++i)
            {
                rotation_free_ = rotation_free_ and (range_[3 + i] >= full_turn);
                rot_fixed_[i] = range_[3 + i] < detail::fixed_bound_eps;
                all_rot_fixed_ = all_rot_fixed_ and rot_fixed_[i];
            }

            if ((not rotation_free_) and all_rot_fixed_)
            {
                fixed_rotation_ = detail::exp_so3(Eigen::Vector3f(mid_[3], mid_[4], mid_[5]));
            }
        }

        inline void reset() noexcept override
        {
            inner->reset();
            inner->dist.reset();
        }

        inline auto next() noexcept -> FloatVector<Space::dimension> override
        {
            // Space::sample() via Halton: position box + free-SO(3) quaternion + whatever the
            // generated redundancy-parameter tail (psi, ...) does. buf[7+] is kept as-is below;
            // buf[0..6] are recomputed/overridden.
            alignas(FloatVectorAlignment) auto buf = inner->next().to_array();

            Eigen::Vector3f translation;
            if (all_pos_fixed_)
            {
                // Bound is a point (or near enough): skip sampling, use the midpoint.
                translation = fixed_translation_;
            }
            else
            {
                for (std::size_t i = 0; i < 3; ++i)
                {
                    if (pos_fixed_[i])
                    {
                        translation[static_cast<Eigen::Index>(i)] = mid_[i];
                        continue;
                    }
                    const float digit = (buf[i] - default_position_lower) / default_position_range;
                    translation[static_cast<Eigen::Index>(i)] = lower_[i] + digit * range_[i];
                }
            }

            Eigen::Quaternionf reference_rotation;
            if (rotation_free_)
            {
                // Already Shoemake-uniform over SO(3); reuse as the local rotation as-is.
                reference_rotation = Eigen::Quaternionf(buf[6], buf[3], buf[4], buf[5]);
            }
            else if (all_rot_fixed_)
            {
                // Rotation bound is a point (or near enough): reuse the precomputed midpoint
                // quaternion instead of re-running exp_so3 every call.
                reference_rotation = fixed_rotation_;
            }
            else
            {
                Eigen::Vector3f rotvec;
                for (std::size_t i = 0; i < 3; ++i)
                {
                    rotvec[static_cast<Eigen::Index>(i)] = rot_fixed_[i]
                        ? mid_[3 + i]
                        : inner->dist.uniform_real(lower_[3 + i], upper_[3 + i]);
                }
                reference_rotation = detail::exp_so3(rotvec);
            }

            // world_to_offset = world_to_reference * reference_to_offset, then
            // world_to_eef = world_to_offset * offset_to_eef, done as quaternion/vector
            // composition instead of building an Isometry3f (3x3 rotation matrix) for each
            // factor and multiplying those out.
            const Eigen::Quaternionf world_to_offset_q = world_to_reference_q_ * reference_rotation;
            const Eigen::Vector3f world_to_offset_t =
                world_to_reference_t_ + (world_to_reference_q_ * translation);

            Eigen::Quaternionf world_to_eef_q = world_to_offset_q * offset_to_eef_q_;
            world_to_eef_q.normalize();
            const Eigen::Vector3f world_to_eef_t = world_to_offset_t + (world_to_offset_q * offset_to_eef_t_);

            buf[0] = world_to_eef_t.x();
            buf[1] = world_to_eef_t.y();
            buf[2] = world_to_eef_t.z();
            buf[3] = world_to_eef_q.x();
            buf[4] = world_to_eef_q.y();
            buf[5] = world_to_eef_q.z();
            buf[6] = world_to_eef_q.w();
            // buf[7+] (psi, ...) is untouched: Space::sample()'s own output.
            return Configuration(buf.data());
        }

    private:
        static auto to_quat_trans(const Transform &t) noexcept
            -> std::pair<Eigen::Quaternionf, Eigen::Vector3f>
        {
            return {Eigen::Quaternionf(t[6], t[3], t[4], t[5]), Eigen::Vector3f(t[0], t[1], t[2])};
        }

        Bound lower_;
        Bound upper_;
        // Precomputed once at construction (constant across all next() calls).
        Eigen::Quaternionf offset_to_eef_q_;
        Eigen::Vector3f offset_to_eef_t_;
        Eigen::Quaternionf world_to_reference_q_;
        Eigen::Vector3f world_to_reference_t_;
        Bound range_;  // upper_ - lower_
        Bound mid_;    // (lower_ + upper_) / 2
        std::array<bool, 3> pos_fixed_{};
        std::array<bool, 3> rot_fixed_{};
        bool all_pos_fixed_ = false;
        bool all_rot_fixed_ = false;
        Eigen::Vector3f fixed_translation_ = Eigen::Vector3f::Zero();  // valid iff all_pos_fixed_
        Eigen::Quaternionf fixed_rotation_ =
            Eigen::Quaternionf::Identity();  // valid iff (!rotation_free_ && all_rot_fixed_)
        bool rotation_free_ = false;
        // Kept for callers/future use (e.g. an eef-vs-environment prefilter on the sampled
        // pose); not read by next() itself.
        vamp::collision::Environment<float> environment_;
        typename rng::RNG<Robot, Space>::Ptr inner;
    };

    template <typename Robot, typename Space = Robot>
    inline auto make_task_space_informed_sampler(
        const typename TaskSpaceInformedSampler<Robot, Space>::Transform &eef_to_offset,
        const typename TaskSpaceInformedSampler<Robot, Space>::Transform &world_to_reference,
        const typename TaskSpaceInformedSampler<Robot, Space>::Bound &lower,
        const typename TaskSpaceInformedSampler<Robot, Space>::Bound &upper,
        vamp::collision::Environment<float> environment,
        typename rng::RNG<Robot, Space>::Ptr inner) -> std::shared_ptr<TaskSpaceInformedSampler<Robot, Space>>
    {
        return std::make_shared<TaskSpaceInformedSampler<Robot, Space>>(
            eef_to_offset,
            world_to_reference,
            lower,
            upper,
            std::move(environment),
            std::move(inner));
    }
}  // namespace vamp::planning
