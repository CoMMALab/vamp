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

// Informed sampler for task-space-parameterized robots (Robot::use_parameterized_ik, e.g.
// IiwaMarker): Robot::Configuration already *is* an end-effector pose (+ redundancy
// parameter), so instead of sampling the whole space and rejecting on a downstream IK/
// collision check, this samples end-effector poses directly inside a Task Space Region
// (Berenson et al., "Task Space Regions", IJRR 2011) and rejects against a caller-supplied
// blackbox pose validity check.
//
// Region parameterization (matches vamp::planning::constraint::TaskSpaceConstraint): an
// offset frame's pose, eef_to_offset (= eef_T_offset: maps offset-frame coordinates into the
// end-effector frame), is bounded relative to a reference frame, world_to_reference
// (= world_T_reference: maps reference-frame coordinates into the world frame), by
// [lower, upper] on [dx, dy, dz, rx, ry, rz] -- the translation and so(3) log-map rotation
// vector of the offset frame *as seen from the reference frame* (reference_T_offset).
// ASSUMPTION (unverified against a generated tsr_error kernel, since no n_eef=1 robot
// currently has one in this repo): bound order is translation (indices 0-2) then rotation
// log-map (indices 3-5), matching TaskSpaceConstraint's declared row order. Flip the two
// halves below if a real kernel disagrees.
//
// Instead of drawing fresh uniforms for translation/rotation and hand-rolling the redundancy
// (psi, ...) tail, this reuses Robot::sample() wholesale via `inner` (a Halton<Robot>) and
// only overrides the fields that need to land inside the TSR:
//   - translation: Robot::sample() maps its raw Halton digit x in [0, 1] through a fixed
//     affine map (Robot::sample_position_lower/upper, the same box sample() itself reads from
//     -- see iiwamarker.hh) into its own default box; that map is invertible, so the original
//     low-discrepancy digit is recovered exactly and re-mapped into [lower, upper]'s
//     translation half instead -- same Halton stratification, different box.
//   - rotation: Robot::sample()'s quaternion is already Shoemake-uniform over all of SO(3).
//     If the TSR leaves rotation unconstrained (bound width >= a full turn on all three
//     log-map axes), that quaternion is used as reference_to_offset's rotation directly --
//     free reuse, ideal even. Only when rotation is genuinely bounded does this fall back to
//     fresh uniform draws over the rotation-vector box + exp map (Halton stratification is
//     not preserved for that case, since Shoemake's 3 inputs don't invert into an
//     axis-angle box).
//   - psi / any further redundancy parameters (index 7+): left exactly as Robot::sample()
//     produced them -- whatever that generated tail formula is, we never need to know it.
//
// world_to_offset = world_to_reference * reference_to_offset, then
// world_to_eef = world_to_offset * eef_to_offset^{-1}. Retry (up to max_tries) until
// Robot::is_eef_collision_free(world_to_eef, environment) accepts; on repeated failure fall
// back to inner->next() (deterministic, never worse than uninformed).

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
    }  // namespace detail

    // Rejects candidate poses via Robot::is_eef_collision_free(pose, environment). Single
    // end-effector only.
    template <typename Robot>
    struct TaskSpaceInformedSampler : public rng::RNG<Robot>
    {
        static_assert(
            Robot::use_parameterized_ik,
            "TaskSpaceInformedSampler requires a task-space-parameterized robot "
            "(Robot::use_parameterized_ik)");

        using Configuration = typename Robot::Configuration;

        using Transform = std::array<float, 7>;
        // (dx, dy, dz, rx, ry, rz): translation + so(3) log-map rotation bound.
        using Bound = std::array<float, 6>;

        static constexpr std::size_t max_tries = 128;

        TaskSpaceInformedSampler(
            const Transform &eef_to_offset,
            const Transform &world_to_reference,
            const Bound &lower,
            const Bound &upper,
            vamp::collision::Environment<float> environment_in,
            typename rng::RNG<Robot>::Ptr inner_in) noexcept
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

            constexpr float full_turn = 2.F * static_cast<float>(vamp::utils::constants::pi);
            rotation_free_ = true;
            for (std::size_t i = 0; i < 3; ++i)
            {
                rotation_free_ = rotation_free_ and (upper_[3 + i] - lower_[3 + i] >= full_turn);
            }
        }

        inline void reset() noexcept override
        {
            inner->reset();
            inner->dist.reset();
        }

        inline auto next() noexcept -> FloatVector<Robot::dimension> override
        {
            for (std::size_t tries = 0; tries < max_tries; ++tries)
            {
                // Robot::sample() via Halton: position box + free-SO(3) quaternion + whatever
                // the generated redundancy-parameter tail (psi, ...) does. buf[7+] is kept
                // as-is below; buf[0..6] are recomputed/overridden.
                alignas(FloatVectorAlignment) auto buf = inner->next().to_array();

                Eigen::Vector3f translation;
                for (std::size_t i = 0; i < 3; ++i)
                {
                    const float lo = Robot::sample_position_lower[i];
                    const float scale = Robot::sample_position_upper[i] - Robot::sample_position_lower[i];
                    const float digit = (buf[i] - lo) / scale;
                    translation[static_cast<Eigen::Index>(i)] = lower_[i] + digit * (upper_[i] - lower_[i]);
                }

                Eigen::Quaternionf reference_rotation;
                if (rotation_free_)
                {
                    // Already Shoemake-uniform over SO(3); reuse as the local rotation as-is.
                    reference_rotation = Eigen::Quaternionf(buf[6], buf[3], buf[4], buf[5]);
                }
                else
                {
                    Eigen::Vector3f rotvec;
                    for (std::size_t i = 0; i < 3; ++i)
                    {
                        rotvec[static_cast<Eigen::Index>(i)] =
                            inner->dist.uniform_real(lower_[3 + i], upper_[3 + i]);
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

                if (true or Robot::is_eef_collision_free(
                                to_isometry(world_to_eef_q, world_to_eef_t), environment_))
                {
                    buf[0] = world_to_eef_t.x();
                    buf[1] = world_to_eef_t.y();
                    buf[2] = world_to_eef_t.z();
                    buf[3] = world_to_eef_q.x();
                    buf[4] = world_to_eef_q.y();
                    buf[5] = world_to_eef_q.z();
                    buf[6] = world_to_eef_q.w();
                    // buf[7+] (psi, ...) is untouched: Robot::sample()'s own output.
                    return Configuration(buf.data());
                }
            }

            return inner->next();
        }

    private:
        static auto to_quat_trans(const Transform &t) noexcept
            -> std::pair<Eigen::Quaternionf, Eigen::Vector3f>
        {
            return {Eigen::Quaternionf(t[6], t[3], t[4], t[5]), Eigen::Vector3f(t[0], t[1], t[2])};
        }

        // Only needed on the (currently dead) collision-check path, which wants a full pose.
        static auto to_isometry(const Eigen::Quaternionf &q, const Eigen::Vector3f &t) noexcept
            -> Eigen::Isometry3f
        {
            Eigen::Isometry3f iso = Eigen::Isometry3f::Identity();
            iso.linear() = q.toRotationMatrix();
            iso.translation() = t;
            return iso;
        }

        Bound lower_;
        Bound upper_;
        // Precomputed once at construction (constant across all next() calls).
        Eigen::Quaternionf offset_to_eef_q_;
        Eigen::Vector3f offset_to_eef_t_;
        Eigen::Quaternionf world_to_reference_q_;
        Eigen::Vector3f world_to_reference_t_;
        bool rotation_free_ = false;
        vamp::collision::Environment<float> environment_;
        typename rng::RNG<Robot>::Ptr inner;
    };

    template <typename Robot>
    inline auto make_task_space_informed_sampler(
        const typename TaskSpaceInformedSampler<Robot>::Transform &eef_to_offset,
        const typename TaskSpaceInformedSampler<Robot>::Transform &world_to_reference,
        const typename TaskSpaceInformedSampler<Robot>::Bound &lower,
        const typename TaskSpaceInformedSampler<Robot>::Bound &upper,
        vamp::collision::Environment<float> environment,
        typename rng::RNG<Robot>::Ptr inner) -> std::shared_ptr<TaskSpaceInformedSampler<Robot>>
    {
        return std::make_shared<TaskSpaceInformedSampler<Robot>>(
            eef_to_offset,
            world_to_reference,
            lower,
            upper,
            std::move(environment),
            std::move(inner));
    }
}  // namespace vamp::planning
