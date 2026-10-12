#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/validity.hh>
#include <vamp/planning/constraints/goal_region.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/random/rng.hh>

namespace vamp::planning::constraint
{
    struct TaskSpaceSamplerSettings
    {
        // Object position box and nominal orientation (wxyz), world frame.
        std::array<float, 3> lower{};
        std::array<float, 3> upper{};
        std::array<float, 4> orientation{1.F, 0.F, 0.F, 0.F};
        // Orientation draws rotate the nominal by up to +- yaw about the object's z axis and
        // +- tilt about its x and y axes.
        float tilt = 0.2F;
        float yaw = 0.5F;
        // Linear speed uniform in [0, max_speed]; each angular velocity component in
        // +- max_angular_speed (world frame).
        float max_speed = 0.8F;
        float max_angular_speed = 0.3F;
        // Biased linear-velocity directions (x, y, z, weight), world frame, each jittered by a
        // uniform direction scaled by direction_jitter; the remaining probability (1 - total
        // weight) draws a uniform direction.
        std::vector<std::array<float, 4>> directions;
        float direction_jitter = 0.3F;
        // Posture seeds: candidate projections start within +- spread / 2 of a random seed's
        // joint ranges (keeping their inverse-kinematics branches); without seeds, anywhere in
        // the joint box.
        std::vector<std::vector<float>> seeds;
        float spread = 0.3F;
        // Posture candidates per draw (the lowest static torque ratio wins), the cap
        // 1 + slack on the zero-acceleration torque ratio, and rejected draws before falling back
        // to the inner sampler.
        std::size_t candidates = 4;
        float slack = 0.2F;
        std::size_t max_attempts = 20;
        // Fraction of draws that are task-space draws; the rest come from the mixture sampler (or
        // the inner sampler without one), keeping cheap exploration draws in the stream.
        float fraction = 1.F;
        // Posture memory: up to `memory` accepted (object position, posture) pairs; half the
        // candidates start from the postures remembered nearest the drawn position, each joint
        // jittered by +- memory_spread / 2 of its range. 0 disables.
        std::size_t memory = 512;
        float memory_spread = 0.05F;
        // Null-space refinement of the chosen posture: up to refine_steps steps of refine_step
        // (rad) along the self-motion that keeps the object pose, descending a smooth maximum of
        // the static torque ratios; a step is kept only if the true ratio drops. 0 disables.
        std::size_t refine_steps = 3;
        float refine_step = 0.1F;
        // Local draws (next_near, asked for by planners that expand locally, e.g. BKPIECE): the
        // object moves from the given state's pose and twist and the posture continues from the
        // state's -- candidates project from it (jittered by +- memory_spread / 2), keeping its
        // inverse-kinematics branch. local_mode 0 disables them (planners fall back to next()); 1
        // perturbs the pose by up to local_step (m) and local_angle (rad) and the linear velocity by
        // up to local_speed (m/s); 2 rolls the object forward for a duration in [rollout_min,
        // rollout_max] (s) under a linear acceleration drawn along a uniform direction within the
        // torque limits at the state (inertia, gravity and velocity terms; at most max_acceleration
        // m/s^2). A fraction `fraction` of local draws are task-space; the rest are mixture draws.
        // Local draws need the object on end effector 0 (the robot's eefk).
        int local_mode = 0;
        float local_step = 0.05F;
        float local_angle = 0.1F;
        float local_speed = 0.2F;
        float rollout_min = 0.1F;
        float rollout_max = 0.3F;
        float max_acceleration = 4.F;
    };

    struct TaskSpaceSamplerStats
    {
        std::size_t draws = 0;        // next() calls
        std::size_t task_draws = 0;   // next() calls that attempted task-space draws
        std::size_t attempts = 0;     // object pose/twist draws
        std::size_t pose_collisions = 0;  // attempts whose object pose collides with the obstacles
        std::size_t projections = 0;  // candidate projections tried
        std::size_t projected = 0;    // candidates that projected (in bounds, collision free)
        std::size_t refined = 0;      // refinement steps kept
        std::size_t held = 0;         // attempts whose best posture held the payload within the cap
        std::size_t fallbacks = 0;    // task-space draws that gave up after max_attempts
        std::size_t local_draws = 0;  // next_near calls that attempted local task-space draws
        std::size_t local_held = 0;   // local draws returned
    };

    // Task-space state sampler for constrained flask planning with a held object. Each draw picks
    // the object's pose (position in a box, orientation near a nominal) and twist, then puts the
    // robot there: candidate postures are exact projections onto the planning constraints plus a
    // pin of the object at the drawn pose (end-effector `eef` carries the object), and the one
    // with the lowest static torque ratio under the environment's payloads wins -- heavy objects
    // leave few postures that can hold them. The twist is realized at minimum kinetic energy with
    // the planning constraints' rows still,
    //
    //   qd = M^-1 J^T (J M^-1 J^T)^-1 [0; twist],
    //
    // so velocity draws move the object rather than the arms' self-motion. Joint velocities beyond
    // their limits are scaled down along their direction, and the speed is halved until the
    // zero-acceleration torque ratio max_i |tau_i(q, qd, 0)| / limit_i is within 1 + slack.
    // Draws whose best posture exceeds that cap at rest are rejected; after max_attempts
    // rejections the inner sampler's draw is returned. The inner sampler supplies the uniform
    // draws (its unit-cube preimage), so it should be uniform; a `fraction` < 1 of task-space
    // draws mixes in the mixture sampler's draws. Holds mutable evaluation state: not thread-safe.
    template <typename ZRobot, std::size_t rake>
    struct TaskSpaceSampler : public rng::RNG<ZRobot>
    {
        using Ambient = typename ZRobot::Ambient;
        static constexpr std::size_t d = Ambient::dimension;
        using Configuration = typename ZRobot::Configuration;
        using TSR = TaskSpaceConstraint<Ambient, rake>;
        using Transform = typename TSR::Transform;
        using ConstraintPtr = typename ConstraintSet<Ambient, rake>::Ptr;

        TaskSpaceSampler(
            typename rng::RNG<ZRobot>::Ptr inner,
            std::vector<ConstraintPtr> constraints,
            ConstraintSettings constraint_settings,
            std::size_t eef,
            const Transform &object_in_eef,
            TaskSpaceSamplerSettings settings,
            ChartSettings chart_settings,
            const collision::Environment<float> &environment,
            typename rng::RNG<ZRobot>::Ptr mixture = nullptr) noexcept
          : inner_(std::move(inner))
          , mixture_(mixture ? std::move(mixture) : inner_)
          , pin_(make_pin(eef, object_in_eef))
          , eef_(eef)
          , settings_(std::move(settings))
          , region_(
                with_pin(std::move(constraints), pin_, constraint_settings),
                {},
                0.F,
                0.F,
                chart_settings,
                environment)
          , payload_(collision::attachment_payload<ZRobot::payload_dimension / 10>(environment))
          , rates_(region_.rows(), 0.F)
          , object_spheres_(object_spheres(environment, eef, object_in_eef))
          , obstacles_(environment)
          , nominal_(Eigen::Quaternionf(
                         settings_.orientation[0],
                         settings_.orientation[1],
                         settings_.orientation[2],
                         settings_.orientation[3])
                         .normalized())
        {
            object_.linear() = Eigen::Quaternionf(object_in_eef[0], object_in_eef[1], object_in_eef[2], object_in_eef[3])
                                   .normalized()
                                   .toRotationMatrix();
            object_.translation() = Eigen::Vector3f(object_in_eef[4], object_in_eef[5], object_in_eef[6]);
        }

        // Whether the object pin has an equality form on all six rows.
        auto valid() const noexcept -> bool
        {
            return pin_->equality_rows() == 6 and region_.rows() >= 6;
        }

        inline void reset() noexcept override final
        {
            inner_->reset();
            mixture_->reset();
            this->dist.reset();
        }

        inline auto next() noexcept -> FloatVector<ZRobot::dimension> override final
        {
            ++stats_.draws;
            if (settings_.fraction < 1.F and this->dist.uniform_01() >= settings_.fraction)
            {
                return mixture_->next();
            }

            ++stats_.task_draws;
            for (std::size_t a = 0; a < settings_.max_attempts; ++a)
            {
                if (const auto z = draw())
                {
                    return *z;
                }
            }

            ++stats_.fallbacks;
            return inner_->next();
        }

        inline auto next_near(const FloatVector<ZRobot::dimension> &from, float range) noexcept
            -> std::optional<FloatVector<ZRobot::dimension>> override final
        {
            if (settings_.local_mode == 0 or eef_ != 0)
            {
                return std::nullopt;
            }

            if (settings_.fraction < 1.F and this->dist.uniform_01() >= settings_.fraction)
            {
                return mixture_->next();
            }

            ++stats_.local_draws;
            const auto arr = from.to_array();
            std::array<float, d> q0, qd0;
            std::copy_n(arr.begin(), d, q0.begin());
            std::copy_n(arr.begin() + d, d, qd0.begin());
            for (std::size_t a = 0; a < settings_.max_attempts; ++a)
            {
                if (const auto z = local_draw(q0, qd0))
                {
                    ++stats_.local_held;
                    return *z;
                }
            }

            (void)range;
            return std::nullopt;
        }

        auto stats() const noexcept -> const TaskSpaceSamplerStats &
        {
            return stats_;
        }

        // One draw; empty if no candidate posture projects, or the best cannot hold the payload
        // at rest within 1 + slack.
        auto draw() noexcept -> std::optional<Configuration>
        {
            ++stats_.attempts;
            const auto u = unit_draw();
            std::size_t k = 0;
            const auto take = [&]() { return u[k++]; };

            // Object pose.
            Eigen::Vector3f p;
            for (auto i = 0U; i < 3; ++i)
            {
                p[i] = settings_.lower[i] + take() * (settings_.upper[i] - settings_.lower[i]);
            }

            const float yaw = settings_.yaw * (2.F * take() - 1.F);
            const float tx = settings_.tilt * (2.F * take() - 1.F);
            const float ty = settings_.tilt * (2.F * take() - 1.F);
            const Eigen::Quaternionf rot =
                (nominal_ * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
                 Eigen::AngleAxisf(tx, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(ty, Eigen::Vector3f::UnitY()))
                    .normalized();
            if (not place_object(rot, p))
            {
                return std::nullopt;
            }

            // Object twist, world frame.
            const float speed = settings_.max_speed * take();
            Eigen::Vector3f dir = sphere(take(), take());
            float pick = take();
            for (const auto &b : settings_.directions)
            {
                if (pick < b[3])
                {
                    dir = (Eigen::Vector3f(b[0], b[1], b[2]).normalized() + settings_.direction_jitter * dir)
                              .normalized();
                    break;
                }

                pick -= b[3];
            }

            Eigen::Vector3f w;
            for (auto i = 0U; i < 3; ++i)
            {
                w[i] = settings_.max_angular_speed * (2.F * take() - 1.F);
            }

            // Strongest projected posture; candidates project rake at a time as SIMD lanes.
            std::optional<std::array<float, d>> best;
            float best_ratio = std::numeric_limits<float>::infinity();
            const auto near = nearest_memories(p, (settings_.candidates + 1) / 2);
            for (std::size_t c0 = 0; c0 < settings_.candidates; c0 += rake)
            {
                const std::size_t n = std::min(rake, settings_.candidates - c0);
                for (std::size_t i = 0; i < n; ++i)
                {
                    const std::size_t c = c0 + i;
                    lanes_[i] =
                        (c % 2 == 0 and c / 2 < near.size()) ? jittered(memory_[near[c / 2]].second) : posture_seed();
                }

                strongest_lane(n, best, best_ratio);
            }

            if (not hold(p, best, best_ratio))
            {
                return std::nullopt;
            }

            return realize(*best, rot, speed * dir, w);
        }

        // One local draw from the state (q0, qd0); see TaskSpaceSamplerSettings::local_mode.
        auto local_draw(const std::array<float, d> &q0, const std::array<float, d> &qd0) noexcept
            -> std::optional<Configuration>
        {
            ++stats_.attempts;
            const auto u = unit_draw();
            std::size_t k = 0;
            const auto take = [&]() { return u[k++]; };

            // Object pose and twist at the state (twist by a forward difference along qd0).
            constexpr float h = 1e-3F;
            std::array<float, d> q1;
            for (auto j = 0U; j < d; ++j)
            {
                q1[j] = q0[j] + h * qd0[j];
            }

            const Eigen::Isometry3f X0 = Ambient::eefk(q0) * object_;
            const Eigen::Isometry3f X1 = Ambient::eefk(q1) * object_;
            const Eigen::Vector3f p0 = X0.translation();
            const Eigen::Vector3f v0 = (X1.translation() - p0) / h;
            const Eigen::AngleAxisf body(X0.linear().transpose() * X1.linear());
            const Eigen::Vector3f w0 = X0.linear() * (body.axis() * (body.angle() / h));
            const Eigen::Quaternionf rot0(X0.linear());

            Eigen::Vector3f p, v, w = w0;
            Eigen::Quaternionf rot;
            if (settings_.local_mode == 1)
            {
                p = p0 + settings_.local_step * ball(take(), take(), take());
                v = v0 + settings_.local_speed * ball(take(), take(), take());
                rot = rot0 *
                      Eigen::AngleAxisf(settings_.local_angle * (2.F * take() - 1.F), Eigen::Vector3f::UnitZ()) *
                      Eigen::AngleAxisf(settings_.local_angle * (2.F * take() - 1.F), Eigen::Vector3f::UnitX()) *
                      Eigen::AngleAxisf(settings_.local_angle * (2.F * take() - 1.F), Eigen::Vector3f::UnitY());
            }
            else
            {
                // Torque headroom along a uniform direction: tau(alpha) = tau0 + alpha tau1 with tau1
                // the torque of the minimum-energy joint acceleration realizing a unit object
                // acceleration (drift terms neglected), alpha within every joint's limit.
                const float dt = settings_.rollout_min + take() * (settings_.rollout_max - settings_.rollout_min);
                const Eigen::Vector3f dir = sphere(take(), take());
                pin_->set_reference(eef_, transform(rot0, p0));
                std::fill(rates_.begin(), rates_.end(), 0.F);
                const Eigen::Vector3f dir_ref = X0.linear().transpose() * dir;
                const std::size_t r0 = rates_.size() - 6;
                for (auto i = 0U; i < 3; ++i)
                {
                    rates_[r0 + i] = dir_ref[i];
                }

                const auto qdd = region_.velocity(q0, rates_);
                if (not qdd)
                {
                    return std::nullopt;
                }

                std::array<float, 3 * d> x{};
                std::copy(q0.begin(), q0.end(), x.begin());
                std::copy(qd0.begin(), qd0.end(), x.begin() + d);
                const auto tau0 = ZRobot::torques(x, payload_);
                std::copy(qdd->begin(), qdd->end(), x.begin() + 2 * d);
                const auto tau = ZRobot::torques(x, payload_);
                float lo = 0.F, hi = settings_.max_acceleration;
                for (auto i = 0U; i < d; ++i)
                {
                    const float limit = ZRobot::effort_limits[i] * ZRobot::effort_scale;
                    const float slope = tau[i] - tau0[i];
                    if (std::abs(slope) < 1e-9F)
                    {
                        if (std::abs(tau0[i]) > limit)
                        {
                            return std::nullopt;
                        }

                        continue;
                    }

                    const float a = (-limit - tau0[i]) / slope, b = (limit - tau0[i]) / slope;
                    lo = std::max(lo, std::min(a, b));
                    hi = std::min(hi, std::max(a, b));
                }

                if (not (lo <= hi))
                {
                    return std::nullopt;
                }

                const Eigen::Vector3f acc = (lo + take() * (hi - lo)) * dir;
                p = p0 + dt * v0 + (0.5F * dt * dt) * acc;
                v = v0 + dt * acc;
                const float spin = w0.norm();
                rot = (spin > 1e-6F) ? Eigen::Quaternionf(Eigen::AngleAxisf(spin * dt, w0 / spin)) * rot0 : rot0;
            }

            rot.normalize();
            for (auto i = 0U; i < 3; ++i)
            {
                p[i] = std::clamp(p[i], settings_.lower[i], settings_.upper[i]);
            }

            if (v.norm() > settings_.max_speed)
            {
                v *= settings_.max_speed / v.norm();
            }

            // Orientation stays within the global draws' range of the nominal.
            if (rot.angularDistance(nominal_) >
                std::sqrt(settings_.yaw * settings_.yaw + 2.F * settings_.tilt * settings_.tilt))
            {
                return std::nullopt;
            }

            if (not place_object(rot, p))
            {
                return std::nullopt;
            }

            const std::size_t n = std::min(settings_.candidates, rake);
            lanes_[0] = q0;
            for (std::size_t i = 1; i < n; ++i)
            {
                lanes_[i] = jittered(q0);
            }

            std::optional<std::array<float, d>> best;
            float best_ratio = std::numeric_limits<float>::infinity();
            strongest_lane(n, best, best_ratio);
            if (not hold(p, best, best_ratio))
            {
                return std::nullopt;
            }

            return realize(*best, rot, v, w);
        }

    private:
        static auto transform(const Eigen::Quaternionf &rot, const Eigen::Vector3f &p) noexcept -> Transform
        {
            return {rot.w(), rot.x(), rot.y(), rot.z(), p.x(), p.y(), p.z()};
        }

        // Pin the object at (rot, p); false if its own collision spheres (its attachments) hit the
        // obstacles -- rejecting the pose before paying for posture projections.
        auto place_object(const Eigen::Quaternionf &rot, const Eigen::Vector3f &p) noexcept -> bool
        {
            pin_->set_reference(eef_, transform(rot, p));
            for (const auto &sp : object_spheres_)
            {
                const Eigen::Vector3f c = rot * Eigen::Vector3f(sp[0], sp[1], sp[2]) + p;
                if (sphere_environment_in_collision(obstacles_, c.x(), c.y(), c.z(), sp[3]))
                {
                    ++stats_.pose_collisions;
                    return false;
                }
            }

            return true;
        }

        // Project the first n candidate postures in lanes_ (one batch) and keep the lowest static
        // torque ratio in (best, best_ratio).
        void strongest_lane(std::size_t n, std::optional<std::array<float, d>> &best, float &best_ratio) noexcept
        {
            stats_.projections += n;
            stats_.projected += region_.rest_states(lanes_, n, ok_);
            for (std::size_t i = 0; i < n; ++i)
            {
                if (not ok_[i])
                {
                    continue;
                }

                const float ratio = zero_acceleration_torque_ratio<ZRobot>(lanes_[i], zero_, payload_);
                if (ratio < best_ratio)
                {
                    best_ratio = ratio;
                    best = lanes_[i];
                }
            }
        }

        // Refine the chosen posture and accept it if it holds the payload at rest within 1 + slack.
        auto hold(const Eigen::Vector3f &p, std::optional<std::array<float, d>> &best, float &best_ratio) noexcept
            -> bool
        {
            if (best)
            {
                stats_.refined += region_.refine(*best, best_ratio, settings_.refine_steps, settings_.refine_step);
            }

            if (not best or best_ratio > 1.F + settings_.slack)
            {
                return false;
            }

            ++stats_.held;
            remember(p, *best);
            return true;
        }

        // The flask state at posture q (on the manifold pinned at orientation rot) moving the object
        // with world twist (v, w) at minimum kinetic energy, the planning constraints' rows still;
        // velocities scaled into the joint limits and halved until the zero-acceleration torque ratio
        // is within 1 + slack (else zero).
        auto realize(
            const std::array<float, d> &q,
            const Eigen::Quaternionf &rot,
            const Eigen::Vector3f &v,
            const Eigen::Vector3f &w) noexcept -> std::optional<Configuration>
        {
            const Eigen::Matrix3f R = rot.toRotationMatrix();
            const Eigen::Vector3f v_ref = R.transpose() * v;
            const Eigen::Vector3f w_ref = R.transpose() * w;
            std::fill(rates_.begin(), rates_.end(), 0.F);
            const std::size_t r0 = rates_.size() - 6;
            for (auto i = 0U; i < 3; ++i)
            {
                rates_[r0 + i] = v_ref[i];
                rates_[r0 + 3 + i] = w_ref[i];
            }

            auto qd = region_.velocity(q, rates_);
            if (not qd)
            {
                return std::nullopt;
            }

            float over = 0.F;
            for (auto j = 0U; j < d; ++j)
            {
                over = std::max(over, std::abs((*qd)[j]) / ZRobot::velocity_limits[j]);
            }

            if (over > 1.F)
            {
                for (auto &x : *qd)
                {
                    x /= over;
                }
            }

            const float cap = 1.F + settings_.slack;
            for (auto h = 0; h < 4 and zero_acceleration_torque_ratio<ZRobot>(q, *qd, payload_) > cap; ++h)
            {
                for (auto &x : *qd)
                {
                    x *= 0.5F;
                }
            }

            if (zero_acceleration_torque_ratio<ZRobot>(q, *qd, payload_) > cap)
            {
                qd = zero_;
            }

            typename ZRobot::ConfigurationArray arr{};
            std::copy(q.begin(), q.end(), arr.begin());
            std::copy(qd->begin(), qd->end(), arr.begin() + d);
            const Configuration z(arr);
            if (not ZRobot::in_bounds(z))
            {
                return std::nullopt;
            }

            return z;
        }

        static auto make_pin(std::size_t eef, const Transform &object_in_eef) -> std::shared_ptr<TSR>
        {
            constexpr std::size_t n = Ambient::n_eef;
            const Transform identity{1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
            std::array<Transform, n> offsets, references;
            std::array<typename TSR::Bound, n> lower, upper;
            for (auto i = 0U; i < n; ++i)
            {
                offsets[i] = identity;
                references[i] = identity;
                lower[i].fill(-10.F);
                upper[i].fill(10.F);
            }

            // The generated kernels place the offset frame at oMf * rTe^-1: rTe is the
            // end-effector's pose in the object frame.
            const Eigen::Quaternionf q(object_in_eef[0], object_in_eef[1], object_in_eef[2], object_in_eef[3]);
            const Eigen::Vector3f t(object_in_eef[4], object_in_eef[5], object_in_eef[6]);
            const Eigen::Quaternionf qi = q.normalized().conjugate();
            const Eigen::Vector3f ti = -(qi * t);
            offsets[eef] = {qi.w(), qi.x(), qi.y(), qi.z(), ti.x(), ti.y(), ti.z()};
            lower[eef].fill(0.F);
            upper[eef].fill(0.F);
            return std::make_shared<TSR>(offsets, references, lower, upper);
        }

        // Collision spheres (x, y, z, r) of the attachments on end effector `eef`, in the object frame.
        static auto object_spheres(
            const collision::Environment<float> &environment,
            std::size_t eef,
            const Transform &object_in_eef) -> std::vector<std::array<float, 4>>
        {
            Eigen::Isometry3f object = Eigen::Isometry3f::Identity();
            object.linear() =
                Eigen::Quaternionf(object_in_eef[0], object_in_eef[1], object_in_eef[2], object_in_eef[3])
                    .normalized()
                    .toRotationMatrix();
            object.translation() = Eigen::Vector3f(object_in_eef[4], object_in_eef[5], object_in_eef[6]);
            const Eigen::Isometry3f eef_in_object = object.inverse();

            std::vector<std::array<float, 4>> out;
            for (const auto &a : environment.attachments)
            {
                if (a.end_effector != eef)
                {
                    continue;
                }

                for (const auto &sp : a.spheres)
                {
                    const Eigen::Vector3f c = eef_in_object * (a.tf * Eigen::Vector3f(sp.x, sp.y, sp.z));
                    out.push_back({c.x(), c.y(), c.z(), sp.r});
                }
            }

            return out;
        }

        static auto with_pin(
            std::vector<ConstraintPtr> constraints,
            const std::shared_ptr<TSR> &pin,
            const ConstraintSettings &settings) -> ConstraintSet<Ambient, rake>
        {
            constraints.emplace_back(pin);
            return ConstraintSet<Ambient, rake>(std::move(constraints), settings);
        }

        // 2d uniforms in [0, 1]: the unit-cube preimage of an inner draw.
        auto unit_draw() noexcept -> std::array<float, 2 * d>
        {
            auto unit = inner_->next();
            ZRobot::descale_configuration(unit);
            const auto arr = unit.to_array();
            std::array<float, 2 * d> out;
            for (auto i = 0U; i < 2 * d; ++i)
            {
                out[i] = std::clamp(arr[i], 0.F, 1.F);
            }

            return out;
        }

        auto posture_seed() noexcept -> std::array<float, d>
        {
            std::array<float, d> q;
            if (settings_.seeds.empty())
            {
                const auto arr = inner_->next().to_array();
                std::copy_n(arr.begin(), d, q.begin());
                return q;
            }

            const auto u = unit_draw();
            const auto &seed = settings_.seeds[std::min<std::size_t>(
                settings_.seeds.size() - 1, static_cast<std::size_t>(u[d] * settings_.seeds.size()))];
            for (auto j = 0U; j < d; ++j)
            {
                const float range = Ambient::upper_bound[j] - Ambient::lower_bound[j];
                q[j] = seed[j] + settings_.spread * (u[j] - 0.5F) * range;
            }

            return q;
        }

        // Indices of up to k remembered postures nearest (by object position) to p.
        auto nearest_memories(const Eigen::Vector3f &p, std::size_t k) const -> std::vector<std::size_t>
        {
            std::vector<std::pair<float, std::size_t>> d;
            d.reserve(memory_.size());
            for (std::size_t i = 0; i < memory_.size(); ++i)
            {
                d.emplace_back((memory_[i].first - p).squaredNorm(), i);
            }

            k = std::min(k, d.size());
            std::partial_sort(d.begin(), d.begin() + k, d.end());
            std::vector<std::size_t> out(k);
            for (std::size_t i = 0; i < k; ++i)
            {
                out[i] = d[i].second;
            }

            return out;
        }

        void remember(const Eigen::Vector3f &p, const std::array<float, d> &q)
        {
            if (settings_.memory == 0)
            {
                return;
            }

            if (memory_.size() < settings_.memory)
            {
                memory_.emplace_back(p, q);
            }
            else
            {
                memory_[next_memory_++ % settings_.memory] = {p, q};
            }
        }

        auto jittered(const std::array<float, d> &q) noexcept -> std::array<float, d>
        {
            const auto u = unit_draw();
            std::array<float, d> out;
            for (auto j = 0U; j < d; ++j)
            {
                const float range = Ambient::upper_bound[j] - Ambient::lower_bound[j];
                out[j] = q[j] + settings_.memory_spread * (u[j] - 0.5F) * range;
            }

            return out;
        }

        // Uniform unit vector from two uniforms.
        static auto sphere(float a, float b) noexcept -> Eigen::Vector3f
        {
            const float z = 2.F * a - 1.F;
            const float r = std::sqrt(std::max(0.F, 1.F - z * z));
            const float phi = 2.F * static_cast<float>(M_PI) * b;
            return {r * std::cos(phi), r * std::sin(phi), z};
        }

        // Uniform point in the unit ball from three uniforms.
        static auto ball(float a, float b, float c) noexcept -> Eigen::Vector3f
        {
            return std::cbrt(c) * sphere(a, b);
        }

        typename rng::RNG<ZRobot>::Ptr inner_;
        typename rng::RNG<ZRobot>::Ptr mixture_;
        std::shared_ptr<TSR> pin_;
        std::size_t eef_;
        TaskSpaceSamplerSettings settings_;
        ImpactGoalRegion<ZRobot, rake> region_;
        typename ZRobot::Payload payload_;
        std::vector<float> rates_;
        std::array<float, d> zero_{};
        std::vector<std::array<float, 4>> object_spheres_;
        collision::Environment<FloatVector<rake>> obstacles_;
        std::vector<std::pair<Eigen::Vector3f, std::array<float, d>>> memory_;
        std::size_t next_memory_ = 0;
        TaskSpaceSamplerStats stats_;
        Eigen::Quaternionf nominal_;
        Eigen::Isometry3f object_ = Eigen::Isometry3f::Identity();
        std::array<std::array<float, d>, rake> lanes_{};
        std::array<bool, rake> ok_{};
    };
}  // namespace vamp::planning::constraint
