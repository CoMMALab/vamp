#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/chart_local_planner.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/rng.hh>

namespace vamp::planning::constraint
{
    // max_i |tau_i(q, qd, 0)| / (effort_scale * tau_max_i) with the payload: the effort a flask
    // state needs before any acceleration (gravity plus velocity terms).
    template <typename ZRobot, typename Q, typename V>
    inline auto zero_acceleration_torque_ratio(
        const Q &q,
        const V &qd,
        const typename ZRobot::Payload &payload) noexcept -> float
    {
        constexpr std::size_t d = ZRobot::Ambient::dimension;
        std::array<float, 3 * d> x{};
        std::copy_n(q.begin(), d, x.begin());
        std::copy_n(qd.begin(), d, x.begin() + d);
        const auto tau = ZRobot::torques(x, payload);
        float ratio = 0.F;
        for (auto i = 0U; i < d; ++i)
        {
            ratio = std::max(ratio, std::abs(tau[i]) / (ZRobot::effort_limits[i] * ZRobot::effort_scale));
        }

        return ratio;
    }

    // Robots generated with the SIMD RNEA block kernel (torques_block) evaluate rake states per call.
    template <typename R, std::size_t rake, typename = void>
    struct has_torques_block : std::false_type
    {
    };

    template <typename R, std::size_t rake>
    struct has_torques_block<
        R,
        rake,
        std::void_t<decltype(R::template torques_block<rake>(
            std::declval<const typename R::template ConfigurationBlock<rake> &>(),
            std::declval<const typename R::template PayloadBlock<rake> &>()))>> : std::true_type
    {
    };

    // Goal region of flask states z = (q, qd) for chart-based constrained planning, defined by a
    // constraint set with an equality form F (see Constraint::equality_rows): positions lie on
    // its manifold, and velocities realize prescribed rates of its equality rows,
    // dF/dt = s * rate with speed s in [min_speed, max_speed], at minimum kinetic energy:
    //
    //   qd = M^-1 J^T (J M^-1 J^T)^-1 (s * rate),
    //
    // with M(q) the joint-space inertia including the environment's attachment payloads and J the
    // stacked equality Jacobian. That velocity is M-orthogonal to the manifold's self-motion, so
    // an inelastic impact that enforces the constraints (dF/dt -> 0) brings the system exactly to
    // rest: reaching any member is arriving at rest on the manifold through an impact whose
    // impulse (J M^-1 J^T)^-1 (s * rate) is bounded via max_speed. Zero rates give the rest states
    // on the manifold. Example: a box set down on a shelf -- the planning constraints (the rigid
    // grasp) plus a pin of the box pose, whose rows' rate is the box's descent twist.
    //
    // Members are sampled with the planners' primitives: a position seed from a sampler projected
    // by the exact chart, the joint bounds and collision checks of planning, and RNEA columns for
    // M. Holds mutable evaluation state: not thread-safe.
    template <typename ZRobot, std::size_t rake>
    struct ImpactGoalRegion
    {
        using Ambient = typename ZRobot::Ambient;
        static constexpr std::size_t d = Ambient::dimension;

        using Configuration = typename ZRobot::Configuration;
        using AmbientConfiguration = typename Ambient::Configuration;
        using AmbientBlock = typename Ambient::template ConfigurationBlock<rake>;
        using Payload = typename ZRobot::Payload;
        using LP = ChartLocalPlanner<ZRobot, rake, ZRobot::resolution>;

        ImpactGoalRegion(
            ConstraintSet<Ambient, rake> constraints,
            std::vector<float> rate,
            float min_speed,
            float max_speed,
            ChartSettings chart_settings,
            const collision::Environment<float> &environment,
            std::vector<typename PhaseConstraintSet<ZRobot, rake>::Ptr> phase_constraints = {}) noexcept
          : lp_(std::move(constraints),
                exact_settings(chart_settings),
                PhaseConstraintSet<ZRobot, rake>(
                    std::move(phase_constraints),
                    collision::attachment_payload<ZRobot::payload_dimension / 10>(environment)))
          , rate_(std::move(rate))
          , min_speed_(min_speed)
          , max_speed_(max_speed)
          , environment_(environment)
          , payload_(collision::attachment_payload<ZRobot::payload_dimension / 10>(environment))
          , rows_(lp_.constraints.equality_rows())
          , F_(rows_)
          , gamma_(rows_)
          , J_(rows_ * d)
        {
        }

        // Whether the constraint set supports the region (an equality form for every
        // constraint) and `rate` has one entry per equality row.
        auto valid() const noexcept -> bool
        {
            return lp_.exact() and rate_.size() == rows_ and rows_ > 0;
        }

        auto rows() const noexcept -> std::size_t
        {
            return rows_;
        }

        // Draw positions around `seed` instead of over the whole joint box: each joint within
        // +- spread / 2 of its range (spread 1 covers the box). Keeps draws on the seed's
        // inverse-kinematics branch of the manifold.
        void set_seed(const std::array<float, d> &seed, float spread) noexcept
        {
            seed_ = seed;
            spread_ = spread;
        }

        // Refine each drawn position with up to `steps` null-space steps of `step` rad (see
        // refine) before computing its velocity: members whose postures carry the payloads with
        // lower static torque. 0 (the default) disables.
        void set_refinement(std::size_t steps, float step) noexcept
        {
            refine_steps_ = steps;
            refine_step_ = step;
        }

        // One draw: a position from `rng` (over the joint box, or around the seed) projected onto
        // the manifold, bound- and collision-checked, with its minimum-energy velocity at a speed
        // taken uniformly from the same sample. Empty if the draw is rejected.
        auto sample(rng::RNG<ZRobot> &rng) const noexcept -> std::optional<Configuration>
        {
            // Samplers return robot-space states; their unit-cube preimage supplies the uniform
            // draws for the seeded offsets and the speed.
            const auto scaled = rng.next();
            auto unit = scaled;
            ZRobot::descale_configuration(unit);
            const auto unit_arr = unit.to_array();
            const float speed = min_speed_ + std::clamp(unit_arr[d], 0.F, 1.F) * (max_speed_ - min_speed_);

            std::array<float, d> q;
            if (seed_)
            {
                for (auto j = 0U; j < d; ++j)
                {
                    const float range = Ambient::upper_bound[j] - Ambient::lower_bound[j];
                    q[j] = (*seed_)[j] + spread_ * (unit_arr[j] - 0.5F) * range;
                }
            }
            else
            {
                const auto arr = scaled.to_array();
                std::copy_n(arr.begin(), d, q.begin());
            }

            auto rest = rest_state(q);
            if (not rest)
            {
                return std::nullopt;
            }

            if (refine_steps_ > 0)
            {
                float ratio = static_ratio(*rest);
                refine(*rest, ratio, refine_steps_, refine_step_);
            }

            const auto qd = velocity(*rest, speed);
            if (not qd)
            {
                return std::nullopt;
            }

            const auto z = join(*rest, *qd);
            if (not ZRobot::in_bounds(z) or not lp_.satisfied(z))
            {
                return std::nullopt;
            }

            return z;
        }

        // A position projected onto the manifold from `q`, within the joint bounds and collision
        // free; empty if the projection fails or the result is rejected.
        auto rest_state(const std::array<float, d> &q) const noexcept -> std::optional<std::array<float, d>>
        {
            const std::array<float, d> zero{};
            auto z = join(q, zero);
            if (not lp_.project(z))
            {
                return std::nullopt;
            }

            std::array<float, d> out;
            split(z, out);
            if (not Ambient::in_bounds(AmbientConfiguration(out)) or
                not validate_configuration<Ambient, rake>(AmbientConfiguration(out), environment_))
            {
                return std::nullopt;
            }

            return out;
        }

        // rest_state for the first n of up to rake positions at once: one batched exact projection
        // (in place), then the bound and collision checks per converged lane. ok[i] reports
        // acceptance; returns the number accepted.
        auto rest_states(std::array<std::array<float, d>, rake> &q, std::size_t n, std::array<bool, rake> &ok)
            const noexcept -> std::size_t
        {
            lp_.exact_project_batch(q, n, ok);
            std::size_t accepted = 0;
            for (std::size_t i = 0; i < std::min(n, rake); ++i)
            {
                ok[i] = ok[i] and Ambient::in_bounds(AmbientConfiguration(q[i])) and
                        validate_configuration<Ambient, rake>(AmbientConfiguration(q[i]), environment_);
                accepted += ok[i];
            }

            return accepted;
        }

        // Minimum-kinetic-energy velocity at q realizing equality-row rates speed * rate; empty if
        // J M^-1 J^T is singular.
        auto velocity(const std::array<float, d> &q, float speed) const noexcept
            -> std::optional<std::array<float, d>>
        {
            std::vector<float> rates(rows_);
            for (auto r = 0U; r < rows_; ++r)
            {
                rates[r] = speed * rate_[r];
            }

            return velocity(q, rates);
        }

        // Minimum-kinetic-energy velocity at q realizing arbitrary equality-row rates (one per row,
        // in the constraint set's row order); empty if J M^-1 J^T is singular.
        auto velocity(const std::array<float, d> &q, const std::vector<float> &rates) const noexcept
            -> std::optional<std::array<float, d>>
        {
            if (rates.size() != rows_ or not jacobian_and_inertia(q))
            {
                return std::nullopt;
            }

            const Eigen::VectorXf b = Eigen::Map<const Eigen::VectorXf>(rates.data(), rows_);
            const Eigen::VectorXf lambda = S_.ldlt().solve(b);
            if (not lambda.allFinite())
            {
                return std::nullopt;
            }

            const Eigen::VectorXf qd = MinvJt_ * lambda;

            // Near singular postures J M^-1 J^T is ill-conditioned and the float solve leaks motion
            // off the rows it should hold; reject solves that miss the requested rates.
            if (not qd.allFinite() or (Jm_ * qd - b).norm() > 1e-3F * (1.F + b.norm()))
            {
                return std::nullopt;
            }

            std::array<float, d> out;
            for (auto j = 0U; j < d; ++j)
            {
                out[j] = qd[j];
            }

            return out;
        }

        // Constraint impulse of an inelastic impact stopping the equality rows at state z:
        // (J M^-1 J^T)^-1 J qd. For members this is (J M^-1 J^T)^-1 (s * rate), and the impact
        // leaves the system at rest.
        auto impulse(const Configuration &z) const noexcept -> std::vector<float>
        {
            std::array<float, d> q, qd;
            split(z, q, qd);
            if (not jacobian_and_inertia(q))
            {
                return {};
            }

            Eigen::Map<const Eigen::VectorXf> v(qd.data(), d);
            const Eigen::VectorXf lambda = S_.ldlt().solve(Jm_ * v);
            return std::vector<float>(lambda.data(), lambda.data() + rows_);
        }

        // Static torque ratio max_i |tau_i(q, 0, 0)| / limit_i with the payloads.
        auto static_ratio(const std::array<float, d> &q) const noexcept -> float
        {
            return zero_acceleration_torque_ratio<ZRobot>(q, std::array<float, d>{}, payload_);
        }

        // Null-space descent of the static torque at fixed equality-row values: up to `steps` steps
        // of `step` rad along the manifold's self-motion (the null space of the stacked equality
        // Jacobian), descending the smooth maximum sum_i (|tau_i(q, 0, 0)| / limit_i)^8 by central
        // differences, each re-projected and kept only if the true ratio drops. Updates q and its
        // ratio; returns the number of steps kept.
        auto refine(std::array<float, d> &q, float &ratio, std::size_t steps, float step) const noexcept
            -> std::size_t
        {
            constexpr float h = 1e-3F;
            std::size_t kept = 0;
            for (std::size_t k = 0; k < steps; ++k)
            {
                // Null space of the (full row rank) Jacobian: the trailing columns of Q in J^T = Q R.
                const Eigen::HouseholderQR<Eigen::MatrixXf> qr(jacobian(q).transpose());
                const Eigen::Index free = static_cast<Eigen::Index>(d) - static_cast<Eigen::Index>(rows_);
                if (free <= 0)
                {
                    break;
                }

                const Eigen::MatrixXf N = Eigen::MatrixXf(qr.householderQ()).rightCols(free);
                Eigen::VectorXf g(free);
                std::array<std::array<float, d>, 2> pm;
                std::vector<std::array<float, d>> probes;
                probes.reserve(2 * free);
                for (Eigen::Index c = 0; c < free; ++c)
                {
                    pm[0] = q;
                    pm[1] = q;
                    for (auto j = 0U; j < d; ++j)
                    {
                        pm[0][j] += h * N(j, c);
                        pm[1][j] -= h * N(j, c);
                    }

                    probes.push_back(pm[0]);
                    probes.push_back(pm[1]);
                }

                const auto effort = static_efforts(probes);
                for (Eigen::Index c = 0; c < free; ++c)
                {
                    g[c] = (effort[2 * c] - effort[2 * c + 1]) / (2.F * h);
                }

                if (not (g.norm() > 1e-9F))
                {
                    break;
                }

                const Eigen::VectorXf delta = -(step / g.norm()) * (N * g);
                std::array<float, d> trial;
                for (auto j = 0U; j < d; ++j)
                {
                    trial[j] = q[j] + delta[j];
                }

                const auto rest = rest_state(trial);
                if (not rest)
                {
                    break;
                }

                const float r = static_ratio(*rest);
                if (not (r < ratio))
                {
                    break;
                }

                q = *rest;
                ratio = r;
                ++kept;
            }

            return kept;
        }

        // Stacked equality Jacobian (rows x d) at q.
        auto jacobian(const std::array<float, d> &q) const noexcept -> Eigen::MatrixXf
        {
            AmbientBlock qb;
            const AmbientConfiguration cfg(q);
            for (auto i = 0U; i < d; ++i)
            {
                qb[i] = cfg.broadcast(i);
            }

            lp_.constraints.evaluate_equality_position(qb);
            lp_.constraints.extract_equality(0, F_.data(), J_.data(), gamma_.data());
            return Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
                J_.data(), rows_, d);
        }

        // Joint-space inertia M(q) (with payloads), assembled from RNEA columns:
        // M e_i = tau(q, 0, e_i) - tau(q, 0, 0).
        auto inertia(const std::array<float, d> &q) const noexcept -> Eigen::MatrixXf
        {
            if constexpr (has_torques_block<ZRobot, rake>::value)
            {
                // Columns rake at a time: evaluation c < d accelerates joint c, evaluation d is the bias.
                Eigen::MatrixXf M(d, d);
                std::array<float, d> bias{};
                const auto payload = ZRobot::template broadcast_payload<rake>(payload_);
                alignas(FloatVectorAlignment) std::array<FloatT, 3 * d * rake> arr{};
                for (std::size_t c0 = 0; c0 <= d; c0 += rake)
                {
                    arr.fill(0.F);
                    for (std::size_t lane = 0; lane < rake; ++lane)
                    {
                        for (auto j = 0U; j < d; ++j)
                        {
                            arr[lane + j * rake] = q[j];
                        }

                        const std::size_t c = c0 + lane;
                        if (c < d)
                        {
                            arr[lane + (2 * d + c) * rake] = 1.F;
                        }
                    }

                    const auto tau = ZRobot::template torques_block<rake>(
                        typename ZRobot::template ConfigurationBlock<rake>(arr), payload);
                    for (auto r = 0U; r < d; ++r)
                    {
                        const auto lanes = tau[r].to_array();
                        for (std::size_t lane = 0; lane < rake and c0 + lane <= d; ++lane)
                        {
                            if (c0 + lane == d)
                            {
                                bias[r] = lanes[lane];
                            }
                            else
                            {
                                M(r, c0 + lane) = lanes[lane];
                            }
                        }
                    }
                }

                for (auto c = 0U; c < d; ++c)
                {
                    for (auto r = 0U; r < d; ++r)
                    {
                        M(r, c) -= bias[r];
                    }
                }

                return 0.5F * (M + M.transpose());
            }

            std::array<float, 3 * d> x{};
            std::copy(q.begin(), q.end(), x.begin());
            const auto bias = ZRobot::torques(x, payload_);

            Eigen::MatrixXf M(d, d);
            for (auto i = 0U; i < d; ++i)
            {
                x[2 * d + i] = 1.F;
                const auto tau = ZRobot::torques(x, payload_);
                x[2 * d + i] = 0.F;
                for (auto r = 0U; r < d; ++r)
                {
                    M(r, i) = tau[r] - bias[r];
                }
            }

            return 0.5F * (M + M.transpose());
        }

    private:
        // static_effort of several positions, rake per RNEA block call where the robot has one.
        auto static_efforts(const std::vector<std::array<float, d>> &qs) const noexcept -> std::vector<float>
        {
            std::vector<float> out(qs.size());
            if constexpr (has_torques_block<ZRobot, rake>::value)
            {
                const auto payload = ZRobot::template broadcast_payload<rake>(payload_);
                alignas(FloatVectorAlignment) std::array<FloatT, 3 * d * rake> arr{};
                for (std::size_t i0 = 0; i0 < qs.size(); i0 += rake)
                {
                    for (std::size_t lane = 0; lane < rake; ++lane)
                    {
                        const auto &q = qs[std::min(i0 + lane, qs.size() - 1)];
                        for (auto j = 0U; j < d; ++j)
                        {
                            arr[lane + j * rake] = q[j];
                        }
                    }

                    const auto tau = ZRobot::template torques_block<rake>(
                        typename ZRobot::template ConfigurationBlock<rake>(arr), payload);
                    std::array<float, rake> sum{};
                    for (auto j = 0U; j < d; ++j)
                    {
                        const auto lanes = tau[j].to_array();
                        const float limit = ZRobot::effort_limits[j] * ZRobot::effort_scale;
                        for (std::size_t lane = 0; lane < rake; ++lane)
                        {
                            const float r = std::abs(lanes[lane]) / limit;
                            const float r2 = r * r;
                            sum[lane] += r2 * r2 * r2 * r2;
                        }
                    }

                    for (std::size_t lane = 0; lane < rake and i0 + lane < qs.size(); ++lane)
                    {
                        out[i0 + lane] = sum[lane];
                    }
                }
            }
            else
            {
                for (std::size_t i = 0; i < qs.size(); ++i)
                {
                    out[i] = static_effort(qs[i]);
                }
            }

            return out;
        }

        // Smooth maximum of the static torque ratios: sum_i (|tau_i(q, 0, 0)| / limit_i)^8.
        auto static_effort(const std::array<float, d> &q) const noexcept -> float
        {
            std::array<float, 3 * d> x{};
            std::copy(q.begin(), q.end(), x.begin());
            const auto tau = ZRobot::torques(x, payload_);
            float sum = 0.F;
            for (auto i = 0U; i < d; ++i)
            {
                const float r = std::abs(tau[i]) / (ZRobot::effort_limits[i] * ZRobot::effort_scale);
                const float r2 = r * r;
                sum += r2 * r2 * r2 * r2;
            }

            return sum;
        }

        static auto exact_settings(ChartSettings settings) noexcept -> ChartSettings
        {
            settings.exact_chart = true;
            return settings;
        }

        inline static auto join(const std::array<float, d> &q, const std::array<float, d> &v) noexcept
            -> Configuration
        {
            typename ZRobot::ConfigurationArray arr{};
            for (auto i = 0U; i < d; ++i)
            {
                arr[i] = q[i];
                arr[d + i] = v[i];
            }

            return Configuration(arr);
        }

        inline static void split(const Configuration &z, std::array<float, d> &q) noexcept
        {
            const auto arr = z.to_array();
            std::copy_n(arr.begin(), d, q.begin());
        }

        inline static void
        split(const Configuration &z, std::array<float, d> &q, std::array<float, d> &v) noexcept
        {
            const auto arr = z.to_array();
            std::copy_n(arr.begin(), d, q.begin());
            std::copy_n(arr.begin() + d, d, v.begin());
        }

        // Equality Jacobian J (Jm_), M^-1 J^T (MinvJt_), and J M^-1 J^T (S_) at q.
        auto jacobian_and_inertia(const std::array<float, d> &q) const noexcept -> bool
        {
            Jm_ = jacobian(q);
            const Eigen::LLT<Eigen::MatrixXf> llt(inertia(q));
            if (llt.info() != Eigen::Success)
            {
                return false;
            }

            MinvJt_ = llt.solve(Jm_.transpose());
            S_ = Jm_ * MinvJt_;
            return S_.allFinite();
        }

        LP lp_;
        std::optional<std::array<float, d>> seed_;
        float spread_ = 1.F;
        std::size_t refine_steps_ = 0;
        float refine_step_ = 0.1F;
        std::vector<float> rate_;
        float min_speed_;
        float max_speed_;
        collision::Environment<FloatVector<rake>> environment_;
        Payload payload_;
        std::size_t rows_;
        mutable std::vector<float> F_;
        mutable std::vector<float> gamma_;
        mutable std::vector<float> J_;
        mutable Eigen::MatrixXf Jm_;
        mutable Eigen::MatrixXf MinvJt_;
        mutable Eigen::MatrixXf S_;
    };

    // Payload-aware sampler: wraps a flask-robot sampler so a draw is kept only if the robot could
    // hold its payload at the draw's position at rest within (1 + slack) of its effort limits --
    // the static torque ratio max_i |tau_i(q, 0, 0)| / (effort_scale * tau_max_i) from the
    // payload-aware RNEA kernel. Heavy payloads leave a thin slice of capable postures (object
    // orientation x arm self-motion) that uniform draws rarely hit; slack > 0 still proposes
    // postures only momentum can carry the payload through. Per-end-effector payloads make each
    // arm's torque depend on its own joints alone, so raw (off-manifold) draws can be scored
    // directly; with `project` the position is first projected onto the planning constraints'
    // manifold (exact projection), at many times the cost. Velocities come from the inner
    // sampler (the chart planner tangent-projects targets). After max_attempts rejections the
    // last draw is returned. Holds mutable evaluation state: not thread-safe.
    template <typename ZRobot, std::size_t rake>
    struct PayloadShaped : public rng::RNG<ZRobot>
    {
        using Ambient = typename ZRobot::Ambient;
        static constexpr std::size_t d = Ambient::dimension;
        using Configuration = typename ZRobot::Configuration;

        PayloadShaped(
            typename rng::RNG<ZRobot>::Ptr inner,
            ConstraintSet<Ambient, rake> constraints,
            ChartSettings chart_settings,
            const collision::Environment<float> &environment,
            float slack,
            std::size_t max_attempts,
            bool project = false) noexcept
          : inner_(std::move(inner))
          , lp_(std::move(constraints), exact(chart_settings))
          , payload_(collision::attachment_payload<ZRobot::payload_dimension / 10>(environment))
          , slack_(slack)
          , max_attempts_(std::max<std::size_t>(max_attempts, 1))
          , project_(project)
        {
        }

        inline void reset() noexcept override final
        {
            inner_->reset();
        }

        inline auto next() noexcept -> FloatVector<ZRobot::dimension> override final
        {
            std::optional<Configuration> last;
            for (std::size_t a = 0; a < max_attempts_; ++a)
            {
                const Configuration z = inner_->next();
                const auto arr = z.to_array();
                typename ZRobot::ConfigurationArray rest{};
                std::copy_n(arr.begin(), d, rest.begin());

                Configuration zq(rest);
                if (project_ and not lp_.project(zq))
                {
                    continue;
                }

                const auto qa = zq.to_array();
                typename ZRobot::ConfigurationArray out{};
                std::copy_n(qa.begin(), d, out.begin());
                std::copy_n(arr.begin() + d, d, out.begin() + d);
                last = Configuration(out);

                if (static_ratio(qa) <= 1.F + slack_)
                {
                    return *last;
                }
            }

            return last ? *last : inner_->next();
        }

        // max_i |tau_i(q, 0, 0)| / (effort_scale * tau_max_i) with the payload.
        template <typename Array>
        auto static_ratio(const Array &q) const noexcept -> float
        {
            return zero_acceleration_torque_ratio<ZRobot>(q, std::array<float, d>{}, payload_);
        }

    private:
        static auto exact(ChartSettings settings) noexcept -> ChartSettings
        {
            settings.exact_chart = true;
            return settings;
        }

        typename rng::RNG<ZRobot>::Ptr inner_;
        ChartLocalPlanner<ZRobot, rake, ZRobot::resolution> lp_;
        typename ZRobot::Payload payload_;
        float slack_;
        std::size_t max_attempts_;
        bool project_;
    };
}  // namespace vamp::planning::constraint
