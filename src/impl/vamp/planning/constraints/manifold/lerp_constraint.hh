#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    namespace lerp_detail
    {
        // The segment a parameter is on: the first whose end is after t (t = 1 stays in the last one)
        template <std::size_t N>
        inline auto segment_of(const std::array<double, N> &taus, double t) noexcept -> std::size_t
        {
            std::size_t segment = 0;
            while (segment + 2 < N and t >= taus[segment + 1])
            {
                ++segment;
            }

            return segment;
        }

        // The parameter in [0, 1] that minimizes `distance2(t)` along a curve: a scan, then a golden-section refinement
        // of the best bracket. For a curve that comes near itself it can find the wrong branch. Returns {t, distance2(t)}.
        template <typename Distance2>
        inline auto minimize_over_curve(Distance2 distance2) -> std::pair<double, double>
        {
            constexpr int samples = 2000;
            constexpr double inverse_phi = 0.6180339887498949;

            int best = 0;
            double best_distance2 = std::numeric_limits<double>::max();
            for (int k = 0; k <= samples; ++k)
            {
                const double d = distance2(static_cast<double>(k) / samples);
                if (d < best_distance2)
                {
                    best_distance2 = d;
                    best = k;
                }
            }

            double lo = std::max(0.0, static_cast<double>(best - 1) / samples);
            double hi = std::min(1.0, static_cast<double>(best + 1) / samples);
            for (int iteration = 0; iteration < 40; ++iteration)
            {
                const double a = hi - inverse_phi * (hi - lo);
                const double b = lo + inverse_phi * (hi - lo);
                if (distance2(a) < distance2(b))
                {
                    hi = b;
                }
                else
                {
                    lo = a;
                }
            }

            const double t = 0.5 * (lo + hi);
            return {t, distance2(t)};
        }
    }  // namespace lerp_detail

    // What the two lerp constraints share: the curve parameter t, given per lane, at which the error is evaluated and
    // the steps hold (a projection solves for the joints alone); the error and Jacobian come from one generated kernel
    // that also evaluates the curve at t, and the steps from generated solvers. The derived constraint supplies, as
    // private members visible through `friend Base`:
    //   - run_kernel(q, t, solve): evaluate the error/Jacobian kernel at (q, t) into `solve`;
    //   - solve_step(solve, q, method, alpha): dispatch to the generated projection solvers and move q.
    template <typename Derived, typename Robot, std::size_t rake, std::size_t err_size_>
    struct LerpConstraintBase : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = err_size_;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        // The parameter, per lane, that the error is evaluated at and the steps hold. Set by the planner before every
        // projection (ConstraintSet::set_parameter); clear_parameter() has nothing to undo, since this constraint never
        // solves for t.
        void set_parameter(const Row &parameter) const noexcept override
        {
            t_ = parameter.max(0.F).min(1.F);
        }

        // The squared error at the current parameter (for the pose curve with the rotation rows weighted). Caches J and
        // err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            self().run_kernel(q, t_, solve_);

            auto d = solve_.err[0] * solve_.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve_.err[i] * solve_.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
            self().solve_step(solve_, q, method, alpha);
        }

        auto n_rows() const noexcept -> std::size_t override
        {
            return err_size;
        }

        // The error rows pin the end-effector to the curve's pose at t: all manifold-defining
        void active_rows(bool *rows) const noexcept override
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
        }

        void mask_jacobian(const bool *pinned) const noexcept override
        {
            mask_pinned_columns<Robot>(solve_, pinned);
        }

        void evaluate_error_jacobian(const Block &q) const noexcept override
        {
            self().run_kernel(q, t_, solve_);
        }

        // At a fixed t the tangent space of the manifold in q is the null space of the raw q-Jacobian
        void extract_error_jacobian(std::size_t lane, float *err, float *jac) const noexcept override
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = solve_.err[{i, lane}];
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    jac[i * Robot::dimension + j] = solve_.jac[{i * Robot::dimension + j, lane}];
                }
            }
        }

    protected:
        auto self() const noexcept -> const Derived &
        {
            return static_cast<const Derived &>(*this);
        }

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.F);
    };

    // Position constraint against the straight-line (lerp) curve through Robot::lerp_points points in R^3, at a curve
    // parameter that is given: the first end-effector's position must equal P(t). For planning in CurveSpace, where t is
    // part of the state and ConstraintSet::set_parameter hands it to the constraint per lane before every projection.
    //
    // The parameter t in [0, 1] is the normalized chord length along the polyline: tau_0 = 0, tau_{i + 1} - tau_i
    // proportional to |P_{i + 1} - P_i|. The points and their parameters go to the generated kernel as inputs, which
    // picks the segment of t, evaluates the lerp and the error, and differentiates it (recipe key "lerp_r3_constraint").
    // The curve data is broadcast into the kernel's input once, here; an evaluation fills only q and t.
    //
    // There is no closest-point search in planning: a state says where on the curve it is. closest_parameter() is the
    // host-side scan for the t of a configuration that is not given one (a planner's start).
    template <typename Robot, std::size_t rake>
    struct LerpR3Constraint final : LerpConstraintBase<LerpR3Constraint<Robot, rake>, Robot, rake, 3>
    {
        using Base = LerpConstraintBase<LerpR3Constraint<Robot, rake>, Robot, rake, 3>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;
        using Configuration = typename Robot::Configuration;
        using Point = std::array<float, 3>;

        static constexpr std::size_t n_points = Robot::lerp_points;

        // The points the curve passes through, in order (distinct)
        explicit LerpR3Constraint(const std::array<Point, n_points> &points)
        {
            for (auto i = 0U; i < n_points; ++i)
            {
                points_[i] = Eigen::Vector3d(points[i][0], points[i][1], points[i][2]);
            }

            taus_[0] = 0.0;
            for (auto i = 1U; i < n_points; ++i)
            {
                const double distance = (points_[i] - points_[i - 1]).norm();
                if (not(distance > 1e-9))
                {
                    throw std::invalid_argument("consecutive lerp points must be distinct");
                }

                taus_[i] = taus_[i - 1] + distance;
            }

            for (auto i = 0U; i < n_points; ++i)
            {
                taus_[i] /= taus_[n_points - 1];
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    input_.points[i * 3 + axis] = Row::fill(static_cast<float>(points_[i][axis]));
                }

                input_.taus[i] = Row::fill(static_cast<float>(taus_[i]));
            }
        }

        // The parameters of the points
        auto parameters() const noexcept -> const std::array<double, n_points> &
        {
            return taus_;
        }

        // The curve on the host, in double: the point at t (clamped to [0, 1]). The same lerp as the generated kernel, for
        // planners' starts, tests and plots.
        auto value(double t) const noexcept -> Eigen::Vector3d
        {
            t = std::clamp(t, 0.0, 1.0);
            const auto segment = lerp_detail::segment_of(taus_, t);
            const double s = (t - taus_[segment]) / (taus_[segment + 1] - taus_[segment]);
            return points_[segment] + s * (points_[segment + 1] - points_[segment]);
        }

        // The closest point of the curve to the end-effector at q: (t, squared distance). For a curve that comes near
        // itself it can find the wrong branch: pass the t explicitly there.
        auto closest_parameter(const Configuration &q) const -> std::pair<float, float>
        {
            std::array<float, Robot::dimension> values;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                values[j] = array[j];
            }

            const Eigen::Vector3d x = Robot::eefk(values).translation().template cast<double>();
            const auto [t, distance2] =
                lerp_detail::minimize_over_curve([&](double s) { return (x - value(s)).squaredNorm(); });
            return {static_cast<float>(t), static_cast<float>(distance2)};
        }

    private:
        friend Base;

        void run_kernel(const Block &q, const Row &t, SolveBuffer<rake, 3, 3 * Robot::dimension> &solve) const noexcept
        {
            input_.q = q;
            input_.t = t;
            Robot::template lerp_r3_error<rake>(input_, solve);
        }

        void solve_step(
            const SolveBuffer<rake, 3, 3 * Robot::dimension> &solve,
            Block &q,
            ProjMethod method,
            float alpha) const noexcept
        {
            Block gradient;
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_lerp_r3_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_lerp_r3_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_lerp_r3_error_gradient_descent<rake>(solve, gradient);
                    break;
                case ProjMethod::BoundedLS:
                {
                    BoundedSolveInput<Robot, rake, 3, 3 * Robot::dimension> bounded{solve};
                    for (auto i = 0U; i < Robot::dimension; ++i)
                    {
                        bounded.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                        bounded.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                    }

                    Robot::template solve_lerp_r3_error_bounded_ls<rake>(bounded, gradient);
                    break;
                }
            }

            integrate_step<Robot, rake>(q, gradient, alpha);
        }

        // Input layout of the generated Robot::lerp_r3_error: q, t, the points (x y z per point), their parameters
        struct Input
        {
            Block q;
            Row t;
            FloatVector<rake, 3 * n_points> points;
            FloatVector<rake, n_points> taus;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                index -= Robot::dimension;
                if (index == 0)
                {
                    return t;
                }

                index -= 1;
                if (index < 3 * n_points)
                {
                    return points[index];
                }

                return taus[index - 3 * n_points];
            }
        };

        std::array<Eigen::Vector3d, n_points> points_;
        std::array<double, n_points> taus_;

        mutable Input input_;
    };

    // Pose constraint against the straight-line curve through Robot::lerp_points poses in SO(3) x R^3 (positions lerped,
    // rotations slerped), at a curve parameter that is given: the first end-effector's pose must equal the curve's pose
    // at t. The error is the TSR error against that pose, [R_t^T (p - p_t); weight log(R_t^T R)], six rows; the weight
    // (m / rad, adjustable) trades a radian of rotation error against a metre of position error, and is also how rotation
    // enters the length of a segment: the parameters taus are the normalized lengths sqrt(|dp|^2 + weight^2 theta^2) of the
    // segments, theta the angle between a segment's rotations (recipe key "lerp_so3r3_constraint").
    //
    // The keyframes' quaternions are normalized and put on one side (each with a non-negative dot product with the
    // previous one), so the slerp takes the short way, and consecutive rotations more than 175 degrees apart are
    // rejected, where the way round is ambiguous and the error's log map degenerates. Poses must be distinct: a segment
    // needs a positive length. The taus are computed here, on the host, and handed to the kernel with the weight, the
    // positions and the quaternions, which are broadcast into its input once; an evaluation fills only q and t.
    template <typename Robot, std::size_t rake>
    struct LerpSO3R3Constraint final : LerpConstraintBase<LerpSO3R3Constraint<Robot, rake>, Robot, rake, 6>
    {
        using Base = LerpConstraintBase<LerpSO3R3Constraint<Robot, rake>, Robot, rake, 6>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;
        using Configuration = typename Robot::Configuration;
        using Position = std::array<float, 3>;
        using Quaternion = std::array<float, 4>;  // w x y z

        static constexpr std::size_t n_points = Robot::lerp_points;

        LerpSO3R3Constraint(
            const std::array<Position, n_points> &positions,
            const std::array<Quaternion, n_points> &quaternions,
            float rotation_weight = 1.F)
        {
            for (auto i = 0U; i < n_points; ++i)
            {
                positions_[i] = Eigen::Vector3d(positions[i][0], positions[i][1], positions[i][2]);

                Eigen::Quaterniond rotation(quaternions[i][0], quaternions[i][1], quaternions[i][2], quaternions[i][3]);
                if (not(rotation.norm() > 1e-9))
                {
                    throw std::invalid_argument("a keyframe's quaternion must not be zero");
                }

                rotation.normalize();
                if (i > 0 and rotation.dot(rotations_[i - 1]) < 0.0)
                {
                    rotation.coeffs() *= -1.0;
                }

                rotations_[i] = rotation;
            }

            for (auto i = 1U; i < n_points; ++i)
            {
                if (rotations_[i].angularDistance(rotations_[i - 1]) > 175.0 * 3.14159265358979323846 / 180.0)
                {
                    throw std::invalid_argument("consecutive keyframe rotations must be less than 175 degrees apart");
                }
            }

            for (auto i = 0U; i < n_points; ++i)
            {
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    input_.positions[i * 3 + axis] = Row::fill(static_cast<float>(positions_[i][axis]));
                }

                input_.quaternions[i * 4 + 0] = Row::fill(static_cast<float>(rotations_[i].w()));
                input_.quaternions[i * 4 + 1] = Row::fill(static_cast<float>(rotations_[i].x()));
                input_.quaternions[i * 4 + 2] = Row::fill(static_cast<float>(rotations_[i].y()));
                input_.quaternions[i * 4 + 3] = Row::fill(static_cast<float>(rotations_[i].z()));
            }

            set_rotation_weight(rotation_weight);
        }

        // The weight (m / rad) of the rotation error, and with it the parameters of the keyframes: they are recomputed
        void set_rotation_weight(float rotation_weight)
        {
            if (not(rotation_weight >= 0.F))
            {
                throw std::invalid_argument("the rotation weight must not be negative");
            }

            weight_ = rotation_weight;

            taus_[0] = 0.0;
            for (auto i = 1U; i < n_points; ++i)
            {
                const double theta = rotations_[i].angularDistance(rotations_[i - 1]);
                const double length2 = (positions_[i] - positions_[i - 1]).squaredNorm() +
                                       static_cast<double>(weight_) * weight_ * theta * theta;
                if (not(length2 > 1e-18))
                {
                    throw std::invalid_argument("consecutive keyframe poses must be distinct");
                }

                taus_[i] = taus_[i - 1] + std::sqrt(length2);
            }

            for (auto i = 0U; i < n_points; ++i)
            {
                taus_[i] /= taus_[n_points - 1];
                input_.taus[i] = Row::fill(static_cast<float>(taus_[i]));
            }

            input_.weight = Row::fill(weight_);
        }

        auto rotation_weight() const noexcept -> float
        {
            return weight_;
        }

        // The parameters of the keyframes
        auto parameters() const noexcept -> const std::array<double, n_points> &
        {
            return taus_;
        }

        // The curve on the host, in double: the pose at t (clamped to [0, 1]), the lerp of the positions and the slerp of
        // the rotations. For planners' starts, tests and plots.
        auto value(double t) const noexcept -> std::pair<Eigen::Vector3d, Eigen::Quaterniond>
        {
            t = std::clamp(t, 0.0, 1.0);
            const auto segment = lerp_detail::segment_of(taus_, t);
            const double s = (t - taus_[segment]) / (taus_[segment + 1] - taus_[segment]);
            return {
                positions_[segment] + s * (positions_[segment + 1] - positions_[segment]),
                rotations_[segment].slerp(s, rotations_[segment + 1])};
        }

        // The weighted error against the curve's pose at t, in double, written out from its definition:
        // [R_t^T (p - p_t); weight log(R_t^T R)] for the end-effector at (position, rotation)
        auto error(const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation, double t) const noexcept
            -> Eigen::Matrix<double, 6, 1>
        {
            const auto [target_position, target_rotation] = value(t);
            const Eigen::Matrix3d target = target_rotation.toRotationMatrix();

            const Eigen::AngleAxisd relative(target.transpose() * rotation.toRotationMatrix());
            Eigen::Matrix<double, 6, 1> result;
            result.template head<3>() = target.transpose() * (position - target_position);
            result.template tail<3>() = static_cast<double>(weight_) * relative.angle() * relative.axis();
            return result;
        }

        // The closest pose of the curve to the end-effector at q, in the curve's metric (position distance squared plus
        // the weighted rotation angle squared): (t, squared distance).
        auto closest_parameter(const Configuration &q) const -> std::pair<float, float>
        {
            std::array<float, Robot::dimension> values;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                values[j] = array[j];
            }

            const auto pose = Robot::eefk(values);
            const Eigen::Vector3d x = pose.translation().template cast<double>();
            const Eigen::Quaterniond rotation(pose.rotation().template cast<double>());
            const auto [t, distance2] = lerp_detail::minimize_over_curve(
                [&](double s)
                {
                    const auto [position, target_rotation] = value(s);
                    const double theta = target_rotation.angularDistance(rotation);
                    return (x - position).squaredNorm() + static_cast<double>(weight_) * weight_ * theta * theta;
                });
            return {static_cast<float>(t), static_cast<float>(distance2)};
        }

    private:
        friend Base;

        void run_kernel(const Block &q, const Row &t, SolveBuffer<rake, 6, 6 * Robot::dimension> &solve) const noexcept
        {
            input_.q = q;
            input_.t = t;
            Robot::template lerp_so3r3_error<rake>(input_, solve);
        }

        void solve_step(
            const SolveBuffer<rake, 6, 6 * Robot::dimension> &solve,
            Block &q,
            ProjMethod method,
            float alpha) const noexcept
        {
            Block gradient;
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_lerp_so3r3_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_lerp_so3r3_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_lerp_so3r3_error_gradient_descent<rake>(solve, gradient);
                    break;
                case ProjMethod::BoundedLS:
                {
                    BoundedSolveInput<Robot, rake, 6, 6 * Robot::dimension> bounded{solve};
                    for (auto i = 0U; i < Robot::dimension; ++i)
                    {
                        bounded.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                        bounded.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                    }

                    Robot::template solve_lerp_so3r3_error_bounded_ls<rake>(bounded, gradient);
                    break;
                }
            }

            integrate_step<Robot, rake>(q, gradient, alpha);
        }

        // Input layout of the generated Robot::lerp_so3r3_error: q, t, the weight, the positions (x y z per keyframe),
        // the quaternions (w x y z per keyframe), the parameters
        struct Input
        {
            Block q;
            Row t;
            Row weight;
            FloatVector<rake, 3 * n_points> positions;
            FloatVector<rake, 4 * n_points> quaternions;
            FloatVector<rake, n_points> taus;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                index -= Robot::dimension;
                if (index == 0)
                {
                    return t;
                }

                if (index == 1)
                {
                    return weight;
                }

                index -= 2;
                if (index < 3 * n_points)
                {
                    return positions[index];
                }

                index -= 3 * n_points;
                if (index < 4 * n_points)
                {
                    return quaternions[index];
                }

                return taus[index - 4 * n_points];
            }
        };

        std::array<Eigen::Vector3d, n_points> positions_;
        std::array<Eigen::Quaterniond, n_points> rotations_;
        std::array<double, n_points> taus_;
        float weight_ = 1.F;

        mutable Input input_;
    };
}  // namespace vamp::planning::constraint
