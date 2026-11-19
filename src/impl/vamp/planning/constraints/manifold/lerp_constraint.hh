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

    // Input layout of the generated bounded least-squares solvers of the segment form, whose unknowns are the joints and
    // t: the solve buffer (Jacobian with the column of t, then the error; the kernel's two last outputs, the allowed step
    // of t to the ends of its segment, follow in the buffer and are read from it), then the allowed step to the lower
    // bounds (lower - x) and to the upper bounds (upper - x) of every unknown: the joints' limits minus q, and for t
    // those two outputs of the kernel.
    template <typename Robot, std::size_t rake, std::size_t err_size>
    struct SegmentSolveInput
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t n_vars = Robot::dimension + 1;
        static constexpr std::size_t jac_size = err_size * n_vars;

        const SolveBuffer<rake, err_size + 2, jac_size> &solve;
        Block lower;
        Block upper;
        Row lower_t;
        Row upper_t;

        auto operator[](std::size_t index) const noexcept -> Row
        {
            if (index < jac_size)
            {
                return solve.jac[index];
            }

            index -= jac_size;
            if (index < err_size)
            {
                return solve.err[index];
            }

            index -= err_size;
            if (index < Robot::dimension)
            {
                return lower[index];
            }

            if (index == Robot::dimension)
            {
                return lower_t;
            }

            index -= n_vars;
            return (index < Robot::dimension) ? upper[index] : upper_t;
        }
    };

    // What the two lerp constraints share: the curve parameter t, given per lane, at which the error is evaluated; the
    // error and Jacobian come from one generated kernel that also evaluates the curve at t, and the steps from generated
    // solvers. Two forms:
    //
    //   fixed (the default): t is held at the value given (set_parameter), and a projection solves for the joints
    //       alone: the end-effector must reach the curve's pose at t.
    //   segment: t only picks the segment of the curve (the one the given t is on). The end-effector may be anywhere on
    //       that segment: t is an unknown next to the joints, bounded to the segment, and starts at the given value. The
    //       Jacobian has the column of t (the generated segment kernels and solvers), and after a projection
    //       projected_parameter() is the t each lane ended at, which the state takes.
    //
    // The derived constraint supplies, as private members visible through `friend Base`:
    //   - run_kernel(q, t, solve) and solve_step(solve, q, method, alpha), for the fixed form;
    //   - run_segment_kernel(q, t, t_segment, solve), solve_segment(solve, gradient, method) (the solvers other than the
    //     bounded one) and solve_segment_bounded(input, gradient), for the segment form.
    template <typename Derived, typename Robot, std::size_t rake, std::size_t err_size_>
    struct LerpConstraintBase : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = err_size_;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        // The segment form: the unknowns are the joints and t
        static constexpr std::size_t n_vars = Robot::dimension + 1;
        static constexpr std::size_t segment_jac_size = err_size * n_vars;
        using SegmentBuffer = SolveBuffer<rake, err_size + 2, segment_jac_size>;
        using SegmentGradient = FloatVector<rake, n_vars>;

        // The parameter, per lane, that the error is evaluated at, and in the segment form where a projection starts and
        // what picks its segment. Set by the planner before every projection (ConstraintSet::set_parameter);
        // clear_parameter() has nothing to undo.
        void set_parameter(const Row &parameter) const noexcept override
        {
            t_ = parameter.max(0.F).min(1.F);
            t_segment_ = t_;
        }

        // In the segment form, the t each lane ended at; otherwise the parameter given
        auto projected_parameter(const Row &given) const noexcept -> Row override
        {
            return segment_mode_ ? t_ : given;
        }

        // Whether this is the segment form
        auto segment_form() const noexcept -> bool
        {
            return segment_mode_;
        }

        // A copy that holds t at the value given: the fixed form, for the goal region at t = 1
        auto fixed_copy() const -> Derived
        {
            Derived copy(self());
            copy.segment_mode_ = false;
            return copy;
        }

        // The squared error at the current parameter (for the pose curve with the rotation rows weighted). Caches J and
        // err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            if (segment_mode_)
            {
                self().run_segment_kernel(q, t_, t_segment_, segment_solve_);
                return sum_of_squares(segment_solve_.err);
            }

            self().run_kernel(q, t_, solve_);
            return sum_of_squares(solve_.err);
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
            if (not segment_mode_)
            {
                self().solve_step(solve_, q, method, alpha);
                return;
            }

            SegmentGradient gradient;
            if (method == ProjMethod::BoundedLS)
            {
                SegmentSolveInput<Robot, rake, err_size> input{segment_solve_};
                for (auto i = 0U; i < Robot::dimension; ++i)
                {
                    input.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                    input.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                }

                // the kernel's allowed step of t to the start and to the end of its segment
                input.lower_t = segment_solve_.err[err_size];
                input.upper_t = segment_solve_.err[err_size + 1];
                self().solve_segment_bounded(input, gradient);
            }
            else
            {
                self().solve_segment(segment_solve_, gradient, method);
            }

            // t moves by its step, kept in the segment (the bounded solve already keeps it there; the others do not)
            const Row lower_t = t_ + segment_solve_.err[err_size];
            const Row upper_t = t_ + segment_solve_.err[err_size + 1];
            t_ = (t_ - gradient[Robot::dimension] * alpha).max(lower_t).min(upper_t);

            Block step_q;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                step_q[i] = gradient[i];
            }

            integrate_step<Robot, rake>(q, step_q, alpha);
        }

        auto n_rows() const noexcept -> std::size_t override
        {
            return err_size;
        }

        // The error rows pin the end-effector to the curve's pose at t (on the segment): all manifold-defining
        void active_rows(bool *rows) const noexcept override
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
        }

        void evaluate_error_jacobian(const Block &q) const noexcept override
        {
            if (segment_mode_)
            {
                self().run_segment_kernel(q, t_, t_segment_, segment_solve_);
            }
            else
            {
                self().run_kernel(q, t_, solve_);
            }
        }

        // The raw q-Jacobian of one lane (in the segment form without the column of t); at a fixed t the tangent space of
        // the manifold in q is its null space
        void extract_error_jacobian(std::size_t lane, float *err, float *jac) const noexcept override
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = segment_mode_ ? segment_solve_.err[{i, lane}] : solve_.err[{i, lane}];
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    jac[i * Robot::dimension + j] = segment_mode_ ? segment_solve_.jac[{i * n_vars + j, lane}]
                                                                  : solve_.jac[{i * Robot::dimension + j, lane}];
                }
            }
        }

    protected:
        auto self() const noexcept -> const Derived &
        {
            return static_cast<const Derived &>(*this);
        }

        template <typename Errors>
        static auto sum_of_squares(const Errors &err) noexcept -> Row
        {
            auto d = err[0] * err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + err[i] * err[i];
            }

            return d;
        }

        bool segment_mode_ = false;

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable SegmentBuffer segment_solve_;
        mutable Row t_ = Row::fill(0.F);
        mutable Row t_segment_ = Row::fill(0.F);  // the t that picks the segment, in the segment form
    };

    // Position constraint against the straight-line (lerp) curve through Robot::lerp_points points in R^3, at a curve
    // parameter that is given (fixed form), or with t free on the segment it starts on (segment form: see
    // LerpConstraintBase). The first end-effector's position must equal P(t). For planning in CurveSpace, where t is part
    // of the state and ConstraintSet::set_parameter hands it to the constraint per lane before every projection.
    //
    // The parameter t in [0, 1] is the normalized chord length along the polyline: tau_0 = 0, tau_{i + 1} - tau_i
    // proportional to |P_{i + 1} - P_i|. The points and their parameters go to the generated kernel as inputs, which
    // picks the segment of t, evaluates the lerp and the error, and differentiates it (recipe keys "lerp_r3_constraint"
    // and, for the segment form, "lerp_r3_segment_constraint"). The curve data is broadcast into the kernel's input once,
    // here; an evaluation fills only q and t.
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

        // The points the curve passes through, in order (distinct); `segment` selects the segment form
        explicit LerpR3Constraint(const std::array<Point, n_points> &points, bool segment = false)
        {
            this->segment_mode_ = segment;

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
                    segment_input_.points[i * 3 + axis] = input_.points[i * 3 + axis];
                }

                input_.taus[i] = Row::fill(static_cast<float>(taus_[i]));
                segment_input_.taus[i] = input_.taus[i];
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

        using Solve = SolveBuffer<rake, 3, 3 * Robot::dimension>;
        using SegmentBuffer = typename Base::SegmentBuffer;
        using SegmentGradient = typename Base::SegmentGradient;

        void run_kernel(const Block &q, const Row &t, Solve &solve) const noexcept
        {
            input_.q = q;
            input_.t = t;
            Robot::template lerp_r3_error<rake>(input_, solve);
        }

        void solve_step(const Solve &solve, Block &q, ProjMethod method, float alpha) const noexcept
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

        void run_segment_kernel(const Block &q, const Row &t, const Row &t_segment, SegmentBuffer &solve) const noexcept
        {
            segment_input_.q = q;
            segment_input_.t = t;
            segment_input_.t_segment = t_segment;
            Robot::template lerp_r3_segment_error<rake>(segment_input_, solve);
        }

        void solve_segment(const SegmentBuffer &solve, SegmentGradient &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::OuterLM:
                    Robot::template solve_lerp_r3_segment_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_lerp_r3_segment_error_gradient_descent<rake>(solve, gradient);
                    break;
                default:  // InnerLM (the bounded solve is solve_segment_bounded)
                    Robot::template solve_lerp_r3_segment_error_lm_inner<rake>(solve, gradient);
                    break;
            }
        }

        void solve_segment_bounded(
            const SegmentSolveInput<Robot, rake, 3> &input,
            SegmentGradient &gradient) const noexcept
        {
            Robot::template solve_lerp_r3_segment_error_bounded_ls<rake>(input, gradient);
        }

        // Input layout of the generated Robot::lerp_r3_error (and, with the segment's t, of lerp_r3_segment_error): q, t,
        // [t_segment], the points (x y z per point), their parameters
        template <bool Segment>
        struct InputT
        {
            Block q;
            Row t;
            Row t_segment;
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
                if constexpr (Segment)
                {
                    if (index == 0)
                    {
                        return t_segment;
                    }

                    index -= 1;
                }

                if (index < 3 * n_points)
                {
                    return points[index];
                }

                return taus[index - 3 * n_points];
            }
        };

        std::array<Eigen::Vector3d, n_points> points_;
        std::array<double, n_points> taus_;

        mutable InputT<false> input_;
        mutable InputT<true> segment_input_;
    };

    // Pose constraint against the straight-line curve through Robot::lerp_points poses in SO(3) x R^3 (positions lerped,
    // rotations slerped), at a curve parameter that is given (fixed form), or with t free on the segment it starts on
    // (segment form: see LerpConstraintBase): the first end-effector's pose must equal the curve's pose at t. The error is
    // the TSR error against that pose, [R_t^T (p - p_t); weight log(R_t^T R)], six rows; the weight (m / rad, adjustable)
    // trades a radian of rotation error against a metre of position error, and is also how rotation enters the length of
    // a segment: the parameters taus are the normalized lengths sqrt(|dp|^2 + weight^2 theta^2) of the segments, theta the
    // angle between a segment's rotations (recipe keys "lerp_so3r3_constraint" and "lerp_so3r3_segment_constraint").
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
            float rotation_weight = 1.F,
            bool segment = false)
        {
            this->segment_mode_ = segment;

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
                    segment_input_.positions[i * 3 + axis] = input_.positions[i * 3 + axis];
                }

                input_.quaternions[i * 4 + 0] = Row::fill(static_cast<float>(rotations_[i].w()));
                input_.quaternions[i * 4 + 1] = Row::fill(static_cast<float>(rotations_[i].x()));
                input_.quaternions[i * 4 + 2] = Row::fill(static_cast<float>(rotations_[i].y()));
                input_.quaternions[i * 4 + 3] = Row::fill(static_cast<float>(rotations_[i].z()));
                for (auto c = 0U; c < 4; ++c)
                {
                    segment_input_.quaternions[i * 4 + c] = input_.quaternions[i * 4 + c];
                }
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
                segment_input_.taus[i] = input_.taus[i];
            }

            input_.weight = Row::fill(weight_);
            segment_input_.weight = input_.weight;
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

        using Solve = SolveBuffer<rake, 6, 6 * Robot::dimension>;
        using SegmentBuffer = typename Base::SegmentBuffer;
        using SegmentGradient = typename Base::SegmentGradient;

        void run_kernel(const Block &q, const Row &t, Solve &solve) const noexcept
        {
            input_.q = q;
            input_.t = t;
            Robot::template lerp_so3r3_error<rake>(input_, solve);
        }

        void solve_step(const Solve &solve, Block &q, ProjMethod method, float alpha) const noexcept
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

        void run_segment_kernel(const Block &q, const Row &t, const Row &t_segment, SegmentBuffer &solve) const noexcept
        {
            segment_input_.q = q;
            segment_input_.t = t;
            segment_input_.t_segment = t_segment;
            Robot::template lerp_so3r3_segment_error<rake>(segment_input_, solve);
        }

        void solve_segment(const SegmentBuffer &solve, SegmentGradient &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::OuterLM:
                    Robot::template solve_lerp_so3r3_segment_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_lerp_so3r3_segment_error_gradient_descent<rake>(solve, gradient);
                    break;
                default:  // InnerLM (the bounded solve is solve_segment_bounded)
                    Robot::template solve_lerp_so3r3_segment_error_lm_inner<rake>(solve, gradient);
                    break;
            }
        }

        void solve_segment_bounded(
            const SegmentSolveInput<Robot, rake, 6> &input,
            SegmentGradient &gradient) const noexcept
        {
            Robot::template solve_lerp_so3r3_segment_error_bounded_ls<rake>(input, gradient);
        }

        // Input layout of the generated Robot::lerp_so3r3_error (and, with the segment's t, of lerp_so3r3_segment_error):
        // q, t, [t_segment], the weight, the positions (x y z per keyframe), the quaternions (w x y z per keyframe), the
        // parameters
        template <bool Segment>
        struct InputT
        {
            Block q;
            Row t;
            Row t_segment;
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

                index -= 1;
                if constexpr (Segment)
                {
                    if (index == 0)
                    {
                        return t_segment;
                    }

                    index -= 1;
                }

                if (index == 0)
                {
                    return weight;
                }

                index -= 1;
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

        mutable InputT<false> input_;
        mutable InputT<true> segment_input_;
    };
}  // namespace vamp::planning::constraint
