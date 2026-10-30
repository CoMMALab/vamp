#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // The straight-line interpolation of poses in R^3 x SO(3) as a curve P(t), t in [0, 1]: the position moves along
    // the polyline of the keyframe positions and the rotation along the slerp (the shortest-arc geodesic of SO(3),
    // at constant angular velocity) between consecutive keyframe rotations, with one parameter per segment. Position
    // is continuous and rotation is continuous, with the velocity jumping at every keyframe. Any number (at least
    // two) of keyframes.
    //
    // t is the chord-length parameter in a metric that mixes meters and radians: a segment's length is
    //   sqrt(|dp|^2 + w^2 theta^2),
    // dp the change of position, theta the angle between the keyframe rotations and w = rotation_weight in meters per
    // radian. tau_0 = 0, tau_{i+1} - tau_i proportional to the segment's length, normalized to tau_last = 1, so a
    // pose moves at constant speed (in that metric) along the curve as t grows. The same metric and weight are the
    // ones of the rotation rows of PosePolylineConstraint's error.
    //
    // Keyframe rotations are quaternions (w, x, y, z). They are normalized, and signed so that consecutive ones have a
    // positive dot product (q and -q are the same rotation): the shortest arc. A pair that is nearly 180 degrees
    // apart (more than 175) has no unique shortest arc and is rejected.
    struct PosePolylineCurve
    {
        using Point = std::array<float, 3>;
        using Quaternion = std::array<float, 4>;  // w, x, y, z

        struct Pose
        {
            Eigen::Vector3d position;
            Eigen::Quaterniond rotation;
        };

        PosePolylineCurve(
            const std::vector<Point> &points,
            const std::vector<Quaternion> &rotations,
            double rotation_weight)
          : weight_(rotation_weight)
        {
            if (points.size() < 2)
            {
                throw std::invalid_argument("a pose polyline needs at least two keyframes");
            }

            if (points.size() != rotations.size())
            {
                throw std::invalid_argument("a pose polyline needs one rotation per position");
            }

            if (not(rotation_weight > 0.0))
            {
                throw std::invalid_argument("the rotation weight must be positive");
            }

            positions_.reserve(points.size());
            rotations_.reserve(points.size());
            for (std::size_t i = 0; i < points.size(); ++i)
            {
                positions_.emplace_back(points[i][0], points[i][1], points[i][2]);

                Eigen::Quaterniond rotation(rotations[i][0], rotations[i][1], rotations[i][2], rotations[i][3]);
                if (not(rotation.norm() > 1e-6))
                {
                    throw std::invalid_argument("a keyframe rotation must be a nonzero quaternion");
                }

                rotation.normalize();
                if (i > 0 and rotation.dot(rotations_.back()) < 0.0)
                {
                    rotation.coeffs() = -rotation.coeffs();
                }

                rotations_.push_back(rotation);
            }

            tau_.assign(positions_.size(), 0.0);
            for (std::size_t i = 1; i < positions_.size(); ++i)
            {
                // |dot| = cos(theta / 2); 0.0436 is cos(87.5 degrees): theta > 175 degrees
                if (rotations_[i].dot(rotations_[i - 1]) < 0.0436)
                {
                    throw std::invalid_argument(
                        "consecutive keyframe rotations are nearly 180 degrees apart: add an intermediate keyframe");
                }

                const double length = segment_length(i - 1);
                if (not(length > 1e-9))
                {
                    throw std::invalid_argument("consecutive keyframe poses must be distinct");
                }

                tau_[i] = tau_[i - 1] + length;
            }

            for (auto &tau : tau_)
            {
                tau /= tau_.back();
            }
        }

        auto size() const noexcept -> std::size_t
        {
            return positions_.size();
        }

        auto rotation_weight() const noexcept -> double
        {
            return weight_;
        }

        // The segment i with tau_i <= t < tau_{i + 1} (t clamped to [0, 1]); t = 1 belongs to the last one
        auto segment_of(double t) const noexcept -> std::size_t
        {
            t = std::clamp(t, 0.0, 1.0);

            const auto upper = std::upper_bound(tau_.begin(), tau_.end(), t);
            const std::size_t segment =
                (upper == tau_.begin()) ? 0 : static_cast<std::size_t>(upper - tau_.begin()) - 1;
            return std::min(segment, positions_.size() - 2);
        }

        // The parameter t at fraction s of segment i
        auto parameter_on(std::size_t segment, double s) const noexcept -> double
        {
            return tau_[segment] + s * (tau_[segment + 1] - tau_[segment]);
        }

        // The range of t that segment i covers: [segment_begin(i), segment_end(i)]
        auto segment_begin(std::size_t segment) const noexcept -> double
        {
            return tau_[segment];
        }

        auto segment_end(std::size_t segment) const noexcept -> double
        {
            return tau_[segment + 1];
        }

        // How the pose moves with t on segment i, a constant: the position's velocity and the rotation's angular
        // velocity in the frame of the pose itself (the slerp is R(t) = R_i exp((t - tau_i) rotation), so the
        // rotation vector per unit t)
        struct Rate
        {
            Eigen::Vector3d position;
            Eigen::Vector3d rotation;
        };

        auto rate(std::size_t segment) const noexcept -> Rate
        {
            const double length = tau_[segment + 1] - tau_[segment];
            const Eigen::AngleAxisd relative(rotations_[segment].conjugate() * rotations_[segment + 1]);
            return {
                (positions_[segment + 1] - positions_[segment]) / length,
                relative.angle() * relative.axis() / length};
        }

        // The pose at t (clamped to [0, 1])
        auto value(double t) const noexcept -> Pose
        {
            t = std::clamp(t, 0.0, 1.0);

            const std::size_t segment = segment_of(t);
            const double s = (t - tau_[segment]) / (tau_[segment + 1] - tau_[segment]);
            return pose_on(segment, s);
        }

        // The closest pose of segment i to (position, rotation) in the curve's metric: (s, squared distance), s the
        // fraction in [0, 1] along the segment. `samples` points of a scan, then a golden-section refinement of the
        // best bracket (the squared distance along a segment is smooth and has one minimum in practice).
        auto closest_on_segment(
            std::size_t segment,
            const Eigen::Vector3d &position,
            const Eigen::Quaterniond &rotation,
            int samples = 32) const noexcept -> std::pair<double, double>
        {
            constexpr double inverse_phi = 0.6180339887498949;

            const auto distance2 = [&](double s)
            { return squared_distance(pose_on(segment, s), position, rotation); };

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

            const double s = 0.5 * (lo + hi);
            const double d = distance2(s);
            return (d <= best_distance2) ? std::make_pair(s, d)
                                         : std::make_pair(static_cast<double>(best) / samples, best_distance2);
        }

        // The closest pose of the curve to (position, rotation) in the curve's metric, |dp|^2 + w^2 theta^2:
        // (t, that squared distance). The closest pose of every segment, the best of them; for a curve that comes
        // near itself it can find the wrong branch.
        auto closest(const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation) const noexcept
            -> std::pair<double, double>
        {
            double best_t = 0.0;
            double best_distance2 = std::numeric_limits<double>::max();

            for (std::size_t i = 0; i + 1 < positions_.size(); ++i)
            {
                const auto [s, d] = closest_on_segment(i, position, rotation);
                if (d < best_distance2)
                {
                    best_distance2 = d;
                    best_t = parameter_on(i, s);
                }
            }

            return {best_t, best_distance2};
        }

    private:
        // The angle of the rotation between two unit quaternions
        static auto angle_between(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) noexcept -> double
        {
            return 2.0 * std::acos(std::min(1.0, std::abs(a.dot(b))));
        }

        auto squared_distance(const Pose &pose, const Eigen::Vector3d &position, const Eigen::Quaterniond &rotation)
            const noexcept -> double
        {
            const double theta = angle_between(pose.rotation, rotation);
            return (pose.position - position).squaredNorm() + weight_ * weight_ * theta * theta;
        }

        auto segment_length(std::size_t i) const noexcept -> double
        {
            const double theta = angle_between(rotations_[i], rotations_[i + 1]);
            return std::sqrt(
                (positions_[i + 1] - positions_[i]).squaredNorm() + weight_ * weight_ * theta * theta);
        }

        // The pose at fraction s of segment i: the position's lerp and the rotation's slerp
        auto pose_on(std::size_t i, double s) const noexcept -> Pose
        {
            return {
                (1.0 - s) * positions_[i] + s * positions_[i + 1],
                rotations_[i].slerp(s, rotations_[i + 1])};
        }

        double weight_;
        std::vector<Eigen::Vector3d> positions_;
        std::vector<Eigen::Quaterniond> rotations_;
        std::vector<double> tau_;
    };

    // Pose constraint against a PosePolylineCurve at a curve parameter that is given: the first end-effector's pose
    // must equal the curve's pose at t. For planning in Robot::SplineSpace, where t is part of the state and
    // ConstraintSet::set_parameter hands it to the constraint per lane before every projection; a projection solves
    // for the joints alone, and never searches for a closest point.
    //
    // The curve is evaluated on the host, once per lane when the parameter is set, and the pose is the reference
    // frame of the generated TSR kernel (Robot::tsr_error, the equality rows: bounds all zero). So the error is the
    // six rows of the pose of the end-effector in the target's frame, [translation; log of the rotation], and the
    // projection solvers (for a given t) are the TSR ones, the unchanged generated code of TaskSpaceConstraint: this
    // constraint needs no kernel of its own. The cricket recipe keys it needs are "constraints" (the TSR kernel),
    // "spline_constraint" (for SplineSpace) and "pose_slide_constraint" (the solvers with t as an unknown, used when
    // sliding).
    //
    // The rotation rows (and their Jacobian rows) are scaled by the curve's rotation weight, in meters per radian, so
    // the error is |dp|^2 + w^2 |log R|^2: the same metric that parametrizes the curve.
    //
    // Sliding (`slide`): the parameter given to set_parameter only says which segment the lane is on, and t is then an
    // unknown of the projection next to the joints, bounded to the segment, as in the SplineConstraint that solves for
    // t: the Jacobian has a column d(err)/dt after the joints' columns, and every step moves the joints and t together
    // (the generated Robot::solve_pose_slide_error_*; BoundedLS keeps t in the segment's range exactly, the other
    // methods clamp it after the step). The pose curve is evaluated at t once per iteration, on the host.
    //
    // The column of t is not in the kernel (the target is an input of it, not differentiated), so it is built on the
    // host from the segment's constant rate (rotation vector phi and velocity v per unit t) and the target's rotation
    // R_t at t: the translation rows are R_t^T (p - p_t), so d/dt = -phi x e_p - R_t^T v with e_p the translation
    // error; the rotation rows are w log(R_t^T R), so d/dt = -w phi to first order in the error, which is exact on the
    // curve. A column that is off only changes the step's quality, not where the projection ends.
    // The parameter each lane is left at is projected_parameter(): planners write it back into the state.
    //
    // closest_parameter() is the host-side search for the t of a configuration that is not given one (a planner's
    // start). Not thread-safe, like the other constraints: one instance per thread.
    template <typename Robot, std::size_t rake>
    struct PosePolylineConstraint final
      : HingedTSRConstraint<PosePolylineConstraint<Robot, rake>, Robot, rake, 6>
    {
        static_assert(Robot::n_eef == 1, "a pose curve constrains the end-effector: one end-effector");

        using Base = HingedTSRConstraint<PosePolylineConstraint<Robot, rake>, Robot, rake, 6>;
        using Block = typename Base::Block;
        using Row = typename Base::Row;
        using Configuration = typename Robot::Configuration;
        using Point = PosePolylineCurve::Point;
        using Quaternion = PosePolylineCurve::Quaternion;

        // This constraint has a generated bounded least-squares step (ProjMethod::BoundedLS).
        static constexpr bool bounded_ls = true;

        // The keyframes (positions and quaternions w, x, y, z), the rotation weight in meters per radian, and whether
        // the end-effector slides along the parameter's segment during a projection (see above)
        PosePolylineConstraint(
            const std::vector<Point> &points,
            const std::vector<Quaternion> &rotations,
            float rotation_weight = 0.1F,
            bool slide = false)
          : curve_(points, rotations, rotation_weight), weight_(Row::fill(rotation_weight)), slide_(slide)
        {
            // the end-effector frame itself (rTe = identity), with an equality on every row
            for (auto i = 0U; i < 7; ++i)
            {
                input.rTe[i] = Row::fill(i == 0 ? 1.F : 0.F);
            }

            for (auto i = 0U; i < 6; ++i)
            {
                input.lb[i] = Row::fill(0.F);
                input.ub[i] = Row::fill(0.F);
                this->equality_rows[i] = true;
            }

            set_parameter(t_);
        }

        auto curve() const noexcept -> const PosePolylineCurve &
        {
            return curve_;
        }

        auto slide() const noexcept -> bool
        {
            return slide_;
        }

        void set_slide(bool slide) noexcept
        {
            slide_ = slide;
            set_parameter(t_);
        }

        // The curve's pose at t
        auto value(double t) const noexcept -> PosePolylineCurve::Pose
        {
            return curve_.value(t);
        }

        // The closest pose of the curve to the end-effector at q: (t, squared distance in the curve's metric)
        auto closest_parameter(const Configuration &q) const -> std::pair<float, float>
        {
            std::array<float, Robot::dimension> values;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                values[j] = array[j];
            }

            const auto transform = Robot::eefk(values);
            const Eigen::Vector3d position = transform.translation().template cast<double>();
            const Eigen::Quaterniond rotation(transform.rotation().template cast<double>());
            const auto [t, distance2] = curve_.closest(position, rotation);
            return {static_cast<float>(t), static_cast<float>(distance2)};
        }

        // The parameter, per lane, that the error is evaluated at and the steps hold: the curve is evaluated at it
        // here, once, not on every error evaluation. Set by the planner before every projection; clear_parameter()
        // has nothing to undo, since this constraint never solves for t.
        void set_parameter(const Row &parameter) const noexcept override
        {
            t_ = parameter.clamp(0.F, 1.F);

            if (slide_)
            {
                // each lane's segment, from its t: the range t may move in, and how the pose moves with t there
                const auto parameters = t_.to_array();
                std::array<float, rake> lower;
                std::array<float, rake> upper;
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    const auto segment = curve_.segment_of(parameters[lane]);
                    lower[lane] = static_cast<float>(curve_.segment_begin(segment));
                    upper[lane] = static_cast<float>(curve_.segment_end(segment));
                    rate_[lane] = curve_.rate(segment);
                }

                lower_t_ = Row(lower);
                upper_t_ = Row(upper);

                // the hint may be a float rounding off its segment's range
                t_ = t_.max(lower_t_).min(upper_t_);
            }

            load_target();
        }

        // Sliding: the parameter each lane's projection left it at. Otherwise the one given.
        auto projected_parameter(const Row &given) const noexcept -> Row override
        {
            return slide_ ? t_ : given;
        }

        // Sliding: a step on the joints and t, from the cache of the last squared_error(). Otherwise the TSR step.
        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
            if (slide_)
            {
                slide_step(q, method, alpha);
                return;
            }

            Base::step(q, method, alpha);
        }

        // The column d(err)/dt of lane `lane` (6 entries, the unweighted-translation and weighted-rotation rows of the
        // error), at the cache of the last squared_error(): what a sliding projection appends to the Jacobian. For
        // tests, against the finite difference of the error with t.
        void parameter_jacobian(std::size_t lane, float *column) const noexcept
        {
            fill_slide_solve();
            for (auto i = 0U; i < 6; ++i)
            {
                column[i] = slide_solve_.jac[{i * (Robot::dimension + 1) + Robot::dimension, lane}];
            }
        }

    private:
        friend Base;

        // The target of every lane: the curve's pose at the lane's parameter, as [qw qx qy qz x y z]
        void load_target() const noexcept
        {
            const auto parameters = t_.to_array();

            std::array<std::array<float, rake>, 7> target;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                const auto pose = curve_.value(parameters[lane]);
                target_rotation_[lane] = pose.rotation.toRotationMatrix();
                target[0][lane] = static_cast<float>(pose.rotation.w());
                target[1][lane] = static_cast<float>(pose.rotation.x());
                target[2][lane] = static_cast<float>(pose.rotation.y());
                target[3][lane] = static_cast<float>(pose.rotation.z());
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    target[4 + axis][lane] = static_cast<float>(pose.position[axis]);
                }
            }

            for (auto i = 0U; i < 7; ++i)
            {
                input.wTr[i] = Row(target[i]);
            }
        }

        // The kernel's error and Jacobian, with the rotation rows (3 to 5) scaled by the rotation weight. Sliding: at
        // the target of the lanes' current parameters.
        void run_kernel() const noexcept
        {
            if (slide_)
            {
                load_target();
            }

            Robot::template tsr_error<rake>(input, this->solve);

            for (auto i = 3U; i < 6; ++i)
            {
                this->solve.err[i] = this->solve.err[i] * weight_;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    this->solve.jac[i * Robot::dimension + j] =
                        this->solve.jac[i * Robot::dimension + j] * weight_;
                }
            }
        }

        // The Jacobian with the column of t (see the class comment), and the error, from the cache of the last
        // squared_error() (so after its hinge, which masks NaN)
        void fill_slide_solve() const noexcept
        {
            constexpr std::size_t variables = Robot::dimension + 1;
            for (auto i = 0U; i < 6; ++i)
            {
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    slide_solve_.jac[i * variables + j] = this->solve.jac[i * Robot::dimension + j];
                }

                slide_solve_.err[i] = this->solve.err[i];
            }

            std::array<std::array<float, rake>, 3> translation_error;
            for (auto axis = 0U; axis < 3; ++axis)
            {
                translation_error[axis] = this->solve.err[axis].to_array();
            }

            const double weight = curve_.rotation_weight();
            std::array<std::array<float, rake>, 6> column;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                const Eigen::Vector3d e_p(
                    translation_error[0][lane], translation_error[1][lane], translation_error[2][lane]);
                const Eigen::Vector3d translation =
                    -rate_[lane].rotation.cross(e_p) - target_rotation_[lane].transpose() * rate_[lane].position;
                const Eigen::Vector3d rotation = -weight * rate_[lane].rotation;

                for (auto axis = 0U; axis < 3; ++axis)
                {
                    column[axis][lane] = static_cast<float>(translation[axis]);
                    column[3 + axis][lane] = static_cast<float>(rotation[axis]);
                }
            }

            for (auto i = 0U; i < 6; ++i)
            {
                slide_solve_.jac[i * variables + Robot::dimension] = Row(column[i]);
            }
        }

        // One sliding step on the joints and t: t stays in its segment
        void slide_step(Block &q, ProjMethod method, float alpha) const noexcept
        {
            constexpr std::size_t variables = Robot::dimension + 1;
            fill_slide_solve();

            FloatVector<rake, variables> gradient;
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_pose_slide_error_lm_inner<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_pose_slide_error_lm_outer<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_pose_slide_error_gradient_descent<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::BoundedLS:
                {
                    SlideInput slide_input{slide_solve_};
                    for (auto i = 0U; i < Robot::dimension; ++i)
                    {
                        slide_input.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                        slide_input.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                    }

                    slide_input.lower[Robot::dimension] = lower_t_ - t_;
                    slide_input.upper[Robot::dimension] = upper_t_ - t_;

                    Robot::template solve_pose_slide_error_bounded_ls<rake>(slide_input, gradient);
                    break;
                }
            }

            // the joints take their part of the step (clamped to the limits) ...
            Block joint_gradient;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                joint_gradient[i] = gradient[i];
            }

            integrate_step<Robot, rake>(q, joint_gradient, alpha);

            // ... and t the last entry, staying in its segment
            t_ = (t_ - gradient[Robot::dimension] * alpha).max(lower_t_).min(upper_t_);
        }

        // Input layout of the generated Robot::solve_pose_slide_error_bounded_ls: the Jacobian with t's column, then
        // the error, then lower - x and upper - x for the joints and t.
        struct SlideInput
        {
            static constexpr std::size_t variables = Robot::dimension + 1;
            static constexpr std::size_t solve_size = 6 * variables + 6;

            const SolveBuffer<rake, 6, 6 * variables> &solve;
            FloatVector<rake, variables> lower;
            FloatVector<rake, variables> upper;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < solve_size)
                {
                    return solve[index];
                }

                index -= solve_size;
                return (index < variables) ? lower[index] : upper[index - variables];
            }
        };

        // The step of ProjMethod::BoundedLS: `input` is a BoundedSolveInput (solve buffer + step limits).
        template <typename Input>
        void solve_step_bounded(Block &gradient, const Input &bounded_input) const noexcept
        {
            Robot::template solve_tsr_error_bounded_ls<rake>(bounded_input, gradient);
        }

        void solve_step(Block &gradient, ProjMethod method) const noexcept
        {
            switch (method)
            {
                case ProjMethod::BoundedLS:  // dispatched by step(), which has the joint limits
                case ProjMethod::InnerLM:
                    Robot::template solve_tsr_error_lm_inner<rake>(this->solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_tsr_error_lm_outer<rake>(this->solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_tsr_error_gradient_descent<rake>(this->solve, gradient);
                    break;
            }
        }

        // Input layout of the generated tsr_error of one end-effector: q, rTe (7), wTr (7), lower (6), upper (6)
        struct Input
        {
            Block q;
            FloatVector<rake, 7> rTe;
            FloatVector<rake, 7> wTr;
            FloatVector<rake, 6> lb;
            FloatVector<rake, 6> ub;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                index -= Robot::dimension;
                if (index < 7)
                {
                    return rTe[index];
                }

                if (index < 14)
                {
                    return wTr[index - 7];
                }

                if (index < 20)
                {
                    return lb[index - 14];
                }

                return ub[index - 20];
            }
        };

        PosePolylineCurve curve_;
        Row weight_;
        bool slide_ = false;

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable Input input;
        mutable Row t_ = Row::fill(0.F);
        mutable SolveBuffer<rake, 6, 6 *(Robot::dimension + 1)> slide_solve_;
        mutable Row lower_t_ = Row::fill(0.F);
        mutable Row upper_t_ = Row::fill(1.F);
        mutable std::array<PosePolylineCurve::Rate, rake> rate_;
        mutable std::array<Eigen::Matrix3d, rake> target_rotation_;
    };
}  // namespace vamp::planning::constraint
