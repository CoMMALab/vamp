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

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/spline_constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // The straight-line interpolation of points as a curve P(t), t in [0, 1]: a degree-1 spline, position continuous
    // and with a tangent that jumps at every point. Any number (at least two) of distinct points.
    //
    // t is the chord-length parameter: tau_0 = 0, tau_{i+1} - tau_i proportional to |P_{i+1} - P_i|, normalized to
    // tau_last = 1, so a point moves at constant speed along the polyline as t grows, and an equal step in t is an
    // equal distance on every segment.
    //
    // A curve for CurveConstraint: it provides value(t) and closest(x).
    struct PolylineCurve
    {
        using Point = std::array<float, 3>;

        explicit PolylineCurve(const std::vector<Point> &points)
        {
            if (points.size() < 2)
            {
                throw std::invalid_argument("a polyline needs at least two points");
            }

            points_.reserve(points.size());
            for (const auto &point : points)
            {
                points_.emplace_back(point[0], point[1], point[2]);
            }

            tau_.assign(points_.size(), 0.0);
            for (std::size_t i = 1; i < points_.size(); ++i)
            {
                const double length = (points_[i] - points_[i - 1]).norm();
                if (not(length > 1e-9))
                {
                    throw std::invalid_argument("consecutive polyline points must be distinct");
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
            return points_.size();
        }

        // The segment i with tau_i <= t < tau_{i + 1} (t clamped to [0, 1]); t = 1 belongs to the last one
        auto segment_of(double t) const noexcept -> std::size_t
        {
            t = std::clamp(t, 0.0, 1.0);

            const auto upper = std::upper_bound(tau_.begin(), tau_.end(), t);
            const std::size_t segment =
                (upper == tau_.begin()) ? 0 : static_cast<std::size_t>(upper - tau_.begin()) - 1;
            return std::min(segment, points_.size() - 2);
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

        // dP/dt on segment i (a constant)
        auto velocity(std::size_t segment) const noexcept -> Eigen::Vector3d
        {
            return (points_[segment + 1] - points_[segment]) / (tau_[segment + 1] - tau_[segment]);
        }

        // The point at t (clamped to [0, 1])
        auto value(double t) const noexcept -> Eigen::Vector3d
        {
            t = std::clamp(t, 0.0, 1.0);

            const std::size_t segment = segment_of(t);
            const double s = (t - tau_[segment]) / (tau_[segment + 1] - tau_[segment]);
            return (1.0 - s) * points_[segment] + s * points_[segment + 1];
        }

        // The closest point of the polyline to x, exactly: (t, squared distance)
        auto closest(const Eigen::Vector3d &x) const noexcept -> std::pair<double, double>
        {
            double best_t = 0.0;
            double best_distance2 = std::numeric_limits<double>::max();

            for (std::size_t i = 0; i + 1 < points_.size(); ++i)
            {
                const Eigen::Vector3d edge = points_[i + 1] - points_[i];
                const double s = std::clamp(edge.dot(x - points_[i]) / edge.squaredNorm(), 0.0, 1.0);
                const double distance2 = (x - (points_[i] + s * edge)).squaredNorm();
                if (distance2 < best_distance2)
                {
                    best_distance2 = distance2;
                    best_t = tau_[i] + s * (tau_[i + 1] - tau_[i]);
                }
            }

            return {best_t, best_distance2};
        }

    private:
        std::vector<Eigen::Vector3d> points_;
        std::vector<double> tau_;
    };

    // Position constraint against a curve, at a curve parameter that is given: the first end-effector's position must
    // equal Curve::value(t). For planning in Robot::SplineSpace, where t is part of the state and
    // ConstraintSet::set_parameter hands it to the constraint per lane before every projection.
    //
    // The curve is evaluated on the host, once per lane when the parameter is set, and handed to the generated
    // Robot::position_target_error as the target point; a projection solves for the joints alone, with the generated
    // fixed-parameter solvers (Robot::solve_spline_fixed_t_*). So any curve with value(t) and closest(x) works, with
    // any number of points, and nothing is regenerated per curve (cricket recipe keys "position_target_constraint" and
    // "spline_constraint").
    //
    // There is no closest-point search in planning: a state says where on the curve it is. closest_parameter() is the
    // host-side search for the t of a configuration that is not given one (a planner's start).
    //
    // Sliding (`slide`, for a curve that is made of segments: segment_of(t), segment_begin(i), segment_end(i) and
    // velocity(i) = dP/dt on segment i, besides value and closest): the parameter given to set_parameter only says
    // which segment the lane is on, and t is then an unknown of the projection next to the joints, as in the
    // SplineConstraint that solves for t, but bounded to the segment: the error is FK(q) - P(t), its Jacobian has a
    // column -dP/dt for t after the joints' columns, and every step moves the joints and t together (the generated
    // Robot::solve_spline_error_*, whose unknowns are the joints and t; BoundedLS keeps t in the segment's range
    // exactly, the other methods clamp it after the step). The curve is evaluated at t once per iteration, on the
    // host. The parameter the projection leaves each lane at is projected_parameter(): planners write it back into the
    // state. extract_error_jacobian still returns the raw fixed-target rows, which are not the sliding manifold's
    // tangent space.
    //
    // Whole curve (`whole_curve`, the counterpart of the SplineConstraint that solves for t, for a curve of segments as
    // above): no parameter is given. Every projection starts by finding the closest point of the whole curve to each
    // lane's end-effector (begin_projection, on the host, exact), which seeds t; t is then an unknown of the
    // projection next to the joints, bounded to [0, 1], and its column is -dP/dt of whichever segment t is in at the
    // iteration. It is the q-only projection, where the sliding one is bounded to the segment of a given t.
    //
    // Not thread-safe, like the other constraints: one instance per thread.
    template <typename Robot, std::size_t rake, typename Curve>
    struct CurveConstraint : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = 3;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        explicit CurveConstraint(Curve curve, bool slide = false, bool whole_curve = false)
          : curve_(std::move(curve)), slide_(slide), whole_curve_(whole_curve)
        {
            if (slide and whole_curve)
            {
                throw std::invalid_argument("a curve constraint slides along a segment or solves over the whole curve");
            }

            set_parameter(t_);
        }

        // Whether t is solved over the whole curve, seeded by the closest point (see above)
        auto whole_curve() const noexcept -> bool
        {
            return whole_curve_;
        }

        // Whether the end-effector slides along the parameter's segment while a projection runs (see above)
        auto slide() const noexcept -> bool
        {
            return slide_;
        }

        void set_slide(bool slide) noexcept
        {
            slide_ = slide;
            set_parameter(t_);
        }

        auto curve() const noexcept -> const Curve &
        {
            return curve_;
        }

        // The curve's point at t
        auto value(double t) const noexcept -> Eigen::Vector3d
        {
            return curve_.value(t);
        }

        // The closest point of the curve to the end-effector at q: (t, squared distance)
        auto closest_parameter(const Configuration &q) const -> std::pair<float, float>
        {
            std::array<float, Robot::dimension> values;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                values[j] = array[j];
            }

            const Eigen::Vector3d x = Robot::eefk(values).translation().template cast<double>();
            const auto [t, distance2] = curve_.closest(x);
            return {static_cast<float>(t), static_cast<float>(distance2)};
        }

        // The parameter, per lane, that the error is evaluated at and the steps hold: the curve is evaluated at it
        // here, once, not on every error evaluation. Set by the planner before every projection; clear_parameter() has
        // nothing to undo, since this constraint never solves for t.
        void set_parameter(const Row &parameter) const noexcept override
        {
            t_ = parameter.clamp(0.F, 1.F);

            if (slide_)
            {
                // each lane's segment, from its t: the range t may move in, and the curve's velocity there
                const auto parameters = t_.to_array();
                std::array<float, rake> lower;
                std::array<float, rake> upper;
                std::array<std::array<float, rake>, 3> velocity;
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    const auto segment = curve_.segment_of(parameters[lane]);
                    lower[lane] = static_cast<float>(curve_.segment_begin(segment));
                    upper[lane] = static_cast<float>(curve_.segment_end(segment));

                    const Eigen::Vector3d v = curve_.velocity(segment);
                    for (auto axis = 0U; axis < 3; ++axis)
                    {
                        velocity[axis][lane] = static_cast<float>(v[axis]);
                    }
                }

                lower_t_ = Row(lower);
                upper_t_ = Row(upper);
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    velocity_[axis] = Row(velocity[axis]);
                }

                // the hint may be a float rounding off its segment's range
                t_ = t_.max(lower_t_).min(upper_t_);
            }

            load_target();
        }

        // The whole curve: the closest point of the curve to each lane's end-effector seeds t, once per projection
        void begin_projection(const Block &q) const noexcept override
        {
            if (not whole_curve_)
            {
                return;
            }

            std::array<std::array<float, rake>, Robot::dimension> joints;
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                joints[j] = q[j].to_array();
            }

            auto parameters = t_.to_array();
            for (auto lane = 0U; lane < rake; ++lane)
            {
                std::array<float, Robot::dimension> values;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    values[j] = joints[j][lane];
                }

                const Eigen::Vector3d x = Robot::eefk(values).translation().template cast<double>();
                parameters[lane] = static_cast<float>(curve_.closest(x).first);
            }

            t_ = Row(parameters);
            refresh_rates();
            load_target();
        }

        // Solving for t: the parameter each lane's projection left it at. Otherwise the one given.
        auto projected_parameter(const Row &given) const noexcept -> Row override
        {
            return solves_t() ? t_ : given;
        }

        // The squared error |FK(q) - target|^2 at the current target. Caches J and err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            evaluate(q);

            auto d = solve_.err[0] * solve_.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve_.err[i] * solve_.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
            if (solves_t())
            {
                slide_step(q, method, alpha);
                return;
            }

            spline_fixed_step<Robot, rake>(solve_, q, method, alpha);
        }

        auto n_rows() const noexcept -> std::size_t override
        {
            return err_size;
        }

        // The three position rows pin the end-effector to the curve point: all manifold-defining
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
            evaluate(q);
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

    private:
        // The error and Jacobian at q. Sliding: the target is the curve at the lanes' current parameters, and the
        // Jacobian gets the column of t after the joints' ones.
        void evaluate(const Block &q) const noexcept
        {
            input_.q = q;
            if (whole_curve_)
            {
                refresh_rates();  // t moved with the last step: the segment it is in, and the velocity there
            }

            if (solves_t())
            {
                load_target();
            }

            Robot::template position_target_error<rake>(input_, solve_);

            if (solves_t())
            {
                constexpr std::size_t variables = Robot::dimension + 1;
                for (auto i = 0U; i < err_size; ++i)
                {
                    for (auto j = 0U; j < Robot::dimension; ++j)
                    {
                        slide_solve_.jac[i * variables + j] = solve_.jac[i * Robot::dimension + j];
                    }

                    // d(FK - P(t))/dt = -dP/dt
                    slide_solve_.jac[i * variables + Robot::dimension] = Row::fill(0.F) - velocity_[i];
                    slide_solve_.err[i] = solve_.err[i];
                }
            }
        }

        // Whether t is an unknown of the projection (sliding along a segment, or over the whole curve)
        auto solves_t() const noexcept -> bool
        {
            return slide_ or whole_curve_;
        }

        // The curve's velocity at each lane's current t (the segment it is in): the Jacobian's column of t
        void refresh_rates() const noexcept
        {
            const auto parameters = t_.to_array();
            std::array<std::array<float, rake>, 3> velocity;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                const Eigen::Vector3d v = curve_.velocity(curve_.segment_of(parameters[lane]));
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    velocity[axis][lane] = static_cast<float>(v[axis]);
                }
            }

            for (auto axis = 0U; axis < 3; ++axis)
            {
                velocity_[axis] = Row(velocity[axis]);
            }
        }

        // One step on the joints and t, from the cache of the last evaluate(): t stays in its range
        void slide_step(Block &q, ProjMethod method, float alpha) const noexcept
        {
            constexpr std::size_t variables = Robot::dimension + 1;
            FloatVector<rake, variables> gradient;

            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_spline_error_lm_inner<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_spline_error_lm_outer<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_spline_error_gradient_descent<rake>(slide_solve_, gradient);
                    break;
                case ProjMethod::BoundedLS:
                {
                    SlideInput input{slide_solve_};
                    for (auto i = 0U; i < Robot::dimension; ++i)
                    {
                        input.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                        input.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                    }

                    input.lower[Robot::dimension] = lower_t_ - t_;
                    input.upper[Robot::dimension] = upper_t_ - t_;

                    Robot::template solve_spline_error_bounded_ls<rake>(input, gradient);
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

        // Input layout of the generated Robot::solve_spline_error_bounded_ls: the extended error kernel's outputs (the
        // Jacobian with t's column, then the error), then lower - x and upper - x for the joints and t.
        struct SlideInput
        {
            static constexpr std::size_t variables = Robot::dimension + 1;
            static constexpr std::size_t solve_size = err_size * variables + err_size;

            const SolveBuffer<rake, err_size, err_size * variables> &solve;
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

        // Input layout of the generated Robot::position_target_error: q, then the target point.
        struct Input
        {
            Block q;
            FloatVector<rake, 3> target;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                return target[index - Robot::dimension];
            }
        };

        // The target of every lane: the curve at the lane's parameter
        void load_target() const noexcept
        {
            const auto parameters = t_.to_array();

            std::array<std::array<float, rake>, 3> target;
            for (auto lane = 0U; lane < rake; ++lane)
            {
                const Eigen::Vector3d point = curve_.value(parameters[lane]);
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    target[axis][lane] = static_cast<float>(point[axis]);
                }
            }

            for (auto axis = 0U; axis < 3; ++axis)
            {
                input_.target[axis] = Row(target[axis]);
            }
        }

        Curve curve_;
        bool slide_ = false;
        bool whole_curve_ = false;

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable SolveBuffer<rake, err_size, err_size *(Robot::dimension + 1)> slide_solve_;
        mutable Row lower_t_ = Row::fill(0.F);
        mutable Row upper_t_ = Row::fill(1.F);
        mutable std::array<Row, 3> velocity_;
        mutable Input input_;
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.F);
    };

    // The straight-line interpolation of the points, any number of them (at least two)
    template <typename Robot, std::size_t rake>
    struct PolylineConstraint final : CurveConstraint<Robot, rake, PolylineCurve>
    {
        using Point = PolylineCurve::Point;

        explicit PolylineConstraint(const std::vector<Point> &points, bool slide = false, bool whole_curve = false)
          : CurveConstraint<Robot, rake, PolylineCurve>(PolylineCurve(points), slide, whole_curve)
        {
        }
    };
}  // namespace vamp::planning::constraint
