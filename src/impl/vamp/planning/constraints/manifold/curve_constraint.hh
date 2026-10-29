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

        // The point at t (clamped to [0, 1])
        auto value(double t) const noexcept -> Eigen::Vector3d
        {
            t = std::clamp(t, 0.0, 1.0);

            // the segment i with tau_i <= t < tau_{i + 1}; t = 1 belongs to the last one
            const auto upper = std::upper_bound(tau_.begin(), tau_.end(), t);
            std::size_t segment = (upper == tau_.begin()) ? 0 : static_cast<std::size_t>(upper - tau_.begin()) - 1;
            segment = std::min(segment, points_.size() - 2);

            const double s = (t - tau_[segment]) / (tau_[segment + 1] - tau_[segment]);
            return (1.0 - s) * points_[segment] + s * points_[segment + 1];
        }

        // The closest point of the polyline to x, exactly: (t, squared distance)
        auto closest(const Eigen::Vector3d &x) const noexcept -> std::pair<double, double>
        {
            double best_t = 0.0;
            double best_distance2 = std::numeric_limits<double>::infinity();

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
    // Not thread-safe, like the other constraints: one instance per thread.
    template <typename Robot, std::size_t rake, typename Curve>
    struct CurveConstraint : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t err_size = 3;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        explicit CurveConstraint(Curve curve) : curve_(std::move(curve))
        {
            load_target();
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
            load_target();
        }

        // The squared error |FK(q) - target|^2 at the current target. Caches J and err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            input_.q = q;
            Robot::template position_target_error<rake>(input_, solve_);

            auto d = solve_.err[0] * solve_.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve_.err[i] * solve_.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
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
            input_.q = q;
            Robot::template position_target_error<rake>(input_, solve_);
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

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable Input input_;
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.F);
    };

    // The straight-line interpolation of the points, any number of them (at least two)
    template <typename Robot, std::size_t rake>
    struct PolylineConstraint final : CurveConstraint<Robot, rake, PolylineCurve>
    {
        using Point = PolylineCurve::Point;

        explicit PolylineConstraint(const std::vector<Point> &points)
          : CurveConstraint<Robot, rake, PolylineCurve>(PolylineCurve(points))
        {
        }
    };
}  // namespace vamp::planning::constraint
