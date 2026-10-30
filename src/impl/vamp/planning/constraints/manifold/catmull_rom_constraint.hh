#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/spline_constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Position constraint against a Catmull-Rom curve P(t), t in [0, 1], at a curve parameter that is given: the first
    // end-effector's position must equal P(t). For planning in Robot::SplineSpace, where t is part of the state and
    // ConstraintSet::set_parameter hands it to the constraint per lane before every projection; a projection solves for
    // the joints alone, with the generated fixed-parameter solvers (Robot::solve_spline_fixed_t_*).
    //
    // The curve passes through Robot::catmull_rom_points points at parameters spaced by the (centripetal, by default)
    // parametrization: tau_0 = 0, tau_{i+1} - tau_i proportional to |P_{i+1} - P_i|^alpha, normalized to tau_last = 1.
    // alpha = 0 is the uniform curve, 1 the chordal one, 0.5 the centripetal, which does not loop or overshoot on
    // unevenly spaced points. Between two points the curve is the cubic through them and their outer neighbours
    // (Barry-Goldman); the ends use a phantom point, the reflection of the second point through the end one. Unlike the
    // B-spline it is C1 only, but it passes through the points with no fitting.
    //
    // There is no closest-point search in planning: a state says where on the curve it is. closest_parameter() is the
    // host-side scan for the t of a configuration that is not given one (a planner's start).
    //
    // Uses the generated Robot::catmull_rom_error and the fixed-parameter solvers of the spline constraint (cricket
    // recipe keys "catmull_rom_constraint" and "spline_constraint"). Not thread-safe, like the other constraints: one
    // instance per thread.
    template <typename Robot, std::size_t rake>
    struct CatmullRomConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        using Row = FloatVector<rake, 1>;
        using Point = std::array<float, 3>;

        static constexpr std::size_t n_points = Robot::catmull_rom_points;
        static constexpr std::size_t n_extended = n_points + 2;
        static constexpr std::size_t err_size = 3;
        static constexpr std::size_t jac_size = err_size * Robot::dimension;

        // The points the curve passes through, in order (distinct), and the exponent of the parametrization
        explicit CatmullRomConstraint(const std::array<Point, n_points> &points, float alpha = 0.5F)
        {
            std::array<Eigen::Vector3d, n_points> p;
            for (auto i = 0U; i < n_points; ++i)
            {
                p[i] = Eigen::Vector3d(points[i][0], points[i][1], points[i][2]);
            }

            // the parameters of the points
            std::array<double, n_points> tau{};
            for (auto i = 1U; i < n_points; ++i)
            {
                const double distance = (p[i] - p[i - 1]).norm();
                if (not(distance > 1e-9))
                {
                    throw std::invalid_argument("consecutive Catmull-Rom points must be distinct");
                }

                tau[i] = tau[i - 1] + std::pow(distance, static_cast<double>(alpha));
            }

            for (auto i = 0U; i < n_points; ++i)
            {
                tau[i] /= tau[n_points - 1];
            }

            // extended by a phantom point and knot on each side: E_k = P_{k - 1}, T_k = tau_{k - 1}
            for (auto i = 0U; i < n_points; ++i)
            {
                points_[i + 1] = p[i];
                knots_[i + 1] = tau[i];
            }

            points_[0] = 2.0 * p[0] - p[1];
            knots_[0] = knots_[1] - (knots_[2] - knots_[1]);
            points_[n_extended - 1] = 2.0 * p[n_points - 1] - p[n_points - 2];
            knots_[n_extended - 1] = knots_[n_extended - 2] + (knots_[n_extended - 2] - knots_[n_extended - 3]);

            for (auto i = 0U; i < n_extended; ++i)
            {
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    input_.points[i * 3 + axis] = Row::fill(static_cast<float>(points_[i][axis]));
                }

                input_.knots[i] = Row::fill(static_cast<float>(knots_[i]));
            }
        }

        // The curve on the host, in double: the point at t (clamped to [0, 1]). The same Barry-Goldman evaluation as the
        // generated kernel, for planners' starts, tests and plots.
        auto value(double t) const noexcept -> Eigen::Vector3d
        {
            t = std::clamp(t, 0.0, 1.0);

            // the first segment whose end is after t (t = 1 stays in the last one)
            std::size_t segment = 0;
            while (segment + 2 < n_points and knots_[segment + 2] <= t)
            {
                ++segment;
            }

            const auto *P = &points_[segment];
            const auto *T = &knots_[segment];

            const auto blend = [t](const Eigen::Vector3d &a, const Eigen::Vector3d &b, double ta, double tb)
            { return ((tb - t) * a + (t - ta) * b) / (tb - ta); };

            const Eigen::Vector3d a1 = blend(P[0], P[1], T[0], T[1]);
            const Eigen::Vector3d a2 = blend(P[1], P[2], T[1], T[2]);
            const Eigen::Vector3d a3 = blend(P[2], P[3], T[2], T[3]);
            const Eigen::Vector3d b1 = blend(a1, a2, T[0], T[2]);
            const Eigen::Vector3d b2 = blend(a2, a3, T[1], T[3]);
            return blend(b1, b2, T[1], T[2]);
        }

        // The closest point of the curve to the end-effector at q: (t, squared distance), by a scan of the curve and a
        // golden-section refinement of the best bracket. For a curve that comes near itself it can find the wrong
        // branch: pass the t explicitly there.
        auto closest_parameter(const Configuration &q) const -> std::pair<float, float>
        {
            std::array<float, Robot::dimension> values;
            const auto array = q.to_array();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                values[j] = array[j];
            }

            const Eigen::Vector3d x = Robot::eefk(values).translation().template cast<double>();

            constexpr int samples = 2000;
            int best = 0;
            double best_distance2 = std::numeric_limits<double>::max();
            for (int k = 0; k <= samples; ++k)
            {
                const double distance2 = (x - value(static_cast<double>(k) / samples)).squaredNorm();
                if (distance2 < best_distance2)
                {
                    best_distance2 = distance2;
                    best = k;
                }
            }

            double lo = std::max(0.0, static_cast<double>(best - 1) / samples);
            double hi = std::min(1.0, static_cast<double>(best + 1) / samples);
            constexpr double inverse_phi = 0.6180339887498949;
            for (int iteration = 0; iteration < 40; ++iteration)
            {
                const double a = hi - inverse_phi * (hi - lo);
                const double b = lo + inverse_phi * (hi - lo);
                if ((x - value(a)).squaredNorm() < (x - value(b)).squaredNorm())
                {
                    hi = b;
                }
                else
                {
                    lo = a;
                }
            }

            const double t = 0.5 * (lo + hi);
            return {static_cast<float>(t), static_cast<float>((x - value(t)).squaredNorm())};
        }

        // The parameter, per lane, that the error is evaluated at and the steps hold. Set by the planner before every
        // projection; clear_parameter() has nothing to undo, since this constraint never solves for t.
        void set_parameter(const Row &parameter) const noexcept override
        {
            t_ = parameter.clamp(0.F, 1.F);
        }

        // The squared error |FK(q) - P(t)|^2 at the current parameter. Caches J and err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            input_.q = q;
            input_.t = t_;
            Robot::template catmull_rom_error<rake>(input_, solve_);

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
            input_.t = t_;
            Robot::template catmull_rom_error<rake>(input_, solve_);
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
        // Input layout of the generated Robot::catmull_rom_error: q, t, the extended points (x y z per point), the
        // extended knots.
        struct Input
        {
            Block q;
            Row t;
            FloatVector<rake, 3 * n_extended> points;
            FloatVector<rake, n_extended> knots;

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
                if (index < 3 * n_extended)
                {
                    return points[index];
                }

                return knots[index - 3 * n_extended];
            }
        };

        std::array<Eigen::Vector3d, n_extended> points_;
        std::array<double, n_extended> knots_;

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable Input input_;
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.F);
    };
}  // namespace vamp::planning::constraint
