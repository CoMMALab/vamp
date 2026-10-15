#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Position constraint against a cubic B-spline curve: the first end-effector's position must lie on the curve
    // P(t), t in [0, 1]. The manifold is {q : FK(q).translation = P(t) for some t}.
    //
    // The curve parameter t is a second unknown next to q, kept per lane inside the constraint (it is not part of
    // the configuration). A projection works on (q, t): the error is FK(q).translation - P(t), its Jacobian has one
    // more column than the robot has joints, and each step solves for both. t is bounded to [0, 1] by the bounded
    // least-squares step (for the other methods step() clamps it), the joints by their limits.
    //
    // t has to start near the right place, and "the right place" is the closest point of the curve to the
    // end-effector. closest_t(q) finds it, per lane, with the generated Robot::spline_closest_t: a seed (the
    // distance-weighted parameter between the nearest fitted waypoint and its nearer neighbour) refined by a few
    // Gauss-Newton steps on t with q fixed. The same kernel gives the squared distance from the end-effector to the
    // curve, which is distance_to_spline(q), the constraint's error as a function of q alone.
    //
    //   * begin_projection(q) runs closest_t(q): ConstraintSet calls it once at the start of every projection, so
    //     the iterations start from the closest point and carry t from step to step.
    //   * satisfied() and other one-off queries use closest_squared_error(q) = distance_to_spline(q), which does not
    //     depend on whatever t the last projection left.
    //   * Code that drives squared_error()/step() itself must call begin_projection(q) first.
    //
    // What closest_t guarantees: a squared distance near zero proves q is on the curve (the t found is a witness).
    // A larger one only says no nearby point was found, and for a curve that comes near itself it can be the
    // distance to the wrong branch.
    //
    // Uses the generated Robot::spline_closest_t, Robot::spline_error and the four projection solves
    // Robot::solve_spline_error_{lm_inner,lm_outer,gradient_descent,bounded_ls} (cricket, recipe key
    // "spline_constraint"); the spline has Robot::spline_control_points control points.
    template <typename Robot, std::size_t rake>
    struct SplineConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t n_control = Robot::spline_control_points;
        static constexpr std::size_t n_knots = Robot::spline_knots;

        // The unknowns of a projection: the joints, then t
        static constexpr std::size_t variables = Robot::spline_variables;
        static constexpr std::size_t err_size = 3;
        static constexpr std::size_t jac_size = err_size * variables;

        using Point = std::array<float, 3>;

        // The curve is given by its control points and knots (what spline_error evaluates). The waypoints it was
        // fitted through, with the curve parameters at which it passes through each, are what the closest-t kernel
        // seeds from.
        SplineConstraint(
            const std::array<Point, n_control> &control_points,
            const std::array<float, n_knots> &knots,
            const std::array<Point, n_control> &waypoints,
            const std::array<float, n_control> &parameters) noexcept
        {
            for (auto point = 0U; point < n_control; ++point)
            {
                for (auto axis = 0U; axis < 3; ++axis)
                {
                    error_input_.control_points[point * 3 + axis] = Row::fill(control_points[point][axis]);
                    closest_input_.control_points[point * 3 + axis] = Row::fill(control_points[point][axis]);
                    closest_input_.waypoints[point * 3 + axis] = Row::fill(waypoints[point][axis]);
                }

                closest_input_.parameters[point] = Row::fill(parameters[point]);
            }

            for (auto i = 0U; i < n_knots; ++i)
            {
                error_input_.knots[i] = Row::fill(knots[i]);
                closest_input_.knots[i] = Row::fill(knots[i]);
            }
        }

        // The closest point of the curve to the end-effector of each lane of q: sets the per-lane t to it, and keeps
        // the squared distance (see distance_to_spline).
        void closest_t(const Block &q) const noexcept override
        {
            closest_input_.q = q;

            std::array<Row, 2> out;
            Robot::template spline_closest_t<rake>(closest_input_, out);
            t_ = out[0];
            distance2_ = out[1];
        }

        // What starts a projection: t at the closest point.
        void begin_projection(const Block &q) const noexcept override
        {
            closest_t(q);
        }

        // The squared distance from each lane's end-effector to the curve: the constraint's error as a function of
        // q alone. Zero (to rounding) exactly when the closest point found is on the end-effector.
        auto distance_to_spline(const Block &q) const noexcept -> Row
        {
            return closest_squared_error(q);
        }

        auto closest_squared_error(const Block &q) const noexcept -> Row override
        {
            closest_t(q);
            return distance2_;
        }

        // The current per-lane t: the closest point after closest_t/begin_projection, then what the steps made of it.
        auto current_t() const noexcept -> Row
        {
            return t_;
        }

        // The squared error at the current t, |FK(q) - P(t)|^2. Caches J and err for step().
        auto squared_error(const Block &q) const noexcept -> Row override
        {
            error_input_.q = q;
            error_input_.t = t_;
            Robot::template spline_error<rake>(error_input_, solve_);

            auto d = solve_.err[0] * solve_.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve_.err[i] * solve_.err[i];
            }

            return d;
        }

        // One projection step on (q, t) of the requested method, using the cache of the last squared_error() call.
        // BoundedLS takes the joint limits and t's range [0, 1] as the allowed step to each bound, limit - x; the
        // other methods take none, and t is clamped afterwards.
        void step(Block &q, ProjMethod method, float alpha) const noexcept override
        {
            FloatVector<rake, variables> gradient;

            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_spline_error_lm_inner<rake>(solve_, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_spline_error_lm_outer<rake>(solve_, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_spline_error_gradient_descent<rake>(solve_, gradient);
                    break;
                case ProjMethod::BoundedLS:
                {
                    SolveInput input{solve_};
                    for (auto i = 0U; i < Robot::dimension; ++i)
                    {
                        input.lower[i] = Row::fill(Robot::lower_bound[i]) - q[i];
                        input.upper[i] = Row::fill(Robot::upper_bound[i]) - q[i];
                    }

                    input.lower[Robot::dimension] = Row::fill(0.F) - t_;
                    input.upper[Robot::dimension] = Row::fill(1.F) - t_;

                    Robot::template solve_spline_error_bounded_ls<rake>(input, gradient);
                    break;
                }
            }

            // The joints take their part of the step as for any constraint (this also clamps to the limits) ...
            Block joint_gradient;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                joint_gradient[i] = gradient[i];
            }

            integrate_step<Robot, rake>(q, joint_gradient, alpha);

            // ... and t takes the last entry, staying in [0, 1]
            t_ = (t_ - gradient[Robot::dimension] * alpha).clamp(0.F, 1.F);
        }

        auto n_rows() const noexcept -> std::size_t override
        {
            return err_size;
        }

        // The three position rows pin the end-effector to the curve: all manifold-defining
        void active_rows(bool *rows) const noexcept override
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = true;
            }
        }

        // Zero the pinned joints' columns. The Jacobian has `variables` columns (the joints, then t), so the stride
        // is `variables`, not Robot::dimension; t's own column is never pinned.
        void mask_jacobian(const bool *pinned) const noexcept override
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                if (not pinned[j])
                {
                    continue;
                }

                for (auto i = 0U; i < err_size; ++i)
                {
                    solve_.jac[i * variables + j] = Row::fill(0.F);
                }
            }
        }

        // For tangent-space (chart) construction. The configurations passed in are on or near the manifold, so t is
        // set to the closest point first: the Jacobian is then evaluated at the right point of the curve.
        void evaluate_error_jacobian(const Block &q) const noexcept override
        {
            closest_t(q);
            error_input_.q = q;
            error_input_.t = t_;
            Robot::template spline_error<rake>(error_input_, solve_);
        }

        // The error and the Jacobian with respect to the joints only, of one lane. t is a nuisance variable of the
        // manifold, so it is eliminated: with tau = J_t / |J_t| the unit direction in which the curve point moves,
        // both J_q and err are projected onto the plane perpendicular to tau, (I - tau tau^T) J_q and
        // (I - tau tau^T) err. Those rows (rank 2 of 3) vanish exactly when the end-effector is on the curve, and
        // their null space is the tangent space of the manifold in q. If J_t is zero (a stationary curve point),
        // the raw rows are returned.
        void extract_error_jacobian(std::size_t lane, float *err, float *jac) const noexcept override
        {
            float jacobian_q[err_size][Robot::dimension];
            float jacobian_t[err_size];
            float raw_error[err_size];

            for (auto i = 0U; i < err_size; ++i)
            {
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    jacobian_q[i][j] = solve_.jac[{i * variables + j, lane}];
                }

                jacobian_t[i] = solve_.jac[{i * variables + Robot::dimension, lane}];
                raw_error[i] = solve_.err[{i, lane}];
            }

            float norm2 = 0.F;
            for (auto i = 0U; i < err_size; ++i)
            {
                norm2 += jacobian_t[i] * jacobian_t[i];
            }

            float tau[err_size] = {0.F, 0.F, 0.F};
            if (norm2 > 1e-12F)
            {
                const float inverse_norm = 1.F / std::sqrt(norm2);
                for (auto i = 0U; i < err_size; ++i)
                {
                    tau[i] = jacobian_t[i] * inverse_norm;
                }
            }

            float tau_error = 0.F;
            for (auto i = 0U; i < err_size; ++i)
            {
                tau_error += tau[i] * raw_error[i];
            }

            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = raw_error[i] - tau[i] * tau_error;
            }

            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                float tau_column = 0.F;
                for (auto i = 0U; i < err_size; ++i)
                {
                    tau_column += tau[i] * jacobian_q[i][j];
                }

                for (auto i = 0U; i < err_size; ++i)
                {
                    jac[i * Robot::dimension + j] = jacobian_q[i][j] - tau[i] * tau_column;
                }
            }
        }

    private:
        // Input layout of the generated Robot::spline_error: q, t, the control points (x y z per point), the knots.
        struct ErrorInput
        {
            Block q;
            Row t;
            FloatVector<rake, 3 * n_control> control_points;
            FloatVector<rake, n_knots> knots;

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
                if (index < 3 * n_control)
                {
                    return control_points[index];
                }

                return knots[index - 3 * n_control];
            }
        };

        // Input layout of the generated Robot::spline_closest_t: q, the control points (x y z per point), the knots,
        // the waypoints (x y z per waypoint), their parameters.
        struct ClosestInput
        {
            Block q;
            FloatVector<rake, 3 * n_control> control_points;
            FloatVector<rake, n_knots> knots;
            FloatVector<rake, 3 * n_control> waypoints;
            FloatVector<rake, n_control> parameters;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                if (index < Robot::dimension)
                {
                    return q[index];
                }

                index -= Robot::dimension;
                if (index < 3 * n_control)
                {
                    return control_points[index];
                }

                index -= 3 * n_control;
                if (index < n_knots)
                {
                    return knots[index];
                }

                index -= n_knots;
                if (index < 3 * n_control)
                {
                    return waypoints[index];
                }

                return parameters[index - 3 * n_control];
            }
        };

        // Input layout of the generated Robot::solve_spline_error_bounded_ls: the error kernel's outputs (the
        // Jacobian, then the error), then lower - x and upper - x for the `variables` unknowns.
        struct SolveInput
        {
            const SolveBuffer<rake, err_size, jac_size> &solve;
            FloatVector<rake, variables> lower;
            FloatVector<rake, variables> upper;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                constexpr std::size_t solve_size = jac_size + err_size;
                if (index < solve_size)
                {
                    return solve[index];
                }

                index -= solve_size;
                return (index < variables) ? lower[index] : upper[index - variables];
            }
        };

        // Evaluation state, like the other constraints' caches: not thread-safe, one instance per thread.
        mutable ErrorInput error_input_;
        mutable ClosestInput closest_input_;
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.5F);
        mutable Row distance2_ = Row::fill(0.F);
    };
}  // namespace vamp::planning::constraint
