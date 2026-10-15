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
    // the configuration). A projection therefore works on (q, t), with the error  FK(q).translation - P(t)  and
    // a Jacobian with one more column than the robot has joints; t is bounded to [0, 1] by the bounded
    // least-squares step, the joints by their limits.
    //
    // t has to start near the right place. begin_projection(q) seeds it, per lane, with the curve parameter of the
    // fitted waypoint nearest to that lane's end-effector (the generated Robot::spline_seed); a projection then
    // refines it. Every projection must be preceded by begin_projection on the same block (the constraint set's
    // projection entry points do this once the hook is wired in), because squared_error()/step() work with the
    // t left by the previous call.
    //
    // Uses the generated Robot::spline_error, Robot::spline_seed and the four projection solves
    // Robot::solve_spline_error_{lm_inner,lm_outer,gradient_descent,bounded_ls} (cricket, recipe key
    // "spline_constraint"); the spline has Robot::spline_control_points control points. As for the TSR constraint,
    // step() runs the solve of the requested ProjMethod. t stays in [0, 1] either way: the bounded solve enforces it,
    // and for the others step() clamps it.
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
        // fitted through, with the curve parameters at which it passes through each, are only used to seed t.
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
                    seed_input_.waypoints[point * 3 + axis] = Row::fill(waypoints[point][axis]);
                }

                seed_input_.parameters[point] = Row::fill(parameters[point]);
            }

            for (auto i = 0U; i < n_knots; ++i)
            {
                error_input_.knots[i] = Row::fill(knots[i]);
            }
        }

        // Seed t, per lane, from the end-effector position of each lane of q. See the class comment.
        // (Becomes an override of Constraint::begin_projection once the hook exists.)
        void begin_projection(const Block &q) const noexcept
        {
            seed_input_.q = q;

            std::array<Row, 1> seed;
            Robot::template spline_seed<rake>(seed_input_, seed);
            t_ = seed[0];
        }

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
        // seeded here and then taken as is.
        void evaluate_error_jacobian(const Block &q) const noexcept override
        {
            begin_projection(q);
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

        // Input layout of the generated Robot::spline_seed: q, the waypoints (x y z per waypoint), their parameters.
        struct SeedInput
        {
            Block q;
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
        mutable SeedInput seed_input_;
        mutable SolveBuffer<rake, err_size, jac_size> solve_;
        mutable Row t_ = Row::fill(0.5F);
    };
}  // namespace vamp::planning::constraint
