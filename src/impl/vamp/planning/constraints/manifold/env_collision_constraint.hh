#pragma once

#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Push a configuration out of environment collision using ONE representative sphere
    // per link -- the "middle" sphere of that link's collision-sphere list (see cricket's
    // RobotInfo::env_collision_middle_spheres), not every sphere of the link and not a
    // bounding sphere. Each representative sphere contributes one row: the signed distance
    // to the nearest environment primitive (vamp::sphere_environment_min_distance_gradient)
    // hinged at `margin`, so a row is zero (inactive) whenever that sphere is already at
    // least `margin` away from every obstacle, and negative (the amount of violation)
    // otherwise -- the same "hinge, and mask the Jacobian of satisfied rows too" shape as
    // HingedTSRConstraint's bound hinge, and the same "hand-written error/Jacobian, generated
    // solve dispatch" shape as CoMConstraint, whose FK is likewise from a generated kernel
    // (env_collision_jacobian here, com_jacobian there) but whose distance/gradient math is
    // hand-written (there's no generated SDF: the environment's obstacle count and types
    // aren't known at codegen time, only the sphere's own position and Jacobian are).
    //
    // The margin band has positive measure (a slab, same as CoM's polygon interior), so
    // these rows are never manifold-defining (active_rows() reports false for all of them).
    template <typename Robot, std::size_t rake>
    struct EnvironmentCollisionConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t dim = Robot::dimension;
        static constexpr std::size_t err_size = Robot::n_env_collision_spheres;
        static constexpr std::size_t jac_size = err_size * dim;

        // `environment` is copied (not referenced): this constraint is held and reused
        // across many projection steps, possibly outliving whatever scope constructed the
        // environment it was built from (e.g. a Python-side Environment object) -- same
        // reasoning as EnvVec(env) copying at the planning entry points in robot_helper.hh.
        explicit EnvironmentCollisionConstraint(
            const collision::Environment<float> &environment,
            float margin = 0.05F)
          : environment_(environment), margin_(margin)
        {
        }

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            evaluate(q);

            auto d = solve.err[0] * solve.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve.err[i] * solve.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept final
        {
            Block gradient;
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_env_collision_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_env_collision_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_env_collision_error_gradient_descent<rake>(
                        solve, gradient);
                    break;
            }

            integrate_step<Robot, rake>(q, gradient, alpha);
        }

        auto n_rows() const noexcept -> std::size_t final
        {
            return err_size;
        }

        void active_rows(bool *rows) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = false;
            }
        }

        void mask_jacobian(const bool *pinned) const noexcept final
        {
            mask_pinned_columns<Robot>(solve, pinned);
        }

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            evaluate(q);
        }

        void extract_error_jacobian(std::size_t lane, float *err, float *jac)
            const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = solve.err[{i, lane}];
            }

            for (auto i = 0U; i < jac_size; ++i)
            {
                jac[i] = solve.jac[{i, lane}];
            }
        }

    private:
        void evaluate(const Block &q) const noexcept
        {
            input.q = q;
            Robot::template env_collision_jacobian<rake>(input, positions);

            const auto margin = Row::fill(margin_);

            for (auto i = 0U; i < err_size; ++i)
            {
                const Row px = positions.pos[3 * i + 0];
                const Row py = positions.pos[3 * i + 1];
                const Row pz = positions.pos[3 * i + 2];
                const Row radius = Row::fill(Robot::env_collision_radii[i]);

                const auto dg = sphere_environment_min_distance_gradient(environment_, px, py, pz, radius);

                // One-sided hinge (only "too close", no upper bound): zero outside the
                // margin band, the (negative) amount of violation inside it.
                const auto violation = (dg.distance - margin).min(0.F);
                solve.err[i] = violation;

                // Satisfied rows must drop out of the Jacobian too (same reasoning as
                // HingedTSRConstraint): otherwise a zero-residual row still pulls the
                // solve toward "hold this sphere's position", blocking projection.
                const auto active = violation != 0.F;

                for (auto j = 0U; j < dim; ++j)
                {
                    const auto dpx = positions.jac[(3 * i + 0) * dim + j];
                    const auto dpy = positions.jac[(3 * i + 1) * dim + j];
                    const auto dpz = positions.jac[(3 * i + 2) * dim + j];

                    // Chain rule: d(distance)/dq = grad(distance) . d(pos)/dq. No sign
                    // flip needed -- d(violation)/dq equals d(distance)/dq wherever active,
                    // matching err's sign convention exactly (same as CoM/TSR).
                    solve.jac[i * dim + j] = active & (dg.gx * dpx + dg.gy * dpy + dg.gz * dpz);
                }
            }
        }

        // Input layout of the generated env_collision_jacobian: just the configuration.
        struct Input
        {
            Block q;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return q[index];
            }
        };

        // Output layout of env_collision_jacobian: d(pos)/dq (3 * err_size x dim,
        // row-major), then pos (3 * err_size).
        struct Positions
        {
            FloatVector<rake, 3 * err_size * dim> jac;
            FloatVector<rake, 3 * err_size> pos;

            auto operator[](std::size_t index) noexcept -> Row &
            {
                return (index < 3 * err_size * dim) ? jac[index] : pos[index - 3 * err_size * dim];
            }

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return (index < 3 * err_size * dim) ? jac[index] : pos[index - 3 * err_size * dim];
            }
        };

        // Input layout of the generated solvers: d(err)/dq (err_size x dim, row-major),
        // then err (err_size).
        using Solve = SolveBuffer<rake, err_size, jac_size>;

        collision::Environment<FloatVector<rake>> environment_;
        float margin_;

        mutable Input input;
        mutable Positions positions;
        mutable Solve solve;
    };
}  // namespace vamp::planning::constraint
