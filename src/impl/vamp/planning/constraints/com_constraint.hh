#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <vamp/planning/constraints/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Support-polygon constraint on the robot's center of mass: the xy projection of the CoM
    // (from the generated Robot::com_jacobian, possibly expressed relative to a stance frame
    // such as the midpoint of the feet) must lie inside a convex polygon given by its
    // vertices in counterclockwise order. Each edge contributes a half-plane; outside an
    // edge, the error is the outward normal displacement, summed over violated edges. Since
    // the edge normals are constant, d(err)/d(com) is exactly the sum of the violated edges'
    // normal outer products, chained with the CoM Jacobian.
    template <typename Robot, std::size_t rake>
    struct CoMConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t dim = Robot::dimension;
        static constexpr std::size_t err_size = 2;
        static constexpr std::size_t jac_size = err_size * dim;

        using Vertex = std::array<float, 2>;

        explicit CoMConstraint(const std::vector<Vertex> &polygon)
        {
            if (polygon.size() < 3)
            {
                throw std::invalid_argument("CoM support polygon needs at least three vertices");
            }

            edges.reserve(polygon.size());
            for (auto i = 0U; i < polygon.size(); ++i)
            {
                const auto &a = polygon[i];
                const auto &b = polygon[(i + 1) % polygon.size()];

                // Outward normal of a counterclockwise edge.
                float nx = b[1] - a[1];
                float ny = a[0] - b[0];
                const float norm = std::sqrt(nx * nx + ny * ny);
                if (norm <= 0.F)
                {
                    throw std::invalid_argument("CoM support polygon has a degenerate edge");
                }

                edges.push_back({nx / norm, ny / norm, a[0], a[1]});
            }
        }

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            evaluate(q);
            return solve.err[0] * solve.err[0] + solve.err[1] * solve.err[1];
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept final
        {
            Block gradient;
            switch (method)
            {
                case ProjMethod::InnerLM:
                    Robot::template solve_com_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_com_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_com_error_gradient_descent<rake>(solve, gradient);
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
            // The polygon interior has positive measure (a slab): never manifold-defining.
            rows[0] = false;
            rows[1] = false;
        }

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            // The half-plane hinge is intrinsic here (the error has no unhinged form: it is
            // identically zero inside the polygon), but these rows are never chart-active, so
            // the masked Jacobian is safe to report.
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
            Robot::template com_jacobian<rake>(input, com);

            auto ex = Row::fill(0.F);
            auto ey = Row::fill(0.F);
            auto mxx = Row::fill(0.F);
            auto mxy = Row::fill(0.F);
            auto myy = Row::fill(0.F);

            for (const auto &e : edges)
            {
                const auto nx = Row::fill(e[0]);
                const auto ny = Row::fill(e[1]);
                const auto r =
                    nx * (com.pos[0] - Row::fill(e[2])) + ny * (com.pos[1] - Row::fill(e[3]));
                const auto outside = r > 0.F;

                ex = ex + (outside & (r * nx));
                ey = ey + (outside & (r * ny));
                mxx = mxx + (outside & (nx * nx));
                mxy = mxy + (outside & (nx * ny));
                myy = myy + (outside & (ny * ny));
            }

            solve.err[0] = ex;
            solve.err[1] = ey;

            for (auto j = 0U; j < dim; ++j)
            {
                const auto dx = com.jac[j];
                const auto dy = com.jac[dim + j];
                solve.jac[j] = mxx * dx + mxy * dy;
                solve.jac[dim + j] = mxy * dx + myy * dy;
            }
        }

        // Input layout of the generated com_jacobian: just the configuration.
        struct Input
        {
            Block q;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return q[index];
            }
        };

        // Output layout of com_jacobian: d(com)/dq (3 x dim, row-major), then com (3).
        struct Com
        {
            FloatVector<rake, 3 * dim> jac;
            FloatVector<rake, 3> pos;

            auto operator[](std::size_t index) noexcept -> Row &
            {
                return (index < 3 * dim) ? jac[index] : pos[index - 3 * dim];
            }

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return (index < 3 * dim) ? jac[index] : pos[index - 3 * dim];
            }
        };

        // Input layout of the generated solvers: d(err)/dq (2 x dim, row-major), then err (2).
        using Solve = SolveBuffer<rake, err_size, jac_size>;

        // Per-edge (nx, ny, ax, ay): unit outward normal and edge origin.
        std::vector<std::array<float, 4>> edges;

        mutable Input input;
        mutable Com com;
        mutable Solve solve;
    };
}  // namespace vamp::planning::constraint
