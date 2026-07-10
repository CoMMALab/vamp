#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>

#include <vamp/constants.hh>
#include <vamp/planning/cost.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

// Informed sampler for FLASK's C_loc objective; replaces the (L2-metric) PHS gate for cost robots.
//
// Outer set derivation (both bounds admissible for C_loc; the sampler intersects them and rejects
// the residual with an exact directed f-hat check):
//
//   v-space (Option 1): C_loc >= 2 sqrt(rho) * ||dv||, from Cauchy-Schwarz on ||integral of u||^2.
//     Sum s->z->g bounds ||v - v_s|| + ||v - v_g|| <= c_best / (2 sqrt(rho)). A PHS with foci
//     v_s, v_g. Rest-to-rest collapses to a ball of radius c_best / (4 sqrt(rho)).
//
//   y-space (Option 2): C_loc >= rho ||dy|| / V_max. For any admissible trajectory,
//     ||y(t) - y(0)|| = ||integral of v|| <= integral of ||v|| <= T * ||v||_inf * sqrt(1) ...
//     the direction-independent bound is T >= r / V_fast, where V_fast = max_i V_i (any single
//     joint-aligned direction realizes that speed). So C >= rho T >= rho r / V_fast, a linear
//     concave h(r) = (rho / V_fast) r. Concave h with h(0) = 0 is superadditive, so
//     h(r_s) + h(r_g) >= h(r_s + r_g); combined with C(s->z) + C(z->g) <= c_best gives
//     r_s + r_g <= c_best * V_fast / rho -- a genuine y-space PHS with foci y_s, y_g and
//     transverse diameter d_y = c_best * V_fast / rho.
//
// Per proposal:
//   1. inner->next() -> 2n uniforms (descaled to [0,1]^{2n}).
//   2. First n uniforms -> uniform-in-y-ball direction (logit -> Gaussian, normalize) mapped
//      through the y-PHS ellipsoid transform.
//   3. Next n uniforms -> same for v-PHS. Two extra dist.uniform_01() for ball radii.
//   4. Exact rejection: planning::cost<Robot>(start, z) + planning::cost<Robot>(z, goal) <= c_best.
//   5. On max_tries misses, fall back to inner->next() (deterministic, never worse than uninformed).

namespace vamp::planning
{
    namespace detail
    {
        // Uniform in the L2 ball of dimension N via logit-Gaussian direction + r^{1/N} radius.
        // Consumes N uniform_01 values from `dist` for the radius; uses `u` (size N, in [0,1])
        // as the direction seed.
        template <std::size_t N>
        inline auto uniform_in_ball(
            const float *u, float r_uniform) noexcept -> std::array<float, N>
        {
            std::array<float, N> g{};
            float g2 = 0.F;
            for (std::size_t i = 0; i < N; ++i)
            {
                const float U = std::clamp(u[i], 1e-5F, 1.F - 1e-5F);
                g[i] = std::log(U / (1.F - U)) *
                       std::sqrt(static_cast<float>(vamp::utils::constants::pi) / 8.F);
                g2 += g[i] * g[i];
            }
            const float g_norm = std::sqrt(g2);
            const float r = std::pow(std::clamp(r_uniform, 1e-6F, 1.F), 1.F / static_cast<float>(N));
            std::array<float, N> p{};
            if (g_norm > 1e-8F)
            {
                const float scale = r / g_norm;
                for (std::size_t i = 0; i < N; ++i)
                {
                    p[i] = scale * g[i];
                }
            }
            return p;
        }

        // Sample uniformly in a PHS with foci a, b and transverse diameter d, given a
        // pre-drawn unit-ball point `p`. Rest-to-rest (min_diameter = ||a - b|| = 0) is a
        // sphere; general case uses ball -> ellipsoid scaling + Householder that sends the
        // first axis onto (b - a) / ||b - a||.
        template <std::size_t N>
        inline auto phs_transform(
            const std::array<float, N> &a,
            const std::array<float, N> &b,
            float transverse_diameter,
            float min_diameter,
            const std::array<float, N> &axis,  // (b-a)/||b-a|| or zero if rest-to-rest
            const std::array<float, N> &p) noexcept -> std::array<float, N>
        {
            std::array<float, N> center{};
            for (std::size_t i = 0; i < N; ++i)
            {
                center[i] = 0.5F * (a[i] + b[i]);
            }

            if (min_diameter < 1e-6F)
            {
                std::array<float, N> out{};
                const float radius = 0.5F * transverse_diameter;
                for (std::size_t i = 0; i < N; ++i)
                {
                    out[i] = center[i] + radius * p[i];
                }
                return out;
            }

            const float conjugate = std::sqrt(
                std::max(0.F, transverse_diameter * transverse_diameter - min_diameter * min_diameter));

            std::array<float, N> pe{};
            pe[0] = 0.5F * transverse_diameter * p[0];
            for (std::size_t k = 1; k < N; ++k)
            {
                pe[k] = 0.5F * conjugate * p[k];
            }

            // Householder H with H e_0 = axis:  H = I - 2 w w^T / (w^T w), w = e_0 - axis
            std::array<float, N> w{};
            w[0] = 1.F - axis[0];
            for (std::size_t k = 1; k < N; ++k)
            {
                w[k] = -axis[k];
            }
            float w2 = 0.F;
            for (std::size_t k = 0; k < N; ++k)
            {
                w2 += w[k] * w[k];
            }

            std::array<float, N> out{};
            if (w2 > 1e-8F)
            {
                float wdot = 0.F;
                for (std::size_t k = 0; k < N; ++k)
                {
                    wdot += w[k] * pe[k];
                }
                const float coef = 2.F * wdot / w2;
                for (std::size_t k = 0; k < N; ++k)
                {
                    out[k] = pe[k] - coef * w[k] + center[k];
                }
            }
            else
            {
                for (std::size_t k = 0; k < N; ++k)
                {
                    out[k] = pe[k] + center[k];
                }
            }
            return out;
        }
    }  // namespace detail

    template <typename Robot>
    struct FlaskInformedRNG : public rng::RNG<Robot>
    {
        static_assert(Robot::flask, "FlaskInformedRNG requires a flask robot");
        static constexpr std::size_t n = Robot::flat_dimension;
        static constexpr std::size_t dim = Robot::dimension;
        static_assert(dim == 2 * n, "flask Configuration must be (q, v) with dim = 2 * flat_dimension");
        static constexpr std::size_t max_tries = 8;

        using Configuration = typename Robot::Configuration;

        FlaskInformedRNG(
            const Configuration &start_in,
            const Configuration &goal_in,
            typename rng::RNG<Robot>::Ptr inner_in) noexcept
          : start(start_in), goal(goal_in), inner(std::move(inner_in))
        {
            alignas(FloatVectorAlignment) auto sa = start.to_array();
            alignas(FloatVectorAlignment) auto ga = goal.to_array();
            float yd2 = 0.F;
            float vd2 = 0.F;
            for (std::size_t i = 0; i < n; ++i)
            {
                y_s[i] = sa[i];
                y_g[i] = ga[i];
                v_s[i] = sa[n + i];
                v_g[i] = ga[n + i];
                const float dyi = y_g[i] - y_s[i];
                const float dvi = v_g[i] - v_s[i];
                y_axis[i] = dyi;
                v_axis[i] = dvi;
                yd2 += dyi * dyi;
                vd2 += dvi * dvi;
            }
            y_min_diameter = std::sqrt(yd2);
            v_min_diameter = std::sqrt(vd2);
            if (y_min_diameter > 1e-8F)
            {
                for (std::size_t i = 0; i < n; ++i)
                {
                    y_axis[i] /= y_min_diameter;
                }
            }
            if (v_min_diameter > 1e-8F)
            {
                for (std::size_t i = 0; i < n; ++i)
                {
                    v_axis[i] /= v_min_diameter;
                }
            }

            v_fast = 0.F;
            for (std::size_t i = 0; i < n; ++i)
            {
                v_fast = std::max(v_fast, Robot::velocity_limits[i]);
            }
        }

        inline void reset() noexcept override
        {
            inner->reset();
            inner->dist.reset();
        }

        inline void set_cost_bound(float c_best_in) noexcept
        {
            c_best = c_best_in;
            v_transverse = c_best / (2.F * std::sqrt(Robot::rho));
            y_transverse = c_best * v_fast / Robot::rho;
        }

        // Outer set nonempty iff both PHS diameters exceed their foci separations.
        inline auto feasible() const noexcept -> bool
        {
            if (not(c_best < std::numeric_limits<float>::max()))
            {
                return false;
            }
            if (y_transverse <= y_min_diameter)
            {
                return false;
            }
            if (v_transverse <= v_min_diameter)
            {
                return false;
            }
            return true;
        }

        inline auto next() noexcept -> FloatVector<dim> override
        {
            if (not feasible())
            {
                return inner->next();
            }

            alignas(FloatVectorAlignment) std::array<float, Configuration::num_scalars_rounded>
                z_buf{};

            for (std::size_t tries = 0; tries < max_tries; ++tries)
            {
                auto u = inner->next();
                Robot::descale_configuration(u);
                alignas(FloatVectorAlignment) auto ua = u.to_array();

                // y-PHS
                const float r_y = inner->dist.uniform_01();
                const auto p_y = detail::uniform_in_ball<n>(ua.data(), r_y);
                const auto y_sample = detail::phs_transform<n>(
                    y_s, y_g, y_transverse, y_min_diameter, y_axis, p_y);

                // v-PHS
                const float r_v = inner->dist.uniform_01();
                const auto p_v = detail::uniform_in_ball<n>(ua.data() + n, r_v);
                const auto v_sample = detail::phs_transform<n>(
                    v_s, v_g, v_transverse, v_min_diameter, v_axis, p_v);

                for (std::size_t i = 0; i < n; ++i)
                {
                    z_buf[i] = y_sample[i];
                    z_buf[n + i] = v_sample[i];
                }

                Configuration z(z_buf.data());

                if (planning::cost<Robot>(start, z) + planning::cost<Robot>(z, goal) <= c_best)
                {
                    Robot::descale_configuration(z);
                    z = z.clamp(0.F, 1.F);
                    Robot::scale_configuration(z);
                    return z;
                }
            }

            return inner->next();
        }

    private:
        Configuration start;
        Configuration goal;
        typename rng::RNG<Robot>::Ptr inner;

        std::array<float, n> y_s{};
        std::array<float, n> y_g{};
        std::array<float, n> v_s{};
        std::array<float, n> v_g{};
        std::array<float, n> y_axis{};  // unit if y_min_diameter > 0
        std::array<float, n> v_axis{};  // unit if v_min_diameter > 0

        float y_min_diameter = 0.F;
        float v_min_diameter = 0.F;
        float v_fast = 0.F;
        float c_best = std::numeric_limits<float>::max();
        float y_transverse = std::numeric_limits<float>::max();
        float v_transverse = std::numeric_limits<float>::max();
    };
}  // namespace vamp::planning
