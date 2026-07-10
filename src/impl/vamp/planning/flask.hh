#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace vamp::planning::flask
{
    struct LQMTSolution
    {
        float time;
        float cost;
    };

    // Optimal LQMT cubic (r = 2 flat systems, R = I): duration T* is the positive real root of
    //   rho T^4 - 4(|v0|^2 + v0.vf + |vf|^2) T^2 + 24 (v0 + vf).dy T - 36 |dy|^2 = 0
    // minimizing C(T) = rho T + (12|d1|^2 - 12 d1.d2 T + 4|d2|^2 T^2) / T^3,
    // with dy = yf - y0, d1 = dy - T v0, d2 = vf - v0. Returns {T*, C(T*)}.
    //
    // Note C is asymmetric in (a, b); evaluate in execution (a -> b) direction.
    // Generic over any flask robot: depends only on Robot::flat_dimension and Robot::rho.
    template <typename Robot>
    inline auto solve(
        const typename Robot::Configuration &a_in,
        const typename Robot::Configuration &b_in) noexcept -> LQMTSolution
    {
        constexpr std::size_t n = Robot::flat_dimension;
        constexpr double min_time = 1e-4;

        const auto a = a_in.to_array();
        const auto b = b_in.to_array();

        double v00 = 0., v0f = 0., vff = 0., dyv0 = 0., dyvf = 0., dy2 = 0.;
        for (std::size_t j = 0; j < n; ++j)
        {
            const double v0 = a[n + j];
            const double vf = b[n + j];
            const double dy = static_cast<double>(b[j]) - static_cast<double>(a[j]);
            v00 += v0 * v0;
            v0f += v0 * vf;
            vff += vf * vf;
            dyv0 += dy * v0;
            dyvf += dy * vf;
            dy2 += dy * dy;
        }

        // Depressed quartic T^4 + p T^2 + q T + r = 0 (stationarity of C, divided by rho)
        constexpr double rho_d = static_cast<double>(Robot::rho);
        const double p = -4. * (v00 + v0f + vff) / rho_d;
        const double q = 24. * (dyv0 + dyvf) / rho_d;
        const double r = -36. * dy2 / rho_d;

        const double d2d2 = v00 - 2. * v0f + vff;
        const auto cost = [&](double T) -> double
        {
            const double d1d1 = dy2 - 2. * T * dyv0 + T * T * v00;
            const double d1d2 = (dyvf - dyv0) - T * (v0f - v00);
            return rho_d * T + (12. * d1d1 - 12. * d1d2 * T + 4. * d2d2 * T * T) / (T * T * T);
        };

        std::array<double, 4> roots{};
        std::size_t n_roots = 0;

        const auto add_quadratic_roots = [&](double qb, double qc)
        {
            // T^2 + qb T + qc = 0
            const double disc = qb * qb - 4. * qc;
            if (disc < 0.)
            {
                return;
            }

            const double s = std::sqrt(disc);
            roots[n_roots++] = 0.5 * (-qb + s);
            roots[n_roots++] = 0.5 * (-qb - s);
        };

        if (std::abs(q) < 1e-12)
        {
            // Biquadratic: T^2 = (-p +- sqrt(p^2 - 4r)) / 2
            const double disc = p * p - 4. * r;
            if (disc >= 0.)
            {
                const double s = std::sqrt(disc);
                const double u1 = 0.5 * (-p + s);
                const double u2 = 0.5 * (-p - s);
                if (u1 > 0.)
                {
                    roots[n_roots++] = std::sqrt(u1);
                }

                if (u2 > 0.)
                {
                    roots[n_roots++] = std::sqrt(u2);
                }
            }
        }
        else
        {
            // Ferrari: resolvent cubic m^3 + p m^2 + (p^2/4 - r) m - q^2/8 = 0.
            // At m = 0 it is negative (-q^2/8) so a positive real root always exists.
            const double cb = p;
            const double cc = 0.25 * p * p - r;
            const double cd = -0.125 * q * q;

            // Depressed cubic s^3 + ps s + qs = 0 via m = s - cb/3; take the largest real root
            const double ps = cc - cb * cb / 3.;
            const double qs = 2. * cb * cb * cb / 27. - cb * cc / 3. + cd;
            const double cubic_disc = 0.25 * qs * qs + ps * ps * ps / 27.;

            double m = 0.;
            if (cubic_disc > 0.)
            {
                const double sc = std::sqrt(cubic_disc);
                m = std::cbrt(-0.5 * qs + sc) + std::cbrt(-0.5 * qs - sc) - cb / 3.;
            }
            else
            {
                const double rr = std::sqrt(-ps * ps * ps / 27.);
                const double theta = std::acos(std::clamp(-0.5 * qs / rr, -1., 1.));
                m = 2. * std::sqrt(-ps / 3.) * std::cos(theta / 3.) - cb / 3.;
            }

            if (m > 1e-14)
            {
                const double s2m = std::sqrt(2. * m);
                const double half = 0.5 * p + m;
                const double qterm = q / (2. * s2m);
                add_quadratic_roots(-s2m, half + qterm);
                add_quadratic_roots(s2m, half - qterm);
            }
        }

        double best_T = 0.;
        double best_cost = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < n_roots; ++i)
        {
            double T = roots[i];
            if (not (T > min_time))
            {
                continue;
            }

            // Newton polish on the quartic
            for (auto k = 0U; k < 2; ++k)
            {
                const double f = ((T * T + p) * T + q) * T + r;
                const double df = (4. * T * T + 2. * p) * T + q;
                if (std::abs(df) > 1e-14)
                {
                    T -= f / df;
                }
            }

            if (not (T > min_time))
            {
                continue;
            }

            const double c = cost(T);
            if (c < best_cost)
            {
                best_cost = c;
                best_T = T;
            }
        }

        if (not (best_T > 0.))
        {
            return {static_cast<float>(min_time), static_cast<float>(cost(min_time))};
        }

        return {static_cast<float>(best_T), static_cast<float>(best_cost)};
    }

    template <typename Robot>
    inline auto optimal_time(
        const typename Robot::Configuration &a,
        const typename Robot::Configuration &b) noexcept -> float
    {
        return solve<Robot>(a, b).time;
    }

    // LQMT edge cost C_loc(a -> b); asymmetric.
    template <typename Robot>
    inline auto cost(
        const typename Robot::Configuration &a,
        const typename Robot::Configuration &b) noexcept -> float
    {
        return solve<Robot>(a, b).cost;
    }
}  // namespace vamp::planning::flask
