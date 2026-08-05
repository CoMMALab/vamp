// Edge-continuity FK, step 1: the trig recurrence. Sub-configs along an edge are
// collinear (theta_j(t) linear in t), so along the rakes the per-joint angle is an
// arithmetic progression and sin/cos obey an exact 3-term recurrence:
//   s_{r} = s_{r-1} cos(8 delta_j) + c_{r-1} sin(8 delta_j)
//   c_{r} = c_{r-1} cos(8 delta_j) - s_{r-1} sin(8 delta_j)
// Compute rake 0 with full SIMD sin/cos, then recur the rest (4 mul + 2 add each,
// SIMD over the 8 lanes). Exact (no approximation). Measures the trig cost saved.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/vector.hh>
#include <vamp/vector/math.hh>

constexpr std::size_t rake = vamp::FloatVectorWidth, dim = 8;  // Fetch-like
using DataV = vamp::FloatVector<rake>;

int main()
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> ang(-3.f, 3.f), dir(-1.f, 1.f);

    for (std::size_t n : {std::size_t(2), std::size_t(4), std::size_t(8)})
    {
        // one representative edge
        std::array<float, dim> a, delta, C8, S8;
        float L = 1.0f, dt = 1.0f / static_cast<float>(n * rake);
        for (std::size_t j = 0; j < dim; ++j)
        {
            a[j] = ang(rng);
            float vj = dir(rng) * L;          // edge component
            delta[j] = dt * vj;               // angle increment per t-index
            C8[j] = std::cos(rake * delta[j]);
            S8[j] = std::sin(rake * delta[j]);
        }
        // per-(rake,joint) angle block
        auto theta = [&](std::size_t r, std::size_t j)
        {
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
            for (std::size_t l = 0; l < rake; ++l) ln[l] = a[j] + static_cast<float>(r * rake + l) * delta[j];
            return DataV(ln.data());
        };

        std::vector<std::array<DataV, dim>> sA(n), cA(n), sB(n), cB(n);

        // Method A: full SIMD sin/cos every rake
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t j = 0; j < dim; ++j) { auto t = theta(r, j); sA[r][j] = ::sin(t); cA[r][j] = ::cos(t); }

        // Method B: rake 0 full, then recurrence
        for (std::size_t j = 0; j < dim; ++j) { auto t = theta(0, j); sB[0][j] = ::sin(t); cB[0][j] = ::cos(t); }
        for (std::size_t r = 1; r < n; ++r)
            for (std::size_t j = 0; j < dim; ++j)
            {
                DataV c8 = DataV::fill(C8[j]), s8 = DataV::fill(S8[j]);
                sB[r][j] = sB[r - 1][j] * c8 + cB[r - 1][j] * s8;
                cB[r][j] = cB[r - 1][j] * c8 - sB[r - 1][j] * s8;
            }

        // verify
        float maxerr = 0;
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t j = 0; j < dim; ++j)
            {
                auto ea = (sA[r][j] - sB[r][j]).to_array(); auto eb = (cA[r][j] - cB[r][j]).to_array();
                for (std::size_t l = 0; l < rake; ++l) { maxerr = std::max(maxerr, std::abs(ea[l])); maxerr = std::max(maxerr, std::abs(eb[l])); }
            }

        // time (recompute trig K times)
        constexpr int K = 300000;
        auto med = [&](auto fn) {
            std::vector<double> t; volatile float sink = 0;
            for (int rep = 0; rep < 7; ++rep) {
                auto z0 = std::chrono::steady_clock::now();
                float acc = fn();
                auto z1 = std::chrono::steady_clock::now(); sink += acc;
                t.push_back(std::chrono::duration<double>(z1 - z0).count() / K * 1e9);
            }
            (void)sink; std::sort(t.begin(), t.end()); return t[t.size() / 2];
        };
        // rebuild delta/a per iter to avoid caching; measure whole-edge trig
        double tA = med([&]{ float acc = 0; for (int k = 0; k < K; ++k) {
            for (std::size_t r = 0; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) {
                auto t = theta(r, j); acc += (::sin(t) + ::cos(t)).to_array()[0]; } } return acc; });
        double tB = med([&]{ float acc = 0; for (int k = 0; k < K; ++k) {
            std::array<DataV, dim> s, c;
            for (std::size_t j = 0; j < dim; ++j) { auto t = theta(0, j); s[j] = ::sin(t); c[j] = ::cos(t); acc += (s[j]+c[j]).to_array()[0]; }
            for (std::size_t r = 1; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) {
                DataV c8 = DataV::fill(C8[j]), s8 = DataV::fill(S8[j]);
                DataV sn = s[j]*c8 + c[j]*s8, cn = c[j]*c8 - s[j]*s8; s[j]=sn; c[j]=cn; acc += (sn+cn).to_array()[0]; } } return acc; });

        std::printf("n_rakes=%zu dim=%zu: full=%.1f ns  recurrence=%.1f ns  speedup=%.2fx  max_err=%.1e\n",
                    n, dim, tA, tB, tA / tB, maxerr);
    }
    return 0;
}
