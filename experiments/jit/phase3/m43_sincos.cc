// Validate fused sincos: accuracy vs VAMP sin()/cos(), and isolated trig speedup vs sin()+cos().
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "panda_e.hh"   // pulls in vamp::FloatVector

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

int main()
{
    std::mt19937 rng(1); std::uniform_real_distribution<float> u(-8.0f, 8.0f);
    const int N = 4096;
    std::vector<DataV> xs(N);
    for (auto &x : xs) { alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln; for (auto &v : ln) v = u(rng); x = DataV(ln.data()); }

    // accuracy: fused sincos vs sin()/cos()
    float ds = 0, dc = 0;
    for (auto &x : xs) { DataV s, c; x.sincos(s, c); DataV rs = x.sin(), rc = x.cos();
        auto sa = s.to_array(), ca = c.to_array(), rsa = rs.to_array(), rca = rc.to_array();
        for (std::size_t l = 0; l < rake; ++l) { ds = std::max(ds, std::fabs(sa[l] - rsa[l])); dc = std::max(dc, std::fabs(ca[l] - rca[l])); } }
    std::printf("accuracy: max|fused_sin - sin()| = %.2e   max|fused_cos - cos()| = %.2e\n", ds, dc);

    auto med = [&](auto fn) { std::vector<double> t; volatile float sk = 0;
        for (int rp = 0; rp < 11; ++rp) { auto a = std::chrono::steady_clock::now(); float acc = 0;
            for (int it = 0; it < 400; ++it) for (auto &x : xs) acc += fn(x); auto z = std::chrono::steady_clock::now(); sk += acc;
            t.push_back(std::chrono::duration<double>(z - a).count() / (400.0 * N) * 1e9); } (void)sk;
        std::sort(t.begin(), t.end()); return t[t.size() / 2]; };

    double sep = med([](DataV &x) { DataV s = x.sin(), c = x.cos(); return s.to_array()[0] + c.to_array()[0]; });
    double fus = med([](DataV &x) { DataV s, c; x.sincos(s, c); return s.to_array()[0] + c.to_array()[0]; });
    std::printf("timing (per sin+cos pair, %zu-wide SIMD): separate = %.2f ns   fused = %.2f ns   speedup = %.2fx\n",
                rake, sep, fus, sep / fus);
    return 0;
}
