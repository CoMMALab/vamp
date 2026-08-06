// Measure the sincos-hoist opportunity on a real TSR kernel (bimanual panda, 14-DOF constraint
// error+Jacobian). Compares the generated kernel (84 separate sin/cos calls, 14 joints computed
// ~3x each) against a hoisted variant (14 fused sincos in a preamble, all sin/cos reads replaced).
// Settles whether the compiler already CSEs the redundant trig, and sizes the fuse+dedup win.
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <random>
#include <algorithm>
#include <vamp/vector.hh>
#include <vamp/vector/math.hh>

using namespace vamp;
constexpr std::size_t rake = FloatVectorWidth;
using V1 = FloatVector<rake, 1>;

#include "tsr_gen.inc"   // defines K::tsr_orig<rake>(x,out), K::tsr_hoist<rake>(x,out)

int main()
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-3.0f, 3.0f);
    const int N = 200000;
    std::vector<std::array<V1, 14>> xs(N);
    for (auto &x : xs)
        for (auto &j : x) { alignas(FloatVectorAlignment) std::array<float, rake> a; for (auto &f : a) f = u(rng); j = V1(a.data()); }

    // correctness: max abs diff over all outputs/lanes
    std::array<V1, 90> oo, oh; double maxdiff = 0, maxrel = 0;
    for (int i = 0; i < 2000; ++i) {
        K::tsr_orig<rake>(xs[i], oo);
        K::tsr_hoist<rake>(xs[i], oh);
        for (int k = 0; k < 90; ++k) { auto ao = oo[k].to_array(), ah = oh[k].to_array();
            for (std::size_t l = 0; l < rake; ++l) { double d = std::abs(ao[l] - ah[l]); maxdiff = std::max(maxdiff, d);
                maxrel = std::max(maxrel, d / (std::abs((double)ao[l]) + 1e-6)); } }
    }

    auto med = [&](auto fn) { std::vector<double> t; volatile float sk = 0;
        for (int rp = 0; rp < 9; ++rp) { std::array<V1, 90> o; auto a = std::chrono::steady_clock::now();
            for (int i = 0; i < N; ++i) { fn(xs[i], o); sk += o[0].to_array()[0]; }
            auto z = std::chrono::steady_clock::now(); t.push_back(std::chrono::duration<double>(z - a).count() / N * 1e9); }
        (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2]; };

    double to = med([](const std::array<V1, 14> &x, std::array<V1, 90> &o) { K::tsr_orig<rake>(x, o); });
    double th = med([](const std::array<V1, 14> &x, std::array<V1, 90> &o) { K::tsr_hoist<rake>(x, o); });
    std::printf("TSR bimanual error+Jacobian (14-DOF)  max-abs=%.2e max-rel=%.2e\n", maxdiff, maxrel);
    std::printf("  orig (84 sin/cos):     %.1f ns/call\n", to);
    std::printf("  hoist (14 sincos):     %.1f ns/call   speedup=%.2fx\n", th, to / th);
    return 0;
}
