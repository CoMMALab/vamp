// Sanity check for the native xorshift sampler. Robot::sample maps [0,1) -> joint ranges, so we
// center each dimension on its own mean (two deterministic passes) and check serial + cross-dim
// correlation. A working sampler: |serial-corr| and |cross-dim corr| ~ 0.
#include <cstdio>
#include <cmath>
#include <array>
#include <vamp/random/xorshift_native.hh>
#include <vamp/random/xoshiro128.hh>
#include "baxter_e.hh"

using R = vamp::robots::BaxterE;
constexpr std::size_t D = R::dimension;
const int N = 500000;

int main()
{
#ifdef TEST_XOSHIRO
    vamp::rng::Xoshiro128<R> rng;
#else
    vamp::rng::XORShiftNative<R> rng;
#endif
    // pass 1: means
    std::array<double, D> sum{};
    for (int i = 0; i < N; ++i) { auto a = rng.next().to_array(); for (std::size_t j = 0; j < D; ++j) sum[j] += a[j]; }
    std::array<double, D> mean{}; for (std::size_t j = 0; j < D; ++j) mean[j] = sum[j] / N;

    // pass 2: var, serial autocorr (dim j vs its own previous), cross-dim (0 vs 1)
    rng.reset();
    std::array<double, D> var{}, serial{}, prev{}; double cross01 = 0;
    for (int i = 0; i < N; ++i)
    {
        auto a = rng.next().to_array();
        for (std::size_t j = 0; j < D; ++j)
        {
            double d = a[j] - mean[j];
            var[j] += d * d;
            if (i > 0) serial[j] += d * (prev[j] - mean[j]);
            prev[j] = a[j];
        }
        cross01 += (a[0] - mean[0]) * (a[1] - mean[1]);
    }
    int bad = 0;
    for (std::size_t j = 0; j < D; ++j)
    {
        double v = var[j] / N, r1 = (serial[j] / (N - 1)) / v;
        if (std::abs(r1) > 0.01) bad++;
        if (j < 4 || j >= D - 2) std::printf("dim %2zu: mean=%+.4f sd=%.4f serial-corr=%+.5f\n", j, mean[j], std::sqrt(v), r1);
    }
    double xc = (cross01 / N) / std::sqrt((var[0] / N) * (var[1] / N));
    if (std::abs(xc) > 0.01) bad++;
    std::printf("cross-dim(0,1) corr=%+.5f  | %s\n", xc, bad == 0 ? "PASS" : "FAIL");
    return 0;
}
