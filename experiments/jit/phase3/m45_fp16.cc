// fp16 feasibility on this CPU: a representative FK op (rotate+translate a batch of points,
// like sphere placement) in float vs _Float16. Native fp16 arithmetic needs AVX512-FP16; on
// AVX2 (i9-14900K: avx2 f16c, no avx512fp16) _Float16 math is emulated via convert-to-float,
// so it should be SLOWER, not the 2x we'd want. This confirms the idea is hardware-gated.
#include <chrono>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

template <typename T>
static double bench(int NP)
{
    std::mt19937 rng(1); std::uniform_real_distribution<float> u(-1, 1);
    std::vector<T> px(NP), py(NP), pz(NP), ox(NP), oy(NP), oz(NP);
    for (int i = 0; i < NP; ++i) { px[i] = (T)u(rng); py[i] = (T)u(rng); pz[i] = (T)u(rng); }
    T r00 = (T)0.36, r01 = (T)-0.8, r02 = (T)0.48, r10 = (T)0.8, r11 = (T)0.6, r12 = (T)0.0,
      r20 = (T)-0.29, r21 = (T)0.39, r22 = (T)0.88, tx = (T)0.5, ty = (T)-0.3, tz = (T)0.9;
    std::vector<double> t; volatile double sk = 0;
    for (int rp = 0; rp < 9; ++rp) {
        auto a = std::chrono::steady_clock::now();
        for (int it = 0; it < 2000; ++it)
            for (int i = 0; i < NP; ++i) {
                ox[i] = tx + r00 * px[i] + r01 * py[i] + r02 * pz[i];
                oy[i] = ty + r10 * px[i] + r11 * py[i] + r12 * pz[i];
                oz[i] = tz + r20 * px[i] + r21 * py[i] + r22 * pz[i];
            }
        auto z = std::chrono::steady_clock::now();
        sk += (double)ox[NP / 2] + (double)oz[0];
        t.push_back(std::chrono::duration<double>(z - a).count() / (2000.0 * NP) * 1e9);
    }
    (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2];
}

int main()
{
    for (int NP : {75, 512}) {
        double f = bench<float>(NP), h = bench<_Float16>(NP);
        std::printf("points=%d  float=%.3f ns/pt  _Float16=%.3f ns/pt  fp16/float=%.2fx (%s)\n",
                    NP, f, h, h / f, h < f ? "fp16 faster" : "fp16 SLOWER (emulated)");
    }
    return 0;
}
