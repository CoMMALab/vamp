// FLASK edges are polynomial in t (position+velocity BCs -> cubic). sin(theta(t)) is then a
// CHIRP, not a fixed sinusoid, so the linear-edge recurrence breaks. But a polynomial-phase
// chirp has an EXACT generator: the phasor cascade (finite differences). Maintain d+1 unit
// phasors w_0..w_d (w_m = e^{i * m-th forward diff of theta}); per step update w_m *= w_{m+1}
// (m=0..d-1), w_d constant. w_0 = (cos theta_k, sin theta_k), exact. d complex-mul/step vs a
// full sin+cos each step. This is the exact generalization of the RRTC leaf-trig recurrence
// (d=1). Uses FLASK's known polynomial (theta, dtheta=vel, d2theta=accel, ...).
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

int main()
{
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-1.f, 1.f);
    constexpr std::size_t dim = 7;         // joints
    for (int degree : {1, 3, 5})           // 1=RRTC(linear), 3=FLASK cubic, 5=quintic
    {
        for (std::size_t n : {std::size_t(16), std::size_t(48)})
        {
            // random polynomial phase per joint, sampled at k=0..n-1 (small per-step step)
            std::vector<std::array<double, dim>> theta(n);
            std::array<std::array<double, 6>, dim> coef{};   // up to degree 5
            for (std::size_t j = 0; j < dim; ++j) {
                coef[j][0] = u(rng) * 3.0;                    // constant
                for (int d = 1; d <= degree; ++d) coef[j][d] = u(rng) * 0.05 / std::pow((double)n, d - 1);
            }
            for (std::size_t k = 0; k < n; ++k) for (std::size_t j = 0; j < dim; ++j) {
                double t = (double)k, v = 0; for (int d = degree; d >= 0; --d) v = v * t + coef[j][d]; theta[k][j] = v;
            }
            // truth
            std::vector<std::array<float, dim>> S(n), C(n);
            for (std::size_t k = 0; k < n; ++k) for (std::size_t j = 0; j < dim; ++j) { S[k][j] = std::sin(theta[k][j]); C[k][j] = std::cos(theta[k][j]); }

            // cascade: seed d+1 phasors per joint from forward diffs of theta at k=0
            const int D = degree;
            std::array<std::vector<std::array<float, 2>>, dim> wtmp;  // unused; per-joint below
            float maxerr = 0;
            std::vector<std::array<std::array<float, 2>, dim>> W(D + 1);  // W[m][j] = phasor
            for (std::size_t j = 0; j < dim; ++j) {
                for (int m = 0; m <= D; ++m) {
                    // m-th forward difference of theta at 0: sum_{i=0}^m (-1)^{m-i} C(m,i) theta[i][j]
                    double fd = 0, ck = 1; for (int i = 0; i <= m; ++i) { double sign = ((m - i) & 1) ? -1 : 1;
                        double binom = std::tgamma(m + 1) / (std::tgamma(i + 1) * std::tgamma(m - i + 1));
                        fd += sign * binom * theta[i][j]; }
                    W[m][j] = { (float)std::cos(fd), (float)std::sin(fd) };
                }
            }
            auto cmul = [](std::array<float, 2> a, std::array<float, 2> b) { return std::array<float, 2>{ a[0]*b[0]-a[1]*b[1], a[0]*b[1]+a[1]*b[0] }; };
            for (std::size_t k = 0; k < n; ++k) {
                for (std::size_t j = 0; j < dim; ++j) { maxerr = std::max({maxerr, std::abs(W[0][j][0]-C[k][j]), std::abs(W[0][j][1]-S[k][j])}); }
                for (int m = 0; m < D; ++m) for (std::size_t j = 0; j < dim; ++j) W[m][j] = cmul(W[m][j], W[m + 1][j]);
            }

            // timing: full sin+cos vs cascade (advance only), per whole trajectory
            constexpr int K = 200000; volatile float sink = 0;
            auto med=[&](auto fn){std::vector<double> t;for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();float ac=fn();auto z=std::chrono::steady_clock::now();sink+=ac;t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);}std::sort(t.begin(),t.end());return t[t.size()/2];};
            double tf = med([&]{ float ac=0; for(int r=0;r<K;++r) for(std::size_t k=0;k<n;++k) for(std::size_t j=0;j<dim;++j){ ac+=std::sin(theta[k][j])+std::cos(theta[k][j]); } return ac; });
            // cascade: seed first sample full, then advance
            double tc = med([&]{ float ac=0; for(int r=0;r<K;++r){
                std::array<std::array<std::array<float,2>,dim>, 6> w;
                for(std::size_t j=0;j<dim;++j) for(int m=0;m<=D;++m){ double fd=0; for(int i=0;i<=m;++i){double sign=((m-i)&1)?-1:1;double binom=std::tgamma(m+1)/(std::tgamma(i+1)*std::tgamma(m-i+1));fd+=sign*binom*theta[i][j];} w[m][j]={(float)std::cos(fd),(float)std::sin(fd)}; }
                for(std::size_t k=0;k<n;++k){ for(std::size_t j=0;j<dim;++j) ac+=w[0][j][0]+w[0][j][1];
                    for(int m=0;m<D;++m) for(std::size_t j=0;j<dim;++j){auto&a=w[m][j];auto&b=w[m+1][j];w[m][j]={a[0]*b[0]-a[1]*b[1],a[0]*b[1]+a[1]*b[0]};} } } return ac; });
            std::printf("degree=%d (%s) n=%zu: full sin/cos=%.1f ns  cascade=%.1f ns  speedup=%.2fx  max_err=%.1e\n",
                        degree, degree==1?"RRTC linear":degree==3?"FLASK cubic":"quintic", n, tf, tc, tf/tc, maxerr);
        }
    }
    return 0;
}
