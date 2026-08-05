// (d) Structured per-sphere position recurrence. Each sphere traces DC+single-sinusoid
// along the edge -> exact order-3 recurrence p_{k+1}=a(p_k-p_{k-1})+p_{k-2}, a=1+2cos(wD)
// per link. Just 1 mul + 2 add per coord (9 ops/sphere), NO rotation matrix, NO chain.
// Seed 4 rakes with full sphere_fk, derive a per sphere, recur the rest. Compare cost +
// error vs full FK. (Approximate ~ the order-3 fit error; report it.)
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "fetch_base.hh"

using R = vamp::robots::FetchBase;
constexpr std::size_t rake = vamp::FloatVectorWidth, dim = R::dimension, ns = R::n_spheres;
using DataV = vamp::FloatVector<rake>;

int main()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(0.f, 1.f), nd(-1.f, 1.f);
    const float L = 1.0f;
    for (std::size_t n : {std::size_t(8), std::size_t(12), std::size_t(16)})
    {
        R::ConfigurationBlock<rake> s0;
        for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
        R::scale_configuration_block<rake>(s0);
        std::array<float, dim> start, v; float nr = 0;
        for (std::size_t j = 0; j < dim; ++j) { start[j] = s0[j].to_array()[0]; v[j] = nd(rng); nr += v[j] * v[j]; }
        nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) v[j] *= L / nr;
        float dt = 1.0f / static_cast<float>(n * rake);
        std::vector<R::ConfigurationBlock<rake>> B(n);
        for (std::size_t r = 0; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) {
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
            for (std::size_t l = 0; l < rake; ++l) ln[l] = start[j] + static_cast<float>(r * rake + l) * dt * v[j];
            B[r][j] = DataV(ln.data()); }

        // reference: full FK every rake
        std::vector<R::Spheres<rake>> ref(n);
        for (std::size_t r = 0; r < n; ++r) R::sphere_fk<rake>(B[r], ref[r]);

        // recurrence method: seed rakes 0..3 full, derive alpha per sphere, recur 4..n-1
        std::vector<R::Spheres<rake>> got(n);
        for (std::size_t r = 0; r < 4; ++r) R::sphere_fk<rake>(B[r], got[r]);
        std::array<float, ns> alpha;
        for (std::size_t s = 0; s < ns; ++s) {
            // robust alpha: pick the coordinate with largest |p2-p1| (best-conditioned), lane 0
            auto g=[&](const R::Spheres<rake>&S,std::size_t c){ return (c==0?S.x[s]:c==1?S.y[s]:S.z[s]).to_array()[0]; };
            float best=0, a=2.0f;
            for (std::size_t c=0;c<3;++c){ float p0=g(got[0],c),p1=g(got[1],c),p2=g(got[2],c),p3=g(got[3],c); float den=p2-p1;
                if (std::abs(den)>best){ best=std::abs(den); a=(p3-p0)/den; } }
            alpha[s]=a;
        }
        float maxerr = 0;
        for (std::size_t r = 4; r < n; ++r) {
            for (std::size_t s = 0; s < ns; ++s) {
                DataV a = DataV::fill(alpha[s]);
                got[r].x[s] = a*(got[r-1].x[s]-got[r-2].x[s])+got[r-3].x[s];
                got[r].y[s] = a*(got[r-1].y[s]-got[r-2].y[s])+got[r-3].y[s];
                got[r].z[s] = a*(got[r-1].z[s]-got[r-2].z[s])+got[r-3].z[s];
            }
            for (std::size_t s = 0; s < ns; ++s) { auto e=(got[r].x[s]-ref[r].x[s]).to_array(); for(std::size_t l=0;l<rake;++l) maxerr=std::max(maxerr,std::abs(e[l])); }
        }

        constexpr int K = 40000;
        auto med=[&](auto fn){std::vector<double> t;volatile float sk=0;for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();float ac=fn();auto z=std::chrono::steady_clock::now();sk+=ac;t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
        std::vector<R::Spheres<rake>> w(n);
        double t_full = med([&]{float ac=0;for(int k=0;k<K;++k){for(std::size_t r=0;r<n;++r)R::sphere_fk<rake>(B[r],w[r]);ac+=w[0].x[0].to_array()[0];}return ac;});
        double t_rec = med([&]{float ac=0;for(int k=0;k<K;++k){
            for(std::size_t r=0;r<4;++r)R::sphere_fk<rake>(B[r],w[r]);
            for(std::size_t r=4;r<n;++r)for(std::size_t s=0;s<ns;++s){DataV a=DataV::fill(alpha[s]);
                w[r].x[s]=a*(w[r-1].x[s]-w[r-2].x[s])+w[r-3].x[s];
                w[r].y[s]=a*(w[r-1].y[s]-w[r-2].y[s])+w[r-3].y[s];
                w[r].z[s]=a*(w[r-1].z[s]-w[r-2].z[s])+w[r-3].z[s];}
            ac+=w[n-1].x[0].to_array()[0];}return ac;});
        std::printf("n_rakes=%zu (seed 4): full FK=%.1f ns  seed+recur=%.1f ns  speedup=%.2fx  max_pos_err=%.1e (m)\n",
                    n, t_full, t_rec, t_full/t_rec, maxerr);
    }
    return 0;
}
