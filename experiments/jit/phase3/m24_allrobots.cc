// (a)+(b) Leaf-trig recurrence FK win across robots at REALISTIC RRTC edge lengths.
// RRTC extends by up to `range`; at resolution 32 an edge has ~ceil(range*32/rake) rakes.
// range: panda 1.25 (n=5), fetch 1.0 (n=4), baxter 0.5 (n=2), r2c6 ~1.0 (n=4).
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "fetch_pt.hh"
#include "panda_pt.hh"
#include "baxter_pt.hh"
#include "r2c6_pt.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

template <class R, std::size_t ASZ>
static void run(const char *name, std::size_t jlo, std::size_t jhi, float range)
{
    constexpr std::size_t dim = R::dimension, ns = R::n_spheres;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake));
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(0.f, 1.f), nd(-1.f, 1.f);
    typename R::template ConfigurationBlock<rake> s0;
    for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
    R::template scale_configuration_block<rake>(s0);
    std::array<float, dim> start, v; float nr = 0;
    for (std::size_t j = 0; j < dim; ++j) { start[j] = s0[j].to_array()[0]; v[j] = nd(rng); nr += v[j] * v[j]; }
    nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) v[j] *= range / nr;
    float dt = 1.0f / static_cast<float>(n * rake);
    std::vector<typename R::template ConfigurationBlock<rake>> B(n);
    for (std::size_t r = 0; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) {
        alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
        for (std::size_t l = 0; l < rake; ++l) ln[l] = start[j] + static_cast<float>(r * rake + l) * dt * v[j];
        B[r][j] = DataV(ln.data()); }
    std::array<float, dim> C8{}, S8{};
    for (std::size_t j = jlo; j < jhi; ++j) { float d8 = rake * dt * v[j]; C8[j] = std::cos(d8); S8[j] = std::sin(d8); }

    // verify
    typename R::template Spheres<rake> os, orr; std::array<DataV, ASZ> ps{}, pc{}; float me = 0;
    for (std::size_t r = 0; r < n; ++r) {
        R::template sphere_fk<rake>(B[r], os);
        if (r == 0) for (std::size_t j = jlo; j < jhi; ++j) { ps[j] = ::sin(B[0][j]); pc[j] = ::cos(B[0][j]); }
        else for (std::size_t j = jlo; j < jhi; ++j) { DataV c=DataV::fill(C8[j]),s=DataV::fill(S8[j]); DataV sn=ps[j]*c+pc[j]*s,cn=pc[j]*c-ps[j]*s; ps[j]=sn; pc[j]=cn; }
        R::template sphere_fk_pretrig<rake>(B[r], ps, pc, orr);
        for (std::size_t sp=0; sp<ns; ++sp){auto ex=(os.x[sp]-orr.x[sp]).to_array();for(std::size_t l=0;l<rake;++l) me=std::max(me,std::abs(ex[l]));}
    }
    constexpr int K = 60000;
    auto med=[&](auto fn){std::vector<double> t;volatile float sk=0;for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();float ac=fn();auto z=std::chrono::steady_clock::now();sk+=ac;t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    typename R::template Spheres<rake> o;
    double tf=med([&]{float ac=0;for(int k=0;k<K;++k)for(std::size_t r=0;r<n;++r){R::template sphere_fk<rake>(B[r],o);ac+=o.x[0].to_array()[0];}return ac;});
    double tr=med([&]{float ac=0;for(int k=0;k<K;++k){std::array<DataV,ASZ> lp{},lc{};
        for(std::size_t r=0;r<n;++r){if(r==0){for(std::size_t j=jlo;j<jhi;++j){lp[j]=::sin(B[0][j]);lc[j]=::cos(B[0][j]);}}
            else{for(std::size_t j=jlo;j<jhi;++j){DataV c=DataV::fill(C8[j]),s=DataV::fill(S8[j]);DataV sn=lp[j]*c+lc[j]*s,cn=lc[j]*c-lp[j]*s;lp[j]=sn;lc[j]=cn;}}
            R::template sphere_fk_pretrig<rake>(B[r],lp,lc,o);ac+=o.x[0].to_array()[0];}}return ac;});
    std::printf("%-7s dim=%2zu ns=%3zu range=%.2f n_rakes=%zu: stock FK=%.1f recur FK=%.1f  speedup=%.2fx  err=%.1e\n",
                name, dim, ns, range, n, tf, tr, tf/tr, me);
}

int main()
{
    run<vamp::robots::Panda, 7>("panda", 0, 7, 1.25f);
    run<vamp::robots::Fetch, 12>("fetch", 1, 8, 1.0f);
    run<vamp::robots::Baxter, 14>("baxter", 0, 14, 0.5f);
    run<vamp::robots::R2c6, 36>("r2c6", 7, 36, 1.0f);
    return 0;
}
