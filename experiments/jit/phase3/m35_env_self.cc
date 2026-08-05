// Decompose free-edge collision into ENV vs SELF -- the swept broadphase only elides ENV, so
// its ceiling is the env slice, not all collision. FK-only (sphere_fk) = FK; fkcc on an EMPTY
// environment = FK + self (+ trivial env-broadphase); fkcc on the shelf = FK + self + env.
//   self  ~= empty_env_fkcc - fk_only ;  env ~= full_fkcc - empty_env_fkcc
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "panda_e.hh"
#include "fetch_e.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

template <class R>
static auto shelf(std::mt19937 &rng, int nobs)
{
    vamp::collision::Environment<float> ef;
    std::uniform_real_distribution<float> sx(0.45F,0.85F), sy(-0.35F,0.35F), sz(0.45F,1.0F), rad(0.02F,0.05F);
    for (int i=0;i<nobs;++i) ef.spheres.emplace_back(vamp::collision::factory::sphere::array({sx(rng),sy(rng),sz(rng)},rad(rng)));
    ef.sort(); return vamp::collision::Environment<DataV>(ef);
}

template <class R>
static void run(const char *name, float range, int nobs)
{
    constexpr std::size_t dim=R::dimension;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake));
    std::mt19937 srng(0xBEEF), wrng(0x33);
    auto env=shelf<R>(srng,nobs); vamp::collision::Environment<DataV> empty;  // no obstacles
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto fkcc_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    std::vector<std::vector<Block>> edges; int attempts=0;
    while(edges.size()<1500 && attempts<400000){ ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nr=0; for(std::size_t j=0;j<dim;++j){s[j]=tmp[j].to_array()[0];v[j]=nd(wrng);nr+=v[j]*v[j];}
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr; float dt=1.0f/(float)(n*rake);
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        if(fkcc_valid(B)) edges.push_back(std::move(B));
    }

    typename R::template Spheres<rake> sph;
    auto fk_only=[&](const std::vector<Block>&B){ float a=0; for(auto&b:B){ R::template sphere_fk<rake>(b,sph); a+=sph.x[0].to_array()[0]+sph.r[0].to_array()[0]; } return a; };
    auto cc_empty=[&](const std::vector<Block>&B){ std::uint32_t a=0; for(auto&b:B) a+=R::template fkcc<rake>(empty,b)?1:0; return a; };
    auto cc_full=[&](const std::vector<Block>&B){ std::uint32_t a=0; for(auto&b:B) a+=R::template fkcc<rake>(env,b)?1:0; return a; };
    auto med=[&](auto fn){ std::vector<double> t; volatile double sk=0;
        for(int rp=0;rp<9;++rp){auto a=std::chrono::steady_clock::now();double ac=0;for(auto&e:edges)ac+=fn(e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/edges.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    double fk=med(fk_only), emp=med(cc_empty), full=med(cc_full);
    double self=emp-fk, envc=full-emp;
    std::printf("%-6s n=%zu obs=%d free=%zu\n", name, n, nobs, edges.size());
    std::printf("   FK=%.0f  self=%.0f  env=%.0f  (full=%.0f ns)\n", fk, self, envc, full);
    std::printf("   shares: FK=%.0f%%  self=%.0f%%  env=%.0f%%   -> swept-env ceiling=%.2fx\n",
                100*fk/full, 100*self/full, 100*envc/full, full/(full-envc));
}

int main()
{
    run<vamp::robots::PandaE>("panda", 1.25f, 40);
    run<vamp::robots::FetchE>("fetch", 1.0f, 40);
    return 0;
}
