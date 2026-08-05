// Why lazy leaf FK backfires: is FK cost in the leaves, or the shared chain transforms?
// Time bound_fk (bounding spheres only) vs sphere_fk (all leaves) vs fkcc (fused) per block.
// If bound_fk ~= sphere_fk, transforms dominate and skipping leaves saves ~nothing.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

template <class R>
static void run(const char *name)
{
    constexpr std::size_t dim=R::dimension;
    std::mt19937 rng(1); std::uniform_real_distribution<float> u(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    std::vector<Block> blocks(4000);
    for(auto&b:blocks){ for(std::size_t j=0;j<dim;++j) b[j]=static_cast<DataV>(u(rng)); R::template scale_configuration_block<rake>(b); }
    vamp::collision::Environment<DataV> empty;

    typename R::template BoundingSpheres<rake> bs; typename R::template Spheres<rake> sp;
    auto tb=[&]{ float a=0; for(auto&b:blocks){ R::template bound_fk<rake>(b,bs); a+=bs.x[0].to_array()[0]+bs.r[0].to_array()[0]; } return a; };
    auto ts=[&]{ float a=0; for(auto&b:blocks){ R::template sphere_fk<rake>(b,sp); a+=sp.x[0].to_array()[0]+sp.r[0].to_array()[0]; } return a; };
    auto tc=[&]{ std::uint32_t a=0; for(auto&b:blocks) a+=R::template fkcc<rake>(empty,b)?1:0; return (float)a; };
    auto med=[&](auto fn){ std::vector<double> t; volatile double sk=0;
        for(int rp=0;rp<11;++rp){auto x=std::chrono::steady_clock::now();double v=fn();auto y=std::chrono::steady_clock::now();sk+=v;
            t.push_back(std::chrono::duration<double>(y-x).count()/blocks.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    double B=med(tb),S=med(ts),C=med(tc);
    std::printf("%-6s dim=%zu bs=%zu leaves=%zu | bound_fk=%.0f  sphere_fk=%.0f  fkcc(empty)=%.0f ns  | bound/sphere=%.2f leaf_share=%.0f%%\n",
                name,dim,R::n_bounding_spheres,R::n_spheres, B,S,C, B/S, 100.0*(S-B)/S);
}

int main(){ run<vamp::robots::PandaE>("panda"); run<vamp::robots::FetchE>("fetch"); run<vamp::robots::BaxterE>("baxter"); return 0; }
