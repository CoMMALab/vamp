// Staged sphere-aware FK: joint_tf(x) once -> place spheres as world = R*local + t.
// Validates the concept: is transforms-once + cheap placement faster than the fused kernels,
// and does it enable single-pass lazy (bounding + only-needed leaves, no transform recompute)?
//   fused:  bound_fk (bounding), sphere_fk (leaves)
//   staged: joint_tf + place(bounding), joint_tf + place(bounding+leaves)
// Also checks the staged bounding positions match bound_fk (correctness).
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "staged_place.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
using V1 = vamp::FloatVector<rake, 1>;

template <class R>
static void run(const char *name, const std::vector<Place> &leaf, const std::vector<Place> &bsph)
{
    constexpr std::size_t dim=R::dimension;
    std::mt19937 rng(1); std::uniform_real_distribution<float> u(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    std::vector<Block> blocks(4000);
    for(auto&b:blocks){ for(std::size_t j=0;j<dim;++j) b[j]=static_cast<DataV>(u(rng)); R::template scale_configuration_block<rake>(b); }

    // correctness: staged bounding vs bound_fk (max abs diff)
    {
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(blocks[0], bs);
        std::array<V1, 12*R::n_joint_tf> T; R::template joint_tf<rake>(blocks[0], T);
        float md=0; for(std::size_t i=0;i<bsph.size();++i){ V1 x,y,z; R::template place_sphere<rake>(T,bsph[i].slot,bsph[i].lx,bsph[i].ly,bsph[i].lz,x,y,z);
            md=std::max({md, std::fabs(x.to_array()[0]-bs.x[i].to_array()[0]), std::fabs(y.to_array()[0]-bs.y[i].to_array()[0]), std::fabs(z.to_array()[0]-bs.z[i].to_array()[0])}); }
        std::printf("%-6s n_joint_tf=%zu leaves=%zu bsph=%zu  staged-vs-bound_fk max_err=%.2e\n", name, R::n_joint_tf, leaf.size(), bsph.size(), md);
    }

    typename R::template BoundingSpheres<rake> bs; typename R::template Spheres<rake> sp;
    auto med=[&](auto fn){ std::vector<double> t; volatile double sk=0;
        for(int rp=0;rp<11;++rp){auto a=std::chrono::steady_clock::now();double v=fn();auto z=std::chrono::steady_clock::now();sk+=v;
            t.push_back(std::chrono::duration<double>(z-a).count()/blocks.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};

    auto fused_bound=[&]{ float a=0; for(auto&b:blocks){ R::template bound_fk<rake>(b,bs); a+=bs.x[0].to_array()[0]; } return a; };
    auto fused_leaf =[&]{ float a=0; for(auto&b:blocks){ R::template sphere_fk<rake>(b,sp); a+=sp.x[0].to_array()[0]; } return a; };
    auto staged_bound=[&]{ float a=0; std::array<V1,12*R::n_joint_tf> T;
        for(auto&b:blocks){ R::template joint_tf<rake>(b,T); V1 x,y,z;
            for(auto&p:bsph){ R::template place_sphere<rake>(T,p.slot,p.lx,p.ly,p.lz,x,y,z); a+=x.to_array()[0]; } } return a; };
    auto staged_full=[&]{ float a=0; std::array<V1,12*R::n_joint_tf> T;
        for(auto&b:blocks){ R::template joint_tf<rake>(b,T); V1 x,y,z;
            for(auto&p:bsph){ R::template place_sphere<rake>(T,p.slot,p.lx,p.ly,p.lz,x,y,z); a+=x.to_array()[0]; }
            for(auto&p:leaf){ R::template place_sphere<rake>(T,p.slot,p.lx,p.ly,p.lz,x,y,z); a+=x.to_array()[0]; } } return a; };
    // staged lazy proxy: bounding + place leaves for only ~15% of joints (one link) per block
    auto staged_lazy=[&]{ float a=0; std::array<V1,12*R::n_joint_tf> T;
        for(auto&b:blocks){ R::template joint_tf<rake>(b,T); V1 x,y,z;
            for(auto&p:bsph){ R::template place_sphere<rake>(T,p.slot,p.lx,p.ly,p.lz,x,y,z); a+=x.to_array()[0]; }
            std::size_t lim=leaf.size()/7; for(std::size_t i=0;i<lim;++i){ auto&p=leaf[i]; R::template place_sphere<rake>(T,p.slot,p.lx,p.ly,p.lz,x,y,z); a+=x.to_array()[0]; } } return a; };

    double fb=med(fused_bound), fl=med(fused_leaf), sb=med(staged_bound), sf=med(staged_full), sl=med(staged_lazy);
    std::printf("   bound_fk=%.0f  staged_bound=%.0f (%.2fx)  | sphere_fk=%.0f  staged_full=%.0f (%.2fx)  | staged_lazy(bound+1link)=%.0f\n",
                fb, sb, fb/sb, fl, sf, fl/sf, sl);
}

int main()
{
    run<vamp::robots::PandaE>("panda", panda_leaf, panda_bsph);
    run<vamp::robots::FetchE>("fetch", fetch_leaf, fetch_bsph);
    run<vamp::robots::BaxterE>("baxter", baxter_leaf, baxter_bsph);
    return 0;
}
