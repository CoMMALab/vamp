// Headroom for SCENE SPECIALIZATION on real MBM (before building the JIT machinery), per the
// design doc's Component 1. Two measurements:
//  (1) Environment-check share of the kernel: FK-only (sphere_fk) vs FK+self (empty env) vs
//      FK+self+env (scene). Scene specialization only speeds the ENV part -> its ceiling.
//  (2) Compile-time per-bounding-sphere broadphase pruning factor: each bounding sphere is rigid
//      to a link with a bounded reachable region; obstacles outside (reach-ball + radii) can be
//      dropped at specialization time. Report obstacles-checked-per-sphere before/after.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "ur5_e.hh"
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "mbm_envs.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

template <class R>
static vamp::collision::Environment<DataV> build_env(const MbmEnv &me)
{
    vamp::collision::Environment<float> ef;
    for (const auto &p : me.prims) {
        if (p.kind == 0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px, p.py, p.pz, p.a));
        else if (p.kind == 1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b));
        else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b, p.c));
    }
    ef.sort();
    return vamp::collision::Environment<DataV>(ef);
}

// conservative bounding radius of an MBM primitive about its center
static float prim_radius(const MbmPrim &p)
{
    if (p.kind == 0) return p.a;                                   // sphere
    if (p.kind == 1) return std::sqrt(p.a * p.a + (p.b * 0.5f) * (p.b * 0.5f));  // cylinder r,len
    return std::sqrt(p.a * p.a + p.b * p.b + p.c * p.c);          // box half-extents
}

template <class R>
static void run(const char *name, float range, const std::vector<MbmEnv> &mbm)
{
    constexpr std::size_t dim = R::dimension, NBS = R::n_bounding_spheres;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake));
    std::mt19937 rng(7); std::uniform_real_distribution<float> u(0.f, 1.f); std::normal_distribution<float> nd(0.f, 1.f);
    using Block = typename R::template ConfigurationBlock<rake>;

    // (2a) per-bounding-sphere reachable region (robot-intrinsic): sample configs, bound_fk,
    // track center bbox + max radius.
    std::array<float, NBS> cmin_x, cmin_y, cmin_z, cmax_x, cmax_y, cmax_z, bmaxr;
    for (std::size_t k = 0; k < NBS; ++k) { cmin_x[k]=cmin_y[k]=cmin_z[k]=1e9f; cmax_x[k]=cmax_y[k]=cmax_z[k]=-1e9f; bmaxr[k]=0; }
    { typename R::template BoundingSpheres<rake> bs;
      for (int it = 0; it < 4000; ++it) { Block c; for (std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(rng)); R::template scale_configuration_block<rake>(c);
        R::template bound_fk<rake>(c, bs);
        for (std::size_t k=0;k<NBS;++k) for (std::size_t l=0;l<rake;++l) {
            float x=bs.x[k].to_array()[l], y=bs.y[k].to_array()[l], z=bs.z[k].to_array()[l], r=bs.r[k].to_array()[l];
            cmin_x[k]=std::min(cmin_x[k],x);cmin_y[k]=std::min(cmin_y[k],y);cmin_z[k]=std::min(cmin_z[k],z);
            cmax_x[k]=std::max(cmax_x[k],x);cmax_y[k]=std::max(cmax_y[k],y);cmax_z[k]=std::max(cmax_z[k],z); bmaxr[k]=std::max(bmaxr[k],r);} } }
    std::array<std::array<float,3>,NBS> rc; std::array<float,NBS> rr;   // reach center + radius per bs
    for (std::size_t k=0;k<NBS;++k){ rc[k]={0.5f*(cmin_x[k]+cmax_x[k]),0.5f*(cmin_y[k]+cmax_y[k]),0.5f*(cmin_z[k]+cmax_z[k])};
        rr[k]=0.5f*std::sqrt((cmax_x[k]-cmin_x[k])*(cmax_x[k]-cmin_x[k])+(cmax_y[k]-cmin_y[k])*(cmax_y[k]-cmin_y[k])+(cmax_z[k]-cmin_z[k])*(cmax_z[k]-cmin_z[k]))+bmaxr[k]; }

    // (2b) pruning factor over MBM scenes: obstacles-per-sphere kept vs total
    double sum_total=0, sum_kept=0; std::size_t scenes=0;
    for (const auto &me : mbm) { ++scenes; std::size_t nobs=me.prims.size();
        for (std::size_t k=0;k<NBS;++k){ sum_total += nobs;
            for (const auto &p : me.prims) { float dx=rc[k][0]-p.px,dy=rc[k][1]-p.py,dz=rc[k][2]-p.pz;
                if (std::sqrt(dx*dx+dy*dy+dz*dz) < rr[k] + prim_radius(p)) sum_kept += 1; } } }
    double prune_frac = sum_total>0 ? sum_kept/sum_total : 0;

    // (1) env-check share of the kernel over realistic edges on real scenes
    std::vector<vamp::collision::Environment<DataV>> envs; for (auto &me:mbm) envs.push_back(build_env<R>(me));
    vamp::collision::Environment<DataV> empty;
    std::vector<std::pair<int,std::vector<Block>>> free_edges;  // (env idx, blocks)
    std::mt19937 wr(0x33);
    for (std::size_t ei=0; ei<envs.size() && free_edges.size()<envs.size()*40; ++ei) {
        int got=0,att=0;
        while (got<40 && att<20000) { ++att; Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wr)); R::template scale_configuration_block<rake>(tmp);
            std::array<float,dim> s,v; float nr=0; for(std::size_t j=0;j<dim;++j){s[j]=tmp[j].to_array()[0];v[j]=nd(wr);nr+=v[j]*v[j];}
            nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr; float dt=1.0f/(float)(n*rake);
            std::vector<Block> B(n); bool free=true;
            for(std::size_t r=0;r<n;++r){ for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;
                for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
                if(not R::template fkcc<rake>(envs[ei],B[r])){free=false;break;} }
            if(free){ free_edges.push_back({(int)ei,std::move(B)}); ++got; } }
    }
    typename R::template Spheres<rake> sp;
    auto med=[&](auto fn){ std::vector<double> t; volatile double sk=0;
        for(int rp=0;rp<9;++rp){auto a=std::chrono::steady_clock::now();double ac=0;for(auto&e:free_edges)ac+=fn(e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/free_edges.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    double fk = med([&](auto&e){ float a=0; for(auto&b:e.second){ R::template sphere_fk<rake>(b,sp); a+=sp.x[0].to_array()[0];} return a; });
    double emp= med([&](auto&e){ std::uint32_t a=0; for(auto&b:e.second) a+=R::template fkcc<rake>(empty,b)?1:0; return (double)a; });
    double full=med([&](auto&e){ std::uint32_t a=0; for(auto&b:e.second) a+=R::template fkcc<rake>(envs[e.first],b)?1:0; return (double)a; });
    double self=emp-fk, env=full-emp;

    std::printf("%-6s n=%zu NBS=%zu scenes=%zu edges=%zu  | FK=%.0f self=%.0f ENV=%.0f (full=%.0f ns)  env_share=%.0f%%  scene-spec ceiling=%.2fx\n",
                name,n,NBS,scenes,free_edges.size(), fk,self,env,full, 100.0*env/full, full/(full-env));
    std::printf("        compile-time broadphase: obstacles kept/sphere = %.0f%% (prune factor %.2fx); mean obstacles/scene=%.1f\n",
                100.0*prune_frac, prune_frac>0?1.0/prune_frac:0.0, sum_total/(scenes*NBS));
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, ur5_envs);
    run<vamp::robots::PandaE>("panda", 1.25f, panda_envs);
    run<vamp::robots::FetchE>("fetch", 1.0f, fetch_envs);
    run<vamp::robots::BaxterE>("baxter", 0.5f, baxter_envs);
    return 0;
}
