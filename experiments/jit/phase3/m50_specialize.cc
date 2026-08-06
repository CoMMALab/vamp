// Primitive scene specialization on MBM: the compile-time per-sphere broadphase (Component 1.3).
// Each robot bounding sphere is rigid to a link with a bounded reachable region; at scene-spec
// time we keep only the obstacles that region can reach. Measured here as a per-link PRUNED
// sub-environment vs the full env -- the env-check speedup this prune buys (the novel, AOT-
// impossible part), on top of VAMP's already-sorted+culled generic query. (const-fold of obstacle
// fields would add more; measured separately.) Verdicts checked identical to the full env.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include "ur5_e.hh"
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "mbm_envs.hh"
#include "link_groups.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
struct BS { float x,y,z,r; };

static float prim_radius(const MbmPrim &p)
{
    if (p.kind==0) return p.a;
    if (p.kind==1) return std::sqrt(p.a*p.a+(p.b*0.5f)*(p.b*0.5f));
    return std::sqrt(p.a*p.a+p.b*p.b+p.c*p.c);
}
static void add_prim(vamp::collision::Environment<float>&ef, const MbmPrim&p)
{
    if (p.kind==0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px,p.py,p.pz,p.a));
    else if (p.kind==1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px,p.py,p.pz,p.ex,p.ey,p.ez,p.a,p.b));
    else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px,p.py,p.pz,p.ex,p.ey,p.ez,p.a,p.b,p.c));
}

template <class R>
static void run(const char *name, float range, const std::vector<MbmEnv> &mbm, const std::vector<std::vector<int>> &links)
{
    constexpr std::size_t dim=R::dimension, NBS=R::n_bounding_spheres, NS=R::n_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake));
    std::mt19937 rng(7); std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;

    // per-bounding-sphere reachable region (center bbox + max radius) -- robot-intrinsic
    std::array<float,NBS> rcx,rcy,rcz,rr,mnx,mny,mnz,mxx,mxy,mxz,mr;
    for(std::size_t k=0;k<NBS;++k){mnx[k]=mny[k]=mnz[k]=1e9f;mxx[k]=mxy[k]=mxz[k]=-1e9f;mr[k]=0;}
    { typename R::template BoundingSpheres<rake> bs;
      for(int it=0;it<4000;++it){ Block c; for(std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(rng)); R::template scale_configuration_block<rake>(c);
        R::template bound_fk<rake>(c,bs);
        for(std::size_t k=0;k<NBS;++k) for(std::size_t l=0;l<rake;++l){ float x=bs.x[k].to_array()[l],y=bs.y[k].to_array()[l],z=bs.z[k].to_array()[l],r=bs.r[k].to_array()[l];
            mnx[k]=std::min(mnx[k],x);mny[k]=std::min(mny[k],y);mnz[k]=std::min(mnz[k],z);mxx[k]=std::max(mxx[k],x);mxy[k]=std::max(mxy[k],y);mxz[k]=std::max(mxz[k],z);mr[k]=std::max(mr[k],r);} } }
    for(std::size_t k=0;k<NBS;++k){ rcx[k]=0.5f*(mnx[k]+mxx[k]);rcy[k]=0.5f*(mny[k]+mxy[k]);rcz[k]=0.5f*(mnz[k]+mxz[k]);
        rr[k]=0.5f*std::sqrt((mxx[k]-mnx[k])*(mxx[k]-mnx[k])+(mxy[k]-mny[k])*(mxy[k]-mny[k])+(mxz[k]-mnz[k])*(mxz[k]-mnz[k]))+mr[k]; }

    // full env + per-link (=per-bounding-sphere) pruned env per scene
    std::vector<vamp::collision::Environment<DataV>> full;
    std::vector<std::array<vamp::collision::Environment<DataV>,NBS>> pruned(mbm.size());
    double kept=0,tot=0;
    for(std::size_t s=0;s<mbm.size();++s){ vamp::collision::Environment<float> ef; for(auto&p:mbm[s].prims) add_prim(ef,p); ef.sort(); full.push_back(vamp::collision::Environment<DataV>(ef));
        for(std::size_t k=0;k<NBS;++k){ vamp::collision::Environment<float> pf;
            for(auto&p:mbm[s].prims){ float dx=rcx[k]-p.px,dy=rcy[k]-p.py,dz=rcz[k]-p.pz; if(std::sqrt(dx*dx+dy*dy+dz*dz)<rr[k]+prim_radius(p)){ add_prim(pf,p); kept++; } tot++; }
            pf.sort(); pruned[s][k]=vamp::collision::Environment<DataV>(pf); } }

    // free MBM configs (blocks)
    std::vector<std::pair<int,Block>> cfgs; std::mt19937 wr(0x33);
    for(std::size_t s=0;s<full.size()&&cfgs.size()<full.size()*60;++s){ int got=0,att=0;
        while(got<60&&att<20000){ ++att; Block b; for(std::size_t j=0;j<dim;++j) b[j]=static_cast<DataV>(u(wr)); R::template scale_configuration_block<rake>(b);
            if(R::template fkcc<rake>(full[s],b)){ cfgs.push_back({(int)s,b}); ++got; } } }

    // precompute sphere positions per config
    struct Pos{ std::array<BS,NBS> bs; std::array<std::array<float,4>,NS> lf; int s; };
    std::vector<Pos> pos(cfgs.size());
    { typename R::template BoundingSpheres<rake> b; typename R::template Spheres<rake> l;
      for(std::size_t i=0;i<cfgs.size();++i){ R::template bound_fk<rake>(cfgs[i].second,b); R::template sphere_fk<rake>(cfgs[i].second,l); pos[i].s=cfgs[i].first;
        for(std::size_t k=0;k<NBS;++k) pos[i].bs[k]={b.x[k].to_array()[0],b.y[k].to_array()[0],b.z[k].to_array()[0],b.r[k].to_array()[0]};
        for(std::size_t k=0;k<NS;++k) pos[i].lf[k]={l.x[k].to_array()[0],l.y[k].to_array()[0],l.z[k].to_array()[0],l.r[k].to_array()[0]}; } }

    auto q=[&](const vamp::collision::Environment<DataV>&e, const BS&s){ return vamp::sphere_environment_in_collision(e,DataV::fill(s.x),DataV::fill(s.y),DataV::fill(s.z),DataV::fill(s.r)); };
    // env-vs-robot check (mirrors fkcc's structure): bounding sphere broadphase -> leaf narrowphase
    auto gen=[&](const Pos&P){ for(std::size_t k=0;k<NBS;++k){ if(q(full[P.s],P.bs[k])){ for(int lf:links[k]){ BS L{P.lf[lf][0],P.lf[lf][1],P.lf[lf][2],P.lf[lf][3]}; if(q(full[P.s],L)) return true; } } } return false; };
    auto spc=[&](const Pos&P){ for(std::size_t k=0;k<NBS;++k){ auto&pe=pruned[P.s][k]; if(q(pe,P.bs[k])){ for(int lf:links[k]){ BS L{P.lf[lf][0],P.lf[lf][1],P.lf[lf][2],P.lf[lf][3]}; if(q(pe,L)) return true; } } } return false; };

    std::size_t mm=0; for(auto&P:pos) if(gen(P)!=spc(P)) ++mm;
    auto med=[&](auto fn){ std::vector<double> t; volatile std::uint64_t sk=0;
        for(int rp=0;rp<9;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(auto&P:pos)ac+=fn(P)?1:0;auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/pos.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    double g=med(gen), s=med(spc);
    std::printf("%-6s NBS=%zu obs/scene=%.1f  kept/sphere=%.0f%% (prune %.2fx)  cfgs=%zu mism=%zu | env-check: full=%.0f  pruned=%.0f ns  speedup=%.2fx\n",
                name, NBS, tot/(mbm.size()*NBS), 100.0*kept/tot, tot/kept, pos.size(), mm, g, s, g/s);
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, ur5_envs, ur5_links);
    run<vamp::robots::PandaE>("panda", 1.25f, panda_envs, panda_links);
    run<vamp::robots::FetchE>("fetch", 1.0f, fetch_envs, fetch_links);
    run<vamp::robots::BaxterE>("baxter", 0.5f, baxter_envs, baxter_links);
    return 0;
}
