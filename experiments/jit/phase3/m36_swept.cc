// Swept broadphase PROOF. Per edge: bound_fk at 3 configs (t=0,0.5,1) -> per bounding-sphere
// swept sphere (center=midpoint, radius=max endpoint deviation + curvature remainder + max bs
// radius). env_clear[bs] = swept sphere clear of all obstacles; pair_clear[pi] = the pair's two
// swept spheres disjoint. Then fkcc_swept per rake skips certified links/pairs. Verify verdicts
// vs baseline fkcc on FREE and COLLIDING edges (rigor: swept must NEVER pass a colliding edge),
// and time. SAFETY inflates the curvature remainder until zero rigor violations.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "panda_e.hh"   // unrolled baseline (ships today): PandaE
#include "fetch_e.hh"   // FetchE
#include "panda_c.hh"   // compact + swept: PandaEC
#include "fetch_c.hh"   // FetchEC
#include "swept_aux.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
struct Obs { float x, y, z, r; };

template <class R>
static std::vector<Obs> shelf(std::mt19937 &rng, int nobs, vamp::collision::Environment<DataV> &envv)
{
    vamp::collision::Environment<float> ef; std::vector<Obs> obs;
    std::uniform_real_distribution<float> sx(0.45F,0.85F), sy(-0.35F,0.35F), sz(0.45F,1.0F), rad(0.02F,0.05F);
    for (int i=0;i<nobs;++i){ float x=sx(rng),y=sy(rng),z=sz(rng),rr=rad(rng);
        ef.spheres.emplace_back(vamp::collision::factory::sphere::array({x,y,z},rr)); obs.push_back({x,y,z,rr}); }
    ef.sort(); envv=vamp::collision::Environment<DataV>(ef); return obs;
}

template <class RB, class R>   // RB = unrolled baseline, R = compact+swept (same robot)
static void run(const char *name, float range, int nobs, const std::vector<std::array<int,2>> &pair_bs, float SAFETY)
{
    constexpr std::size_t dim=R::dimension, NBS=R::n_bounding_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake;
    std::size_t NP=pair_bs.size();
    std::mt19937 srng(0xBEEF), wrng(0x33);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;

    struct Edge { std::vector<Block> B; std::array<float,dim> s,v; float dt; bool free; };
    std::vector<Edge> edges; int attempts=0;
    auto base_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not RB::template fkcc<rake>(env,b)) return false; return true; };
    auto cbase_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };
    while(edges.size()<3000 && attempts<600000){ ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nr=0; for(std::size_t j=0;j<dim;++j){s[j]=tmp[j].to_array()[0];v[j]=nd(wrng);nr+=v[j]*v[j];}
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr; float dt=1.0f/(float)N;
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        bool fr=base_valid(B);
        if(fr || edges.size()%2==0) edges.push_back({std::move(B),s,v,dt,fr});
    }

    // swept: build masks from bound_fk at 3 configs (indices 0, N/2, N-1)
    auto masks=[&](const Edge&e, std::array<bool,NBS>&env_clear, std::vector<char>&pair_clear){
        std::size_t I[3]={0,N/2,N-1};
        Block b3;
        for(std::size_t j=0;j<dim;++j){ alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;
            for(int t=0;t<3;++t) ln[t]=e.s[j]+(float)I[t]*e.dt*e.v[j];
            for(std::size_t l=3;l<rake;++l) ln[l]=ln[0]; b3[j]=DataV(ln.data()); }
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b3, bs);
        std::array<std::array<float,3>,NBS> E; std::array<float,NBS> RAD;
        for(std::size_t k=0;k<NBS;++k){
            std::array<float,3> c0{bs.x[k].to_array()[0],bs.y[k].to_array()[0],bs.z[k].to_array()[0]};
            std::array<float,3> cm{bs.x[k].to_array()[1],bs.y[k].to_array()[1],bs.z[k].to_array()[1]};
            std::array<float,3> c1{bs.x[k].to_array()[2],bs.y[k].to_array()[2],bs.z[k].to_array()[2]};
            float r0=bs.r[k].to_array()[0],rm=bs.r[k].to_array()[1],r1=bs.r[k].to_array()[2];
            auto d=[&](std::array<float,3>a,std::array<float,3>b){return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));};
            std::array<float,3> chord{(c0[0]+c1[0])/2,(c0[1]+c1[1])/2,(c0[2]+c1[2])/2};
            float sag=d(chord,cm)*SAFETY;                       // curvature remainder
            float rho=std::max(d(c0,cm),d(c1,cm))+sag;
            float Rr=std::max({r0,rm,r1})+sag;
            E[k]=cm; RAD[k]=rho+Rr;
        }
        for(std::size_t k=0;k<NBS;++k){ float mn=1e9f; for(auto&o:obs){float dx=E[k][0]-o.x,dy=E[k][1]-o.y,dz=E[k][2]-o.z;mn=std::min(mn,std::sqrt(dx*dx+dy*dy+dz*dz)-o.r);} env_clear[k]=(mn>RAD[k]); }
        pair_clear.assign(NP,0);
        for(std::size_t pi=0;pi<NP;++pi){ int a=pair_bs[pi][0],b=pair_bs[pi][1];
            float dx=E[a][0]-E[b][0],dy=E[a][1]-E[b][1],dz=E[a][2]-E[b][2];
            pair_clear[pi]=(std::sqrt(dx*dx+dy*dy+dz*dz) > RAD[a]+RAD[b]); }
    };
    auto swept_valid=[&](const Edge&e){ std::array<bool,NBS> env_clear; std::vector<char> pc;
        masks(e,env_clear,pc);
        std::array<bool,R::n_self_pairs> pcarr{}; for(std::size_t i=0;i<NP;++i) pcarr[i]=pc[i];
        for(auto&b:e.B) if(not R::template fkcc_swept<rake>(env,b,env_clear,pcarr)) return false; return true; };

    // rigor: swept must match baseline exactly; unsafe = swept FREE but baseline COLLIDE
    std::size_t mm=0, unsafe=0, nfree=0;
    for(auto&e:edges){ bool bv=base_valid(e.B), sv=swept_valid(e); nfree+=e.free; if(bv!=sv){++mm; if(sv&&!bv)++unsafe;} }

    auto med=[&](auto fn,int mode){ std::vector<double> t; volatile std::uint64_t sk=0; std::vector<Edge*> sub;
        for(auto&e:edges) if(mode==0||(mode==1&&e.free)||(mode==2&&!e.free)) sub.push_back(&e); if(sub.empty())return 0.0;
        for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(auto*e:sub)ac+=fn(*e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/sub.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    auto Uf=[&](const Edge&e){return base_valid(e.B)?1U:0U;};    // unrolled baseline (ships)
    auto Cf=[&](const Edge&e){return cbase_valid(e.B)?1U:0U;};   // compact baseline (no mask)
    auto Sf=[&](const Edge&e){return swept_valid(e)?1U:0U;};     // compact + swept mask
    std::printf("%-6s n=%zu NBS=%zu pairs=%zu free=%zu/%zu SAFETY=%.1f  mismatch=%zu unsafe=%zu\n",
                name,n,NBS,NP,nfree,edges.size(),SAFETY,mm,unsafe);
    for(int m=0;m<3;++m){ const char*lbl=m==0?"all":m==1?"free":"colliding";
        double U=med(Uf,m),C=med(Cf,m),S=med(Sf,m);
        std::printf("   %-9s: unrolled=%.0f compact=%.0f swept=%.0f ns | swept-vs-unrolled=%.2fx  swept-vs-compact=%.2fx\n",
                    lbl,U,C,S,U/S,C/S); }
}

int main()
{
    run<vamp::robots::PandaE, vamp::robots::PandaEC>("panda",1.25f,40,panda_pair_bs,2.0f);
    run<vamp::robots::FetchE, vamp::robots::FetchEC>("fetch",1.0f,40,fetch_pair_bs,2.0f);
    return 0;
}
