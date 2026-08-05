// Node-level caching prototype. In RRTC an edge connects two EXISTING tree nodes, so both
// endpoints' bound_fk are cacheable. Using endpoints-only swept spheres (m34: 2 samples capture
// the extent) + a conservative remainder, the per-edge setup needs ZERO FK -- both endpoint
// bounding spheres come from the node cache. Compare per-edge cost:
//   baseline      : fkcc per rake
//   swept (m36)   : recompute 3-config bound_fk per edge + masks + fkcc_swept
//   swept+cache   : endpoint bound_fk precomputed (cache hit) -> 0 FK/edge + masks + fkcc_swept
// Endpoints-only swept sphere + a per-link sagitta remainder calibrated offline (C[k] = max
// bulge over sampled edges); REM = safety multiplier on C[k]. Rigor checked on free+colliding.
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
#include "swept_aux.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
struct Obs { float x,y,z,r; };
struct BS { float x,y,z,r; };   // one bounding sphere at one config

template <class R>
static std::vector<Obs> shelf(std::mt19937 &rng, int nobs, vamp::collision::Environment<DataV> &envv)
{
    vamp::collision::Environment<float> ef; std::vector<Obs> obs;
    std::uniform_real_distribution<float> sx(0.45F,0.85F), sy(-0.35F,0.35F), sz(0.45F,1.0F), rad(0.02F,0.05F);
    for (int i=0;i<nobs;++i){ float x=sx(rng),y=sy(rng),z=sz(rng),rr=rad(rng);
        ef.spheres.emplace_back(vamp::collision::factory::sphere::array({x,y,z},rr)); obs.push_back({x,y,z,rr}); }
    ef.sort(); envv=vamp::collision::Environment<DataV>(ef); return obs;
}

template <class R>
static void run(const char *name, float range, int nobs, const std::vector<std::array<int,2>> &pair_bs, float REM)
{
    constexpr std::size_t dim=R::dimension, NBS=R::n_bounding_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake, NP=pair_bs.size();
    std::mt19937 srng(0xBEEF), wrng(0x33);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto base_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    // bound_fk at a single config (broadcast into all lanes, read lane 0)
    auto node_bound=[&](const std::array<float,dim>&q){ Block b;
        for(std::size_t j=0;j<dim;++j) b[j]=DataV::fill(q[j]);
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b, bs);
        std::array<BS,NBS> out; for(std::size_t k=0;k<NBS;++k) out[k]={bs.x[k].to_array()[0],bs.y[k].to_array()[0],bs.z[k].to_array()[0],bs.r[k].to_array()[0]};
        return out; };

    // Calibrate a per-bounding-sphere sagitta constant C[k] = max over sampled edges of the
    // bulge |c_mid - (c0+c1)/2|. Precomputed robot+range constant (needs midpoints ONLY offline);
    // at runtime the remainder is SAFETY*C[k] -- tight, no per-edge midpoint FK.
    std::array<float,NBS> C{}; { std::mt19937 crng(0x5151);
        for(int it=0;it<4000;++it){ Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(crng));
            R::template scale_configuration_block<rake>(tmp);
            std::array<float,dim> a,vv,b,mid; float nr=0; for(std::size_t j=0;j<dim;++j){a[j]=tmp[j].to_array()[0];vv[j]=nd(crng);nr+=vv[j]*vv[j];}
            nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j){vv[j]*=range/nr;b[j]=a[j]+vv[j];mid[j]=a[j]+0.5f*vv[j];}
            auto ca=node_bound(a),cb=node_bound(b),cm=node_bound(mid);
            for(std::size_t k=0;k<NBS;++k){ float mx=(ca[k].x+cb[k].x)*0.5f,my=(ca[k].y+cb[k].y)*0.5f,mz=(ca[k].z+cb[k].z)*0.5f;
                float sag=std::sqrt((cm[k].x-mx)*(cm[k].x-mx)+(cm[k].y-my)*(cm[k].y-my)+(cm[k].z-mz)*(cm[k].z-mz)); C[k]=std::max(C[k],sag); } } }

    struct Edge { std::vector<Block> B; std::array<BS,NBS> ca, cb; bool free; };
    std::vector<Edge> edges; int attempts=0;
    while(edges.size()<3000 && attempts<600000){ ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> a,v,b; float nr=0; for(std::size_t j=0;j<dim;++j){a[j]=tmp[j].to_array()[0];v[j]=nd(wrng);nr+=v[j]*v[j];}
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j){ v[j]*=range/nr; b[j]=a[j]+v[j]; }
        float dt=1.0f/(float)N;
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=a[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        bool fr=base_valid(B);
        if(fr || edges.size()%2==0) edges.push_back({std::move(B), node_bound(a), node_bound(b), fr});   // endpoints cached
    }

    // masks from two cached endpoint bounding spheres (endpoints-only + proportional remainder)
    auto cache_masks=[&](const Edge&e, std::array<bool,NBS>&env_clear, std::array<bool,R::n_self_pairs>&pair_clear){
        std::array<std::array<float,3>,NBS> E; std::array<float,NBS> RAD;
        for(std::size_t k=0;k<NBS;++k){ auto&A=e.ca[k]; auto&Bb=e.cb[k];
            float ex=(A.x+Bb.x)*0.5f,ey=(A.y+Bb.y)*0.5f,ez=(A.z+Bb.z)*0.5f;
            float half=0.5f*std::sqrt((A.x-Bb.x)*(A.x-Bb.x)+(A.y-Bb.y)*(A.y-Bb.y)+(A.z-Bb.z)*(A.z-Bb.z));
            float rem=REM*C[k];                                  // calibrated sagitta remainder (tight)
            E[k]={ex,ey,ez}; RAD[k]=half+std::max(A.r,Bb.r)+rem; }
        for(std::size_t k=0;k<NBS;++k){ float mn=1e9f; for(auto&o:obs){float dx=E[k][0]-o.x,dy=E[k][1]-o.y,dz=E[k][2]-o.z;mn=std::min(mn,std::sqrt(dx*dx+dy*dy+dz*dz)-o.r);} env_clear[k]=(mn>RAD[k]); }
        for(std::size_t pi=0;pi<NP;++pi){ int a=pair_bs[pi][0],b=pair_bs[pi][1]; float dx=E[a][0]-E[b][0],dy=E[a][1]-E[b][1],dz=E[a][2]-E[b][2];
            pair_clear[pi]=(std::sqrt(dx*dx+dy*dy+dz*dz) > RAD[a]+RAD[b]); }
    };
    auto cached_valid=[&](const Edge&e){ std::array<bool,NBS> ec; std::array<bool,R::n_self_pairs> pc{}; cache_masks(e,ec,pc);
        for(auto&b:e.B) if(not R::template fkcc_swept<rake>(env,b,ec,pc)) return false; return true; };

    // rigor
    std::size_t mm=0, unsafe=0, nfree=0;
    for(auto&e:edges){ bool bv=base_valid(e.B), cv=cached_valid(e); nfree+=e.free; if(bv!=cv){++mm; if(cv&&!bv)++unsafe;} }

    auto med=[&](auto fn,int mode){ std::vector<double> t; volatile std::uint64_t sk=0; std::vector<Edge*> sub;
        for(auto&e:edges) if(mode==0||(mode==1&&e.free)||(mode==2&&!e.free)) sub.push_back(&e); if(sub.empty())return 0.0;
        for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(auto*e:sub)ac+=fn(*e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/sub.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    auto Uf=[&](const Edge&e){return base_valid(e.B)?1U:0U;};
    auto Cf=[&](const Edge&e){return cached_valid(e)?1U:0U;};
    std::printf("%-6s n=%zu NBS=%zu pairs=%zu free=%zu/%zu REM=%.1f  mismatch=%zu unsafe=%zu\n",
                name,n,NBS,NP,nfree,edges.size(),REM,mm,unsafe);
    for(int m=0;m<3;++m){ const char*lbl=m==0?"all":m==1?"free":"colliding";
        double U=med(Uf,m),C=med(Cf,m);
        std::printf("   %-9s: fkcc=%.0f  swept+cache=%.0f ns  %.2fx\n", lbl,U,C,U/C); }
}

int main()
{
    run<vamp::robots::PandaE>("panda",1.25f,40,panda_pair_bs,1.5f);
    run<vamp::robots::FetchE>("fetch",1.0f,40,fetch_pair_bs,1.5f);
    run<vamp::robots::BaxterE>("baxter",0.5f,40,baxter_pair_bs,1.5f);
    return 0;
}
