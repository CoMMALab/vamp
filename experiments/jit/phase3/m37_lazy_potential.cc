// Lazy-leaf-FK ceiling: on free edges, how often would a lazy kernel need leaves? Per rake,
// after the swept mask, an ACTIVE (non-certified) link needs its leaves only if its bounding
// sphere actually hits the env this rake; a pair needs leaves only if its two bounding spheres
// overlap. Count rakes needing NO leaves -> leaf-FK reclaimable. Uses bound_fk (bounding spheres
// per rake) + brute-force obstacle/pair tests; masks from bound_fk at 3 configs (as m36).
#include <array>
#include <cmath>
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
static void run(const char *name, float range, int nobs, const std::vector<std::array<int,2>> &pair_bs, float SAFETY)
{
    constexpr std::size_t dim=R::dimension, NBS=R::n_bounding_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake, NP=pair_bs.size();
    std::mt19937 srng(0xBEEF), wrng(0x33);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto base_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    // hit test: bounding sphere (center c, radius rr) vs any obstacle, brute force
    auto hit_env=[&](float cx,float cy,float cz,float rr){ for(auto&o:obs){float dx=cx-o.x,dy=cy-o.y,dz=cz-o.z; if(std::sqrt(dx*dx+dy*dy+dz*dz)<rr+o.r) return true;} return false; };

    std::size_t got=0; int attempts=0;
    std::size_t rakes_total=0, rakes_need_leaves=0, edges_all_free_rakes=0;
    while(got<2000 && attempts<600000){ ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nr=0; for(std::size_t j=0;j<dim;++j){s[j]=tmp[j].to_array()[0];v[j]=nd(wrng);nr+=v[j]*v[j];}
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr; float dt=1.0f/(float)N;
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        if(!base_valid(B)) continue;
        ++got;

        // swept masks from bound_fk at 3 configs
        std::size_t I[3]={0,N/2,N-1}; Block b3;
        for(std::size_t j=0;j<dim;++j){ alignas(vamp::FloatVectorAlignment) std::array<float,rake> ln;
            for(int t=0;t<3;++t) ln[t]=s[j]+(float)I[t]*dt*v[j]; for(std::size_t l=3;l<rake;++l) ln[l]=ln[0]; b3[j]=DataV(ln.data()); }
        typename R::template BoundingSpheres<rake> bs3; R::template bound_fk<rake>(b3,bs3);
        std::array<std::array<float,3>,NBS> E; std::array<float,NBS> RAD; std::array<bool,NBS> env_clear;
        for(std::size_t k=0;k<NBS;++k){ auto g=[&](int t){return std::array<float,3>{bs3.x[k].to_array()[t],bs3.y[k].to_array()[t],bs3.z[k].to_array()[t]};};
            auto c0=g(0),cm=g(1),c1=g(2); float r0=bs3.r[k].to_array()[0],rm=bs3.r[k].to_array()[1],r1=bs3.r[k].to_array()[2];
            auto d=[&](std::array<float,3>a,std::array<float,3>b){return std::sqrt((a[0]-b[0])*(a[0]-b[0])+(a[1]-b[1])*(a[1]-b[1])+(a[2]-b[2])*(a[2]-b[2]));};
            std::array<float,3> chord{(c0[0]+c1[0])/2,(c0[1]+c1[1])/2,(c0[2]+c1[2])/2}; float sag=d(chord,cm)*SAFETY;
            float rho=std::max(d(c0,cm),d(c1,cm))+sag, Rr=std::max({r0,rm,r1})+sag; E[k]=cm; RAD[k]=rho+Rr; }
        for(std::size_t k=0;k<NBS;++k){ float mn=1e9f; for(auto&o:obs){float dx=E[k][0]-o.x,dy=E[k][1]-o.y,dz=E[k][2]-o.z;mn=std::min(mn,std::sqrt(dx*dx+dy*dy+dz*dz)-o.r);} env_clear[k]=(mn>RAD[k]); }
        std::vector<char> pair_clear(NP,0);
        for(std::size_t pi=0;pi<NP;++pi){int a=pair_bs[pi][0],b=pair_bs[pi][1];float dx=E[a][0]-E[b][0],dy=E[a][1]-E[b][1],dz=E[a][2]-E[b][2];pair_clear[pi]=(std::sqrt(dx*dx+dy*dy+dz*dz)>RAD[a]+RAD[b]);}

        // per rake: does a lazy kernel need leaves? (any active link bs hits env, or any active pair bs overlap)
        bool edge_all_clear=true;
        for(std::size_t r=0;r<n;++r){
            typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(B[r],bs);
            bool need=false;
            for(std::size_t k=0;k<NBS && !need;++k){ if(env_clear[k]) continue;
                for(std::size_t l=0;l<rake;++l) if(hit_env(bs.x[k].to_array()[l],bs.y[k].to_array()[l],bs.z[k].to_array()[l],bs.r[k].to_array()[l])){need=true;break;} }
            for(std::size_t pi=0;pi<NP && !need;++pi){ if(pair_clear[pi]) continue; int a=pair_bs[pi][0],b=pair_bs[pi][1];
                for(std::size_t l=0;l<rake;++l){ float dx=bs.x[a].to_array()[l]-bs.x[b].to_array()[l],dy=bs.y[a].to_array()[l]-bs.y[b].to_array()[l],dz=bs.z[a].to_array()[l]-bs.z[b].to_array()[l];
                    if(std::sqrt(dx*dx+dy*dy+dz*dz)<bs.r[a].to_array()[l]+bs.r[b].to_array()[l]){need=true;break;} } }
            ++rakes_total; if(need){++rakes_need_leaves; edge_all_clear=false;}
        }
        if(edge_all_clear) ++edges_all_free_rakes;
    }
    double avoid=100.0*(1.0-(double)rakes_need_leaves/rakes_total);
    std::printf("%-6s n=%zu free_edges=%zu  rakes needing leaves=%.1f%%  leaf-FK avoided=%.1f%%  edges w/ 0 leaf rakes=%.1f%%\n",
                name,n,got,100.0*rakes_need_leaves/rakes_total,avoid,100.0*edges_all_free_rakes/got);
}

int main()
{
    run<vamp::robots::PandaE>("panda",1.25f,40,panda_pair_bs,2.0f);
    run<vamp::robots::FetchE>("fetch",1.0f,40,fetch_pair_bs,2.0f);
    run<vamp::robots::BaxterE>("baxter",0.5f,40,baxter_pair_bs,2.0f);
    return 0;
}
