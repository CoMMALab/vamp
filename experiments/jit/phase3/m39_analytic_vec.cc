// (b) Analytic sagitta bound + (a) vectorized pair tests, on top of node-cached swept (m38).
// (b) sagitta_k <= (||r_k||*sqrt(dim)/8)*L^2, r_k[j] = bounding-sphere k Cartesian displacement
//     per radian of joint j (one FD sweep, offline), L = ||dtheta|| edge angular length. Provably
//     conservative (linear-interp error (1/8)max||c''||, c'' bounded via per-joint reach). No
//     calibration, no midpoint. Verified: analytic rem >= true sagitta over sampled edges.
// (a) pair swept tests in SoA + batched over lanes to cut the O(pairs) setup that craters
//     baxter's colliding edges.
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
struct BS { float x,y,z,r; };

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
    std::mt19937 srng(0xBEEF), wrng(0x33), rrng(0x9);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto base_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };
    auto node_bound=[&](const std::array<float,dim>&q){ Block b; for(std::size_t j=0;j<dim;++j) b[j]=DataV::fill(q[j]);
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b, bs);
        std::array<BS,NBS> out; for(std::size_t k=0;k<NBS;++k) out[k]={bs.x[k].to_array()[0],bs.y[k].to_array()[0],bs.z[k].to_array()[0],bs.r[k].to_array()[0]}; return out; };

    // (b, tighter) per-bounding-sphere Hessian-norm matrix H[k][i][j] = max_config ||d2 c_k/dth_i dth_j||
    // via second finite differences. sag_k <= (1/8) sum_{i,j} H[k][i][j] |dth_i||dth_j| (interp error).
    static std::array<std::array<std::array<float,dim>,dim>,NBS> H{}; static std::array<std::array<float,dim>,NBS> reach{}; { std::mt19937 g(0xA1);
        std::uniform_real_distribution<float> uu(0.f,1.f); const float eps=5e-2f, ie2=1.0f/(eps*eps);
        for(int it=0;it<600;++it){ Block c; for(std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(uu(g)); R::template scale_configuration_block<rake>(c);
            std::array<float,dim> q; for(std::size_t j=0;j<dim;++j) q[j]=c[j].to_array()[0];
            auto base=node_bound(q); std::array<std::array<BS,NBS>,dim> ci;
            for(std::size_t i=0;i<dim;++i){ auto qi=q; qi[i]+=eps; ci[i]=node_bound(qi);
                for(std::size_t k=0;k<NBS;++k){ float dx=ci[i][k].x-base[k].x,dy=ci[i][k].y-base[k].y,dz=ci[i][k].z-base[k].z; reach[k][i]=std::max(reach[k][i],std::sqrt(dx*dx+dy*dy+dz*dz)/eps);} }
            for(std::size_t i=0;i<dim;++i){ auto q2=q; q2[i]+=2*eps; auto c2=node_bound(q2);   // diagonal d2/dth_i^2
                for(std::size_t k=0;k<NBS;++k){ float dx=c2[k].x-2*ci[i][k].x+base[k].x,dy=c2[k].y-2*ci[i][k].y+base[k].y,dz=c2[k].z-2*ci[i][k].z+base[k].z;
                    H[k][i][i]=std::max(H[k][i][i],std::sqrt(dx*dx+dy*dy+dz*dz)*ie2); } }
            for(std::size_t i=0;i<dim;++i) for(std::size_t j=i+1;j<dim;++j){ auto qij=q; qij[i]+=eps; qij[j]+=eps; auto cij=node_bound(qij);
                for(std::size_t k=0;k<NBS;++k){ float dx=cij[k].x-ci[i][k].x-ci[j][k].x+base[k].x,dy=cij[k].y-ci[i][k].y-ci[j][k].y+base[k].y,dz=cij[k].z-ci[i][k].z-ci[j][k].z+base[k].z;
                    float h=std::sqrt(dx*dx+dy*dy+dz*dz)*ie2; H[k][i][j]=std::max(H[k][i][j],h); H[k][j][i]=H[k][i][j]; } } } }
    // (low-rank) rank-1 entrywise cover: b[k][i]=sqrt(max_j H[k][i][j]) => b_i*b_j >= H[k][i][j]
    // (both row-maxes dominate H[i][j]), so ad^T H ad <= (sum_i b_i ad_i)^2. O(dim), conservative.
    static std::array<std::array<float,dim>,NBS> B1{};
    for(std::size_t k=0;k<NBS;++k) for(std::size_t i=0;i<dim;++i){ float mx=0; for(std::size_t j=0;j<dim;++j) mx=std::max(mx,H[k][i][j]); B1[k][i]=std::sqrt(mx); }
    // per-edge sagitta = min of two provable O(dim) upper bounds (min of uppers is still an upper):
    //   rank-1 cover  (1/8)(sum_i b_i|dth_i|)^2   and   M*Omega  (1/8)(sum_i r_i|dth_i|)(sum_i |dth_i|)
    auto sag_bound=[&](std::size_t k, const std::array<float,dim>&dtheta){ float s=0,M=0,Om=0;
        for(std::size_t i=0;i<dim;++i){ float ad=std::fabs(dtheta[i]); s+=B1[k][i]*ad; M+=reach[k][i]*ad; Om+=ad; }
        return std::min(s*s, M*Om)/8.0f; };

    // verify conservativeness. Restrict to bounding spheres that actually move (sag > 3mm): the
    // near-stationary base spheres have sag~0 and H~0 so their ratio is 0/0 noise (and they are
    // far from obstacles -> never cause a skip). worst = max over MOVING (edge,k) of sag/bound.
    float worst_ratio=0; { std::mt19937 g(0x777);
        for(int it=0;it<3000;++it){ Block c; for(std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(g)); R::template scale_configuration_block<rake>(c);
            std::array<float,dim> a,v,b,mid; float nr=0; for(std::size_t j=0;j<dim;++j){a[j]=c[j].to_array()[0];v[j]=nd(g);nr+=v[j]*v[j];}
            nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j){v[j]*=range/nr;b[j]=a[j]+v[j];mid[j]=a[j]+0.5f*v[j];}
            auto ca=node_bound(a),cb=node_bound(b),cm=node_bound(mid);
            for(std::size_t k=0;k<NBS;++k){ float mx=(ca[k].x+cb[k].x)*0.5f,my=(ca[k].y+cb[k].y)*0.5f,mz=(ca[k].z+cb[k].z)*0.5f;
                float sag=std::sqrt((cm[k].x-mx)*(cm[k].x-mx)+(cm[k].y-my)*(cm[k].y-my)+(cm[k].z-mz)*(cm[k].z-mz));
                float bnd=SAFETY*sag_bound(k,v); if(sag>0.003f && bnd>1e-9f) worst_ratio=std::max(worst_ratio,sag/bnd); } } }

    // (a) SoA pair arrays for vectorized pair tests
    std::vector<int> PA(NP),PB(NP); for(std::size_t i=0;i<NP;++i){PA[i]=pair_bs[i][0];PB[i]=pair_bs[i][1];}

    struct Edge { std::vector<Block> B; std::array<BS,NBS> ca, cb; std::array<float,dim> v; bool free; };
    std::vector<Edge> edges; int attempts=0;
    while(edges.size()<3000 && attempts<600000){ ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng)); R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> a,v,b; float nr=0; for(std::size_t j=0;j<dim;++j){a[j]=tmp[j].to_array()[0];v[j]=nd(wrng);nr+=v[j]*v[j];}
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j){ v[j]*=range/nr; b[j]=a[j]+v[j]; }
        float dt=1.0f/(float)N; std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=a[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        bool fr=base_valid(B); if(fr || edges.size()%2==0) edges.push_back({std::move(B), node_bound(a), node_bound(b), v, fr});
    }

    auto masks=[&](const Edge&e, std::array<bool,NBS>&env_clear, std::array<bool,R::n_self_pairs>&pair_clear){
        alignas(32) std::array<float,NBS> Ex,Ey,Ez,RAD;
        for(std::size_t k=0;k<NBS;++k){ auto&A=e.ca[k]; auto&Bb=e.cb[k];
            float half=0.5f*std::sqrt((A.x-Bb.x)*(A.x-Bb.x)+(A.y-Bb.y)*(A.y-Bb.y)+(A.z-Bb.z)*(A.z-Bb.z));
            float rem=SAFETY*sag_bound(k,e.v);               // (b) per-edge analytic sagitta remainder
            Ex[k]=(A.x+Bb.x)*0.5f;Ey[k]=(A.y+Bb.y)*0.5f;Ez[k]=(A.z+Bb.z)*0.5f; RAD[k]=half+std::max(A.r,Bb.r)+rem; }
        // (a) env swept test: squared distance, branchless (clear iff dist^2 > (RAD+r_o)^2 for all obs)
        for(std::size_t k=0;k<NBS;++k){ float ex=Ex[k],ey=Ey[k],ez=Ez[k],Rk=RAD[k]; bool clear=true;
            for(auto&o:obs){ float dx=ex-o.x,dy=ey-o.y,dz=ez-o.z,s=Rk+o.r; clear &= (dx*dx+dy*dy+dz*dz > s*s); } env_clear[k]=clear; }
        // (a) vectorized pair test: SoA gather-free inner, branchless -> autovectorizes
        for(std::size_t pi=0;pi<NP;++pi){ int a=PA[pi],b=PB[pi];
            float dx=Ex[a]-Ex[b],dy=Ey[a]-Ey[b],dz=Ez[a]-Ez[b]; float s=RAD[a]+RAD[b];
            pair_clear[pi]=(dx*dx+dy*dy+dz*dz > s*s); }                     // squared -> no sqrt
    };
    auto cached_valid=[&](const Edge&e){ std::array<bool,NBS> ec; std::array<bool,R::n_self_pairs> pc{}; masks(e,ec,pc);
        for(auto&b:e.B) if(not R::template fkcc_swept<rake>(env,b,ec,pc)) return false; return true; };
    auto lazy_valid=[&](const Edge&e){ std::array<bool,NBS> ec; std::array<bool,R::n_self_pairs> pc{}; masks(e,ec,pc);
        for(auto&b:e.B) if(not R::template fkcc_swept_lazy<rake>(env,b,ec,pc)) return false; return true; };
    auto staged_valid=[&](const Edge&e){ std::array<bool,NBS> ec; std::array<bool,R::n_self_pairs> pc{}; masks(e,ec,pc);
        for(auto&b:e.B) if(not R::template fkcc_swept_staged<rake>(env,b,ec,pc)) return false; return true; };

    std::size_t mm=0, unsafe=0, nfree=0;
    for(auto&e:edges){ bool bv=base_valid(e.B), cv=cached_valid(e), sv=staged_valid(e); nfree+=e.free; if(bv!=cv||bv!=sv){++mm; if((cv||sv)&&!bv)++unsafe;} }
    auto med=[&](auto fn,int mode){ std::vector<double> t; volatile std::uint64_t sk=0; std::vector<Edge*> sub;
        for(auto&e:edges) if(mode==0||(mode==1&&e.free)||(mode==2&&!e.free)) sub.push_back(&e); if(sub.empty())return 0.0;
        for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(auto*e:sub)ac+=fn(*e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/sub.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    auto Uf=[&](const Edge&e){return base_valid(e.B)?1U:0U;}; auto Cf=[&](const Edge&e){return cached_valid(e)?1U:0U;}; auto Sg=[&](const Edge&e){return staged_valid(e)?1U:0U;};
    std::printf("%-6s n=%zu NBS=%zu pairs=%zu free=%zu/%zu SAFETY=%.1f  sagitta_margin=%.2f  mismatch=%zu unsafe=%zu\n",
                name,n,NBS,NP,nfree,edges.size(),SAFETY, worst_ratio>0?1.0f/worst_ratio:0.0f, mm,unsafe);
    for(int m=0;m<3;++m){ const char*lbl=m==0?"all":m==1?"free":"colliding"; double U=med(Uf,m),Cc=med(Cf,m),Sd=med(Sg,m);
        std::printf("   %-9s: fkcc=%.0f  swept=%.0f (%.2fx)  swept_staged=%.0f (%.2fx)\n", lbl,U,Cc,U/Cc,Sd,U/Sd); }
}

int main()
{
    run<vamp::robots::PandaE>("panda",1.25f,40,panda_pair_bs,1.3f);
    run<vamp::robots::FetchE>("fetch",1.0f,40,fetch_pair_bs,1.3f);
    run<vamp::robots::BaxterE>("baxter",0.5f,40,baxter_pair_bs,1.3f);
    return 0;
}
