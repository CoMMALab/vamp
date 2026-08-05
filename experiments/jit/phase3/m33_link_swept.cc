// Per-LINK swept certifiability -- the number that actually drives a swept broadphase, since
// VAMP prunes per-link bounding sphere (not per-leaf). For each link: bounding sphere per
// config (centroid of its leaves, enclosing radius); enclose the bs-center trajectory over the
// edge (center=midpoint, rho=max deviation, R=max bs radius); certified free for the WHOLE edge
// iff min_o dist(center,o)-R-r_o > rho -> that link skips all n rakes of broadphase.
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "panda_e.hh"
#include "fetch_e.hh"
#include "link_groups.hh"

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

template <class R>
static void run(const char *name, std::size_t jlo, std::size_t jhi, float range, int nobs,
                const std::vector<std::vector<int>> &links)
{
    constexpr std::size_t dim=R::dimension, NS=R::n_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake, L=links.size();
    std::mt19937 srng(0xBEEF), wrng(0x33);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto fkcc_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    std::size_t NE=2000, got=0; int attempts=0; double link_cert=0, sphere_cert=0;
    while(got<NE && attempts<600000){
        ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nr=0;
        for(std::size_t j=0;j<dim;++j){ s[j]=tmp[j].to_array()[0]; v[j]=nd(wrng); nr+=v[j]*v[j]; }
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr;
        float dt=1.0f/(float)N;
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        if(!fkcc_valid(B)) continue;
        ++got;

        // leaf positions over all configs
        std::vector<std::array<float,3>> pos(NS*N); std::vector<float> lr(NS);
        for(std::size_t r=0;r<n;++r){ typename R::template Spheres<rake> sp; R::template sphere_fk<rake>(B[r],sp);
            for(std::size_t l=0;l<rake;++l){ std::size_t idx=r*rake+l; for(std::size_t k=0;k<NS;++k){
                pos[k*N+idx]={sp.x[k].to_array()[l],sp.y[k].to_array()[l],sp.z[k].to_array()[l]}; if(idx==0) lr[k]=sp.r[k].to_array()[l]; } } }

        // per-link bounding sphere per config, then swept-certify
        std::size_t mid=N/2, cert=0;
        for(auto&grp:links){
            // bs center per config = centroid of leaves; radius = max(dist+leaf_r)
            std::array<float,3> emid{0,0,0};
            auto bscenter=[&](std::size_t idx){ std::array<float,3> ct{0,0,0}; for(int k:grp){auto&p=pos[k*N+idx]; ct[0]+=p[0];ct[1]+=p[1];ct[2]+=p[2];} float m=grp.size(); ct[0]/=m;ct[1]/=m;ct[2]/=m; return ct; };
            emid=bscenter(mid);
            float rho=0,R_=0;
            for(std::size_t idx=0;idx<N;++idx){ auto ct=bscenter(idx); float dx=ct[0]-emid[0],dy=ct[1]-emid[1],dz=ct[2]-emid[2]; rho=std::max(rho,std::sqrt(dx*dx+dy*dy+dz*dz));
                float br=0; for(int k:grp){auto&p=pos[k*N+idx]; float ex=p[0]-ct[0],ey=p[1]-ct[1],ez=p[2]-ct[2]; br=std::max(br,std::sqrt(ex*ex+ey*ey+ez*ez)+lr[k]);} R_=std::max(R_,br); }
            float clr=1e9f; for(auto&o:obs){ float dx=emid[0]-o.x,dy=emid[1]-o.y,dz=emid[2]-o.z; clr=std::min(clr,std::sqrt(dx*dx+dy*dy+dz*dz)-R_-o.r); }
            if(clr>rho) ++cert;
        }
        link_cert += (double)cert/L;

        // per-sphere for reference
        std::size_t sc=0; for(std::size_t k=0;k<NS;++k){ auto e=pos[k*N+mid]; float rho=0,clr=1e9f;
            for(std::size_t idx=0;idx<N;++idx){auto&p=pos[k*N+idx];float dx=p[0]-e[0],dy=p[1]-e[1],dz=p[2]-e[2];rho=std::max(rho,std::sqrt(dx*dx+dy*dy+dz*dz));}
            for(auto&o:obs){float dx=e[0]-o.x,dy=e[1]-o.y,dz=e[2]-o.z;clr=std::min(clr,std::sqrt(dx*dx+dy*dy+dz*dz)-lr[k]-o.r);} if(clr>rho)++sc; }
        sphere_cert += (double)sc/NS;
    }
    std::printf("%-6s n=%zu links=%zu free=%zu  per-LINK swept-certifiable=%.1f%%  (per-sphere ref=%.1f%%)  broadphase work skipped~=%.1f%%\n",
                name, n, L, got, 100.0*link_cert/got, 100.0*sphere_cert/got, 100.0*link_cert/got*(double)(n-1)/n);
}

int main()
{
    run<vamp::robots::PandaE>("panda",0,7,1.25f,40,panda_links);
    run<vamp::robots::FetchE>("fetch",1,8,1.0f,40,fetch_links);
    return 0;
}
