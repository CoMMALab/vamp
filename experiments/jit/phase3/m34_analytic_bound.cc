// De-risk the swept broadphase: can a CHEAP analytic bound (K sampled configs + Lipschitz
// remainder) match the exact-trajectory certified% from m33? A real bound_fk kernel evaluates
// only K endpoints/samples, not all n rakes. For a link with per-joint reach r[link][j],
// M_link = sum_j r[link][j]*|dtheta_j| bounds its Cartesian sweep over the whole edge; over a
// t-subinterval of length g the center bulges from the nearer sample by <= M_link*g/2. So:
//   e = midpoint config; rho = max_i |c(s_i)-e| + M_link*gap/2; R = max_i bs_r(s_i) + M_link*gap/2
//   certified iff clearance(e) - R > rho.
// Sweep K = 2,3,5,9 samples vs the exact bound.
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

// per-link per-joint reach r[link][j] = max over link's leaves of sphere displacement / radian
template <class R>
static std::vector<std::array<float, R::dimension>> link_reach(const std::vector<std::vector<int>> &links,
                                                               std::size_t jlo, std::size_t jhi, std::mt19937 &rng)
{
    constexpr std::size_t dim=R::dimension;
    std::vector<std::array<float,dim>> reach(links.size(), std::array<float,dim>{});
    std::uniform_real_distribution<float> u(0.f,1.f); typename R::template Spheres<rake> a,b; const float eps=1e-3f;
    for (int it=0; it<4000; ++it){
        typename R::template ConfigurationBlock<rake> c; for(std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(rng));
        R::template scale_configuration_block<rake>(c); R::template sphere_fk<rake>(c,a);
        for(std::size_t j=jlo;j<jhi;++j){ auto c2=c; c2[j]=c[j]+static_cast<DataV>(eps); R::template sphere_fk<rake>(c2,b);
            for(std::size_t li=0;li<links.size();++li){ float md=0; for(int k:links[li]){
                float dx=a.x[k].to_array()[0]-b.x[k].to_array()[0],dy=a.y[k].to_array()[0]-b.y[k].to_array()[0],dz=a.z[k].to_array()[0]-b.z[k].to_array()[0];
                md=std::max(md,std::sqrt(dx*dx+dy*dy+dz*dz)); } reach[li][j]=std::max(reach[li][j], md/eps); } }
    }
    return reach;
}

template <class R>
static void run(const char *name, std::size_t jlo, std::size_t jhi, float range, int nobs,
                const std::vector<std::vector<int>> &links)
{
    constexpr std::size_t dim=R::dimension, NS=R::n_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake, L=links.size();
    std::mt19937 srng(0xBEEF), wrng(0x33), rrng(0x77);
    vamp::collision::Environment<DataV> env; auto obs=shelf<R>(srng,nobs,env);
    auto reach=link_reach<R>(links,jlo,jhi,rrng);
    std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    auto fkcc_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    const std::vector<int> Ks={2,3,5,9}; std::vector<double> cert(Ks.size(),0), cert_nr(Ks.size(),0); double cert_exact=0;
    std::size_t NE=2000, got=0; int attempts=0;
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
        std::vector<std::array<float,3>> pos(NS*N); std::vector<float> lr(NS);
        for(std::size_t r=0;r<n;++r){ typename R::template Spheres<rake> sp; R::template sphere_fk<rake>(B[r],sp);
            for(std::size_t l=0;l<rake;++l){ std::size_t idx=r*rake+l; for(std::size_t k=0;k<NS;++k){
                pos[k*N+idx]={sp.x[k].to_array()[l],sp.y[k].to_array()[l],sp.z[k].to_array()[l]}; if(idx==0) lr[k]=sp.r[k].to_array()[l]; } } }

        auto bscenter=[&](const std::vector<int>&grp,std::size_t idx){ std::array<float,3> ct{0,0,0}; for(int k:grp){auto&p=pos[k*N+idx];ct[0]+=p[0];ct[1]+=p[1];ct[2]+=p[2];} float m=grp.size(); ct[0]/=m;ct[1]/=m;ct[2]/=m; return ct; };
        auto bsrad2=[&](const std::vector<int>&grp,std::size_t idx,std::array<float,3> ct){ float br=0; for(int k:grp){auto&p=pos[k*N+idx]; float ex=p[0]-ct[0],ey=p[1]-ct[1],ez=p[2]-ct[2]; br=std::max(br,std::sqrt(ex*ex+ey*ey+ez*ez)+lr[k]); } return br; };
        auto clearance=[&](std::array<float,3> e){ float c=1e9f; for(auto&o:obs){float dx=e[0]-o.x,dy=e[1]-o.y,dz=e[2]-o.z;c=std::min(c,std::sqrt(dx*dx+dy*dy+dz*dz)-o.r);} return c; };

        std::size_t mid=N/2;
        // exact (m33): full trajectory
        { std::size_t c=0; for(auto&grp:links){ auto e=bscenter(grp,mid); float rho=0,Rr=0;
            for(std::size_t idx=0;idx<N;++idx){ auto ct=bscenter(grp,idx); float dx=ct[0]-e[0],dy=ct[1]-e[1],dz=ct[2]-e[2]; rho=std::max(rho,std::sqrt(dx*dx+dy*dy+dz*dz)); Rr=std::max(Rr,bsrad2(grp,idx,ct)); }
            if(clearance(e)-Rr>rho) ++c; } cert_exact+=(double)c/L; }

        // analytic: K samples, with Lipschitz remainder (cert) and without (cert_nr = sampling ceiling)
        for(std::size_t ki=0;ki<Ks.size();++ki){ int K=Ks[ki]; float gap=1.0f/(float)(K-1); std::size_t c=0, cnr=0;
            for(std::size_t li=0;li<L;++li){ auto&grp=links[li];
                float M=0; for(std::size_t j=jlo;j<jhi;++j) M+=reach[li][j]*std::fabs(v[j]);
                float rem=M*gap*0.5f;
                std::array<float,3> e = bscenter(grp, mid);      // real kernel evaluates the midpoint sample
                float rho=0,Rr=0;
                for(int t=0;t<K;++t){ float tt=(float)t/(float)(K-1); std::size_t idx=std::min<std::size_t>(N-1,(std::size_t)std::lround(tt*(N-1)));
                    auto ct=bscenter(grp,idx); float dx=ct[0]-e[0],dy=ct[1]-e[1],dz=ct[2]-e[2]; rho=std::max(rho,std::sqrt(dx*dx+dy*dy+dz*dz)); Rr=std::max(Rr,bsrad2(grp,idx,ct)); }
                if(clearance(e)-Rr>rho) ++cnr;                    // no remainder = sampling ceiling
                if(clearance(e)-(Rr+rem)>rho+rem) ++c; }          // with Lipschitz remainder
            cert[ki]+=(double)c/L; cert_nr[ki]+=(double)cnr/L; }
    }
    std::printf("%-6s n=%zu links=%zu free=%zu  exact=%.1f%%\n", name, n, L, got, 100.0*cert_exact/got);
    std::printf("   with Lipschitz remainder: "); for(std::size_t ki=0;ki<Ks.size();++ki) std::printf("K=%d:%.1f%%  ", Ks[ki], 100.0*cert[ki]/got); std::printf("\n");
    std::printf("   sampling ceiling (no rem): "); for(std::size_t ki=0;ki<Ks.size();++ki) std::printf("K=%d:%.1f%%  ", Ks[ki], 100.0*cert_nr[ki]/got); std::printf("\n");
}

int main()
{
    run<vamp::robots::PandaE>("panda",0,7,1.25f,40,panda_links);
    run<vamp::robots::FetchE>("fetch",1,8,1.0f,40,fetch_links);
    return 0;
}
