// Skippability + conservative advancement (CA) probe.
//  Q1 (skippability): along a free edge, how large does robot->obstacle clearance stay? Can a
//      single clearance check at the midpoint certify the WHOLE edge free?
//  Q2 (skip subsequent rakes): CA -- at a config with clearance d, every config within joint-
//      motion d/M is provably free (M = max Cartesian sweep rate = sum_j reach_j*|dtheta_j|).
//      So one clearance eval certifies a variable-length safe interval; count how many evals
//      certify a whole free edge vs the n rake-blocks the kernel evaluates today.
// Clearance is the true min over all leaf spheres x all obstacles (brute force; profile only).
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "panda_e.hh"
#include "fetch_e.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
struct Obs { float x, y, z, r; };

template <class R>
static std::vector<Obs> shelf(std::mt19937 &rng, int nobs, vamp::collision::Environment<DataV> &envv)
{
    vamp::collision::Environment<float> ef; std::vector<Obs> obs;
    std::uniform_real_distribution<float> sx(0.45F, 0.85F), sy(-0.35F, 0.35F), sz(0.45F, 1.0F), rad(0.02F, 0.05F);
    for (int i = 0; i < nobs; ++i) { float x=sx(rng),y=sy(rng),z=sz(rng),rr=rad(rng);
        ef.spheres.emplace_back(vamp::collision::factory::sphere::array({x,y,z}, rr)); obs.push_back({x,y,z,rr}); }
    ef.sort(); envv = vamp::collision::Environment<DataV>(ef); return obs;
}

// per-joint reach r_j = max over sampled configs of max sphere displacement per radian of joint j
template <class R>
static std::array<float, R::dimension> joint_reach(std::size_t jlo, std::size_t jhi, std::mt19937 &rng)
{
    constexpr std::size_t dim = R::dimension, NS = R::n_spheres;
    std::array<float, dim> reach{}; std::uniform_real_distribution<float> u(0.f, 1.f);
    typename R::template Spheres<rake> a, b; const float eps = 1e-3f;
    for (int it = 0; it < 4000; ++it) {
        typename R::template ConfigurationBlock<rake> c;
        for (std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(rng));
        R::template scale_configuration_block<rake>(c);
        R::template sphere_fk<rake>(c, a);
        for (std::size_t j=jlo;j<jhi;++j){
            auto c2=c; c2[j]=c[j]+static_cast<DataV>(eps); R::template sphere_fk<rake>(c2, b);
            float md=0; for(std::size_t k=0;k<NS;++k){ float dx=a.x[k].to_array()[0]-b.x[k].to_array()[0],
                dy=a.y[k].to_array()[0]-b.y[k].to_array()[0], dz=a.z[k].to_array()[0]-b.z[k].to_array()[0];
                md=std::max(md,std::sqrt(dx*dx+dy*dy+dz*dz)); }
            reach[j]=std::max(reach[j], md/eps);
        }
    }
    return reach;
}

template <class R>
static void run(const char *name, std::size_t jlo, std::size_t jhi, float range, int nobs, int resmul = 1)
{
    constexpr std::size_t dim = R::dimension, NS = R::n_spheres;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(resmul * range * 32.0f / rake));
    std::size_t N = n * rake;                       // total configs on the edge (resmul x native)
    std::mt19937 srng(0xBEEF), wrng(0x33), rrng(0x77);
    vamp::collision::Environment<DataV> env; auto obs = shelf<R>(srng, nobs, env);
    auto reach = joint_reach<R>(jlo, jhi, rrng);
    std::uniform_real_distribution<float> u(0.f, 1.f); std::normal_distribution<float> nd(0.f, 1.f);
    using Block = typename R::template ConfigurationBlock<rake>;
    auto fkcc_valid=[&](const std::vector<Block>&B){ for(auto&b:B) if(not R::template fkcc<rake>(env,b)) return false; return true; };

    // clearance of one config (given as Spheres block, lane l) vs all obstacles
    auto clear_lane=[&](const typename R::template Spheres<rake>&s, std::size_t l){ float mn=1e9f;
        for(std::size_t k=0;k<NS;++k){ float cx=s.x[k].to_array()[l],cy=s.y[k].to_array()[l],cz=s.z[k].to_array()[l],cr=s.r[k].to_array()[l];
            for(auto&o:obs){ float dx=cx-o.x,dy=cy-o.y,dz=cz-o.z; mn=std::min(mn,std::sqrt(dx*dx+dy*dy+dz*dz)-cr-o.r); } }
        return mn; };

    std::size_t NE=2000, got=0; int attempts=0;
    double sum_evals_M=0, sum_evals_ideal=0, one_check=0, sum_minclr=0;
    double sum_sphere_cert=0;                          // per-sphere swept-certifiable fraction
    std::size_t n_blocks_total=0;
    while (got<NE && attempts<600000){
        ++attempts;
        Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nr=0;
        for(std::size_t j=0;j<dim;++j){ s[j]=tmp[j].to_array()[0]; v[j]=nd(wrng); nr+=v[j]*v[j]; }
        nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr;
        float dt=1.0f/static_cast<float>(N);
        std::vector<Block> B(n);
        for(std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+static_cast<float>(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        if(!fkcc_valid(B)) continue;
        ++got; n_blocks_total+=n;

        // full sphere trajectories: pos[k] over all N configs, plus whole-robot clearance c[N]
        std::vector<std::array<float,3>> pos(NS*N); std::vector<float> sr(NS);
        std::vector<float> c(N);
        for(std::size_t r=0;r<n;++r){ typename R::template Spheres<rake> sp; R::template sphere_fk<rake>(B[r], sp);
            for(std::size_t l=0;l<rake;++l){ std::size_t idx=r*rake+l;
                for(std::size_t k=0;k<NS;++k){ pos[k*N+idx]={sp.x[k].to_array()[l],sp.y[k].to_array()[l],sp.z[k].to_array()[l]}; if(idx==0) sr[k]=sp.r[k].to_array()[l]; }
                c[idx]=clear_lane(sp,l); } }
        float minclr=*std::min_element(c.begin(),c.end()); sum_minclr+=minclr;

        // Q1' per-sphere swept test: enclose each sphere's center trajectory (center=midpoint
        // config, radius rho=max deviation); certified free for the WHOLE edge iff
        // min_o dist(e_s,o)-r_s-r_o > rho_s. Fraction certified = spheres that skip all n rakes.
        { std::size_t cert=0; std::size_t mid=N/2;
          for(std::size_t k=0;k<NS;++k){ auto e=pos[k*N+mid]; float rho=0, clr=1e9f;
            for(std::size_t idx=0;idx<N;++idx){ auto&p=pos[k*N+idx]; float dx=p[0]-e[0],dy=p[1]-e[1],dz=p[2]-e[2]; rho=std::max(rho,std::sqrt(dx*dx+dy*dy+dz*dz)); }
            for(auto&o:obs){ float dx=e[0]-o.x,dy=e[1]-o.y,dz=e[2]-o.z; clr=std::min(clr,std::sqrt(dx*dx+dy*dy+dz*dz)-sr[k]-o.r); }
            if(clr>rho) ++cert; }
          sum_sphere_cert += (double)cert/NS; }

        // M = max Cartesian sweep of the whole edge (Lipschitz const over t in [0,1])
        float M=0; for(std::size_t j=jlo;j<jhi;++j) M+=reach[j]*std::fabs(v[j]);
        float per_step = M/(float)N;                 // displacement per single config step

        // Q1: single clearance check at midpoint certifies whole edge?  c_mid > M/2
        if (c[N/2] > 0.5f*M) one_check += 1.0;

        // Q2a: CA with the global (loose) Lipschitz bound M
        { std::size_t i=0, ev=0; while(i<N){ ++ev; float d=c[i]; std::size_t step=(std::size_t)std::floor(d/per_step);
            if(step<1) step=1; i+=step; } sum_evals_M+=ev; }
        // Q2b: CA with the tightest possible (empirical per-step) displacement -> headroom bound
        { std::vector<float> disp(N,per_step); // recompute true per-step max sphere displacement
          // approximate ideal via actual clearance-limited jumps using local disp = per_step (edge is affine, per_step is already the exact per-step bound for the loose M; the tight version needs per-joint local reach which equals global here). Use half M as a proxy for a 2x-tighter bound.
          std::size_t i=0,ev=0; float ps2=0.5f*per_step; while(i<N){ ++ev; std::size_t step=(std::size_t)std::floor(c[i]/ps2); if(step<1)step=1; i+=step; } sum_evals_ideal+=ev; }
    }
    std::printf("%-6s dim=%zu n=%zu N=%zu obs=%d free=%zu  reach_max=%.2fm\n",
                name, dim, n, N, nobs, got, *std::max_element(reach.begin()+jlo, reach.begin()+jhi));
    std::printf("   median min-clearance over free edge = %.3f m\n", sum_minclr/got);
    std::printf("   Q1  single midpoint check certifies whole edge: %.1f%%\n", 100.0*one_check/got);
    std::printf("   Q1' per-sphere swept-certifiable (skip all rakes): %.1f%% of spheres\n", 100.0*sum_sphere_cert/got);
    std::printf("   Q2 CA evals/edge: loose-M=%.2f  2x-tighter=%.2f   vs n_blocks=%zu  -> block-skip %.2fx / %.2fx\n",
                sum_evals_M/got, sum_evals_ideal/got, n, (double)n/(sum_evals_M/got), (double)n/(sum_evals_ideal/got));
}

int main()
{
    // realistic RRTC ranges
    run<vamp::robots::PandaE>("panda", 0, 7, 1.25f, 40);
    run<vamp::robots::FetchE>("fetch", 1, 8, 1.0f, 40);
    // CA crossover: longer edges (more rakes) -- CA evals ~ M/clearance is ~n-independent,
    // so it should overtake the n-block kernel as edges lengthen.
    std::printf("--- CA vs edge LENGTH (M and n both grow -> ratio flat, CA never wins) ---\n");
    run<vamp::robots::PandaE>("panda", 0, 7, 0.5f, 40);
    run<vamp::robots::PandaE>("panda", 0, 7, 2.5f, 40);
    std::printf("--- CA vs RESOLUTION at fixed length 1.25 (M fixed, n grows -> CA crosses over) ---\n");
    run<vamp::robots::PandaE>("panda", 0, 7, 1.25f, 40, 2);
    run<vamp::robots::PandaE>("panda", 0, 7, 1.25f, 40, 4);
    run<vamp::robots::PandaE>("panda", 0, 7, 1.25f, 40, 8);
    return 0;
}
