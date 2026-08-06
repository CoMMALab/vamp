// Real-time regime: MBM scenes as DENSE surface pointclouds behind an MVT voxel table (the
// structure sensor-based planners use), instead of sparse primitives. Measures whether the
// JIT/collision ideas behave differently when the per-rake environment query is EXPENSIVE.
//   - baseline fkcc cost: MVT vs sparse primitives (confirms env cost grows)
//   - env-check share of the kernel against the MVT (scene-spec / swept ceiling)
//   - swept broadphase IDEAL (free masks) and REAL (cached setup) vs baseline, against the MVT
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <unordered_set>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include "ur5_e.hh"
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "swept_aux.hh"
#include "mbm_envs.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
using Point = vamp::collision::Point;
struct BS { float x, y, z, r; };

static void rotmat(float rho, float th, float phi, float R[9])
{
    float cr=std::cos(rho),sr=std::sin(rho),ct=std::cos(th),st=std::sin(th),cp=std::cos(phi),sp=std::sin(phi);
    // Rz(phi)*Ry(th)*Rx(rho), row-major
    R[0]=cp*ct;            R[1]=cp*st*sr-sp*cr;  R[2]=cp*st*cr+sp*sr;
    R[3]=sp*ct;            R[4]=sp*st*sr+cp*cr;  R[5]=sp*st*cr-cp*sr;
    R[6]=-st;              R[7]=ct*sr;           R[8]=ct*cr;
}
static Point xf(const float R[9], float cx, float cy, float cz, float x, float y, float z)
{ return {cx+R[0]*x+R[1]*y+R[2]*z, cy+R[3]*x+R[4]*y+R[5]*z, cz+R[6]*x+R[7]*y+R[8]*z}; }

// dense surface pointcloud (~1.5cm spacing) of the MBM primitives
static std::vector<Point> surface_points(const MbmEnv &me)
{
    std::vector<Point> pts; const float sp = 0.015f;
    for (const auto &p : me.prims) {
        if (p.kind == 0) {                                 // sphere: fibonacci
            int nq = std::max(24, (int)(4.0f*M_PI*p.a*p.a/(sp*sp)));
            const float ga = M_PI*(3.f-std::sqrt(5.f));
            for (int i=0;i<nq;++i){ float y=1.f-2.f*(i+0.5f)/nq, r=std::sqrt(std::max(0.f,1.f-y*y)), t=ga*i;
                pts.push_back({p.px+p.a*r*std::cos(t), p.py+p.a*y, p.pz+p.a*r*std::sin(t)}); }
        } else if (p.kind == 2) {                          // box: 6 faces
            float R[9]; rotmat(p.ex,p.ey,p.ez,R); float h[3]={p.a,p.b,p.c};
            for (int ax=0;ax<3;++ax){ int u=(ax+1)%3,v=(ax+2)%3;
                int nu=std::max(2,(int)(2*h[u]/sp)), nv=std::max(2,(int)(2*h[v]/sp));
                for (int s=-1;s<=1;s+=2) for(int iu=0;iu<=nu;++iu) for(int iv=0;iv<=nv;++iv){
                    float lc[3]; lc[ax]=s*h[ax]; lc[u]=-h[u]+2*h[u]*iu/nu; lc[v]=-h[v]+2*h[v]*iv/nv;
                    pts.push_back(xf(R,p.px,p.py,p.pz,lc[0],lc[1],lc[2])); } }
        } else {                                           // cylinder: curved + caps
            float R[9]; rotmat(p.ex,p.ey,p.ez,R); float r=p.a,hl=p.b*0.5f;
            int nth=std::max(8,(int)(2*M_PI*r/sp)), nz=std::max(2,(int)(2*hl/sp));
            for(int ia=0;ia<nth;++ia){ float a=2*M_PI*ia/nth;
                for(int iz=0;iz<=nz;++iz){ float z=-hl+2*hl*iz/nz; pts.push_back(xf(R,p.px,p.py,p.pz,r*std::cos(a),r*std::sin(a),z)); } }
            for(int s=-1;s<=1;s+=2){ int nr=std::max(1,(int)(r/sp));
                for(int ir=0;ir<=nr;++ir){ float rr=r*ir/nr; int na=std::max(4,(int)(2*M_PI*rr/sp));
                    for(int ia=0;ia<na;++ia){ float a=2*M_PI*ia/na; pts.push_back(xf(R,p.px,p.py,p.pz,rr*std::cos(a),rr*std::sin(a),s*hl)); } } }
        }
    }
    return pts;
}

// Coarse occupancy grid (voxel ~ swept-sphere size) for a CHEAP swept certification: instead of
// scanning fine MVT voxels + points with a large radius, test the swept sphere against occupied
// COARSE voxels. Conservative: clear() true => no point within radius (uses radius+point_radius).
struct CoarseGrid
{
    float vs = 0.3f, pr = 0.0075f; std::array<float,3> o{0,0,0}; int nx=0, ny=0, nz=0;
    std::vector<std::uint8_t> g;
    inline int idx(int ix,int iy,int iz) const { return ix + nx*(iy + ny*iz); }
    void build(const std::vector<Point> &pts, float voxel, float point_r)
    {
        vs = voxel; pr = point_r; if (pts.empty()) { nx=ny=nz=1; g.assign(1,0); return; }
        std::array<float,3> mn{1e9f,1e9f,1e9f}, mx{-1e9f,-1e9f,-1e9f};
        for (auto &p : pts) for (int d=0;d<3;++d){ mn[d]=std::min(mn[d],p[d]); mx[d]=std::max(mx[d],p[d]); }
        for (int d=0;d<3;++d) o[d]=mn[d]-vs;
        nx=(int)((mx[0]-o[0])/vs)+2; ny=(int)((mx[1]-o[1])/vs)+2; nz=(int)((mx[2]-o[2])/vs)+2;
        g.assign((std::size_t)nx*ny*nz, 0);
        for (auto &p : pts){ int ix=(int)((p[0]-o[0])/vs),iy=(int)((p[1]-o[1])/vs),iz=(int)((p[2]-o[2])/vs); g[idx(ix,iy,iz)]=1; }
    }
    bool clear(float cx, float cy, float cz, float r) const
    {
        const float rr = r + pr;
        int lx=std::max(0,(int)((cx-rr-o[0])/vs)), hx=std::min(nx-1,(int)((cx+rr-o[0])/vs));
        int ly=std::max(0,(int)((cy-rr-o[1])/vs)), hy=std::min(ny-1,(int)((cy+rr-o[1])/vs));
        int lz=std::max(0,(int)((cz-rr-o[2])/vs)), hz=std::min(nz-1,(int)((cz+rr-o[2])/vs));
        for (int iz=lz;iz<=hz;++iz) for (int iy=ly;iy<=hy;++iy) for (int ix=lx;ix<=hx;++ix) {
            if (not g[idx(ix,iy,iz)]) continue;
            float ax=o[0]+ix*vs, ay=o[1]+iy*vs, az=o[2]+iz*vs;
            float qx=std::max(ax,std::min(cx,ax+vs)), qy=std::max(ay,std::min(cy,ay+vs)), qz=std::max(az,std::min(cz,az+vs));
            float dx=cx-qx,dy=cy-qy,dz=cz-qz; if (dx*dx+dy*dy+dz*dz < rr*rr) return false;
        }
        return true;
    }
};

template <class R>
static vamp::collision::Environment<DataV> build_sparse(const MbmEnv &me)
{
    vamp::collision::Environment<float> ef;
    for (const auto &p : me.prims) {
        if (p.kind==0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px,p.py,p.pz,p.a));
        else if (p.kind==1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px,p.py,p.pz,p.ex,p.ey,p.ez,p.a,p.b));
        else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px,p.py,p.pz,p.ex,p.ey,p.ez,p.a,p.b,p.c));
    }
    ef.sort(); return vamp::collision::Environment<DataV>(ef);
}
static vamp::collision::Environment<DataV> build_mvt(const std::vector<Point> &pts, float max_q, float r_point)
{
    vamp::collision::Environment<float> ef;
    Point mn{1e9f,1e9f,1e9f}, mx{-1e9f,-1e9f,-1e9f};
    for (auto &q : pts) for (int d=0;d<3;++d){ mn[d]=std::min(mn[d],q[d]); mx[d]=std::max(mx[d],q[d]); }
    for (int d=0;d<3;++d){ mn[d]-=max_q+r_point+0.05f; mx[d]+=max_q+r_point+0.05f; }
    if (not pts.empty()) ef.pointclouds_mvt.emplace_back(pts, max_q, mn, mx, r_point);
    return vamp::collision::Environment<DataV>(ef);
}

template <class R>
static void run(const char *name, float range, const std::vector<MbmEnv> &mbm)
{
    constexpr std::size_t dim=R::dimension, NBS=R::n_bounding_spheres;
    std::size_t n=std::max<std::size_t>(1,(std::size_t)std::ceil(range*32.0f/rake)), N=n*rake, NP=ur5_pair_bs.size();
    const std::vector<std::array<int,2>> *PB = nullptr; { (void)NP; }
    std::mt19937 rng(7); std::uniform_real_distribution<float> u(0.f,1.f); std::normal_distribution<float> nd(0.f,1.f);
    using Block=typename R::template ConfigurationBlock<rake>;
    const float SAFETY=1.3f;

    // per-bs reach for M*Omega sagitta
    std::array<std::array<float,dim>,NBS> reach{};
    { std::mt19937 g(0xA1); const float eps=1e-3f; typename R::template BoundingSpheres<rake> a,b;
      auto nb=[&](const std::array<float,dim>&q){ Block bl; for(std::size_t j=0;j<dim;++j) bl[j]=DataV::fill(q[j]);
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(bl,bs); std::array<BS,NBS> o;
        for(std::size_t k=0;k<NBS;++k) o[k]={bs.x[k].to_array()[0],bs.y[k].to_array()[0],bs.z[k].to_array()[0],bs.r[k].to_array()[0]}; return o; };
      for(int it=0;it<2500;++it){ Block c; for(std::size_t j=0;j<dim;++j) c[j]=static_cast<DataV>(u(g)); R::template scale_configuration_block<rake>(c);
        std::array<float,dim> q; for(std::size_t j=0;j<dim;++j) q[j]=c[j].to_array()[0]; auto A=nb(q);
        for(std::size_t j=0;j<dim;++j){ auto q2=q; q2[j]+=eps; auto B=nb(q2);
            for(std::size_t k=0;k<NBS;++k){ float dx=A[k].x-B[k].x,dy=A[k].y-B[k].y,dz=A[k].z-B[k].z; reach[k][j]=std::max(reach[k][j],std::sqrt(dx*dx+dy*dy+dz*dz)/eps);} } } }
    auto sag=[&](std::size_t k,const std::array<float,dim>&v){ float M=0,Om=0; for(std::size_t j=0;j<dim;++j){float a=std::fabs(v[j]);M+=reach[k][j]*a;Om+=a;} return SAFETY*M*Om/8.0f; };

    // pick the robot's pair_bs
    const std::vector<std::array<int,2>> &pair_bs =
        (std::string(name)=="ur5")?ur5_pair_bs:(std::string(name)=="panda")?panda_pair_bs:(std::string(name)=="fetch")?fetch_pair_bs:baxter_pair_bs;
    std::size_t NPP=pair_bs.size();

    // build both env representations + a shared pointcount tally
    std::vector<vamp::collision::Environment<DataV>> mvt, sparse; std::vector<CoarseGrid> coarse; std::size_t total_pts=0;
    const float max_q=R::max_radius, r_point=0.0075f;
    for(auto&me:mbm){ auto pts=surface_points(me); total_pts+=pts.size(); mvt.push_back(build_mvt(pts,max_q,r_point)); sparse.push_back(build_sparse<R>(me));
        CoarseGrid cg; cg.build(pts, 0.3f, r_point); coarse.push_back(std::move(cg)); }
    vamp::collision::Environment<DataV> empty;

    // realistic edges (free under MVT), with cached endpoint bounding spheres
    struct Edge{int e;std::vector<Block>B;std::array<float,dim>v;std::array<BS,NBS>ca,cb;};
    std::vector<Edge> edges; std::mt19937 wr(0x33);
    auto bnd=[&](const std::array<float,dim>&q){ Block bl; for(std::size_t j=0;j<dim;++j) bl[j]=DataV::fill(q[j]);
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(bl,bs); std::array<BS,NBS> o;
        for(std::size_t k=0;k<NBS;++k) o[k]={bs.x[k].to_array()[0],bs.y[k].to_array()[0],bs.z[k].to_array()[0],bs.r[k].to_array()[0]}; return o; };
    for(std::size_t ei=0;ei<mvt.size();++ei){ int got=0,att=0;
        while(got<30&&att<20000){ ++att; Block tmp; for(std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wr)); R::template scale_configuration_block<rake>(tmp);
            std::array<float,dim> s,v; float nr=0; for(std::size_t j=0;j<dim;++j){s[j]=tmp[j].to_array()[0];v[j]=nd(wr);nr+=v[j]*v[j];}
            nr=std::sqrt(nr); for(std::size_t j=0;j<dim;++j) v[j]*=range/nr; float dt=1.0f/(float)N;
            std::vector<Block> B(n); bool free=true;
            for(std::size_t r=0;r<n;++r){ for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
                for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+(float)(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
                if(not R::template fkcc<rake>(mvt[ei],B[r])){free=false;break;} }
            std::array<float,dim> a,bb; for(std::size_t j=0;j<dim;++j){a[j]=s[j];bb[j]=s[j]+v[j];}
            if(free){ edges.push_back({(int)ei,std::move(B),v,bnd(a),bnd(bb)}); ++got; } }
    }

    auto masks=[&](const Edge&e, std::array<bool,NBS>&ec, std::array<bool,R::n_self_pairs>&pc){
        std::array<std::array<float,3>,NBS> E; std::array<float,NBS> RAD;
        for(std::size_t k=0;k<NBS;++k){ auto&A=e.ca[k];auto&Bb=e.cb[k];
            float half=0.5f*std::sqrt((A.x-Bb.x)*(A.x-Bb.x)+(A.y-Bb.y)*(A.y-Bb.y)+(A.z-Bb.z)*(A.z-Bb.z));
            E[k]={(A.x+Bb.x)*0.5f,(A.y+Bb.y)*0.5f,(A.z+Bb.z)*0.5f}; RAD[k]=half+std::max(A.r,Bb.r)+sag(k,e.v); }
        for(std::size_t k=0;k<NBS;++k) ec[k]=not vamp::sphere_environment_in_collision(mvt[e.e],DataV::fill(E[k][0]),DataV::fill(E[k][1]),DataV::fill(E[k][2]),DataV::fill(RAD[k]));
        for(std::size_t pi=0;pi<NPP;++pi){int a=pair_bs[pi][0],b=pair_bs[pi][1];float dx=E[a][0]-E[b][0],dy=E[a][1]-E[b][1],dz=E[a][2]-E[b][2];pc[pi]=(std::sqrt(dx*dx+dy*dy+dz*dz)>RAD[a]+RAD[b]);}
    };
    // COARSE certification: env_clear via the coarse occupancy grid (cheap) instead of a large-
    // radius fine-MVT query. Self-pair test unchanged (robot-robot).
    auto masks_coarse=[&](const Edge&e, std::array<bool,NBS>&ec, std::array<bool,R::n_self_pairs>&pc){
        std::array<std::array<float,3>,NBS> E; std::array<float,NBS> RAD;
        for(std::size_t k=0;k<NBS;++k){ auto&A=e.ca[k];auto&Bb=e.cb[k];
            float half=0.5f*std::sqrt((A.x-Bb.x)*(A.x-Bb.x)+(A.y-Bb.y)*(A.y-Bb.y)+(A.z-Bb.z)*(A.z-Bb.z));
            E[k]={(A.x+Bb.x)*0.5f,(A.y+Bb.y)*0.5f,(A.z+Bb.z)*0.5f}; RAD[k]=half+std::max(A.r,Bb.r)+sag(k,e.v); }
        for(std::size_t k=0;k<NBS;++k) ec[k]=coarse[e.e].clear(E[k][0],E[k][1],E[k][2],RAD[k]);
        for(std::size_t pi=0;pi<NPP;++pi){int a=pair_bs[pi][0],b=pair_bs[pi][1];float dx=E[a][0]-E[b][0],dy=E[a][1]-E[b][1],dz=E[a][2]-E[b][2];pc[pi]=(std::sqrt(dx*dx+dy*dy+dz*dz)>RAD[a]+RAD[b]);}
    };
    // rigor: coarse-swept must match baseline on every edge
    std::size_t mm=0;
    for(std::size_t i=0;i<edges.size();++i){ std::array<bool,NBS> ec; std::array<bool,R::n_self_pairs> pc{}; masks_coarse(edges[i],ec,pc);
        bool sv=true; for(auto&b:edges[i].B){ if(not R::template fkcc_swept<rake>(mvt[edges[i].e],b,ec,pc)){sv=false;break;} }
        bool bv=true; for(auto&b:edges[i].B){ if(not R::template fkcc<rake>(mvt[edges[i].e],b)){bv=false;break;} }
        if(sv!=bv) ++mm; }
    // precompute masks (free) for the IDEAL ceiling
    std::vector<std::array<bool,NBS>> ecp(edges.size()); std::vector<std::array<bool,R::n_self_pairs>> pcp(edges.size());
    for(std::size_t i=0;i<edges.size();++i) masks(edges[i],ecp[i],pcp[i]);

    typename R::template Spheres<rake> sp;
    auto med=[&](auto fn){ std::vector<double> t; volatile std::uint64_t sk=0;
        for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(std::size_t i=0;i<edges.size();++i)ac+=fn(i);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/edges.size()*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    double base_mvt = med([&](std::size_t i){std::uint32_t a=0;for(auto&b:edges[i].B)a+=R::template fkcc<rake>(mvt[edges[i].e],b)?1:0;return (std::uint64_t)a;});
    double base_spa = med([&](std::size_t i){std::uint32_t a=0;for(auto&b:edges[i].B)a+=R::template fkcc<rake>(sparse[edges[i].e],b)?1:0;return (std::uint64_t)a;});
    double fk = med([&](std::size_t i){float a=0;for(auto&b:edges[i].B){R::template sphere_fk<rake>(b,sp);a+=sp.x[0].to_array()[0];}return (std::uint64_t)a;});
    double emp= med([&](std::size_t i){std::uint32_t a=0;for(auto&b:edges[i].B)a+=R::template fkcc<rake>(empty,b)?1:0;return (std::uint64_t)a;});
    double envc=base_mvt-emp, self=emp-fk;
    double ideal = med([&](std::size_t i){for(auto&b:edges[i].B) if(not R::template fkcc_swept<rake>(mvt[edges[i].e],b,ecp[i],pcp[i])) return (std::uint64_t)0; return (std::uint64_t)1;});
    double real  = med([&](std::size_t i){std::array<bool,NBS> ec;std::array<bool,R::n_self_pairs> pc{};masks(edges[i],ec,pc);for(auto&b:edges[i].B) if(not R::template fkcc_swept<rake>(mvt[edges[i].e],b,ec,pc)) return (std::uint64_t)0; return (std::uint64_t)1;});
    double crs   = med([&](std::size_t i){std::array<bool,NBS> ec;std::array<bool,R::n_self_pairs> pc{};masks_coarse(edges[i],ec,pc);for(auto&b:edges[i].B) if(not R::template fkcc_swept<rake>(mvt[edges[i].e],b,ec,pc)) return (std::uint64_t)0; return (std::uint64_t)1;});

    std::printf("%-6s n=%zu NBS=%zu pts/scene=%.0f edges=%zu mism=%zu | base: sparse=%.0f  MVT=%.0f ns (%.1fx costlier)\n",
                name,n,NBS,(double)total_pts/mbm.size(),edges.size(),mm, base_spa, base_mvt, base_mvt/base_spa);
    std::printf("        MVT kernel: FK=%.0f self=%.0f ENV=%.0f  env_share=%.0f%%  scene-spec ceiling=%.2fx\n",
                fk, self, envc, 100.0*envc/base_mvt, base_mvt/(base_mvt-envc));
    std::printf("        swept vs base(MVT): IDEAL=%.0f (%.2fx)  REAL/fine=%.0f (%.2fx)  COARSE=%.0f (%.2fx)\n",
                ideal, base_mvt/ideal, real, base_mvt/real, crs, base_mvt/crs);
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, ur5_envs);
    run<vamp::robots::PandaE>("panda", 1.25f, panda_envs);
    run<vamp::robots::FetchE>("fetch", 1.0f, fetch_envs);
    run<vamp::robots::BaxterE>("baxter", 0.5f, baxter_envs);
    return 0;
}
