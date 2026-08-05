// Is the per-sphere recurrence error small enough to absorb as radius inflation?
// The recurrence p_{k+1}=a(p_k-p_{k-1})+p_{k-2} is exact for DC+sinusoid; error comes
// from (a) imperfect alpha and (b) >order-3 content. Measure the accumulated position
// error (=required radius inflation) for: crude alpha (4-pt), and best-fit alpha
// (least-squares over the edge = the floor if alpha were known well). Also vs #seeds.
#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "fetch_base.hh"

using R = vamp::robots::FetchBase;
constexpr std::size_t rake = vamp::FloatVectorWidth, dim = R::dimension, ns = R::n_spheres;
using DataV = vamp::FloatVector<rake>;

int main()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(0.f, 1.f), nd(-1.f, 1.f);
    for (float L : {0.5f, 1.0f, 1.5f})
    {
        const std::size_t n = 12;
        R::ConfigurationBlock<rake> s0;
        for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
        R::scale_configuration_block<rake>(s0);
        std::array<float, dim> st, v; float nr = 0;
        for (std::size_t j = 0; j < dim; ++j) { st[j] = s0[j].to_array()[0]; v[j] = nd(rng); nr += v[j] * v[j]; }
        nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) v[j] *= L / nr;
        float dt = 1.0f / static_cast<float>(n * rake);
        std::vector<R::Spheres<rake>> P(n);
        for (std::size_t r = 0; r < n; ++r) {
            R::ConfigurationBlock<rake> b;
            for (std::size_t j = 0; j < dim; ++j) { alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = st[j] + static_cast<float>(r * rake + l) * dt * v[j]; b[j] = DataV(ln.data()); }
            R::sphere_fk<rake>(b, P[r]);
        }
        auto co = [&](const R::Spheres<rake>&S, std::size_t s, std::size_t c){ return (c==0?S.x[s]:c==1?S.y[s]:S.z[s]); };

        // alpha estimators (lane 0, best-varying coord)
        auto alpha_from = [&](std::size_t s, std::size_t kseed, bool lsq){
            // pick best coord by variation over seeds
            std::size_t bc=0; float bv=0;
            for (std::size_t c=0;c<3;++c){ float mn=1e30,mx=-1e30; for(std::size_t r=0;r<kseed;++r){float x=co(P[r],s,c).to_array()[0];mn=std::min(mn,x);mx=std::max(mx,x);} if(mx-mn>bv){bv=mx-mn;bc=c;} }
            if (bv<1e-6f) return 2.0f;
            if (!lsq){ // 4-pt: a=(p3-p0)/(p2-p1)
                float p0=co(P[0],s,bc).to_array()[0],p1=co(P[1],s,bc).to_array()[0],p2=co(P[2],s,bc).to_array()[0],p3=co(P[3],s,bc).to_array()[0];
                return (p3-p0)/(p2-p1);
            }
            // least squares over kseed: a = sum((p_{k+1}-p_{k-2})(p_k-p_{k-1}))/sum((p_k-p_{k-1})^2)
            double num=0,den=0; for(std::size_t r=3;r<kseed;++r){float pk1=co(P[r],s,bc).to_array()[0],pk=co(P[r-1],s,bc).to_array()[0],pkm1=co(P[r-2],s,bc).to_array()[0],pkm2=co(P[r-3],s,bc).to_array()[0];
                double d=(pk-pkm1); num+=(double)(pk1-pkm2)*d; den+=d*d; } return den>1e-9?(float)(num/den):2.0f;
        };
        auto run_err = [&](std::size_t kseed, bool lsq){
            float maxerr=0;
            for (std::size_t s=0;s<ns;++s){ float a=alpha_from(s,kseed,lsq);
                std::array<std::array<float,3>,3> h; // last 3 positions per coord, lane 0... use full DataV
                std::array<DataV,3> hx,hy,hz;
                for(std::size_t r=0;r<3;++r){hx[r]=P[r].x[s];hy[r]=P[r].y[s];hz[r]=P[r].z[s];}
                DataV A=DataV::fill(a);
                for(std::size_t r=3;r<n;++r){ DataV nx=A*(hx[2]-hx[1])+hx[0],ny=A*(hy[2]-hy[1])+hy[0],nz=A*(hz[2]-hz[1])+hz[0];
                    auto ex=(nx-P[r].x[s]).to_array();auto ey=(ny-P[r].y[s]).to_array();auto ez=(nz-P[r].z[s]).to_array();
                    for(std::size_t l=0;l<rake;++l) maxerr=std::max({maxerr,std::abs(ex[l]),std::abs(ey[l]),std::abs(ez[l])});
                    hx={hx[1],hx[2],nx};hy={hy[1],hy[2],ny};hz={hz[1],hz[2],nz}; }
            }
            return maxerr;
        };
        std::printf("L=%.2f n=%zu: infl(=max pos err, m) | crude-4pt=%.1e | lsq-seed6=%.1e | lsq-seed8=%.1e | best(lsq-all)=%.1e\n",
                    L, n, run_err(4,false), run_err(6,true), run_err(8,true), run_err(n,true));
    }
    return 0;
}
