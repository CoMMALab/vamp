// Edge-continuity FK integrated: real sphere_fk with the leaf trig hoisted + recurred
// across rakes. stock (full sphere_fk per rake) vs recur (rake-0 trig full, recur the
// rest, sphere_fk_pretrig per rake). Verify sphere positions match; time whole-edge FK.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
    const float L = 1.0f;

    for (std::size_t n : {std::size_t(2), std::size_t(4), std::size_t(8)})
    {
        // random edge start + direction (scaled joint space)
        R::ConfigurationBlock<rake> s0;
        for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
        R::scale_configuration_block<rake>(s0);
        std::array<float, dim> start, v; float nrm = 0;
        for (std::size_t j = 0; j < dim; ++j) { start[j] = s0[j].to_array()[0]; v[j] = nd(rng); nrm += v[j] * v[j]; }
        nrm = std::sqrt(nrm);
        for (std::size_t j = 0; j < dim; ++j) v[j] *= L / nrm;
        float dt = 1.0f / static_cast<float>(n * rake);

        // pre-build blocks (interpolation) for both methods
        std::vector<R::ConfigurationBlock<rake>> B(n);
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t j = 0; j < dim; ++j)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = start[j] + static_cast<float>(r * rake + l) * dt * v[j];
                B[r][j] = DataV(ln.data());
            }
        // recurrence constants: joints 1..7 (ps/pc index j-1)
        std::array<float, 7> C8, S8;
        for (std::size_t j = 1; j <= 7; ++j) { float d8 = static_cast<float>(rake) * dt * v[j]; C8[j - 1] = std::cos(d8); S8[j - 1] = std::sin(d8); }

        // verify: run both, compare sphere positions
        R::Spheres<rake> o_stock, o_recur; float maxerr = 0;
        std::array<DataV, 7> ps, pc;
        for (std::size_t r = 0; r < n; ++r)
        {
            R::sphere_fk<rake>(B[r], o_stock);
            if (r == 0) for (std::size_t j = 0; j < 7; ++j) { ps[j] = ::sin(B[0][j + 1]); pc[j] = ::cos(B[0][j + 1]); }
            else for (std::size_t j = 0; j < 7; ++j) { DataV c8 = DataV::fill(C8[j]), s8 = DataV::fill(S8[j]); DataV sn = ps[j]*c8+pc[j]*s8, cn = pc[j]*c8-ps[j]*s8; ps[j]=sn; pc[j]=cn; }
            R::sphere_fk_pretrig<rake>(B[r], ps, pc, o_recur);
            for (std::size_t sp = 0; sp < ns; ++sp)
            {
                auto ex = (o_stock.x[sp]-o_recur.x[sp]).to_array(); auto ey=(o_stock.y[sp]-o_recur.y[sp]).to_array(); auto ez=(o_stock.z[sp]-o_recur.z[sp]).to_array();
                for (std::size_t l=0;l<rake;++l) maxerr=std::max({maxerr,std::abs(ex[l]),std::abs(ey[l]),std::abs(ez[l])});
            }
        }

        constexpr int K = 100000;
        auto med = [&](auto fn) { std::vector<double> t; volatile float sink=0;
            for (int rep=0;rep<7;++rep){ auto a=std::chrono::steady_clock::now(); float acc=fn(); auto z=std::chrono::steady_clock::now(); sink+=acc;
            t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);} (void)sink; std::sort(t.begin(),t.end()); return t[t.size()/2]; };

        R::Spheres<rake> o;
        double t_stock = med([&]{ float acc=0; for(int k=0;k<K;++k){ for(std::size_t r=0;r<n;++r){ R::sphere_fk<rake>(B[r],o); acc+=o.x[0].to_array()[0]; } } return acc; });
        double t_recur = med([&]{ float acc=0; for(int k=0;k<K;++k){ std::array<DataV,7> lps,lpc;
            for(std::size_t r=0;r<n;++r){ if(r==0){ for(std::size_t j=0;j<7;++j){lps[j]=::sin(B[0][j+1]);lpc[j]=::cos(B[0][j+1]);} }
                else { for(std::size_t j=0;j<7;++j){DataV c8=DataV::fill(C8[j]),s8=DataV::fill(S8[j]);DataV sn=lps[j]*c8+lpc[j]*s8,cn=lpc[j]*c8-lps[j]*s8;lps[j]=sn;lpc[j]=cn;} }
                R::sphere_fk_pretrig<rake>(B[r],lps,lpc,o); acc+=o.x[0].to_array()[0]; } } return acc; });

        std::printf("n_rakes=%zu: stock FK/edge=%.1f ns  recur FK/edge=%.1f ns  speedup=%.2fx  max_pos_err=%.1e\n",
                    n, t_stock, t_recur, t_stock / t_recur, maxerr);
    }
    return 0;
}
