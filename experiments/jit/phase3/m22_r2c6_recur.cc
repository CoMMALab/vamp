// r2c6 (36-DoF, 211 spheres, 29 revolute joints -> 58 leaf trig). Leaf-trig
// recurrence FK win, and the data to judge per-sphere/per-link vs shared FK.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include "r2c6_pt.hh"

using R = vamp::robots::R2c6;
constexpr std::size_t rake = vamp::FloatVectorWidth, dim = R::dimension, ns = R::n_spheres;
using DataV = vamp::FloatVector<rake>;
constexpr std::size_t J0 = 7, J1 = 36;  // revolute joints [7,35]

int main()
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<float> u(0.f, 1.f), nd(-1.f, 1.f);
    const float L = 1.0f;

    for (std::size_t n : {std::size_t(2), std::size_t(4), std::size_t(8)})
    {
        R::ConfigurationBlock<rake> s0;
        for (std::size_t j = 0; j < dim; ++j) s0[j] = static_cast<DataV>(u(rng));
        R::scale_configuration_block<rake>(s0);
        std::array<float, dim> start, v; float nrm = 0;
        for (std::size_t j = 0; j < dim; ++j) { start[j] = s0[j].to_array()[0]; v[j] = nd(rng); nrm += v[j] * v[j]; }
        nrm = std::sqrt(nrm);
        for (std::size_t j = 0; j < dim; ++j) v[j] *= L / nrm;
        float dt = 1.0f / static_cast<float>(n * rake);

        std::vector<R::ConfigurationBlock<rake>> B(n);
        for (std::size_t r = 0; r < n; ++r)
            for (std::size_t j = 0; j < dim; ++j) {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = start[j] + static_cast<float>(r * rake + l) * dt * v[j];
                B[r][j] = DataV(ln.data());
            }
        std::array<float, dim> C8{}, S8{};
        for (std::size_t j = J0; j < J1; ++j) { float d8 = static_cast<float>(rake) * dt * v[j]; C8[j] = std::cos(d8); S8[j] = std::sin(d8); }

        // verify
        R::Spheres<rake> o_s, o_r; std::array<DataV, 36> ps{}, pc{}; float maxerr = 0;
        for (std::size_t r = 0; r < n; ++r) {
            R::sphere_fk<rake>(B[r], o_s);
            if (r == 0) for (std::size_t j = J0; j < J1; ++j) { ps[j] = ::sin(B[0][j]); pc[j] = ::cos(B[0][j]); }
            else for (std::size_t j = J0; j < J1; ++j) { DataV c8 = DataV::fill(C8[j]), s8 = DataV::fill(S8[j]); DataV sn = ps[j]*c8+pc[j]*s8, cn = pc[j]*c8-ps[j]*s8; ps[j]=sn; pc[j]=cn; }
            R::sphere_fk_pretrig<rake>(B[r], ps, pc, o_r);
            for (std::size_t sp = 0; sp < ns; ++sp) { auto ex=(o_s.x[sp]-o_r.x[sp]).to_array(); for (std::size_t l=0;l<rake;++l) maxerr=std::max(maxerr,std::abs(ex[l])); }
        }

        constexpr int K = 60000;
        auto med = [&](auto fn) { std::vector<double> t; volatile float sink=0;
            for (int rep=0;rep<7;++rep){ auto a=std::chrono::steady_clock::now(); float acc=fn(); auto z=std::chrono::steady_clock::now(); sink+=acc;
            t.push_back(std::chrono::duration<double>(z-a).count()/K*1e9);} (void)sink; std::sort(t.begin(),t.end()); return t[t.size()/2]; };
        R::Spheres<rake> o;
        double t_stock = med([&]{ float acc=0; for(int k=0;k<K;++k) for(std::size_t r=0;r<n;++r){ R::sphere_fk<rake>(B[r],o); acc+=o.x[0].to_array()[0]; } return acc; });
        double t_recur = med([&]{ float acc=0; for(int k=0;k<K;++k){ std::array<DataV,36> lps{},lpc{};
            for(std::size_t r=0;r<n;++r){ if(r==0){ for(std::size_t j=J0;j<J1;++j){lps[j]=::sin(B[0][j]);lpc[j]=::cos(B[0][j]);} }
                else { for(std::size_t j=J0;j<J1;++j){DataV c8=DataV::fill(C8[j]),s8=DataV::fill(S8[j]);DataV sn=lps[j]*c8+lpc[j]*s8,cn=lpc[j]*c8-lps[j]*s8;lps[j]=sn;lpc[j]=cn;} }
                R::sphere_fk_pretrig<rake>(B[r],lps,lpc,o); acc+=o.x[0].to_array()[0]; } } return acc; });
        std::printf("r2c6 n_rakes=%zu: stock FK=%.1f ns  recur FK=%.1f ns  speedup=%.2fx  max_err=%.1e\n",
                    n, t_stock, t_recur, t_stock / t_recur, maxerr);
    }
    return 0;
}
