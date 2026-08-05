// End-to-end edge validation: full fused FK+CC per rake (VAMP's fkcc) vs the same edge
// validated with the leaf-trig recurrence (rake-0 sin/cos, advance per rake, fkcc_pretrig).
// Verifies identical edge verdicts; times the whole edge validation, split free/colliding.
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

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

template <class R>
static auto shelf(std::mt19937 &rng, int nobs)
{
    vamp::collision::Environment<float> ef;
    std::uniform_real_distribution<float> sx(0.45F, 0.85F), sy(-0.35F, 0.35F), sz(0.45F, 1.0F), rad(0.02F, 0.05F);
    for (int i = 0; i < nobs; ++i) ef.spheres.emplace_back(vamp::collision::factory::sphere::array({sx(rng), sy(rng), sz(rng)}, rad(rng)));
    ef.sort();
    return vamp::collision::Environment<DataV>(ef);
}

template <class R>
static void run(const char *name, std::size_t jlo, std::size_t jhi, float range, int nobs)
{
    constexpr std::size_t dim = R::dimension;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake));
    std::mt19937 srng(0xBEEF), wrng(0x33);
    auto env = shelf<R>(srng, nobs);
    std::uniform_real_distribution<float> u(0.f, 1.f); std::normal_distribution<float> nd(0.f, 1.f);

    struct Edge { std::vector<typename R::template ConfigurationBlock<rake>> B; std::array<float, dim> C8, S8; bool free; };
    std::vector<Edge> edges; int attempts = 0;
    auto build_blocks = [&](const std::array<float,dim>&s, const std::array<float,dim>&v, float dt){
        std::vector<typename R::template ConfigurationBlock<rake>> B(n);
        for (std::size_t r=0;r<n;++r) for(std::size_t j=0;j<dim;++j){alignas(vamp::FloatVectorAlignment)std::array<float,rake> ln;
            for(std::size_t l=0;l<rake;++l) ln[l]=s[j]+static_cast<float>(r*rake+l)*dt*v[j]; B[r][j]=DataV(ln.data());}
        return B;
    };
    auto base_valid = [&](const std::vector<typename R::template ConfigurationBlock<rake>>&B){
        for (auto &b : B) if (not R::template fkcc<rake>(env, b)) return false; return true; };
    auto recur_valid = [&](const Edge &e){
        std::array<DataV, dim> ps{}, pc{};
        for (std::size_t r = 0; r < e.B.size(); ++r) {
            if (r == 0) for (std::size_t j=jlo;j<jhi;++j){ ps[j]=::sin(e.B[0][j]); pc[j]=::cos(e.B[0][j]); }
            else for (std::size_t j=jlo;j<jhi;++j){ DataV c=DataV::fill(e.C8[j]),s=DataV::fill(e.S8[j]); DataV sn=ps[j]*c+pc[j]*s,cn=pc[j]*c-ps[j]*s; ps[j]=sn; pc[j]=cn; }
            if (not R::template fkcc_pretrig<rake>(env, e.B[r], ps, pc)) return false;
        }
        return true;
    };

    while (edges.size() < 3000 && attempts < 300000) {
        ++attempts;
        typename R::template ConfigurationBlock<rake> tmp;
        for (std::size_t j=0;j<dim;++j) tmp[j]=static_cast<DataV>(u(wrng));
        R::template scale_configuration_block<rake>(tmp);
        std::array<float,dim> s,v; float nrm=0;
        for (std::size_t j=0;j<dim;++j){ s[j]=tmp[j].to_array()[0]; v[j]=nd(wrng); nrm+=v[j]*v[j]; }
        nrm=std::sqrt(nrm); for(std::size_t j=0;j<dim;++j) v[j]*=range/nrm;
        float dt=1.0f/static_cast<float>(n*rake);
        Edge e; e.B=build_blocks(s,v,dt);
        for(std::size_t j=jlo;j<jhi;++j){ float d8=rake*dt*v[j]; e.C8[j]=std::cos(d8); e.S8[j]=std::sin(d8); }
        e.free=base_valid(e.B);
        if (e.free || edges.size()%2==0) edges.push_back(std::move(e));
    }
    // verify verdicts match
    std::size_t mm=0, nfree=0; for (auto &e : edges){ if (base_valid(e.B)!=recur_valid(e)) ++mm; nfree+=e.free; }

    auto med=[&](auto fn, bool wantfree, bool both){ std::vector<double> t; volatile std::uint64_t sk=0; std::size_t cnt=0;
        std::vector<Edge*> sub; for(auto&e:edges) if(both||e.free==wantfree) sub.push_back(&e); cnt=sub.size(); if(!cnt)return 0.0;
        for(int rp=0;rp<7;++rp){auto a=std::chrono::steady_clock::now();std::uint64_t ac=0;for(auto*e:sub)ac+=fn(*e);auto z=std::chrono::steady_clock::now();sk+=ac;
            t.push_back(std::chrono::duration<double>(z-a).count()/cnt*1e9);}(void)sk;std::sort(t.begin(),t.end());return t[t.size()/2];};
    auto B=[&](const Edge&e){return base_valid(e.B)?1U:0U;}; auto Rc=[&](const Edge&e){return recur_valid(e)?1U:0U;};
    std::printf("%-6s dim=%zu n=%zu obs=%d free=%zu/%zu mm=%zu\n", name, dim, n, nobs, nfree, edges.size(), mm);
    std::printf("   all      : fkcc=%.0f recur=%.0f ns  speedup=%.2fx\n", med(B,false,true), med(Rc,false,true), med(B,false,true)/med(Rc,false,true));
    std::printf("   free     : fkcc=%.0f recur=%.0f ns  speedup=%.2fx\n", med(B,true,false), med(Rc,true,false), med(B,true,false)/med(Rc,true,false));
    std::printf("   colliding: fkcc=%.0f recur=%.0f ns  speedup=%.2fx\n", med(B,false,false), med(Rc,false,false), med(B,false,false)/med(Rc,false,false));
}

int main()
{
    run<vamp::robots::PandaE>("panda", 0, 7, 1.25f, 40);
    run<vamp::robots::FetchE>("fetch", 1, 8, 1.0f, 40);
    return 0;
}
