// Per-edge broadphase timing (Fetch, env-collision only). Compares, per edge:
//   baseline : FetchEnvbase::fkcc (VAMP's real fused env kernel: radial early-exit + gate)
//   ref_full : sphere_fk + full manual env check (same FK as flows -> isolates broadphase)
//   flowA    : FK all sub-configs, per-sphere swept AABB, prune, check pruned (exact)
//   flowB    : 3-pt swept bound + margin, prune, fused check w/ early-exit (fewer FK on
//              colliding edges; slight margin)
// Reports speedups and correctness (flow verdict vs ref_full, same FK), split free/colliding.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/collision/sphere_sphere.hh>
#include "fetch_base.hh"
#include "fetch_envbase.hh"

using RB = vamp::robots::FetchBase;
using RE = vamp::robots::FetchEnvbase;
constexpr std::size_t rake = vamp::FloatVectorWidth, ns = RB::n_spheres, dim = RB::dimension;
using DataV = vamp::FloatVector<rake>;
using EnvV = vamp::collision::Environment<DataV>;

namespace
{
    struct Obs { float x, y, z, r; };

    inline bool hit(const Obs &o, DataV sx, DataV sy, DataV sz, DataV sr)
    {
        return not vamp::collision::sphere_sphere_sql2(
                       static_cast<DataV>(o.x), static_cast<DataV>(o.y),
                       static_cast<DataV>(o.z), static_cast<DataV>(o.r), sx, sy, sz, sr).test_zero();
    }

    // build n_rakes interpolation blocks along edge start->start+v
    auto make_blocks(const std::array<float, dim> &s, const std::array<float, dim> &v, std::size_t nr)
        -> std::vector<RB::ConfigurationBlock<rake>>
    {
        std::vector<RB::ConfigurationBlock<rake>> B(nr);
        const float total = static_cast<float>(nr * rake);
        for (std::size_t r = 0; r < nr; ++r)
            for (std::size_t j = 0; j < dim; ++j)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l)
                {
                    float t = (static_cast<float>(r * rake + l) + 0.5F) / total;
                    ln[l] = s[j] + t * v[j];
                }
                B[r][j] = DataV(ln.data());
            }
        return B;
    }

    // ---- validators (return true = edge free) ----
    bool baseline(const EnvV &env, const std::vector<RB::ConfigurationBlock<rake>> &B)
    {
        for (const auto &b : B) if (not RE::fkcc<rake>(env, b)) return false;
        return true;
    }
    RB::Spheres<rake> sph;
    bool ref_full(const std::vector<Obs> &obs, const std::vector<RB::ConfigurationBlock<rake>> &B)
    {
        for (const auto &b : B)
        {
            RB::sphere_fk<rake>(b, sph);
            for (std::size_t s = 0; s < ns; ++s)
                for (const auto &o : obs)
                    if (hit(o, sph.x[s], sph.y[s], sph.z[s], sph.r[s])) return false;
        }
        return true;
    }

    std::vector<RB::Spheres<rake>> store;
    std::vector<std::vector<int>> pruned;  // per sphere
    bool flowA(const std::vector<Obs> &obs, const std::vector<RB::ConfigurationBlock<rake>> &B)
    {
        std::size_t nr = B.size();
        store.resize(nr);
        std::array<std::array<float, 3>, ns> mn, mx; std::array<float, ns> rr;
        for (std::size_t s = 0; s < ns; ++s) { mn[s] = {1e30F, 1e30F, 1e30F}; mx[s] = {-1e30F, -1e30F, -1e30F}; }
        for (std::size_t r = 0; r < nr; ++r)
        {
            RB::sphere_fk<rake>(B[r], store[r]);
            for (std::size_t s = 0; s < ns; ++s)
            {
                auto ax = store[r].x[s].to_array(); auto ay = store[r].y[s].to_array(); auto az = store[r].z[s].to_array();
                if (r == 0) rr[s] = store[r].r[s].to_array()[0];
                for (std::size_t l = 0; l < rake; ++l)
                {
                    mn[s][0] = std::min(mn[s][0], ax[l]); mx[s][0] = std::max(mx[s][0], ax[l]);
                    mn[s][1] = std::min(mn[s][1], ay[l]); mx[s][1] = std::max(mx[s][1], ay[l]);
                    mn[s][2] = std::min(mn[s][2], az[l]); mx[s][2] = std::max(mx[s][2], az[l]);
                }
            }
        }
        pruned.assign(ns, {});
        for (std::size_t s = 0; s < ns; ++s)
        {
            float lo0 = mn[s][0] - rr[s], hi0 = mx[s][0] + rr[s], lo1 = mn[s][1] - rr[s], hi1 = mx[s][1] + rr[s],
                  lo2 = mn[s][2] - rr[s], hi2 = mx[s][2] + rr[s];
            for (int i = 0; i < (int)obs.size(); ++i)
            {
                const auto &o = obs[i];
                if (o.x >= lo0 - o.r && o.x <= hi0 + o.r && o.y >= lo1 - o.r && o.y <= hi1 + o.r &&
                    o.z >= lo2 - o.r && o.z <= hi2 + o.r) pruned[s].push_back(i);
            }
        }
        for (std::size_t r = 0; r < nr; ++r)
            for (std::size_t s = 0; s < ns; ++s)
                for (int i : pruned[s])
                    if (hit(obs[i], store[r].x[s], store[r].y[s], store[r].z[s], store[r].r[s])) return false;
        return true;
    }

    bool flowB(const std::vector<Obs> &obs, const std::array<float, dim> &s0, const std::array<float, dim> &v,
               const std::vector<RB::ConfigurationBlock<rake>> &B, float margin)
    {
        // 3-pt bound (t=0,0.5,1) broadcast
        std::array<std::array<float, 3>, ns> mn, mx; std::array<float, ns> rr;
        for (std::size_t s = 0; s < ns; ++s) { mn[s] = {1e30F, 1e30F, 1e30F}; mx[s] = {-1e30F, -1e30F, -1e30F}; }
        for (float t : {0.0F, 0.5F, 1.0F})
        {
            RB::ConfigurationBlock<rake> b;
            for (std::size_t j = 0; j < dim; ++j) b[j] = static_cast<DataV>(s0[j] + t * v[j]);
            RB::sphere_fk<rake>(b, sph);
            for (std::size_t s = 0; s < ns; ++s)
            {
                float x = sph.x[s].to_array()[0], y = sph.y[s].to_array()[0], z = sph.z[s].to_array()[0];
                if (t == 0.0F) rr[s] = sph.r[s].to_array()[0];
                mn[s][0] = std::min(mn[s][0], x); mx[s][0] = std::max(mx[s][0], x);
                mn[s][1] = std::min(mn[s][1], y); mx[s][1] = std::max(mx[s][1], y);
                mn[s][2] = std::min(mn[s][2], z); mx[s][2] = std::max(mx[s][2], z);
            }
        }
        pruned.assign(ns, {});
        for (std::size_t s = 0; s < ns; ++s)
        {
            float m = rr[s] + margin;
            for (int i = 0; i < (int)obs.size(); ++i)
            {
                const auto &o = obs[i];
                if (o.x >= mn[s][0] - m - o.r && o.x <= mx[s][0] + m + o.r && o.y >= mn[s][1] - m - o.r &&
                    o.y <= mx[s][1] + m + o.r && o.z >= mn[s][2] - m - o.r && o.z <= mx[s][2] + m + o.r)
                    pruned[s].push_back(i);
            }
        }
        for (const auto &b : B)  // fused check w/ early-exit
        {
            RB::sphere_fk<rake>(b, sph);
            for (std::size_t s = 0; s < ns; ++s)
                for (int i : pruned[s])
                    if (hit(obs[i], sph.x[s], sph.y[s], sph.z[s], sph.r[s])) return false;
        }
        return true;
    }
}  // namespace

int main()
{
    std::mt19937 rng(0xED6E);
    const int N = 40;
    // front "shelf" region away from the static base/torso -> free configs exist
    std::uniform_real_distribution<float> sx(0.45F, 0.85F), sy(-0.35F, 0.35F), sz(0.45F, 1.0F), rad(0.02F, 0.05F);
    std::vector<Obs> obs(N);
    vamp::collision::Environment<float> ef;
    for (auto &o : obs) { o = {sx(rng), sy(rng), sz(rng), rad(rng)}; ef.spheres.emplace_back(vamp::collision::factory::sphere::array({o.x, o.y, o.z}, o.r)); }
    ef.sort(); EnvV env(ef);

    std::uniform_real_distribution<float> u(0.F, 1.F); std::normal_distribution<float> nd(0.F, 1.F);
    const float L = 1.0F; const std::size_t nr = 4;
    // generate edges, classify
    struct Edge { std::array<float, dim> s, v; std::vector<RB::ConfigurationBlock<rake>> B; bool free; };
    std::vector<Edge> edges;
    int attempts = 0;
    while (edges.size() < 4000 && attempts < 400000)
    {
        ++attempts;
        std::array<float, dim> s, v; RB::ConfigurationBlock<rake> tmp;
        for (std::size_t j = 0; j < dim; ++j) tmp[j] = static_cast<DataV>(u(rng));
        RB::scale_configuration_block<rake>(tmp);
        float nrm = 0; std::array<float, dim> d;
        for (std::size_t j = 0; j < dim; ++j) { s[j] = tmp[j].to_array()[0]; d[j] = nd(rng); nrm += d[j] * d[j]; }
        nrm = std::sqrt(nrm);
        for (std::size_t j = 0; j < dim; ++j) v[j] = d[j] / nrm * L;
        auto B = make_blocks(s, v, nr);
        bool fr = baseline(env, B);
        // keep a ~50/50 mix so both classes are populated
        if (fr || (edges.size() % 2 == 0)) edges.push_back({s, v, std::move(B), fr});
    }
    std::size_t nfree = 0; for (auto &e : edges) nfree += e.free;

    auto med = [&](auto &&fn, const std::vector<Edge> &es) -> double
    {
        std::vector<double> t; volatile std::uint64_t sink = 0;
        for (int r = 0; r < 7; ++r)
        {
            auto a = std::chrono::steady_clock::now();
            std::uint64_t acc = 0; for (const auto &e : es) acc += fn(e);
            auto b = std::chrono::steady_clock::now(); sink += acc;
            t.push_back(std::chrono::duration<double>(b - a).count() / es.size() * 1e9);
        }
        (void)sink; std::sort(t.begin(), t.end()); return t[t.size() / 2];
    };

    // correctness: flow verdict vs ref_full (same FK)
    std::size_t mmA = 0, mmB = 0;
    for (auto &e : edges) { bool rf = ref_full(obs, e.B); if (flowA(obs, e.B) != rf) ++mmA; if (flowB(obs, e.s, e.v, e.B, 0.05F) != rf) ++mmB; }

    auto run_subset = [&](const char *lab, bool wantfree, bool both)
    {
        std::vector<Edge> sub;
        for (auto &e : edges) if (both || e.free == wantfree) { sub.push_back(Edge{e.s, e.v, e.B, e.free}); }
        if (sub.empty()) { std::printf("%s: (none)\n", lab); return; }
        double tb = med([&](const Edge &e){ return baseline(env, e.B) ? 1U : 0U; }, sub);
        double tr = med([&](const Edge &e){ return ref_full(obs, e.B) ? 1U : 0U; }, sub);
        double ta = med([&](const Edge &e){ return flowA(obs, e.B) ? 1U : 0U; }, sub);
        double tB = med([&](const Edge &e){ return flowB(obs, e.s, e.v, e.B, 0.05F) ? 1U : 0U; }, sub);
        std::printf("%-9s (%4zu edges): baseline=%.0f ref_full=%.0f flowA=%.0f flowB=%.0f ns | "
                    "A vs baseline=%.2fx  B vs baseline=%.2fx  A vs ref_full=%.2fx\n",
                    lab, sub.size(), tb, tr, ta, tB, tb / ta, tb / tB, tr / ta);
    };
    std::printf("Fetch env-only, N=%d obstacles, edge L=%.1f, %zu rakes; %zu/%zu free. mismatches A=%zu B=%zu\n",
                N, L, nr, nfree, edges.size(), mmA, mmB);
    run_subset("all", false, true);
    run_subset("free", true, false);
    run_subset("colliding", false, false);
    return 0;
}
