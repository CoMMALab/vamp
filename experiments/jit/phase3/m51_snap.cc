// Model-noise snap (idea #1): sub-1e-9 joint-placement entries snapped to exact 0 in the FK
// trace, sparsifying every joint transform. This bench is header-agnostic: it just times the
// fused fkcc kernel on real MBM edges (free + colliding) and prints a verdict fingerprint.
// Run it against the SNAPPED cricket build and the baseline build; the fingerprint must match
// (correctness) and the time delta is the win. dim/n/edges identical across builds by construction.
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include "ur5_e.hh"
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "mbm_envs.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

static vamp::collision::Environment<DataV> build_env(const MbmEnv &me)
{
    vamp::collision::Environment<float> ef;
    for (const auto &p : me.prims) {
        if (p.kind == 0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px, p.py, p.pz, p.a));
        else if (p.kind == 1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b));
        else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b, p.c));
    }
    ef.sort();
    return vamp::collision::Environment<DataV>(ef);
}

template <class R>
static void run(const char *name, float range, const std::vector<MbmEnv> &mbm)
{
    constexpr std::size_t dim = R::dimension;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake)), N = n * rake;
    using Block = typename R::template ConfigurationBlock<rake>;
    std::vector<vamp::collision::Environment<DataV>> envs;
    for (const auto &me : mbm) envs.push_back(build_env(me));

    struct Edge { int env; std::vector<Block> B; bool free; };
    std::vector<Edge> edges;
    auto valid = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc<rake>(envs[e.env], b)) return false; return true; };
    std::mt19937 wr(0x33); std::uniform_real_distribution<float> u(0.f, 1.f); std::normal_distribution<float> nd(0.f, 1.f);
    for (std::size_t ei = 0; ei < envs.size(); ++ei) {
        std::size_t got = 0; int att = 0; const std::size_t PER = 320;
        while (got < PER && att < 40000) {
            ++att; Block tmp; for (std::size_t j = 0; j < dim; ++j) tmp[j] = static_cast<DataV>(u(wr)); R::template scale_configuration_block<rake>(tmp);
            std::array<float, dim> a, v; float nr = 0; for (std::size_t j = 0; j < dim; ++j) { a[j] = tmp[j].to_array()[0]; v[j] = nd(wr); nr += v[j] * v[j]; }
            nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) v[j] *= range / nr;
            float dt = 1.0f / (float)N; std::vector<Block> B(n);
            for (std::size_t r = 0; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) { alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = a[j] + (float)(r * rake + l) * dt * v[j]; B[r][j] = DataV(ln.data()); }
            Edge e{(int)ei, std::move(B), false}; e.free = valid(e); edges.push_back(std::move(e)); ++got;
        }
    }

    // verdict fingerprint over all edges (FNV-1a over free/collide bits) + collision count
    std::uint64_t h = 1469598103934665603ULL; std::size_t nfree = 0;
    for (auto &e : edges) { std::uint8_t b = e.free ? 1 : 0; nfree += b; h = (h ^ b) * 1099511628211ULL; }

    auto sincos_valid = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc_sincos<rake>(envs[e.env], b)) return false; return true; };
    auto med = [&](auto fn) { std::vector<double> t; volatile std::uint64_t sk = 0;
        for (int rp = 0; rp < 9; ++rp) { auto a = std::chrono::steady_clock::now(); std::uint64_t ac = 0; for (auto &e : edges) ac += fn(e); auto z = std::chrono::steady_clock::now(); sk += ac;
            t.push_back(std::chrono::duration<double>(z - a).count() / edges.size() * 1e9); } (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2]; };

    double fk = med([&](const Edge &e){ return valid(e); }), sc = med([&](const Edge &e){ return sincos_valid(e); });
    std::printf("%-6s dim=%2zu n=%zu edges=%zu (free %zu)  fingerprint=%016llx  ns/edge: fkcc=%.0f  fkcc_sincos=%.0f\n",
                name, dim, n, edges.size(), nfree, (unsigned long long)h, fk, sc);
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, ur5_envs);
    run<vamp::robots::PandaE>("panda", 1.25f, panda_envs);
    run<vamp::robots::FetchE>("fetch", 1.0f, fetch_envs);
    run<vamp::robots::BaxterE>("baxter", 0.5f, baxter_envs);
    return 0;
}
