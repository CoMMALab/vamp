// FK-side ideas on real MBM environments: fused sincos vs baseline, and vs / combined-with the
// leaf-trig recurrence. Every method drives the same fkcc collision check; only how the per-joint
// sin/cos are produced differs. Verdicts checked vs baseline; time is ns/edge, free/colliding.
//   base          fkcc (separate sin+cos internally, every rake)
//   sincos        fused sincos -> fkcc_pretrig, every rake
//   recur         leaf-trig recurrence -> fkcc_pretrig (trig only at rake 0, separate sin/cos)
//   recur+sincos  recurrence, with the rake-0 seed via fused sincos
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
using V1 = vamp::FloatVector<rake, 1>;

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
static void run(const char *name, float range, std::size_t jlo, std::size_t jhi, const std::vector<MbmEnv> &mbm)
{
    constexpr std::size_t dim = R::dimension;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake)), N = n * rake;
    using Block = typename R::template ConfigurationBlock<rake>;
    std::vector<vamp::collision::Environment<DataV>> envs;
    for (const auto &me : mbm) envs.push_back(build_env(me));

    struct Edge { int env; std::vector<Block> B; std::array<float, dim> v; float dt; bool free; };
    std::vector<Edge> edges;
    auto valid_base = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc<rake>(envs[e.env], b)) return false; return true; };
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
            Edge e{(int)ei, std::move(B), v, dt, false}; e.free = valid_base(e); edges.push_back(std::move(e)); ++got;
        }
    }

    auto valid_sincos = [&](const Edge &e) { std::array<V1, dim> ps, pc;
        for (auto &b : e.B) { for (std::size_t j = jlo; j < jhi; ++j) b[j].sincos(ps[j], pc[j]);
            if (not R::template fkcc_pretrig<rake>(envs[e.env], b, ps, pc)) return false; } return true; };
    auto valid_recur = [&](const Edge &e) { std::array<V1, dim> ps{}, pc{}; std::array<float, dim> C8, S8;
        for (std::size_t j = jlo; j < jhi; ++j) { float d8 = rake * e.dt * e.v[j]; C8[j] = std::cos(d8); S8[j] = std::sin(d8); }
        for (std::size_t r = 0; r < e.B.size(); ++r) {
            if (r == 0) for (std::size_t j = jlo; j < jhi; ++j) { ps[j] = e.B[0][j].sin(); pc[j] = e.B[0][j].cos(); }
            else for (std::size_t j = jlo; j < jhi; ++j) { V1 c = V1::fill(C8[j]), s = V1::fill(S8[j]); V1 sn = ps[j] * c + pc[j] * s, cn = pc[j] * c - ps[j] * s; ps[j] = sn; pc[j] = cn; }
            if (not R::template fkcc_pretrig<rake>(envs[e.env], e.B[r], ps, pc)) return false; }
        return true; };
    auto valid_recur_sc = [&](const Edge &e) { std::array<V1, dim> ps{}, pc{}; std::array<float, dim> C8, S8;
        for (std::size_t j = jlo; j < jhi; ++j) { float d8 = rake * e.dt * e.v[j]; C8[j] = std::cos(d8); S8[j] = std::sin(d8); }
        for (std::size_t r = 0; r < e.B.size(); ++r) {
            if (r == 0) for (std::size_t j = jlo; j < jhi; ++j) e.B[0][j].sincos(ps[j], pc[j]);
            else for (std::size_t j = jlo; j < jhi; ++j) { V1 c = V1::fill(C8[j]), s = V1::fill(S8[j]); V1 sn = ps[j] * c + pc[j] * s, cn = pc[j] * c - ps[j] * s; ps[j] = sn; pc[j] = cn; }
            if (not R::template fkcc_pretrig<rake>(envs[e.env], e.B[r], ps, pc)) return false; }
        return true; };

    auto valid_native = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc_sincos<rake>(envs[e.env], b)) return false; return true; };

    std::size_t nfree = 0; std::array<std::size_t, 4> mm{};
    for (auto &e : edges) { bool bv = valid_base(e); nfree += e.free;
        if (valid_sincos(e) != bv) mm[0]++; if (valid_recur(e) != bv) mm[1]++; if (valid_recur_sc(e) != bv) mm[2]++; if (valid_native(e) != bv) mm[3]++; }

    auto med = [&](auto fn, int mode) { std::vector<double> t; volatile std::uint64_t sk = 0; std::vector<Edge *> sub;
        for (auto &e : edges) if (mode == 0 || (mode == 1 && e.free) || (mode == 2 && not e.free)) sub.push_back(&e); if (sub.empty()) return 0.0;
        for (int rp = 0; rp < 7; ++rp) { auto a = std::chrono::steady_clock::now(); std::uint64_t ac = 0; for (auto *e : sub) ac += fn(*e); auto z = std::chrono::steady_clock::now(); sk += ac;
            t.push_back(std::chrono::duration<double>(z - a).count() / sub.size() * 1e9); } (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2]; };
    auto B0 = [&](const Edge &e) { return valid_base(e) ? 1U : 0U; };
    auto Nv = [&](const Edge &e) { return valid_native(e) ? 1U : 0U; };
    auto Sx = [&](const Edge &e) { return valid_sincos(e) ? 1U : 0U; };
    auto Rc = [&](const Edge &e) { return valid_recur(e) ? 1U : 0U; };
    auto Rs = [&](const Edge &e) { return valid_recur_sc(e) ? 1U : 0U; };

    std::printf("\n== %-6s ==  dim=%zu n=%zu revolute=[%zu,%zu)  edges=%zu (free %zu)  mism sincos/recur/recur+sc/native = %zu/%zu/%zu/%zu\n",
                name, dim, n, jlo, jhi, edges.size(), nfree, mm[0], mm[1], mm[2], mm[3]);
    const char *lbls[3] = {"all", "free", "colliding"};
    std::printf("   %-9s | %8s %10s %8s %8s %10s\n", "edges", "base", "fkcc_sincos", "sincos*", "recur", "recur+sc");
    for (int m = 0; m < 3; ++m) {
        double bb = med(B0, m), nv = med(Nv, m), sx = med(Sx, m), rc = med(Rc, m), rs = med(Rs, m);
        std::printf("   %-9s | %8.0f %10.0f %8.0f %8.0f %10.0f\n", lbls[m], bb, nv, sx, rc, rs);
        std::printf("   %-9s | %8s %9.2fx %7.2fx %7.2fx %9.2fx\n", "", "1.00x", bb / nv, bb / sx, bb / rc, bb / rs);
    }
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, 0, 6, ur5_envs);
    run<vamp::robots::PandaE>("panda", 1.25f, 0, 7, panda_envs);
    run<vamp::robots::FetchE>("fetch", 1.0f, 1, 8, fetch_envs);
    run<vamp::robots::BaxterE>("baxter", 0.5f, 0, 14, baxter_envs);
    return 0;
}
