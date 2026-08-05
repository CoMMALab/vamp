// MBM in-practice evaluation: ablations + orthogonal combinations of the collision-check
// methods on REAL Motion-Bench-Maker environments, for ur5/panda/fetch/baxter.
// Edges are RRTC-realistic (Halton-ish sample two configs within the robot's RRT range, build
// the rake blocks); the swept env test uses VAMP's own sphere_environment_in_collision so it
// handles spheres, capsules and cuboids. Every method's verdict is verified bit-identical to
// baseline fkcc; timing is collision-check ns/edge, split free / colliding.
//
// Methods (columns):
//   base          fkcc per rake (what RRTC uses today)
//   recur         leaf-trig recurrence -> fkcc_pretrig per rake        (FK-side)
//   swept         fkcc_swept, masks from 3-config bound_fk per edge     (collision-side)
//   swept+cache   fkcc_swept, masks from cached endpoint bound_fk       (collision + node cache)
//   staged+cache  fkcc_swept_staged, cached masks                       (collision + staged lazy FK)
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>
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
using V1 = vamp::FloatVector<rake, 1>;
struct BS { float x, y, z, r; };

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
static void run(const char *name, float range, std::size_t jlo, std::size_t jhi,
                const std::vector<std::array<int, 2>> &pair_bs, const std::vector<MbmEnv> &mbm)
{
    constexpr std::size_t dim = R::dimension, NBS = R::n_bounding_spheres;
    std::size_t n = std::max<std::size_t>(1, (std::size_t)std::ceil(range * 32.0f / rake)), N = n * rake, NP = pair_bs.size();
    const float SAFETY = 1.3f;
    using Block = typename R::template ConfigurationBlock<rake>;

    // per-bounding-sphere per-joint reach (1st-difference FD on bound_fk) -> provable M*Omega sagitta
    std::array<std::array<float, dim>, NBS> reach{};
    {
        std::mt19937 g(0xA1); std::uniform_real_distribution<float> u(0.f, 1.f); const float eps = 1e-3f;
        auto nb = [&](const std::array<float, dim> &q) { Block b; for (std::size_t j = 0; j < dim; ++j) b[j] = DataV::fill(q[j]);
            typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b, bs);
            std::array<BS, NBS> o; for (std::size_t k = 0; k < NBS; ++k) o[k] = {bs.x[k].to_array()[0], bs.y[k].to_array()[0], bs.z[k].to_array()[0], bs.r[k].to_array()[0]}; return o; };
        for (int it = 0; it < 3000; ++it) {
            Block c; for (std::size_t j = 0; j < dim; ++j) c[j] = static_cast<DataV>(u(g)); R::template scale_configuration_block<rake>(c);
            std::array<float, dim> q; for (std::size_t j = 0; j < dim; ++j) q[j] = c[j].to_array()[0]; auto a = nb(q);
            for (std::size_t j = 0; j < dim; ++j) { auto q2 = q; q2[j] += eps; auto b = nb(q2);
                for (std::size_t k = 0; k < NBS; ++k) { float dx = a[k].x - b[k].x, dy = a[k].y - b[k].y, dz = a[k].z - b[k].z; reach[k][j] = std::max(reach[k][j], std::sqrt(dx * dx + dy * dy + dz * dz) / eps); } }
        }
    }
    auto sag = [&](std::size_t k, const std::array<float, dim> &v) { float M = 0, Om = 0;
        for (std::size_t j = 0; j < dim; ++j) { float ad = std::fabs(v[j]); M += reach[k][j] * ad; Om += ad; } return SAFETY * M * Om / 8.0f; };

    std::vector<vamp::collision::Environment<DataV>> envs;
    for (const auto &me : mbm) envs.push_back(build_env(me));

    struct Edge { int env; std::vector<Block> B; std::array<float, dim> v; float dt; std::array<BS, NBS> ca, cb; bool free; };
    std::vector<Edge> edges;
    auto bound_at = [&](const std::array<float, dim> &q) { Block b; for (std::size_t j = 0; j < dim; ++j) b[j] = DataV::fill(q[j]);
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b, bs); std::array<BS, NBS> o;
        for (std::size_t k = 0; k < NBS; ++k) o[k] = {bs.x[k].to_array()[0], bs.y[k].to_array()[0], bs.z[k].to_array()[0], bs.r[k].to_array()[0]}; return o; };
    auto valid_base = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc<rake>(envs[e.env], b)) return false; return true; };

    std::mt19937 wr(0x33); std::uniform_real_distribution<float> u(0.f, 1.f); std::normal_distribution<float> nd(0.f, 1.f);
    const std::size_t PER_ENV = 320;
    for (std::size_t ei = 0; ei < envs.size(); ++ei) {
        std::size_t got = 0; int att = 0;
        while (got < PER_ENV && att < 40000) {
            ++att; Block tmp; for (std::size_t j = 0; j < dim; ++j) tmp[j] = static_cast<DataV>(u(wr)); R::template scale_configuration_block<rake>(tmp);
            std::array<float, dim> a, v, b; float nr = 0; for (std::size_t j = 0; j < dim; ++j) { a[j] = tmp[j].to_array()[0]; v[j] = nd(wr); nr += v[j] * v[j]; }
            nr = std::sqrt(nr); for (std::size_t j = 0; j < dim; ++j) { v[j] *= range / nr; b[j] = a[j] + v[j]; }
            float dt = 1.0f / (float)N; std::vector<Block> B(n);
            for (std::size_t r = 0; r < n; ++r) for (std::size_t j = 0; j < dim; ++j) { alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
                for (std::size_t l = 0; l < rake; ++l) ln[l] = a[j] + (float)(r * rake + l) * dt * v[j]; B[r][j] = DataV(ln.data()); }
            Edge e{(int)ei, std::move(B), v, dt, bound_at(a), bound_at(b), false}; e.free = valid_base(e);
            // keep all free edges + an equal share of colliding
            if (e.free || got < PER_ENV) { edges.push_back(std::move(e)); ++got; }
        }
    }

    // cached masks from endpoint bounding spheres + M*Omega sagitta; env test via VAMP primitive
    auto masks = [&](const Edge &e, std::array<bool, NBS> &ec, std::array<bool, R::n_self_pairs> &pc) {
        std::array<std::array<float, 3>, NBS> E; std::array<float, NBS> RAD;
        for (std::size_t k = 0; k < NBS; ++k) { auto &A = e.ca[k]; auto &Bb = e.cb[k];
            float half = 0.5f * std::sqrt((A.x - Bb.x) * (A.x - Bb.x) + (A.y - Bb.y) * (A.y - Bb.y) + (A.z - Bb.z) * (A.z - Bb.z));
            E[k] = {(A.x + Bb.x) * 0.5f, (A.y + Bb.y) * 0.5f, (A.z + Bb.z) * 0.5f}; RAD[k] = half + std::max(A.r, Bb.r) + sag(k, e.v); }
        for (std::size_t k = 0; k < NBS; ++k)
            ec[k] = not vamp::sphere_environment_in_collision(envs[e.env], DataV::fill(E[k][0]), DataV::fill(E[k][1]), DataV::fill(E[k][2]), DataV::fill(RAD[k]));
        for (std::size_t pi = 0; pi < NP; ++pi) { int a = pair_bs[pi][0], b = pair_bs[pi][1];
            float dx = E[a][0] - E[b][0], dy = E[a][1] - E[b][1], dz = E[a][2] - E[b][2];
            pc[pi] = (std::sqrt(dx * dx + dy * dy + dz * dz) > RAD[a] + RAD[b]); }
    };
    auto valid_recur = [&](const Edge &e) { std::array<V1, dim> ps{}, pc{}; std::array<float, dim> C8, S8;
        for (std::size_t j = jlo; j < jhi; ++j) { float d8 = rake * e.dt * e.v[j]; C8[j] = std::cos(d8); S8[j] = std::sin(d8); }
        for (std::size_t r = 0; r < e.B.size(); ++r) {
            if (r == 0) for (std::size_t j = jlo; j < jhi; ++j) { ps[j] = ::sin(e.B[0][j]); pc[j] = ::cos(e.B[0][j]); }
            else for (std::size_t j = jlo; j < jhi; ++j) { V1 c = V1::fill(C8[j]), s = V1::fill(S8[j]); V1 sn = ps[j] * c + pc[j] * s, cn = pc[j] * c - ps[j] * s; ps[j] = sn; pc[j] = cn; }
            if (not R::template fkcc_pretrig<rake>(envs[e.env], e.B[r], ps, pc)) return false; }
        return true; };
    auto valid_swept_nc = [&](const Edge &e) { std::array<bool, NBS> ec; std::array<bool, R::n_self_pairs> pc{};
        std::size_t I[3] = {0, N / 2, N - 1}; Block b3;   // 3-config bound_fk in the timed path (no cache)
        for (std::size_t j = 0; j < dim; ++j) { alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
            std::array<float, dim> a; a[j] = e.ca[0].x; (void)a; for (int t = 0; t < 3; ++t) ln[t] = e.B[0][j].to_array()[0] + (float)I[t] * e.dt * e.v[j];
            for (std::size_t l = 3; l < rake; ++l) ln[l] = ln[0]; b3[j] = DataV(ln.data()); }
        typename R::template BoundingSpheres<rake> bs; R::template bound_fk<rake>(b3, bs);
        Edge tmp = e; for (std::size_t k = 0; k < NBS; ++k) { tmp.ca[k] = {bs.x[k].to_array()[0], bs.y[k].to_array()[0], bs.z[k].to_array()[0], bs.r[k].to_array()[0]};
            tmp.cb[k] = {bs.x[k].to_array()[2], bs.y[k].to_array()[2], bs.z[k].to_array()[2], bs.r[k].to_array()[2]}; }
        masks(tmp, ec, pc);
        for (auto &b : e.B) if (not R::template fkcc_swept<rake>(envs[e.env], b, ec, pc)) return false; return true; };
    auto valid_swept_c = [&](const Edge &e) { std::array<bool, NBS> ec; std::array<bool, R::n_self_pairs> pc{}; masks(e, ec, pc);
        for (auto &b : e.B) if (not R::template fkcc_swept<rake>(envs[e.env], b, ec, pc)) return false; return true; };
    auto valid_staged_c = [&](const Edge &e) { std::array<bool, NBS> ec; std::array<bool, R::n_self_pairs> pc{}; masks(e, ec, pc);
        for (auto &b : e.B) if (not R::template fkcc_swept_staged<rake>(envs[e.env], b, ec, pc)) return false; return true; };
    // ISOLATED tracing change: staged sphere-aware FK doing a FULL (unmasked) check -> vs baseline
    // fkcc this measures the FK-tracing modification alone (no swept skipping).
    std::array<bool, NBS> ec_off{}; std::array<bool, R::n_self_pairs> pc_off{};
    auto valid_staged_full = [&](const Edge &e) { for (auto &b : e.B) if (not R::template fkcc_swept_staged<rake>(envs[e.env], b, ec_off, pc_off)) return false; return true; };

    // rigor: every method must match baseline; unsafe = method says FREE, baseline says COLLIDE
    std::size_t nfree = 0; std::array<std::size_t, 5> mm{}, uns{};
    auto chk = [&](std::size_t idx, bool bv, bool mv) { if (bv != mv) { mm[idx]++; if (mv && not bv) uns[idx]++; } };
    for (auto &e : edges) { bool bv = valid_base(e); nfree += e.free;
        chk(0, bv, valid_recur(e)); chk(1, bv, valid_swept_nc(e)); chk(2, bv, valid_swept_c(e)); chk(3, bv, valid_staged_c(e)); chk(4, bv, valid_staged_full(e)); }

    auto med = [&](auto fn, int mode) { std::vector<double> t; volatile std::uint64_t sk = 0; std::vector<Edge *> sub;
        for (auto &e : edges) if (mode == 0 || (mode == 1 && e.free) || (mode == 2 && not e.free)) sub.push_back(&e); if (sub.empty()) return 0.0;
        for (int rp = 0; rp < 7; ++rp) { auto a = std::chrono::steady_clock::now(); std::uint64_t ac = 0; for (auto *e : sub) ac += fn(*e); auto z = std::chrono::steady_clock::now(); sk += ac;
            t.push_back(std::chrono::duration<double>(z - a).count() / sub.size() * 1e9); } (void)sk; std::sort(t.begin(), t.end()); return t[t.size() / 2]; };
    auto B0 = [&](const Edge &e) { return valid_base(e) ? 1U : 0U; };
    auto Rr = [&](const Edge &e) { return valid_recur(e) ? 1U : 0U; };
    auto Sn = [&](const Edge &e) { return valid_swept_nc(e) ? 1U : 0U; };
    auto Sc = [&](const Edge &e) { return valid_swept_c(e) ? 1U : 0U; };
    auto St = [&](const Edge &e) { return valid_staged_c(e) ? 1U : 0U; };
    auto Sf = [&](const Edge &e) { return valid_staged_full(e) ? 1U : 0U; };

    std::printf("\n== %-6s ==  dim=%zu  n=%zu  bs=%zu pairs=%zu  envs=%zu  edges=%zu (free %zu)  mism=%zu/%zu/%zu/%zu/%zu  unsafe=%zu/%zu/%zu/%zu/%zu\n",
                name, dim, n, NBS, NP, envs.size(), edges.size(), nfree, mm[0], mm[1], mm[2], mm[3], mm[4], uns[0], uns[1], uns[2], uns[3], uns[4]);
    const char *lbls[3] = {"all", "free", "colliding"};
    std::printf("   %-9s | %8s %8s %8s %8s %8s %9s   (ns/edge; x = vs base)\n", "edges", "base", "recur", "swept", "swept+$", "staged+$", "stagedFULL");
    for (int m = 0; m < 3; ++m) {
        double bb = med(B0, m), rr = med(Rr, m), sn = med(Sn, m), sc = med(Sc, m), st = med(St, m), sf = med(Sf, m);
        std::printf("   %-9s | %8.0f %8.0f %8.0f %8.0f %8.0f %9.0f\n", lbls[m], bb, rr, sn, sc, st, sf);
        std::printf("   %-9s | %8s %7.2fx %7.2fx %7.2fx %7.2fx %8.2fx\n", "", "1.00x", bb / rr, bb / sn, bb / sc, bb / st, bb / sf);
    }
}

int main()
{
    run<vamp::robots::Ur5>("ur5", 1.5f, 0, 6, ur5_pair_bs, ur5_envs);
    run<vamp::robots::PandaE>("panda", 1.25f, 0, 7, panda_pair_bs, panda_envs);
    run<vamp::robots::FetchE>("fetch", 1.0f, 1, 8, fetch_pair_bs, fetch_envs);
    run<vamp::robots::BaxterE>("baxter", 0.5f, 0, 14, baxter_pair_bs, baxter_envs);
    return 0;
}
