// Compiled reordering via the compact collision table. The compact kernel loops a
// small data table (cc_env_links: link order + fine-sphere ranges). Specialization =
// permute that table by collision frequency (done once/scene) -> the compiled loop
// early-exits on the likely-colliding link first, with NO per-config/per-edge runtime
// partition (the overhead that killed the runtime broadphase).
//   1) compact (data-driven) vs unrolled (straight-line) baseline speed
//   2) compact default order vs frequency-reordered cc_env_links
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include "fetch_base.hh"      // FetchBase   : unrolled env+self
#include "fetch_compact.hh"   // FetchCompact: compact env+self (cc_env_links mutable)

namespace vr = vamp::robots;
constexpr std::size_t rake = vamp::FloatVectorWidth, ns = vr::FetchBase::n_spheres;
using DataV = vamp::FloatVector<rake>;
using EnvV = vamp::collision::Environment<DataV>;

static EnvV shelf(std::mt19937 &rng, int n)
{
    vamp::collision::Environment<float> ef;
    std::uniform_real_distribution<float> sx(0.45F, 0.85F), sy(-0.35F, 0.35F), sz(0.45F, 1.0F), rad(0.02F, 0.05F);
    for (int i = 0; i < n; ++i)
        ef.spheres.emplace_back(vamp::collision::factory::sphere::array({sx(rng), sy(rng), sz(rng)}, rad(rng)));
    ef.sort();
    return EnvV(ef);
}
template <class R>
static auto mblk(std::mt19937 &rng)
{
    std::uniform_real_distribution<float> u(0.F, 1.F); std::normal_distribution<float> st(0.F, 0.02F);
    typename R::template ConfigurationBlock<rake> b;
    for (std::size_t j = 0; j < R::dimension; ++j)
    {
        float base = u(rng), d = st(rng);
        alignas(vamp::FloatVectorAlignment) std::array<float, rake> ln;
        for (std::size_t l = 0; l < rake; ++l) ln[l] = base + (float(l) - 3.5F) * d;
        b[j] = DataV(ln.data());
    }
    R::template scale_configuration_block<rake>(b);
    return b;
}

int main()
{
    std::mt19937 srng(0xBEEF), wrng(0x33), trng(0x55);
    auto env = shelf(srng, 40);
    constexpr std::size_t M = 200000;
    std::vector<vr::FetchBase::ConfigurationBlock<rake>> blk(M);
    for (auto &b : blk) b = mblk<vr::FetchBase>(wrng);

    // ---- profile per-link env-collision frequency (held-out training set) ----
    std::array<long, 15> freq{}; vr::FetchBase::Spheres<rake> sph;
    for (int i = 0; i < 30000; ++i)
    {
        auto b = mblk<vr::FetchBase>(trng); vr::FetchBase::sphere_fk<rake>(b, sph);
        for (std::size_t li = 0; li < vr::FetchCompact::cc_env_links.size(); ++li)
        {
            const auto &el = vr::FetchCompact::cc_env_links[li];
            for (unsigned k = 0; k < el.body_count; ++k)
            {
                std::size_t s = vr::FetchCompact::cc_env_body_idx[el.body_start + k];
                if (vamp::sphere_environment_in_collision(env, sph.x[s], sph.y[s], sph.z[s], sph.r[s])) { ++freq[li]; break; }
            }
        }
    }

    auto med = [&](auto kern) {
        std::vector<double> t; volatile std::uint64_t s = 0;
        for (int r = 0; r < 7; ++r) {
            auto a = std::chrono::steady_clock::now(); std::uint64_t acc = 0;
            for (auto &b : blk) acc += kern(b); auto z = std::chrono::steady_clock::now();
            s += acc; t.push_back(std::chrono::duration<double>(z - a).count() / M * 1e9);
        }
        (void)s; std::sort(t.begin(), t.end()); return t[t.size() / 2];
    };

    double t_unroll = med([&](auto &b){ return vr::FetchBase::fkcc<rake>(env, b) ? 1U : 0U; });
    auto default_order = vr::FetchCompact::cc_env_links;  // save
    double t_compact = med([&](auto &b){ return vr::FetchCompact::fkcc<rake>(env, b) ? 1U : 0U; });

    // ---- reorder cc_env_links by descending frequency ----
    std::array<std::size_t, 15> ord; std::iota(ord.begin(), ord.end(), 0);
    std::stable_sort(ord.begin(), ord.end(), [&](std::size_t a, std::size_t c){ return freq[a] > freq[c]; });
    auto reordered = default_order;
    for (std::size_t i = 0; i < 15; ++i) reordered[i] = default_order[ord[i]];
    vr::FetchCompact::cc_env_links = reordered;
    double t_reorder = med([&](auto &b){ return vr::FetchCompact::fkcc<rake>(env, b) ? 1U : 0U; });

    // collision frac for context
    vr::FetchCompact::cc_env_links = default_order;
    std::size_t coll = 0; for (auto &b : blk) if (not vr::FetchBase::fkcc<rake>(env, b)) ++coll;

    std::printf("Fetch env+self, shelf N=40, collision_frac=%.3f\n", double(coll) / M);
    std::printf("unrolled        fkcc = %.1f ns\n", t_unroll);
    std::printf("compact default fkcc = %.1f ns  (compact/unrolled = %.2fx)\n", t_compact, t_unroll / t_compact);
    std::printf("compact reorder fkcc = %.1f ns  (reorder speedup vs compact = %.2fx, vs unrolled = %.2fx)\n",
                t_reorder, t_compact / t_reorder, t_unroll / t_reorder);
    return 0;
}
