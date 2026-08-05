// P0.2 — bounding-sphere gate potential: on realistic (localized) scenes, what
// fraction of fine-sphere FK would a per-link bounding-sphere gate let us SKIP?
//
// The generated fkcc already gates fine-sphere *checks* behind a per-link bounding
// sphere, but computes all fine-sphere FK up front. If fine-sphere FK moved inside
// the gate, a link whose bounding sphere clears (all 8 SIMD lanes) would skip its
// fine spheres' FK. This measures the ceiling of that saving.
//
// Self-contained: link grouping is recovered by RIGIDITY (fine spheres on one link
// keep constant pairwise distance across configs), and each link's bounding sphere
// is the enclosing sphere of its fine spheres (a valid, if loose, superset of
// cricket's -> a conservative/lower-bound estimate of the gate's clear-rate).
//
// CSV: robot,n_spheres,dim,scene,n_links,mean_clear_rate,fine_fk_skip_frac,
//      links_always_clear
#include <array>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;

    template <typename Robot>
    auto rand_block(std::mt19937 &rng, std::uniform_real_distribution<float> &u)
        -> typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> b;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
            for (std::size_t l = 0; l < rake; ++l) lane[l] = u(rng);
            b[j] = DataV(lane.data());
        }
        Robot::template scale_configuration_block<rake>(b);
        return b;
    }

    // Correlated motion rake: 8 lanes are 8 nearby configs along a short segment --
    // this is how VAMP actually fills a rake (validate_vector). The SIMD all-lanes-clear
    // gate fires far more often here than for 8 independent random configs.
    template <typename Robot>
    auto motion_block(std::mt19937 &rng) -> typename Robot::template ConfigurationBlock<rake>
    {
        std::uniform_real_distribution<float> u(0.F, 1.F);
        std::normal_distribution<float> step(0.F, 0.02F);  // per lane-step in [0,1] space: a local segment
        typename Robot::template ConfigurationBlock<rake> b;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            float base = u(rng), d = step(rng);
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
            for (std::size_t l = 0; l < rake; ++l)
                lane[l] = base + (static_cast<float>(l) - 3.5F) * d;  // in [0,1] pre-scale space
            b[j] = DataV(lane.data());
        }
        Robot::template scale_configuration_block<rake>(b);
        return b;
    }

    // Recover rigid link groups by joint-dependency signature: a sphere's link is the
    // set of joints that move it (perturb each joint, see which spheres move). Same
    // signature = same rigid link (unique per link in a kinematic tree). Robust, unlike
    // pairwise-distance clustering which chains adjacent links via near-axis spheres.
    template <typename Robot>
    auto link_groups(std::mt19937 &rng) -> std::vector<std::vector<std::size_t>>
    {
        constexpr std::size_t ns = Robot::n_spheres, dim = Robot::dimension;
        std::uniform_real_distribution<float> u(0.F, 1.F);
        typename Robot::template Spheres<rake> sph;

        std::vector<std::uint64_t> sig(ns, 0);  // bit jt set if joint jt moves the sphere
        constexpr int baselines = 4;
        for (int base = 0; base < baselines; ++base)
        {
            auto b0 = rand_block<Robot>(rng, u);
            std::vector<std::array<float, 3>> p0(ns);
            Robot::template sphere_fk<rake>(b0, sph);
            for (std::size_t j = 0; j < ns; ++j)
                p0[j] = {sph.x[j].to_array()[0], sph.y[j].to_array()[0], sph.z[j].to_array()[0]};
            for (std::size_t jt = 0; jt < dim && jt < 64; ++jt)
            {
                auto b = b0;
                b[jt] = b[jt] + DataV(0.3F);  // perturb one joint (post-scale radians)
                Robot::template sphere_fk<rake>(b, sph);
                for (std::size_t j = 0; j < ns; ++j)
                {
                    float dx = sph.x[j].to_array()[0] - p0[j][0];
                    float dy = sph.y[j].to_array()[0] - p0[j][1];
                    float dz = sph.z[j].to_array()[0] - p0[j][2];
                    if (dx * dx + dy * dy + dz * dz > 1e-8F) sig[j] |= (1ULL << jt);
                }
            }
        }
        std::vector<std::uint64_t> keys;
        std::vector<std::vector<std::size_t>> out;
        for (std::size_t j = 0; j < ns; ++j)
        {
            auto it = std::find(keys.begin(), keys.end(), sig[j]);
            if (it == keys.end()) { keys.push_back(sig[j]); out.emplace_back(1, j); }
            else out[it - keys.begin()].push_back(j);
        }
        return out;
    }

    // Uniform box, or localized box offset to one side (a table/shelf) that distal
    // links reach but the base/torso does not.
    auto make_env(const char *scene, std::mt19937 &rng) -> vamp::collision::Environment<DataV>
    {
        vamp::collision::Environment<float> ef;
        std::uniform_real_distribution<float> rad(0.02F, 0.06F);
        if (std::string(scene) == "uniform")
        {
            std::uniform_real_distribution<float> p(-0.8F, 0.8F);
            for (int i = 0; i < 300; ++i)
                ef.spheres.emplace_back(vamp::collision::factory::sphere::array({p(rng), p(rng), p(rng)}, rad(rng)));
        }
        else  // localized: cluster around (0.55, 0.0, 0.55), half-extent 0.22
        {
            std::normal_distribution<float> nx(0.55F, 0.13F), ny(0.0F, 0.13F), nz(0.55F, 0.13F);
            for (int i = 0; i < 300; ++i)
                ef.spheres.emplace_back(vamp::collision::factory::sphere::array({nx(rng), ny(rng), nz(rng)}, rad(rng)));
        }
        ef.sort();
        return vamp::collision::Environment<DataV>(ef);
    }

    template <typename Robot>
    void run(const char *name, const char *scene, const char *workload)
    {
        constexpr std::size_t ns = Robot::n_spheres, M = 50000;
        std::mt19937 grng(0x11), crng(0x22), wrng(0x33);
        auto groups = link_groups<Robot>(grng);
        auto env = make_env(scene, crng);
        const bool motion = std::string(workload) == "motion";

        std::uniform_real_distribution<float> u(0.F, 1.F);
        typename Robot::template Spheres<rake> sph;

        std::vector<std::size_t> clear_cnt(groups.size(), 0);
        for (std::size_t s = 0; s < M; ++s)
        {
            auto b = motion ? motion_block<Robot>(wrng) : rand_block<Robot>(wrng, u);
            Robot::template sphere_fk<rake>(b, sph);
            for (std::size_t g = 0; g < groups.size(); ++g)
            {
                // enclosing bounding sphere of this link's fine spheres (per lane)
                DataV cx(0.F), cy(0.F), cz(0.F);
                for (auto j : groups[g]) { cx = cx + sph.x[j]; cy = cy + sph.y[j]; cz = cz + sph.z[j]; }
                float inv = 1.F / static_cast<float>(groups[g].size());
                cx = cx * DataV(inv); cy = cy * DataV(inv); cz = cz * DataV(inv);
                DataV r2(0.F);
                for (auto j : groups[g])
                {
                    DataV dx = sph.x[j] - cx, dy = sph.y[j] - cy, dz = sph.z[j] - cz;
                    DataV d = (dx * dx + dy * dy + dz * dz).sqrt() + sph.r[j];
                    r2 = r2.max(d);
                }
                // clear iff bounding sphere collides nothing across all lanes
                if (not vamp::sphere_environment_in_collision(env, cx, cy, cz, r2)) ++clear_cnt[g];
            }
        }

        double skip_num = 0, always_clear = 0;
        double sum_rate = 0;
        for (std::size_t g = 0; g < groups.size(); ++g)
        {
            double rate = static_cast<double>(clear_cnt[g]) / M;
            sum_rate += rate;
            skip_num += rate * groups[g].size();  // fine-sphere FK skipped by this link
            if (clear_cnt[g] == M) ++always_clear;
        }
        std::printf("%s,%zu,%zu,%s,%s,%zu,%.4f,%.4f,%.0f\n",
                    name, ns, Robot::dimension, scene, workload, groups.size(),
                    sum_rate / groups.size(), skip_num / ns, always_clear);
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    std::printf("robot,n_spheres,dim,scene,workload,n_links,mean_clear_rate,fine_fk_skip_frac,links_always_clear\n");
    for (const char *scene : {"uniform", "localized"})
        for (const char *wl : {"random", "motion"})
        {
            run<vamp::robots::UR5>("ur5", scene, wl);
            run<vamp::robots::Panda>("panda", scene, wl);
            run<vamp::robots::Baxter>("baxter", scene, wl);
            run<vamp::robots::Fetch>("fetch", scene, wl);
        }
    return 0;
}
