// M2.5 broadphase pruning-factor analysis (design doc Component 1 level 3 + §4
// "report the pruning factor per scene class"). NO codegen -- this measures the
// *ceiling* of the AOT-impossible lever before we build the pruned kernel.
//
// The honest bar is NOT "check everything": VAMP's generic loop already prunes
// far obstacles per query sphere via the sorted min_distance early-exit (each
// sphere's max_extent = |center|+r). So we compare, per robot sphere j:
//   frac_radial : obstacles the runtime radial early-exit leaves (mean over configs)
//   frac_aabb   : obstacles surviving a compile-time reachable-AABB test (static)
// If frac_aabb << frac_radial, the compile-time broadphase beats the early-exit.
//
// Two config distributions feed the reachable AABB:
//   full   : the whole joint-limit box (conservative; the doc's weak-pruning case)
//   corridor: configs near a start->goal motion (query-time knowledge = the JIT lever)
//
// CSV: robot,scene,N,param,dist,n_spheres,frac_radial,frac_aabb,aabb_beats_radial,reach_frac

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;

    struct Obs { float x, y, z, r, mind; };

    auto make_box(std::size_t n, float extent, std::mt19937 &rng) -> std::vector<Obs>
    {
        std::uniform_real_distribution<float> pos(-extent, extent), rad(0.02F, 0.08F);
        std::vector<Obs> o(n);
        for (auto &e : o)
        {
            e.x = pos(rng); e.y = pos(rng); e.z = pos(rng); e.r = rad(rng);
            e.mind = std::sqrt(e.x * e.x + e.y * e.y + e.z * e.z) - e.r;
        }
        std::sort(o.begin(), o.end(), [](const Obs &a, const Obs &b) { return a.mind < b.mind; });
        return o;
    }

    // Obstacles on a spherical shell radius [rmin,rmax], full solid angle: same
    // radial extent as the arm's reach, so the radial early-exit can't prune them
    // but a directional AABB might.
    auto make_shell(std::size_t n, float rmin, float rmax, std::mt19937 &rng) -> std::vector<Obs>
    {
        std::uniform_real_distribution<float> u(0.F, 1.F), rad(0.02F, 0.08F);
        std::vector<Obs> o(n);
        for (auto &e : o)
        {
            float rr = std::cbrt(u(rng) * (rmax * rmax * rmax - rmin * rmin * rmin) + rmin * rmin * rmin);
            float ct = 2.F * u(rng) - 1.F, st = std::sqrt(std::max(0.F, 1.F - ct * ct));
            float ph = 6.2831853F * u(rng);
            e.x = rr * st * std::cos(ph); e.y = rr * st * std::sin(ph); e.z = rr * ct; e.r = rad(rng);
            e.mind = std::sqrt(e.x * e.x + e.y * e.y + e.z * e.z) - e.r;
        }
        std::sort(o.begin(), o.end(), [](const Obs &a, const Obs &b) { return a.mind < b.mind; });
        return o;
    }

    template <typename Robot>
    auto random_block(std::mt19937 &rng, std::uniform_real_distribution<float> &u)
        -> typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> blk;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
            for (std::size_t l = 0; l < rake; ++l) lane[l] = u(rng);
            blk[j] = DataV(lane.data());
        }
        Robot::template scale_configuration_block<rake>(blk);
        return blk;
    }

    // Corridor block: interpolate start->goal (already scaled joint space) at random
    // t per lane, plus small joint-space gaussian jitter (planner exploration margin).
    template <typename Robot>
    auto corridor_block(
        const std::array<float, Robot::dimension> &start,
        const std::array<float, Robot::dimension> &goal,
        float jitter,
        std::mt19937 &rng) -> typename Robot::template ConfigurationBlock<rake>
    {
        std::uniform_real_distribution<float> t01(0.F, 1.F);
        std::normal_distribution<float> nj(0.F, jitter);
        typename Robot::template ConfigurationBlock<rake> blk;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
            for (std::size_t l = 0; l < rake; ++l)
            {
                float t = t01(rng);
                lane[l] = start[j] + t * (goal[j] - start[j]) + nj(rng);
            }
            blk[j] = DataV(lane.data());
        }
        return blk;
    }

    struct AABB { float lo[3], hi[3]; };

    // Accumulate per-sphere reachable AABBs and radii from a supplied config sampler.
    template <typename Robot, typename Sampler>
    void reach_aabbs(std::size_t k, Sampler &&sample, std::vector<AABB> &box, std::vector<float> &rj)
    {
        constexpr std::size_t ns = Robot::n_spheres;
        box.assign(ns, AABB{{1e30F, 1e30F, 1e30F}, {-1e30F, -1e30F, -1e30F}});
        rj.assign(ns, 0.F);
        typename Robot::template Spheres<rake> sph;
        for (std::size_t s = 0; s < k; ++s)
        {
            auto blk = sample();
            Robot::template sphere_fk<rake>(blk, sph);
            for (std::size_t j = 0; j < ns; ++j)
            {
                auto ax = sph.x[j].to_array();
                auto ay = sph.y[j].to_array();
                auto az = sph.z[j].to_array();
                if (s == 0) rj[j] = sph.r[j].to_array()[0];
                for (std::size_t l = 0; l < rake; ++l)
                {
                    box[j].lo[0] = std::min(box[j].lo[0], ax[l]); box[j].hi[0] = std::max(box[j].hi[0], ax[l]);
                    box[j].lo[1] = std::min(box[j].lo[1], ay[l]); box[j].hi[1] = std::max(box[j].hi[1], ay[l]);
                    box[j].lo[2] = std::min(box[j].lo[2], az[l]); box[j].hi[2] = std::max(box[j].hi[2], az[l]);
                }
            }
        }
    }

    // frac_aabb: mean_j (#obstacles whose center lies in AABB_j inflated by rj+maxr) / N.
    auto frac_aabb(const std::vector<AABB> &box, const std::vector<float> &rj,
                   const std::vector<Obs> &obs) -> double
    {
        float maxr = 0.F;
        for (const auto &o : obs) maxr = std::max(maxr, o.r);
        double acc = 0;
        for (std::size_t j = 0; j < box.size(); ++j)
        {
            const float inf = rj[j] + maxr;
            std::size_t k = 0;
            for (const auto &o : obs)
            {
                if (o.x >= box[j].lo[0] - inf && o.x <= box[j].hi[0] + inf &&
                    o.y >= box[j].lo[1] - inf && o.y <= box[j].hi[1] + inf &&
                    o.z >= box[j].lo[2] - inf && o.z <= box[j].hi[2] + inf)
                    ++k;
            }
            acc += static_cast<double>(k) / static_cast<double>(obs.size());
        }
        return acc / static_cast<double>(box.size());
    }

    // frac_radial: mean over (config, sphere) of (#obstacles with min_distance <=
    // |center_j|+rj) / N -- what the sorted radial early-exit leaves per query.
    template <typename Robot, typename Sampler>
    auto frac_radial(std::size_t k, Sampler &&sample, const std::vector<Obs> &obs,
                     const std::vector<float> &rj) -> double
    {
        constexpr std::size_t ns = Robot::n_spheres;
        std::vector<float> mind(obs.size());
        for (std::size_t i = 0; i < obs.size(); ++i) mind[i] = obs[i].mind;  // sorted ascending
        typename Robot::template Spheres<rake> sph;
        double acc = 0; std::size_t cnt = 0;
        for (std::size_t s = 0; s < k; ++s)
        {
            auto blk = sample();
            Robot::template sphere_fk<rake>(blk, sph);
            for (std::size_t j = 0; j < ns; ++j)
            {
                auto ax = sph.x[j].to_array();
                auto ay = sph.y[j].to_array();
                auto az = sph.z[j].to_array();
                for (std::size_t l = 0; l < rake; ++l)
                {
                    float me = std::sqrt(ax[l] * ax[l] + ay[l] * ay[l] + az[l] * az[l]) + rj[j];
                    std::size_t surv = std::upper_bound(mind.begin(), mind.end(), me) - mind.begin();
                    acc += static_cast<double>(surv) / static_cast<double>(obs.size());
                    ++cnt;
                }
            }
        }
        return acc / static_cast<double>(cnt);
    }

    template <typename Robot>
    void analyze(const char *name, const char *scene, std::size_t N, float param,
                 const std::vector<Obs> &obs)
    {
        constexpr std::size_t k_aabb = 100000, k_rad = 3000;
        std::mt19937 rng(0xA1B2);
        std::uniform_real_distribution<float> u(0.F, 1.F);
        auto sampler = [&]() { return random_block<Robot>(rng, u); };

        std::vector<AABB> box; std::vector<float> rj;
        reach_aabbs<Robot>(k_aabb, sampler, box, rj);
        double fa = frac_aabb(box, rj, obs);
        std::mt19937 rng2(0xA1B2);
        std::uniform_real_distribution<float> u2(0.F, 1.F);
        auto sampler2 = [&]() { return random_block<Robot>(rng2, u2); };
        double fr = frac_radial<Robot>(k_rad, sampler2, obs, rj);

        // reachable-box union volume fraction vs scene box (rough context).
        double reach = 0;
        for (const auto &b : box)
            reach += std::max(0.F, b.hi[0] - b.lo[0]) * std::max(0.F, b.hi[1] - b.lo[1]) *
                     std::max(0.F, b.hi[2] - b.lo[2]);
        double scenevol = std::pow(2.0 * param, 3);

        std::printf("%s,%s,%zu,%.2f,full,%zu,%.4f,%.4f,%.3f,%.3f\n",
                    name, scene, N, param, Robot::n_spheres, fr, fa, fr / std::max(1e-9, fa),
                    reach / std::max(1e-9, scenevol * box.size()));
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    std::printf("robot,scene,N,param,dist,n_spheres,frac_radial,frac_aabb,aabb_beats_radial,reach_frac\n");
    std::mt19937 seed(7);

    for (float extent : {0.6F, 1.5F, 3.0F})
    {
        for (std::size_t N : {std::size_t(200), std::size_t(800)})
        {
            auto obs = make_box(N, extent, seed);
            analyze<vamp::robots::UR5>("ur5", "box", N, extent, obs);
            analyze<vamp::robots::Panda>("panda", "box", N, extent, obs);
            analyze<vamp::robots::Baxter>("baxter", "box", N, extent, obs);
            analyze<vamp::robots::Fetch>("fetch", "box", N, extent, obs);
        }
    }
    // Shell at arm-reach radius: same radial extent, directional structure.
    for (std::size_t N : {std::size_t(400)})
    {
        auto obs = make_shell(N, 0.5F, 0.9F, seed);
        analyze<vamp::robots::UR5>("ur5", "shell", N, 0.9F, obs);
        analyze<vamp::robots::Panda>("panda", "shell", N, 0.9F, obs);
        analyze<vamp::robots::Fetch>("fetch", "shell", N, 0.9F, obs);
    }

    // Corridor (start/goal-informed) for Panda: the query-time JIT lever. Same box
    // scene; AABBs from configs near one representative start->goal motion.
    {
        std::array<float, 7> start = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
        std::array<float, 7> goal = {2.35F, 1.F, 0.F, -0.8F, 0.F, 2.5F, 0.785F};
        for (float extent : {1.5F, 3.0F})
        {
            auto obs = make_box(800, extent, seed);
            std::mt19937 rng(0xC0DE);
            std::vector<AABB> box; std::vector<float> rj;
            auto sampler = [&]() { return corridor_block<vamp::robots::Panda>(start, goal, 0.25F, rng); };
            reach_aabbs<vamp::robots::Panda>(100000, sampler, box, rj);
            double fa = frac_aabb(box, rj, obs);
            std::mt19937 rng2(0xC0DE);
            auto sampler2 = [&]() { return corridor_block<vamp::robots::Panda>(start, goal, 0.25F, rng2); };
            double fr = frac_radial<vamp::robots::Panda>(3000, sampler2, obs, rj);
            std::printf("panda,box,%d,%.2f,corridor,%zu,%.4f,%.4f,%.3f,%.3f\n",
                        800, extent, vamp::robots::Panda::n_spheres, fr, fa, fr / std::max(1e-9, fa), 0.0);
            std::fflush(stdout);
        }
    }
    return 0;
}
