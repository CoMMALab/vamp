// M2.5 pruned-kernel throughput + correctness: does the per-sphere reachable-AABB
// pruning (measured in m25_pruning) translate into real speedup?
//
// Method (honest, reuses VAMP's own optimized kernel): give each robot sphere j a
// pruned Environment containing only obstacles whose center lies in sphere j's
// sampled+inflated reachable AABB. Compare:
//   T0    : sphere_fk + generic sphere_environment_in_collision on the FULL env
//   Tprune: sphere_fk + generic sphere_environment_in_collision on env_j (pruned)
// Both use the identical (vectorized, early-exit) kernel and identical FK -> the
// delta is purely the obstacle-set reduction.
//
// The reachable AABB over the full joint box is ROBOT-intrinsic (scene-independent)
// -> computable offline once; only the obstacle partition is per-scene (O(ns*N)).
//
// Correctness: sampled AABBs are not provably conservative, so we VERIFY: over an
// independent config set, Tprune's verdict must equal T0's. Any false negative (a
// missed collision) is reported as fn_rate -- a real limitation of the sampled
// fallback vs the doc's preferred interval-FK reachability.
//
// CSV: robot,scene,N,extent,margin,n_spheres,mean_frac_kept,t0_ns,tprune_ns,speedup,
//      verify_blocks,fn_blocks,fn_rate

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;
    using EnvF = vamp::collision::Environment<float>;
    using EnvV = vamp::collision::Environment<DataV>;

    struct RawObs { float x, y, z, r; };

    auto make_box(std::size_t n, float extent, std::mt19937 &rng) -> std::vector<RawObs>
    {
        std::uniform_real_distribution<float> pos(-extent, extent), rad(0.02F, 0.08F);
        std::vector<RawObs> o(n);
        for (auto &e : o) { e.x = pos(rng); e.y = pos(rng); e.z = pos(rng); e.r = rad(rng); }
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

    struct AABB { float lo[3], hi[3]; };

    template <typename Robot>
    void run(const char *name, std::size_t N, float extent, float margin)
    {
        constexpr std::size_t ns = Robot::n_spheres;
        std::mt19937 rng(0x1234);
        std::uniform_real_distribution<float> u(0.F, 1.F);
        typename Robot::template Spheres<rake> sph;

        // 1. Robot-intrinsic per-sphere reachable AABBs (sampled over joint box).
        std::vector<AABB> box(ns, AABB{{1e30F, 1e30F, 1e30F}, {-1e30F, -1e30F, -1e30F}});
        std::vector<float> rj(ns, 0.F);
        for (std::size_t s = 0; s < 100000; ++s)
        {
            auto blk = random_block<Robot>(rng, u);
            Robot::template sphere_fk<rake>(blk, sph);
            for (std::size_t j = 0; j < ns; ++j)
            {
                auto ax = sph.x[j].to_array(); auto ay = sph.y[j].to_array(); auto az = sph.z[j].to_array();
                if (s == 0) rj[j] = sph.r[j].to_array()[0];
                for (std::size_t l = 0; l < rake; ++l)
                {
                    box[j].lo[0] = std::min(box[j].lo[0], ax[l]); box[j].hi[0] = std::max(box[j].hi[0], ax[l]);
                    box[j].lo[1] = std::min(box[j].lo[1], ay[l]); box[j].hi[1] = std::max(box[j].hi[1], ay[l]);
                    box[j].lo[2] = std::min(box[j].lo[2], az[l]); box[j].hi[2] = std::max(box[j].hi[2], az[l]);
                }
            }
        }

        // 2. Scene + full env + per-sphere pruned envs.
        std::mt19937 srng(0xBEEF);
        auto obs = make_box(N, extent, srng);
        float maxr = 0.F; for (auto &o : obs) maxr = std::max(maxr, o.r);

        EnvF fullf;
        for (auto &o : obs) fullf.spheres.emplace_back(vamp::collision::Sphere<float>(o.x, o.y, o.z, o.r));
        fullf.sort();
        EnvV full(fullf);

        // Per-query specialization cost = the obstacle partition (AABBs are
        // robot-intrinsic/offline). Time building all ns pruned envs; median of 20.
        auto build_pruned = [&](std::vector<EnvV> &out) -> std::size_t
        {
            out.clear();
            out.resize(ns);
            std::size_t kept = 0;
            for (std::size_t j = 0; j < ns; ++j)
            {
                const float inf = rj[j] + maxr + margin;
                EnvF pf;
                for (auto &o : obs)
                    if (o.x >= box[j].lo[0] - inf && o.x <= box[j].hi[0] + inf &&
                        o.y >= box[j].lo[1] - inf && o.y <= box[j].hi[1] + inf &&
                        o.z >= box[j].lo[2] - inf && o.z <= box[j].hi[2] + inf)
                        pf.spheres.emplace_back(vamp::collision::Sphere<float>(o.x, o.y, o.z, o.r));
                pf.sort();
                kept += pf.spheres.size();
                out[j] = EnvV(pf);
            }
            return kept;
        };
        std::vector<EnvV> pruned;
        std::size_t kept_total = 0;
        std::vector<double> pt;
        for (int r = 0; r < 20; ++r)
        {
            auto t0 = std::chrono::steady_clock::now();
            kept_total = build_pruned(pruned);
            auto t1 = std::chrono::steady_clock::now();
            pt.push_back(std::chrono::duration<double>(t1 - t0).count() * 1e6);  // us
        }
        std::sort(pt.begin(), pt.end());
        double partition_us = pt[pt.size() / 2];
        double mean_frac_kept = static_cast<double>(kept_total) / static_cast<double>(ns * N);

        // 3. Workload (independent stream) + correctness verify (false negatives).
        constexpr std::size_t m = 100000;
        std::mt19937 wrng(0x9999);
        std::uniform_real_distribution<float> wu(0.F, 1.F);
        std::vector<typename Robot::template ConfigurationBlock<rake>> blocks(m);
        for (auto &b : blocks) b = random_block<Robot>(wrng, wu);

        std::size_t fn = 0;  // pruned says free but full says collision (missed collision)
        for (const auto &b : blocks)
        {
            Robot::template sphere_fk<rake>(b, sph);
            bool full_free = true, prune_free = true;
            for (std::size_t j = 0; j < ns; ++j)
            {
                if (vamp::sphere_environment_in_collision(full, sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                    full_free = false;
                if (vamp::sphere_environment_in_collision(pruned[j], sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                    prune_free = false;
            }
            if (prune_free && not full_free) ++fn;  // false negative = unsafe prune
        }

        auto median = [&](auto &&fn_kernel) -> double
        {
            std::vector<double> t; volatile std::uint64_t sink = 0;
            for (int r = 0; r < 5; ++r)
            {
                auto t0 = std::chrono::steady_clock::now();
                std::uint64_t acc = fn_kernel();
                auto t1 = std::chrono::steady_clock::now();
                sink += acc;
                t.push_back(std::chrono::duration<double>(t1 - t0).count() / static_cast<double>(m) * 1e9);
            }
            (void)sink; std::sort(t.begin(), t.end()); return t[t.size() / 2];
        };

        double t0 = median([&]() -> std::uint64_t
        {
            std::uint64_t a = 0;
            for (const auto &b : blocks)
            {
                Robot::template sphere_fk<rake>(b, sph);
                bool free = true;
                for (std::size_t j = 0; j < ns; ++j)
                    if (vamp::sphere_environment_in_collision(full, sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                    { free = false; break; }
                a += free;
            }
            return a;
        });

        double tp = median([&]() -> std::uint64_t
        {
            std::uint64_t a = 0;
            for (const auto &b : blocks)
            {
                Robot::template sphere_fk<rake>(b, sph);
                bool free = true;
                for (std::size_t j = 0; j < ns; ++j)
                    if (vamp::sphere_environment_in_collision(pruned[j], sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                    { free = false; break; }
                a += free;
            }
            return a;
        });

        std::printf("%s,box,%zu,%.2f,%.3f,%zu,%.4f,%.3f,%.3f,%.3f,%.1f,%zu,%zu,%.6f\n",
                    name, N, extent, margin, ns, mean_frac_kept, t0, tp, t0 / tp,
                    partition_us, m, fn, static_cast<double>(fn) / static_cast<double>(m));
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    std::printf("robot,scene,N,extent,margin,n_spheres,mean_frac_kept,t0_ns,tprune_ns,speedup,"
                "partition_us,verify_blocks,fn_blocks,fn_rate\n");
    for (float margin : {0.05F})
        for (float extent : {0.6F, 1.5F})
            for (std::size_t N : {std::size_t(200), std::size_t(800)})
            {
                run<vamp::robots::Fetch>("fetch", N, extent, margin);
                run<vamp::robots::Baxter>("baxter", N, extent, margin);
                run<vamp::robots::UR5>("ur5", N, extent, margin);
                run<vamp::robots::Panda>("panda", N, extent, margin);
            }
    return 0;
}
