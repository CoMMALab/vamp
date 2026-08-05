// M1 baseline microbenchmark: generic (T0) fused FK + collision-check throughput.
//
// Measures isolated Robot::fkcc<rake> calls/sec over a scene of N random sphere
// obstacles, sweeping N and workspace extent. This is the T0 bar the scene-
// specialized kernels (M2/M3) must beat, and validates the premise (design doc
// §13 action 2, E1/E2 baselines). No JIT / cricket needed -- header-only.
//
// Output: CSV to stdout: robot,n_spheres_robot,dim,n_obstacles,extent,rake,
//         m_blocks,repeats,median_ns_per_fkcc,fkcc_per_sec,configs_per_sec,collision_frac

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/baxter.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;

    // Build a scene of `n` random sphere obstacles uniformly in a cube of
    // half-extent `extent` centered at the origin (robot base). Obstacle radius
    // drawn from [0.02, 0.08] m -- typical spherized-scene granularity.
    auto make_scene(std::size_t n, float extent, std::mt19937 &rng)
        -> vamp::collision::Environment<vamp::FloatVector<rake>>
    {
        std::uniform_real_distribution<float> pos(-extent, extent);
        std::uniform_real_distribution<float> rad(0.02F, 0.08F);

        vamp::collision::Environment<float> env;
        env.spheres.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            std::array<float, 3> c = {pos(rng), pos(rng), pos(rng)};
            env.spheres.emplace_back(vamp::collision::factory::sphere::array(c, rad(rng)));
        }
        env.sort();  // establishes the sorted min_distance early-exit order
        return vamp::collision::Environment<vamp::FloatVector<rake>>(env);
    }

    // Pre-generate `m` random in-bounds configuration blocks for `Robot`.
    template <typename Robot>
    auto make_blocks(std::size_t m, std::mt19937 &rng)
        -> std::vector<typename Robot::template ConfigurationBlock<rake>>
    {
        std::uniform_real_distribution<float> u(0.0F, 1.0F);
        std::vector<typename Robot::template ConfigurationBlock<rake>> blocks(m);
        for (std::size_t b = 0; b < m; ++b)
        {
            auto &blk = blocks[b];
            for (std::size_t j = 0; j < Robot::dimension; ++j)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
                for (std::size_t l = 0; l < rake; ++l)
                {
                    lane[l] = u(rng);
                }
                blk[j] = vamp::FloatVector<rake>(lane.data());
            }
            Robot::template scale_configuration_block<rake>(blk);  // [0,1] -> joint space
        }
        return blocks;
    }

    template <typename Robot>
    void run(const char *robot_name, const std::vector<std::size_t> &ns, float extent)
    {
        constexpr std::size_t m = 200000;  // blocks per repeat
        constexpr int repeats = 5;
        std::mt19937 rng(0xC0FFEE);

        auto blocks = make_blocks<Robot>(m, rng);

        for (auto n : ns)
        {
            auto env = make_scene(n, extent, rng);

            // Warm + collision fraction.
            std::size_t collisions = 0;
            for (const auto &blk : blocks)
            {
                if (not vamp::planning::fkcc_block<Robot, rake>(env, blk))
                {
                    ++collisions;
                }
            }
            const double collision_frac = static_cast<double>(collisions) / static_cast<double>(m);

            std::vector<double> ns_per_fkcc;
            ns_per_fkcc.reserve(repeats);
            volatile std::uint64_t sink = 0;
            for (int r = 0; r < repeats; ++r)
            {
                std::uint64_t acc = 0;
                auto t0 = std::chrono::steady_clock::now();
                for (const auto &blk : blocks)
                {
                    acc += vamp::planning::fkcc_block<Robot, rake>(env, blk) ? 1U : 0U;
                }
                auto t1 = std::chrono::steady_clock::now();
                sink += acc;
                double sec = std::chrono::duration<double>(t1 - t0).count();
                ns_per_fkcc.push_back(sec / static_cast<double>(m) * 1e9);
            }
            (void)sink;
            std::sort(ns_per_fkcc.begin(), ns_per_fkcc.end());
            double med = ns_per_fkcc[ns_per_fkcc.size() / 2];
            double fkcc_per_sec = 1e9 / med;
            double configs_per_sec = fkcc_per_sec * rake;

            std::printf(
                "%s,%zu,%zu,%zu,%.2f,%zu,%zu,%d,%.3f,%.0f,%.0f,%.4f\n",
                robot_name, Robot::n_spheres, Robot::dimension, n, extent, rake, m, repeats,
                med, fkcc_per_sec, configs_per_sec, collision_frac);
            std::fflush(stdout);
        }
    }
}  // namespace

auto main() -> int
{
    std::printf(
        "robot,n_spheres_robot,dim,n_obstacles,extent,rake,m_blocks,repeats,"
        "median_ns_per_fkcc,fkcc_per_sec,configs_per_sec,collision_frac\n");
    const std::vector<std::size_t> ns = {10, 25, 50, 100, 200, 400, 800};
    for (float extent : {0.6F, 1.5F})
    {
        run<vamp::robots::UR5>("ur5", ns, extent);      // 40 spheres
        run<vamp::robots::Panda>("panda", ns, extent);  // 59
        run<vamp::robots::Baxter>("baxter", ns, extent);  // 75, dim 14
        run<vamp::robots::Fetch>("fetch", ns, extent);    // 111, dim 8
    }
    return 0;
}
