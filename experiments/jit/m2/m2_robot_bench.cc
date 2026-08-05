// M2 robot FK + collision benchmark: how scene specialization composes through a
// real robot's per-sphere collision checks. For each robot we measure three kernels
// on the SAME random-config workload + the SAME baked scene:
//   T0_fused : Robot::fkcc<rake>          (generic fused FK + generic scene loop)
//   T0_split : sphere_fk + generic scene loop, per robot sphere
//   T2_split : sphere_fk + SPECIALIZED scene loop, per robot sphere
// The (T0_split vs T2_split) ratio isolates the collision-kernel specialization
// (common FK); n_spheres scales it -> Baxter(75)/Fetch(111) are the interesting ones.
//
// Correctness: per robot sphere, generic vs specialized on identical inputs must
// match bit-for-bit; block verdict fused-vs-specialized reported separately (FK path).
//
// CSV row: robot,n_spheres,dim,N,extent,rake,m_blocks,
//          t0_fused_ns,t0_split_ns,t2_split_ns,speedup_split,speedup_vs_fused,
//          collision_frac,sphere_mismatch,verdict_mismatch

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/planning/validate.hh>
#include <vamp/collision/validity.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

#include "specialized_kernel.hh"

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;

    auto build_env() -> vamp::collision::Environment<DataV>
    {
        vamp::collision::Environment<float> env;
        env.spheres.reserve(scene_gen::N);
        for (std::size_t i = 0; i < scene_gen::N; ++i)
        {
            env.spheres.emplace_back(vamp::collision::Sphere<float>(
                scene_gen::OBS_X[i], scene_gen::OBS_Y[i], scene_gen::OBS_Z[i], scene_gen::OBS_R[i]));
        }
        env.sort();
        return vamp::collision::Environment<DataV>(env);
    }

    template <typename Robot>
    auto make_blocks(std::size_t m, std::mt19937 &rng)
        -> std::vector<typename Robot::template ConfigurationBlock<rake>>
    {
        std::uniform_real_distribution<float> u(0.0F, 1.0F);
        std::vector<typename Robot::template ConfigurationBlock<rake>> blocks(m);
        for (auto &blk : blocks)
        {
            for (std::size_t j = 0; j < Robot::dimension; ++j)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
                for (std::size_t l = 0; l < rake; ++l) lane[l] = u(rng);
                blk[j] = DataV(lane.data());
            }
            Robot::template scale_configuration_block<rake>(blk);
        }
        return blocks;
    }

    template <typename Fn>
    auto median_ns(Fn &&fn, std::size_t m, int repeats) -> double
    {
        std::vector<double> t;
        volatile std::uint64_t sink = 0;
        for (int r = 0; r < repeats; ++r)
        {
            auto t0 = std::chrono::steady_clock::now();
            std::uint64_t acc = fn();
            auto t1 = std::chrono::steady_clock::now();
            sink += acc;
            t.push_back(std::chrono::duration<double>(t1 - t0).count() / static_cast<double>(m) * 1e9);
        }
        (void)sink;
        std::sort(t.begin(), t.end());
        return t[t.size() / 2];
    }

    template <typename Robot>
    void run(const char *name, const vamp::collision::Environment<DataV> &env)
    {
        constexpr std::size_t m = 100000;
        constexpr int repeats = 5;
        std::mt19937 rng(0xB10C);
        auto blocks = make_blocks<Robot>(m, rng);
        typename Robot::template Spheres<rake> sph;

        // Correctness + collision fraction.
        std::size_t sphere_mm = 0, verdict_mm = 0, coll = 0;
        for (const auto &blk : blocks)
        {
            Robot::template sphere_fk<rake>(blk, sph);
            bool spec_free = true;
            for (std::size_t j = 0; j < Robot::n_spheres; ++j)
            {
                bool g = vamp::sphere_environment_in_collision(env, sph.x[j], sph.y[j], sph.z[j], sph.r[j]);
                bool s = vamp::m2::scene_in_collision_spec<DataV>(sph.x[j], sph.y[j], sph.z[j], sph.r[j]);
                if (g != s) ++sphere_mm;
                if (s) spec_free = false;
            }
            bool fused_free = Robot::template fkcc<rake>(env, blk);
            if (fused_free != spec_free) ++verdict_mm;
            if (not fused_free) ++coll;
        }

        double t0_fused = median_ns(
            [&]() -> std::uint64_t
            {
                std::uint64_t a = 0;
                for (const auto &blk : blocks) a += Robot::template fkcc<rake>(env, blk) ? 1U : 0U;
                return a;
            }, m, repeats);

        double t0_split = median_ns(
            [&]() -> std::uint64_t
            {
                std::uint64_t a = 0;
                for (const auto &blk : blocks)
                {
                    Robot::template sphere_fk<rake>(blk, sph);
                    bool free = true;
                    for (std::size_t j = 0; j < Robot::n_spheres; ++j)
                        if (vamp::sphere_environment_in_collision(env, sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                        { free = false; break; }
                    a += free;
                }
                return a;
            }, m, repeats);

        double t2_split = median_ns(
            [&]() -> std::uint64_t
            {
                std::uint64_t a = 0;
                for (const auto &blk : blocks)
                {
                    Robot::template sphere_fk<rake>(blk, sph);
                    bool free = true;
                    for (std::size_t j = 0; j < Robot::n_spheres; ++j)
                        if (vamp::m2::scene_in_collision_spec<DataV>(sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                        { free = false; break; }
                    a += free;
                }
                return a;
            }, m, repeats);

        std::printf(
            "%s,%zu,%zu,%zu,%.2f,%zu,%zu,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%zu,%zu\n",
            name, Robot::n_spheres, Robot::dimension, scene_gen::N, scene_gen::SCENE_EXTENT, rake, m,
            t0_fused, t0_split, t2_split, t0_split / t2_split, t0_fused / t2_split,
            static_cast<double>(coll) / static_cast<double>(m), sphere_mm, verdict_mm);
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    auto env = build_env();
    run<vamp::robots::UR5>("ur5", env);
    run<vamp::robots::Panda>("panda", env);
    run<vamp::robots::Baxter>("baxter", env);
    run<vamp::robots::Fetch>("fetch", env);
    return 0;
}
