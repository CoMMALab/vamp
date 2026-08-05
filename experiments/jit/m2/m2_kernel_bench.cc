// M2 isolated-kernel benchmark: pure scene-collision throughput, generic (T0) vs
// scene-specialized (T2), plus the bit-exact correctness gate. No robot / FK --
// this is Component 1 in isolation, so it compiles fast (sweep many N) and lets
// us control the early-exit regime via the query-sphere distribution.
//
// Both paths read the SAME obstacle constants (scene_gen.hh): the generic path
// builds an Environment from them; the specialized path bakes them as immediates.
// Identical inputs -> results MUST match bit-for-bit (design doc §5 must-have).
//
// CSV row: kernel,N,extent,rake,k_queries,dist,med_ns_generic,med_ns_spec,
//          speedup,collision_frac,mismatches

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>

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
        env.sort();  // already sorted by construction; keeps min_distance order canonical
        return vamp::collision::Environment<DataV>(env);
    }

    struct Queries
    {
        std::vector<DataV> sx, sy, sz, sr;
    };

    // box_scale controls the early-exit regime: >1 places queries beyond the
    // obstacle box (max_extent large -> few distance early-exits -> full loop).
    auto make_queries(std::size_t k, float extent, float box_scale, std::mt19937 &rng) -> Queries
    {
        std::uniform_real_distribution<float> pos(-extent * box_scale, extent * box_scale);
        std::uniform_real_distribution<float> rad(0.02F, 0.10F);
        Queries q;
        q.sx.resize(k); q.sy.resize(k); q.sz.resize(k); q.sr.resize(k);
        auto fill = [&](std::vector<DataV> &v, std::uniform_real_distribution<float> &d)
        {
            for (std::size_t i = 0; i < k; ++i)
            {
                alignas(vamp::FloatVectorAlignment) std::array<float, rake> lane;
                for (std::size_t l = 0; l < rake; ++l) lane[l] = d(rng);
                v[i] = DataV(lane.data());
            }
        };
        fill(q.sx, pos); fill(q.sy, pos); fill(q.sz, pos); fill(q.sr, rad);
        return q;
    }

    template <typename Fn>
    auto median_ns(Fn &&fn, std::size_t k, int repeats) -> double
    {
        std::vector<double> t;
        volatile std::uint64_t sink = 0;
        for (int r = 0; r < repeats; ++r)
        {
            auto t0 = std::chrono::steady_clock::now();
            std::uint64_t acc = fn();
            auto t1 = std::chrono::steady_clock::now();
            sink += acc;
            t.push_back(std::chrono::duration<double>(t1 - t0).count() / static_cast<double>(k) * 1e9);
        }
        (void)sink;
        std::sort(t.begin(), t.end());
        return t[t.size() / 2];
    }

    void run(const char *dist, float box_scale)
    {
        constexpr std::size_t k = 500000;
        constexpr int repeats = 5;
        std::mt19937 rng(0x5CE7E);

        auto env = build_env();
        auto q = make_queries(k, scene_gen::SCENE_EXTENT, box_scale, rng);

        // Correctness gate + collision fraction (identical inputs to both kernels).
        std::size_t mism = 0, coll = 0;
        for (std::size_t i = 0; i < k; ++i)
        {
            bool g = vamp::sphere_environment_in_collision(env, q.sx[i], q.sy[i], q.sz[i], q.sr[i]);
            bool s = vamp::m2::scene_in_collision_spec<DataV>(q.sx[i], q.sy[i], q.sz[i], q.sr[i]);
            if (g != s) ++mism;
            if (g) ++coll;
        }

        double gen = median_ns(
            [&]() -> std::uint64_t
            {
                std::uint64_t a = 0;
                for (std::size_t i = 0; i < k; ++i)
                    a += vamp::sphere_environment_in_collision(env, q.sx[i], q.sy[i], q.sz[i], q.sr[i]);
                return a;
            }, k, repeats);

        double spec = median_ns(
            [&]() -> std::uint64_t
            {
                std::uint64_t a = 0;
                for (std::size_t i = 0; i < k; ++i)
                    a += vamp::m2::scene_in_collision_spec<DataV>(q.sx[i], q.sy[i], q.sz[i], q.sr[i]);
                return a;
            }, k, repeats);

        std::printf(
            "kernel,%zu,%.2f,%zu,%zu,%s,%.3f,%.3f,%.3f,%.4f,%zu\n",
            scene_gen::N, scene_gen::SCENE_EXTENT, rake, k, dist, gen, spec,
            gen / spec, static_cast<double>(coll) / static_cast<double>(k), mism);
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    // "near": queries inside the obstacle box (early-exit likely, planning-typical).
    // "full": queries beyond the box (full obstacle loop -- specialization ceiling).
    run("near", 1.0F);
    run("full", 1.6F);
    return 0;
}
