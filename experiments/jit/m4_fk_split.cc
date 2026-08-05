// E-FK1: how much of a collision check is FK? Measures sphere_fk-only vs
// sphere_fk + generic collision (T0_split), per robot. FK fraction = t_fk/t_cc
// sets the ceiling on any collision-side optimization: pruning collision can at
// best reach t_fk, so max collision-only speedup = t_cc/t_fk. Going below t_fk
// requires pruning FK itself (the scene-pruned-FK idea).
//
// CSV: robot,n_spheres,dim,N,extent,t_fk_ns,t_cc_ns,fk_fraction,collision_ceiling
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>
#include <algorithm>

#include <vamp/collision/factory.hh>
#include <vamp/collision/validity.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/baxter.hh>
#include <vamp/robots/fetch.hh>

namespace
{
    constexpr std::size_t rake = vamp::FloatVectorWidth;
    using DataV = vamp::FloatVector<rake>;

    template <typename Fn>
    auto median_ns(Fn &&fn, std::size_t m, int reps) -> double
    {
        std::vector<double> t; volatile std::uint64_t sink = 0;
        for (int r = 0; r < reps; ++r)
        {
            auto a = std::chrono::steady_clock::now();
            std::uint64_t acc = fn();
            auto b = std::chrono::steady_clock::now();
            sink += acc;
            t.push_back(std::chrono::duration<double>(b - a).count() / m * 1e9);
        }
        (void)sink; std::sort(t.begin(), t.end()); return t[t.size() / 2];
    }

    template <typename Robot>
    void run(const char *name, std::size_t N, float extent)
    {
        constexpr std::size_t m = 100000, ns = Robot::n_spheres;
        constexpr int reps = 7;
        std::mt19937 rng(0xF00D);
        std::uniform_real_distribution<float> u(0.F, 1.F), pos(-extent, extent), rad(0.02F, 0.08F);

        vamp::collision::Environment<float> ef;
        for (std::size_t i = 0; i < N; ++i)
            ef.spheres.emplace_back(vamp::collision::factory::sphere::array(
                {pos(rng), pos(rng), pos(rng)}, rad(rng)));
        ef.sort();
        vamp::collision::Environment<DataV> env(ef);

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
        typename Robot::template Spheres<rake> sph;

        double t_fk = median_ns([&]() -> std::uint64_t
        {
            std::uint64_t a = 0;
            for (const auto &b : blocks)
            {
                Robot::template sphere_fk<rake>(b, sph);
                a += static_cast<std::uint64_t>(sph.x[0].to_array()[0] > 0.F);  // keep FK live
            }
            return a;
        }, m, reps);

        double t_cc = median_ns([&]() -> std::uint64_t
        {
            std::uint64_t a = 0;
            for (const auto &b : blocks)
            {
                Robot::template sphere_fk<rake>(b, sph);
                bool free = true;
                for (std::size_t j = 0; j < ns; ++j)
                    if (vamp::sphere_environment_in_collision(env, sph.x[j], sph.y[j], sph.z[j], sph.r[j]))
                    { free = false; break; }
                a += free;
            }
            return a;
        }, m, reps);

        std::printf("%s,%zu,%zu,%zu,%.2f,%.3f,%.3f,%.4f,%.3f\n",
                    name, ns, Robot::dimension, N, extent, t_fk, t_cc, t_fk / t_cc, t_cc / t_fk);
        std::fflush(stdout);
    }
}  // namespace

auto main() -> int
{
    std::printf("robot,n_spheres,dim,N,extent,t_fk_ns,t_cc_ns,fk_fraction,collision_ceiling\n");
    for (float extent : {0.6F, 1.5F})
        for (std::size_t N : {std::size_t(200), std::size_t(800)})
        {
            run<vamp::robots::UR5>("ur5", N, extent);
            run<vamp::robots::Panda>("panda", N, extent);
            run<vamp::robots::Baxter>("baxter", N, extent);
            run<vamp::robots::Fetch>("fetch", N, extent);
        }
    return 0;
}
