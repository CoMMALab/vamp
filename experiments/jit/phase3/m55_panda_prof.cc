// Isolated baxter RRTC workload for profiling the NN-vs-collision split (idea #3 sizing).
// Runs all solvable baxter problems in a loop so perf gets a clean, baxter-only sample.
#include <array>
#include <cstdio>
#include <memory>
#include <vector>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/random/halton.hh>
#include "panda_e.hh"
#include "mbm_problems.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;
using R = vamp::robots::PandaE;

static vamp::collision::Environment<DataV> build_env(const std::vector<PPrim> &prims)
{
    vamp::collision::Environment<float> ef;
    for (const auto &p : prims) {
        if (p.kind == 0) ef.add_sphere(vamp::collision::factory::sphere::flat(p.px, p.py, p.pz, p.a));
        else if (p.kind == 1) ef.add_capsule(vamp::collision::factory::capsule::center::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b));
        else ef.add_cuboid(vamp::collision::factory::cuboid::flat(p.px, p.py, p.pz, p.ex, p.ey, p.ez, p.a, p.b, p.c));
    }
    ef.sort();
    return vamp::collision::Environment<DataV>(ef);
}

static R::Configuration to_cfg(const std::vector<float> &v)
{
    std::array<float, R::dimension> a{};
    for (std::size_t i = 0; i < R::dimension && i < v.size(); ++i) a[i] = v[i];
    return R::Configuration(a);
}

int main(int argc, char **argv)
{
    constexpr std::size_t res = R::resolution;
    vamp::planning::RRTCSettings settings;
    settings.range = 1.25f;
    settings.max_iterations = 10'000'000;
    settings.max_samples = 1'000'000;
    int reps = (argc > 1) ? std::atoi(argv[1]) : 1500;

    // prebuild solvable problems once
    struct Prob { vamp::collision::Environment<DataV> env; R::Configuration start; std::vector<R::Configuration> goals; };
    std::vector<Prob> ps;
    for (const auto &pr : panda_probs) {
        auto env = build_env(pr.prims);
        auto start = to_cfg(pr.start);
        std::vector<R::Configuration> goals; for (const auto &g : pr.goals) goals.push_back(to_cfg(g));
        bool ok_start = vamp::planning::validate_configuration<R, rake>(start, env);
        bool ok_goal = false; for (auto &g : goals) ok_goal |= vamp::planning::validate_configuration<R, rake>(g, env);
        if (ok_start and ok_goal) ps.push_back({std::move(env), start, std::move(goals)});
    }

    std::size_t solved = 0; double sum_iter = 0;
    for (int r = 0; r < reps; ++r)
        for (auto &p : ps) {
            auto rng = std::make_shared<vamp::rng::Halton<R>>();
            auto result = vamp::planning::RRTC<R, rake, res>::solve(p.start, p.goals, p.env, settings, rng);
            solved += result.solved; sum_iter += result.iterations;
        }
    std::printf("panda: probs=%zu reps=%d solved=%zu total-iters=%.0f\n", ps.size(), reps, solved, sum_iter);
    return 0;
}
