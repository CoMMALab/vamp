// End-to-end RRTC on real MBM problems: baseline fkcc vs fkcc_sincos (selected at compile time
// by -DVAMP_FKCC_SINCOS in fkcc_block). Same problems, same deterministic Halton sampler, 5
// timing trials/problem, 10M max iterations. Reports solved/total, planner time, iterations.
#include <array>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/random/halton.hh>
#if defined(VAMP_XOSHIRO)
#include <vamp/random/xoshiro128.hh>
template <class R> using Sampler = vamp::rng::Xoshiro128<R>;
static const char *sampler_tag = "xo";
#elif defined(VAMP_XORSHIFT_NATIVE)
#include <vamp/random/xorshift_native.hh>
template <class R> using Sampler = vamp::rng::XORShiftNative<R>;
static const char *sampler_tag = "xsn";
#elif defined(VAMP_XORSHIFT)
#include <vamp/random/xorshift.hh>
template <class R> using Sampler = vamp::rng::XORShift<R>;
static const char *sampler_tag = "xs";
#else
template <class R> using Sampler = vamp::rng::Halton<R>;
static const char *sampler_tag = "ht";
#endif
#include "ur5_e.hh"
#include "panda_e.hh"
#include "fetch_e.hh"
#include "baxter_e.hh"
#include "mbm_problems.hh"

constexpr std::size_t rake = vamp::FloatVectorWidth;
using DataV = vamp::FloatVector<rake>;

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

static std::uint32_t g_seed = 0;
static int g_trials = 10;

template <class R>
static std::shared_ptr<Sampler<R>> make_sampler(std::uint32_t seed)
{
#if defined(VAMP_XOSHIRO) || defined(VAMP_XORSHIFT_NATIVE)
    return std::make_shared<Sampler<R>>(seed ? seed : 0x9e3779b9u);
#elif defined(VAMP_XORSHIFT)
    // two distinct odd keys derived from the seed
    return std::make_shared<Sampler<R>>(
        static_cast<std::uint64_t>(seed) * 2862933555777941757ull + 3ull,
        static_cast<std::uint64_t>(seed) * 3202034522624059733ull + 7ull);
#else
    (void)seed;  // Halton is deterministic; seed ignored
    return std::make_shared<Sampler<R>>();
#endif
}

template <class R>
static typename R::Configuration to_cfg(const std::vector<float> &v)
{
    std::array<float, R::dimension> a{};
    for (std::size_t i = 0; i < R::dimension && i < v.size(); ++i) a[i] = v[i];
    return typename R::Configuration(a);
}

template <class R>
static void run(const char *name, float range, const std::vector<MbmProb> &probs)
{
    constexpr std::size_t res = R::resolution;
    vamp::planning::RRTCSettings settings;
    settings.range = range;
    settings.max_iterations = 10'000'000;
    settings.max_samples = 1'000'000;
    const int TRIALS = g_trials;

    std::size_t valid = 0, solved = 0;
    std::vector<double> times_us; std::vector<double> iters;
    for (const auto &pr : probs) {
        auto env = build_env(pr.prims);
        auto start = to_cfg<R>(pr.start);
        std::vector<typename R::Configuration> goals;
        for (const auto &g : pr.goals) goals.push_back(to_cfg<R>(g));
        // sanity: start + at least one goal collision-free (validates the joint ordering)
        bool ok_start = vamp::planning::validate_configuration<R, rake>(start, env);
        bool ok_goal = false; for (auto &g : goals) ok_goal |= vamp::planning::validate_configuration<R, rake>(g, env);
        if (not ok_start or not ok_goal) continue;
        ++valid;
        double best = 1e18; bool any = false; double it = 0;
        for (int t = 0; t < TRIALS; ++t) {
            auto rng = make_sampler<R>(g_seed);
            auto result = vamp::planning::RRTC<R, rake, res>::solve(start, goals, env, settings, rng);
            best = std::min(best, result.nanoseconds / 1000.0);   // us, min over timing trials
            any = result.solved; it = (double)result.iterations;
        }
        if (any) { ++solved; times_us.push_back(best); iters.push_back(it); }
    }
    auto pct = [&](std::vector<double> &v, double q) { if (v.empty()) return 0.0; std::sort(v.begin(), v.end()); return v[(std::size_t)(q * (v.size() - 1))]; };
    double mean = 0; for (double x : times_us) mean += x; if (!times_us.empty()) mean /= times_us.size();
#if defined(VAMP_RECUR_SINCOS)
    const char *cfg = "rec+sc ";
#elif defined(VAMP_RECUR)
    const char *cfg = "recur  ";
#elif defined(VAMP_FKCC_SINCOS)
    const char *cfg = "sincos ";
#else
    const char *cfg = "base   ";
#endif
    std::printf("%-6s [%s/%s] valid=%zu solved=%zu/%zu  planner us: mean=%.1f  p50=%.1f  p95=%.1f  iters p50=%.0f\n",
                name, sampler_tag, cfg, valid, solved, probs.size(), mean, pct(times_us, 0.5), pct(times_us, 0.95), pct(iters, 0.5));
}

int main(int argc, char **argv)
{
    if (argc > 1) g_seed = static_cast<std::uint32_t>(std::strtoul(argv[1], nullptr, 10));
    if (argc > 2) g_trials = std::atoi(argv[2]);
    const char *only = (argc > 3) ? argv[3] : nullptr;  // optional: run one robot
    if (!only || std::string(only) == "ur5") run<vamp::robots::Ur5>("ur5", 1.5f, ur5_probs);
    if (!only || std::string(only) == "panda") run<vamp::robots::PandaE>("panda", 1.25f, panda_probs);
    if (!only || std::string(only) == "fetch") run<vamp::robots::FetchE>("fetch", 1.0f, fetch_probs);
    if (!only || std::string(only) == "baxter") run<vamp::robots::BaxterE>("baxter", 0.5f, baxter_probs);
    return 0;
}
