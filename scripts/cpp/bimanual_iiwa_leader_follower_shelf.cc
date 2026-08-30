// C++ port of scripts/bimanual_iiwa_leader_follower_shelf.py: leader-follower task-space
// RRTC planning over BimanualIiwa::LeaderFollowerSpace, reproducing the
// iiwa_parameterized_ik_planner branch's BimanualIiwa shelf problem (scripts/
// bimanual_iiwa.py / scripts/cpp/iiwa_try.cc) on the current
// `bimanualiiwa.leader_follower_space` mechanism instead of that branch's old flat
// `use_parameterized_ik`/`ik_parameters` one. See the python script's module docstring
// for the full field-mapping rationale (old ik_parameters[0:3] -> smm, ik_parameters[3:10]
// -> rel_pose).
//
// Usage:
//   vamp_bimanual_iiwa_leader_follower_shelf [n_trials] [range]

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/bimanual_iiwa.hh>
#include <vamp/utils/profiling.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::BimanualIiwa;
using LeaderFollowerSpace = Robot::LeaderFollowerSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, LeaderFollowerSpace>;
using TaskLocalPlanner =
    vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution, LeaderFollowerSpace>;

// Default closure parameters from the old branch's BimanualIiwa::ik_parameters
// ({1.0, 1.0, -1.0, 0.0, 0.0, 0.6, 0.927184, -0.374607, 0.0, 0.0}): first 3 are the
// follower's self-motion-manifold branch (smm), last 7 are the fixed leader-to-follower
// hand offset (rel_pose), as (x, y, z, qx, qy, qz, qw).
constexpr std::array<float, 3> kDefaultSmm = {1.0F, 1.0F, -1.0F};
constexpr std::array<float, 7> kDefaultRelPose = {0.0F, 0.0F, 0.6F, 0.927184F, -0.374607F, 0.0F, 0.0F};

// The three named problem states from the old branch's scripts/bimanual_iiwa.py /
// resources/iiwa/example_points.txt: each is (7 leader/left-arm joint angles, psi) --
// exactly LeaderFollowerSpace::State's 8-element layout.
struct NamedState
{
    std::string label;
    LeaderFollowerSpace::StateArray array;
};

const std::vector<NamedState> kNamedStates = {
    {"bottom",
     {{-0.6430910102907225F,
       1.9156121024586796F,
       -1.7968254667817805F,
       1.2945447141185198F,
       -0.023834531305537934F,
       -0.876966810663043F,
       -1.7041643160834519F,
       1.45F}}},
    {"middle",
     {{-0.5997312520566763F,
       1.489780849654964F,
       -1.4739679827359913F,
       1.2905366081785483F,
       -0.04421061906813227F,
       -0.8793712572715165F,
       -1.1603461715511334F,
       1.45F}}},
    {"top",
     {{-0.1994994216078726F,
       0.9140739951190965F,
       -2.236618320862171F,
       0.5238879195899456F,
       0.7998441913611017F,
       -1.3575398006936048F,
       -1.0153092816310436F,
       2.41F}}},
};

// Shelf cuboids from the old branch's resources/iiwa/cuboids/shelf_drake.txt: each row is
// (x, y, z, dx, dy, dz) -- position + *full* extents (halved below to match
// vamp::collision::factory::cuboid's half-extent convention).
const std::vector<std::array<float, 6>> kShelfCuboids = {
    {0.8F, 0.3825F, 0.3F, 0.4F, 1.0F, 0.014F},
    {0.8F, 0.3825F, 0.58F, 0.4F, 1.0F, 0.014F},
    {1.0F, 0.3825F, 0.45F, 0.03F, 1.0F, 0.9F},
    {0.4F, 0.3825F, -0.2F, 5.0F, 5.0F, 0.2F},
};

auto build_environment() -> EnvironmentInput
{
    EnvironmentInput environment;
    for (const auto &c : kShelfCuboids)
    {
        auto cuboid = vamp::collision::factory::cuboid::array(
            std::array<float, 3>{c[0], c[1], c[2]},
            std::array<float, 3>{0.0F, 0.0F, 0.0F},
            std::array<float, 3>{c[3] / 2.0F, c[4] / 2.0F, c[5] / 2.0F});
        environment.cuboids.emplace_back(cuboid);
    }

    return environment;
}

// IK-resolve a task-space state and collision-check the resulting ambient configuration.
// Returns (valid, ambient_block); ambient_block is only meaningful when valid is true.
auto resolve_and_validate(
    const LeaderFollowerSpace::State &state,
    const EnvironmentVector &environment_v) -> std::pair<bool, Robot::ConfigurationBlock<rake>>
{
    LeaderFollowerSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < LeaderFollowerSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [valid, ambient_block] = LeaderFollowerSpace::resolve_block<rake>(block);
    if (not valid)
    {
        return {false, ambient_block};
    }

    const bool collision_free = (environment_v.attachments.empty()) ?
                                     Robot::template fkcc<rake>(environment_v, ambient_block) :
                                     Robot::template fkcc_attach<rake>(environment_v, ambient_block);

    return {collision_free, ambient_block};
}

// Per-(start, goal)-pair planning statistics, accumulated over every trial that drew that
// pair. Time/iteration stats are computed over solved trials only (an unsolved trial has
// no meaningful path cost, but its time/iterations are still noise dominated by whatever
// iteration cap RRTC hit, not by the problem's actual difficulty); `total` counts every
// trial that drew the pair, solved or not.
struct PairStats
{
    std::size_t total = 0;
    std::size_t solved = 0;
    std::vector<double> times_ms;
    std::vector<double> iterations;
};

auto mean(const std::vector<double> &v) -> double
{
    if (v.empty())
    {
        return 0.0;
    }

    return std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
}

auto median(std::vector<double> v) -> double
{
    if (v.empty())
    {
        return 0.0;
    }

    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return (n % 2 == 1) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
}

auto main(int argc, char **argv) -> int
{
    const std::size_t n_trials = (argc > 1) ? static_cast<std::size_t>(std::stoul(argv[1])) : 100;
    const float range = (argc > 2) ? std::stof(argv[2]) : 0.5F;

    std::cout << std::boolalpha;
    std::cout << "Robot::dimension (ambient/joint space): " << Robot::dimension << std::endl;
    std::cout << "LeaderFollowerSpace::dimension (task space): " << LeaderFollowerSpace::dimension << std::endl;

    LeaderFollowerSpace::set_smm(kDefaultSmm);
    LeaderFollowerSpace::rel_pose = kDefaultRelPose;

    const EnvironmentInput environment = build_environment();
    const EnvironmentVector environment_v(environment);
    std::cout << "Environment has " << environment_v.cuboids.size() << " cuboids (shelf)." << std::endl;

    std::cout << "\n--- Checking the three named problem states ---" << std::endl;
    std::vector<NamedState> valid_states;
    for (const auto &named : kNamedStates)
    {
        LeaderFollowerSpace::State state(named.array.data());
        const auto [valid, ambient_block] = resolve_and_validate(state, environment_v);
        std::cout << named.label << ": resolve+validate = " << valid << std::endl;
        if (valid)
        {
            valid_states.push_back(named);
        }
    }

    if (valid_states.size() < 2)
    {
        std::cout << "Fewer than two named states are valid under this rel_pose/smm; nothing to plan."
                   << std::endl;
        return 0;
    }

    vamp::planning::RRTCSettings settings;
    settings.range = range;
    settings.max_iterations = 1000000;

    std::mt19937 rng_engine(0);
    std::uniform_int_distribution<std::size_t> pick(0, valid_states.size() - 1);

    std::size_t solved_count = 0;
    std::map<std::string, PairStats> stats_by_pair;
    auto task_rng = std::make_shared<vamp::rng::Halton<Robot, LeaderFollowerSpace>>();

    for (std::size_t trial = 0; trial < n_trials; ++trial)
    {
        std::size_t start_idx = pick(rng_engine);
        std::size_t goal_idx = pick(rng_engine);
        while (goal_idx == start_idx and valid_states.size() > 1)
        {
            goal_idx = pick(rng_engine);
        }

        const auto &start_named = valid_states[start_idx];
        const auto &goal_named = valid_states[goal_idx];
        const std::string pair_label = start_named.label + " -> " + goal_named.label;
        auto &pair_stats = stats_by_pair[pair_label];
        ++pair_stats.total;

        LeaderFollowerSpace::State start_state(start_named.array.data());
        LeaderFollowerSpace::State goal_state(goal_named.array.data());


        const auto t0 = std::chrono::steady_clock::now();
        auto result = TaskRRTC::solve<TaskLocalPlanner>(
            start_state, goal_state, environment_v, settings, task_rng, TaskLocalPlanner());
        const auto elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        std::cout << "trial " << trial << " (" << pair_label << "): solved=" << result.solved
                   << ", iterations=" << result.iterations << ", " << elapsed_ms << " ms";

        if (result.solved)
        {
            ++solved_count;
            ++pair_stats.solved;
            pair_stats.times_ms.push_back(elapsed_ms);
            pair_stats.iterations.push_back(static_cast<double>(result.iterations));

            const float cost_before = result.path.cost();
            vamp::planning::ShortcutSettings shortcut_settings;
            vamp::planning::shortcut_path<Robot, rake, Robot::resolution, TaskLocalPlanner, LeaderFollowerSpace>(
                result.path, environment_v, shortcut_settings, TaskLocalPlanner());

            std::cout << ", cost " << cost_before << " -> " << result.path.cost() << ", path size "
                       << result.path.size();
        }

        std::cout << std::endl;
    }

    std::cout << "\nSolved " << solved_count << " / " << n_trials << " trials." << std::endl;

    std::cout << "\n--- Per-pair statistics (time/iterations over solved trials) ---" << std::endl;
    for (const auto &[pair_label, pair_stats] : stats_by_pair)
    {
        std::cout << pair_label << ":" << std::endl;
        std::cout << "  total problems: " << pair_stats.total << " (solved " << pair_stats.solved << ")"
                   << std::endl;
        std::cout << "  mean time: " << mean(pair_stats.times_ms) << " ms" << std::endl;
        std::cout << "  median time: " << median(pair_stats.times_ms) << " ms" << std::endl;
        std::cout << "  mean iterations: " << mean(pair_stats.iterations) << std::endl;
        std::cout << "  median iterations: " << median(pair_stats.iterations) << std::endl;
    }

#ifdef VAMP_PROFILING
    std::cout << "\n--- Kernel profiling (aggregated over all trials) ---" << std::endl;
    vamp::utils::profiling::report(std::cout);
#endif

    return 0;
}
