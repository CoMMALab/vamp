// ParameterizedSpace counterpart of bimanual_iiwa_leader_follower_shelf.cc: same shelf
// scenario, but planning over BimanualIiwa::ParameterizedSpace (both arms IK'd from a
// shared mid-frame task-space pose: t_mid_pose(7) + psi_left(1) + psi_right(1)) instead of
// LeaderFollowerSpace (leader sampled directly in joint space, only the follower IK'd).
//
// The fixed hand-to-mid-frame offsets (t_mid_left / t_mid_right) are derived by calling
// ParameterizedSpace::compute_mid_pose() on a known-valid reference ambient configuration
// (kReferenceConfig below, the same "bottom" pose used elsewhere), rather than hand-typed
// literals -- this is the "call compute midpose on the start pose to get and fix the
// relative transforms" step.
//
// TODO(fill in): generate_problem_states() below is a stub. Unlike LeaderFollowerSpace
// (whose named bottom/middle/top states are directly hand-authored 8-dim task-space
// states, reused verbatim from the reference project), ParameterizedSpace's psi_left/
// psi_right have no closed-form inverse from an ambient configuration -- resolve_block()
// only goes task-space -> ambient, not the other way -- so the bottom/middle/top named
// states can't be reconstructed the same way. Two options were discussed and left for
// later:
//   (a) Draw random valid states: ParameterizedSpace::sample() -> resolve_block<rake> ->
//       collision-check, retrying until a valid (start, goal) pair is found per trial.
//   (b) Numerically solve for psi_left/psi_right per named pose (bottom/middle/top) so
//       resolve_block's ambient output FK-matches those references, reproducing the same
//       named problems the other two benchmark files use.
// Until generate_problem_states() returns at least two states, main() prints a notice and
// exits without planning -- everything else in this file (environment, mid-pose fixing,
// RRTC/local-planner wiring, stats, trajectory dump) is fully wired up and ready.
//
// Usage:
//   vamp_bimanual_iiwa_parameterized_shelf [n_trials] [range] [trajectory_dir]

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
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
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::
    ParameterizedLocalPlanner<Robot, rake, Robot::resolution, ParameterizedSpace>;

// A known-valid full 14-dof ambient configuration for this shelf scenario (the "bottom"
// pose from resources/start_end_points/bimanual_iiwa.txt, same as used in
// bimanual_iiwa_projection_shelf.cc / bimanual_iiwa_leader_follower_shelf.cc) -- the
// reference `compute_mid_pose()` is called on to fix t_mid_left/t_mid_right.
constexpr Robot::ConfigurationArray kReferenceConfig = {{
    -0.6430910102907225F,   1.9156121024586796F, -1.7968254667817805F, 1.2945447141185198F,
    -0.023834531305537934F, -0.876966810663043F, -1.7041643160834519F, 0.7137057906077047F,
    1.9675104645685881F,    1.728621289619734F,  1.297295659493332F,   0.16350904416645204F,
    -0.9339939993465736F,   2.3860537824177745F}};

// Shelf cuboids, identical to bimanual_iiwa_leader_follower_shelf.cc's kShelfCuboids: each
// row is (x, y, z, dx, dy, dz) -- position + *full* extents (halved below to match
// vamp::collision::factory::cuboid's half-extent convention).
const std::vector<std::array<float, 6>> kShelfCuboids = {
    {0.8F, 0.3825F, 0.3F, 0.4F, 1.0F, 0.014F},
    {0.8F, 0.3825F, 0.58F, 0.4F, 1.0F, 0.014F},
    {1.0F, 0.3825F, 0.45F, 0.03F, 1.0F, 0.9F},
    {0.4F, 0.3825F, -0.2F, 5.0F, 5.0F, 0.2F},
};

// Sanity-check the reference ambient configuration directly against the environment (no
// IK, no task-space resolution -- it's already a real joint configuration).
auto is_config_valid(const Robot::Configuration &q, const EnvironmentVector &environment_v) -> bool
{
    Robot::ConfigurationBlock<rake> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return (environment_v.attachments.empty()) ? Robot::template fkcc<rake>(environment_v, block) :
                                                  Robot::template fkcc_attach<rake>(environment_v, block);
}

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

struct NamedState
{
    std::string label;
    ParameterizedSpace::StateArray array;
};

// TODO(fill in): see file header. Returns fewer than 2 states until real problem
// generation is implemented, which main() treats as "nothing to plan."
auto generate_problem_states(const EnvironmentVector & /*environment_v*/) -> std::vector<NamedState>
{
    return {};
}

// IK-resolve a task-space state and collision-check the resulting ambient configuration,
// same as bimanual_iiwa_leader_follower_shelf.cc's resolve_and_validate.
auto resolve_and_validate(const ParameterizedSpace::State &state, const EnvironmentVector &environment_v)
    -> std::pair<bool, Robot::ConfigurationBlock<rake>>
{
    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
    if (not valid)
    {
        return {false, ambient_block};
    }

    const bool collision_free = (environment_v.attachments.empty()) ?
                                     Robot::template fkcc<rake>(environment_v, ambient_block) :
                                     Robot::template fkcc_attach<rake>(environment_v, ambient_block);

    return {collision_free, ambient_block};
}

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

// Resolve a task-space path (ParameterizedSpace states) to full 14-dof ambient
// configurations and dump one waypoint per line, comma-separated -- matches
// vamp_bimanual_example_iiwa.cc's trajectory.txt format. A waypoint that fails to resolve
// (shouldn't happen on an already-planned path, but resolve_block can fail numerically)
// is skipped rather than aborting the whole dump.
void write_task_path(const vamp::planning::Path<Robot, ParameterizedSpace> &path, const std::filesystem::path &file)
{
    std::ofstream out(file);
    for (const auto &state : path)
    {
        ParameterizedSpace::StateBlock<rake> block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            block[i] = state.broadcast(i);
        }

        auto [valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
        if (not valid)
        {
            continue;
        }

        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            if (i != 0)
            {
                out << ",";
            }

            out << ambient_block[{i, 0}];
        }

        out << "\n";
    }
}

auto main(int argc, char **argv) -> int
{
    const std::size_t n_trials = (argc > 1) ? static_cast<std::size_t>(std::stoul(argv[1])) : 100;
    const float range = (argc > 2) ? std::stof(argv[2]) : 0.5F;
    const std::filesystem::path trajectory_dir =
        (argc > 3) ? argv[3] : "trajectories/bimanual_iiwa_parameterized_shelf";
    std::filesystem::create_directories(trajectory_dir);
    std::cout << "Writing shortcut trajectories to: " << trajectory_dir << std::endl;

    std::cout << std::boolalpha;
    std::cout << "Robot::dimension (ambient/joint space): " << Robot::dimension << std::endl;
    std::cout << "ParameterizedSpace::dimension (task space): " << ParameterizedSpace::dimension << std::endl;

    const EnvironmentInput environment = build_environment();
    const EnvironmentVector environment_v(environment);
    std::cout << "Environment has " << environment_v.cuboids.size() << " cuboids (shelf)." << std::endl;

    if (not is_config_valid(Robot::Configuration(kReferenceConfig), environment_v))
    {
        std::cout << "Warning: kReferenceConfig is in collision under the current shelf geometry." << std::endl;
    }

    // Fix the hand-to-mid-frame offsets from the reference configuration -- "call compute
    // midpose on the start pose to get and fix the relative transforms."
    ParameterizedSpace::compute_mid_pose(kReferenceConfig);
    std::cout << "t_mid_left:  ";
    for (const auto v : ParameterizedSpace::t_mid_left)
    {
        std::cout << v << " ";
    }
    std::cout << "\nt_mid_right: ";
    for (const auto v : ParameterizedSpace::t_mid_right)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "\n--- Generating problem states (TODO: see file header) ---" << std::endl;
    std::vector<NamedState> valid_states;
    for (const auto &named : generate_problem_states(environment_v))
    {
        ParameterizedSpace::State state(named.array.data());
        const auto [valid, ambient_block] = resolve_and_validate(state, environment_v);
        std::cout << named.label << ": resolve+validate = " << valid << std::endl;
        if (valid)
        {
            valid_states.push_back(named);
        }
    }

    if (valid_states.size() < 2)
    {
        std::cout << "\ngenerate_problem_states() hasn't been filled in yet (see the TODO at the top of "
                     "this file) -- nothing to plan."
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
    auto task_rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();

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

        ParameterizedSpace::State start_state(start_named.array.data());
        ParameterizedSpace::State goal_state(goal_named.array.data());

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
            vamp::planning::shortcut_path<Robot, rake, Robot::resolution, TaskLocalPlanner, ParameterizedSpace>(
                result.path, environment_v, shortcut_settings, TaskLocalPlanner());

            std::cout << ", cost " << cost_before << " -> " << result.path.cost() << ", path size "
                       << result.path.size();

            result.path.interpolate_to_resolution(Robot::resolution);
            const std::string filename =
                "trial_" + std::to_string(trial) + "_" + start_named.label + "_to_" + goal_named.label + ".txt";
            write_task_path(result.path, trajectory_dir / filename);
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
