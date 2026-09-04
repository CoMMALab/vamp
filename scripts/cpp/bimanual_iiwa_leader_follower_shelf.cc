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
//   vamp_bimanual_iiwa_leader_follower_shelf [n_trials] [range] [trajectory_dir] [results_csv]
//
// Every solved trial's shortcut path is resolved from LeaderFollowerSpace states to full
// 14-dof ambient configurations, interpolated to Robot::resolution, and dumped to
// <trajectory_dir>/trial_<n>_<start>_to_<goal>.txt: one waypoint per line, its
// Robot::dimension joint values comma-separated (same format
// vamp_bimanual_example_iiwa.cc wrote its single best trajectory in), for offline
// playback/visualization.
//
// Every trial (solved or not) also gets one row in <results_csv> --
// method,trial,pair,solved,planning_time_ms,iterations,shortcut_time_ms,config_distance,
// eef_distance -- for offline analysis/plotting (see scripts/plot_bimanual_results.py).
// config_distance/eef_distance are the SHORTCUT path's length only, computed on the
// resolved ambient waypoints (task-space distance isn't ambient config-space distance):
// config_distance is summed Robot::Configuration::distance() (ambient 14-dof Euclidean)
// between consecutive resolved waypoints; eef_distance is each hand's SE3 path length
// (translation distance and rotation angle combined in quadrature, matching
// iiwa_maze_tsr_benchmark.cc's se3_distance), averaged over the two hands since they move
// together under the closure constraint.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/bimanual_iiwa.hh>
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
    std::vector<double> shortcut_times_ms;
    std::vector<double> config_distances;
    std::vector<double> eef_distances;
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

// Resolve a task-space path (LeaderFollowerSpace states) to full 14-dof ambient
// configurations, in order. A waypoint that fails to resolve (shouldn't happen on an
// already-planned path, but resolve_block can fail numerically) is skipped rather than
// aborting.
auto resolve_ambient_waypoints(const vamp::planning::Path<Robot, LeaderFollowerSpace> &path)
    -> std::vector<Robot::ConfigurationArray>
{
    std::vector<Robot::ConfigurationArray> waypoints;
    waypoints.reserve(path.size());
    for (const auto &state : path)
    {
        LeaderFollowerSpace::StateBlock<rake> block;
        for (std::size_t i = 0; i < LeaderFollowerSpace::dimension; ++i)
        {
            block[i] = state.broadcast(i);
        }

        auto [valid, ambient_block] = LeaderFollowerSpace::resolve_block<rake>(block);
        if (not valid)
        {
            continue;
        }

        Robot::ConfigurationArray array{};
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            array[i] = ambient_block[{i, 0}];
        }

        waypoints.push_back(array);
    }

    return waypoints;
}

// One waypoint per line, its Robot::dimension joint values comma-separated -- matches
// vamp_bimanual_example_iiwa.cc's trajectory.txt format.
void write_ambient_path(const std::vector<Robot::ConfigurationArray> &waypoints, const std::filesystem::path &file)
{
    std::ofstream out(file);
    for (const auto &array : waypoints)
    {
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            if (i != 0)
            {
                out << ",";
            }

            out << array[i];
        }

        out << "\n";
    }
}

// Combined SE3 distance (translation distance and rotation angle in quadrature) between
// two poses -- same formula as iiwa_maze_tsr_benchmark.cc's se3_distance.
auto se3_distance(
    const Eigen::Vector3f &ta,
    const Eigen::Quaternionf &qa,
    const Eigen::Vector3f &tb,
    const Eigen::Quaternionf &qb) -> float
{
    const float translation_distance = (tb - ta).norm();
    float dot = std::abs(static_cast<float>(qa.dot(qb)));
    dot = std::min(1.0F, dot);
    const float rotation_distance = 2.0F * std::acos(dot);
    return std::sqrt(translation_distance * translation_distance + rotation_distance * rotation_distance);
}

struct PathMetrics
{
    float config_distance;
    float eef_distance;
};

// config_distance: summed ambient (14-dof) Euclidean distance between consecutive
// resolved waypoints. eef_distance: each hand's summed SE3 path length, averaged over
// both hands (they move together under the closure constraint, so this isn't
// double-counting so much as reporting "how far did the coupled pair of hands travel").
auto compute_path_metrics(const std::vector<Robot::ConfigurationArray> &waypoints) -> PathMetrics
{
    PathMetrics metrics{0.0F, 0.0F};
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
        metrics.config_distance += Robot::Configuration(waypoints[i]).distance(Robot::Configuration(waypoints[i + 1]));

        for (const std::size_t hand : {std::size_t{0}, std::size_t{1}})
        {
            const auto a = Robot::eefk(waypoints[i], hand);
            const auto b = Robot::eefk(waypoints[i + 1], hand);
            metrics.eef_distance +=
                0.5F *
                se3_distance(a.translation(), Eigen::Quaternionf(a.rotation()), b.translation(), Eigen::Quaternionf(b.rotation()));
        }
    }

    return metrics;
}

auto main(int argc, char **argv) -> int
{
    const std::size_t n_trials = (argc > 1) ? static_cast<std::size_t>(std::stoul(argv[1])) : 100;
    const float range = (argc > 2) ? std::stof(argv[2]) : 0.5F;
    const std::filesystem::path trajectory_dir =
        (argc > 3) ? argv[3] : "trajectories/bimanual_iiwa_leader_follower_shelf";
    const std::filesystem::path results_csv_path =
        (argc > 4) ? argv[4] : "results/bimanual_iiwa_leader_follower_shelf.csv";
    std::filesystem::create_directories(trajectory_dir);
    std::filesystem::create_directories(results_csv_path.parent_path());
    std::cout << "Writing shortcut trajectories to: " << trajectory_dir << std::endl;
    std::cout << "Writing per-trial results to: " << results_csv_path << std::endl;

    std::ofstream results_csv(results_csv_path);
    if (not results_csv)
    {
        std::cerr << "Failed to open results CSV for writing: " << results_csv_path << std::endl;
        return 1;
    }

    results_csv << "method,trial,pair,solved,planning_time_ms,iterations,shortcut_time_ms,config_distance,"
                   "eef_distance\n";
    results_csv.flush();

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

        results_csv << "leader_follower," << trial << ",\"" << pair_label << "\"," << result.solved << ","
                    << elapsed_ms << "," << result.iterations << ",";

        if (result.solved)
        {
            ++solved_count;
            ++pair_stats.solved;
            pair_stats.times_ms.push_back(elapsed_ms);
            pair_stats.iterations.push_back(static_cast<double>(result.iterations));

            const float cost_before = result.path.cost();
            const auto shortcut_t0 = std::chrono::steady_clock::now();
            vamp::planning::ShortcutSettings shortcut_settings;
            vamp::planning::shortcut_path<Robot, rake, Robot::resolution, TaskLocalPlanner, LeaderFollowerSpace>(
                result.path, environment_v, shortcut_settings, TaskLocalPlanner());
            const auto shortcut_elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - shortcut_t0).count();
            pair_stats.shortcut_times_ms.push_back(shortcut_elapsed_ms);

            std::cout << ", cost " << cost_before << " -> " << result.path.cost() << ", path size "
                       << result.path.size() << ", shortcut " << shortcut_elapsed_ms << " ms";

            result.path.interpolate_to_resolution(Robot::resolution);
            const auto ambient_waypoints = resolve_ambient_waypoints(result.path);
            const auto metrics = compute_path_metrics(ambient_waypoints);
            pair_stats.config_distances.push_back(metrics.config_distance);
            pair_stats.eef_distances.push_back(metrics.eef_distance);

            std::cout << ", config distance " << metrics.config_distance << ", eef distance " << metrics.eef_distance;

            const std::string filename =
                "trial_" + std::to_string(trial) + "_" + start_named.label + "_to_" + goal_named.label + ".txt";
            write_ambient_path(ambient_waypoints, trajectory_dir / filename);

            results_csv << shortcut_elapsed_ms << "," << metrics.config_distance << "," << metrics.eef_distance;
        }

        results_csv << "\n";
        results_csv.flush();
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
        std::cout << "  mean shortcut time: " << mean(pair_stats.shortcut_times_ms) << " ms" << std::endl;
        std::cout << "  median shortcut time: " << median(pair_stats.shortcut_times_ms) << " ms" << std::endl;
        std::cout << "  mean config distance: " << mean(pair_stats.config_distances) << std::endl;
        std::cout << "  median config distance: " << median(pair_stats.config_distances) << std::endl;
        std::cout << "  mean eef distance: " << mean(pair_stats.eef_distances) << std::endl;
        std::cout << "  median eef distance: " << median(pair_stats.eef_distances) << std::endl;
    }

    return 0;
}
