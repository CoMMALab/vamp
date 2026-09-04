// ParameterizedSpace counterpart of bimanual_iiwa_leader_follower_shelf.cc: same shelf
// scenario, but planning over BimanualIiwa::ParameterizedSpace (both arms IK'd from a
// shared mid-frame task-space pose: t_mid_pose(7) + psi_left(1) + psi_right(1)) instead of
// LeaderFollowerSpace (leader sampled directly in joint space, only the follower IK'd).
//
// The fixed hand-to-mid-frame offsets (t_mid_left / t_mid_right) are derived by calling
// ParameterizedSpace::compute_mid_pose() on a known-valid reference ambient configuration
// (kNamedAmbientConfigs' "bottom" entry below, same pose used elsewhere), rather than
// hand-typed literals -- this is the "call compute midpose on the start pose to get and
// fix the relative transforms" step.
//
// The bottom/middle/top named problem states are reconstructed as follows: each
// reference's mid-frame *position* (x, y, z) is read off directly via forward kinematics
// (the midpoint of the two hands' translations, identity orientation -- same convention
// compute_mid_pose() itself uses for T_w_mid), since that part has a closed form. There's
// no closed-form inverse for psi_left/psi_right, though (resolve_block() only goes
// task-space -> ambient, not the other way), so those two are found by rejection
// sampling: keep drawing random (psi_left, psi_right) pairs at that fixed mid-frame
// position until resolve_block() reports a valid, collision-free ambient configuration.
// This reproduces each named pose's *position* exactly but not necessarily the same joint
// angles as the reference configuration -- any valid arm posture reaching that mid-frame
// position satisfies the problem.
//
// Usage:
//   vamp_bimanual_iiwa_parameterized_shelf [n_trials] [range] [trajectory_dir] [results_csv]
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
#include <optional>
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

struct NamedAmbientConfig
{
    std::string label;
    Robot::ConfigurationArray array;
};

// The three known-valid full 14-dof ambient configurations for this shelf scenario, from
// resources/start_end_points/bimanual_iiwa.txt (same ones used in
// bimanual_iiwa_projection_shelf.cc / bimanual_iiwa_leader_follower_shelf.cc). "bottom" is
// the reference `compute_mid_pose()` is called on to fix t_mid_left/t_mid_right; all three
// are used below to seed the named problem states' mid-frame positions.
const std::vector<NamedAmbientConfig> kNamedAmbientConfigs = {
    {"bottom",
     {{-0.6430910102907225F,   1.9156121024586796F, -1.7968254667817805F, 1.2945447141185198F,
       -0.023834531305537934F, -0.876966810663043F, -1.7041643160834519F, 0.7137057906077047F,
       1.9675104645685881F,    1.728621289619734F,  1.297295659493332F,   0.16350904416645204F,
       -0.9339939993465736F,   2.3860537824177745F}}},
    {"middle",
     {{-0.5997312520566763F,  1.489780849654964F,   -1.4739679827359913F, 1.2905366081785483F,
       -0.04421061906813227F, -0.8793712572715165F, -1.1603461715511334F, 0.6478085507050495F,
       1.5420182953666222F,   1.4012033615433137F,  1.2931428651414443F,  0.14691416900781198F,
       -0.9348727702323719F,  1.864705746202578F}}},
    {"top",
     {{-0.1994994216078726F, 0.9140739951190965F,  -2.236618320862171F, 0.5238879195899456F,
       0.7998441913611017F,  -1.3575398006936048F, -1.0153092816310436F, 0.24160750214358093F,
       0.9022665362154509F,  2.2897413507442623F,  0.5286854254175202F, -0.8636815969096344F,
       -1.4123134496102092F, 1.7961899529805394F}}},
};

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

// IK-resolve a task-space state and collision-check the resulting ambient configuration,
// same as bimanual_iiwa_leader_follower_shelf.cc's resolve_and_validate. resolve_block()
// broadcasts the single state to every SIMD lane and reports valid only if every lane
// (i.e. this one state, replicated) succeeds, so this is a single-candidate check -- no
// per-lane fan-out.
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

// Mid-frame position (x, y, z) for a reference ambient configuration: the midpoint of the
// two hands' translations, same convention compute_mid_pose() uses internally for T_w_mid
// (position only -- no closed-form inverse gives back an orientation or psi here).
auto mid_frame_position(const Robot::ConfigurationArray &q) -> std::array<float, 3>
{
    const auto left = Robot::eefk(q, 0);
    const auto right = Robot::eefk(q, 1);
    const Eigen::Vector3f mid = 0.5F * (left.translation() + right.translation());
    return {mid.x(), mid.y(), mid.z()};
}

// Reject-sample (psi_left, psi_right) at a fixed mid-frame position/orientation until
// resolve_block() reports a valid, collision-free ambient configuration, or give up after
// max_attempts. The found state reaches the same mid-frame position as the reference
// configuration `label` was read off, but not necessarily via the same joint angles --
// any valid arm posture there satisfies the problem.
auto sample_named_state(
    const std::string &label,
    const std::array<float, 3> &mid_position,
    const EnvironmentVector &environment_v,
    std::mt19937 &rng_engine,
    std::size_t max_attempts) -> std::optional<NamedState>
{
    constexpr float kPi = 3.14159265358979323846F;
    std::uniform_real_distribution<float> psi_dist(-kPi, kPi);

    for (std::size_t attempt = 0; attempt < max_attempts; ++attempt)
    {
        const float psi_left = psi_dist(rng_engine);
        const float psi_right = psi_dist(rng_engine);
        NamedState candidate{
            label,
            {{mid_position[0], mid_position[1], mid_position[2], 0.0F, 0.0F, 0.0F, 1.0F, psi_left, psi_right}}};

        ParameterizedSpace::State state(candidate.array.data());
        const auto [valid, ambient_block] = resolve_and_validate(state, environment_v);
        static_cast<void>(ambient_block);
        if (valid)
        {
            std::cout << label << ": found valid (psi_left=" << psi_left << ", psi_right=" << psi_right
                       << ") after " << attempt + 1 << " attempt(s)." << std::endl;
            return candidate;
        }
    }

    std::cout << label << ": no valid psi_left/psi_right found in " << max_attempts << " attempts." << std::endl;
    return std::nullopt;
}

auto generate_problem_states(const EnvironmentVector &environment_v) -> std::vector<NamedState>
{
    constexpr std::size_t kMaxAttemptsPerState = 5000;
    std::mt19937 rng_engine(0);

    std::vector<NamedState> states;
    for (const auto &named : kNamedAmbientConfigs)
    {
        const auto mid_position = mid_frame_position(named.array);
        if (auto state = sample_named_state(named.label, mid_position, environment_v, rng_engine, kMaxAttemptsPerState))
        {
            states.push_back(*state);
        }
    }

    return states;
}

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

// Resolve a task-space path (ParameterizedSpace states) to full 14-dof ambient
// configurations, in order. A waypoint that fails to resolve (shouldn't happen on an
// already-planned path, but resolve_block can fail numerically) is skipped rather than
// aborting.
auto resolve_ambient_waypoints(const vamp::planning::Path<Robot, ParameterizedSpace> &path)
    -> std::vector<Robot::ConfigurationArray>
{
    std::vector<Robot::ConfigurationArray> waypoints;
    waypoints.reserve(path.size());
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
        (argc > 3) ? argv[3] : "trajectories/bimanual_iiwa_parameterized_shelf";
    const std::filesystem::path results_csv_path =
        (argc > 4) ? argv[4] : "results/bimanual_iiwa_parameterized_shelf.csv";
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
    std::cout << "ParameterizedSpace::dimension (task space): " << ParameterizedSpace::dimension << std::endl;

    const EnvironmentInput environment = build_environment();
    const EnvironmentVector environment_v(environment);
    std::cout << "Environment has " << environment_v.cuboids.size() << " cuboids (shelf)." << std::endl;

    const auto &reference = kNamedAmbientConfigs.front();  // "bottom"
    if (not is_config_valid(Robot::Configuration(reference.array), environment_v))
    {
        std::cout << "Warning: reference config '" << reference.label
                   << "' is in collision under the current shelf geometry." << std::endl;
    }

    // Fix the hand-to-mid-frame offsets from the reference configuration -- "call compute
    // midpose on the start pose to get and fix the relative transforms."
    ParameterizedSpace::compute_mid_pose(reference.array);
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

    std::cout << "\n--- Generating problem states (mid-frame position fixed per reference, psi "
                 "rejection-sampled) ---"
              << std::endl;
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
        std::cout << "\nFewer than two named states resolved; nothing to plan." << std::endl;
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

        results_csv << "parameterized," << trial << ",\"" << pair_label << "\"," << result.solved << ","
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
            vamp::planning::shortcut_path<Robot, rake, Robot::resolution, TaskLocalPlanner, ParameterizedSpace>(
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

#ifdef VAMP_PROFILING
    std::cout << "\n--- Kernel profiling (aggregated over all trials) ---" << std::endl;
    vamp::utils::profiling::report(std::cout);
#endif

    return 0;
}
