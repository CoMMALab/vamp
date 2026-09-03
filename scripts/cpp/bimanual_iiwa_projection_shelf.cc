// Projection-based baseline for the same BimanualIiwa shelf scenario that
// bimanual_iiwa_leader_follower_shelf.cc solves via closed-form IK
// (BimanualIiwa::LeaderFollowerSpace): here the two arms' rigid hand-to-hand offset is
// instead enforced as a generic CBiRRT2-style task-space constraint (Berenson et al.,
// "Task Space Regions") -- vamp::planning::constraint::BimanualTaskSpaceConstraint,
// iteratively projected by ConstrainedLocalPlanner -- and RRTC plans directly over
// BimanualIiwa's ambient 14-dof joint space, the way iiwa_maze_tsr_benchmark.cc plans
// IiwaMarker over a generic vamp::planning::constraint::TaskSpaceConstraint instead of
// iiwa_maze_solver_benchmark.cc's closed-form ParameterizedSpace::resolve_block. This is
// the "projections" baseline the leader-follower closed-form planner is meant to be
// compared against on the same problem (same shelf cuboids, same three named
// bottom/middle/top states), so unlike that file this one has no notion of a
// parameterized/task-space Space at all: the three named states below are the already
// IK-resolved full 14-dof ambient configurations from the reference implementation's
// resources/start_end_points/bimanual_iiwa.txt, used here as plain joint configurations.
//
// Usage:
//   vamp_bimanual_iiwa_projection_shelf [n_trials] [range] [trajectory_dir]
//
// Every solved trial's shortcut path, interpolated to Robot::resolution, is dumped to
// <trajectory_dir>/trial_<n>_<start>_to_<goal>.txt: one waypoint per line, its
// Robot::dimension joint values comma-separated (same format
// vamp_bimanual_example_iiwa.cc wrote its single best trajectory in), for offline
// playback/visualization.

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

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/bimanual_iiwa.hh>
#include <vamp/utils/profiling.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::BimanualIiwa;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using ProjMethod = vamp::planning::constraint::ProjMethod;
using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstrainedLP = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
using BimanualTSC = vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// The three named problem states, as plain full 14-dof ambient configurations (7 left-arm
// + 7 right-arm joint angles) -- the reference implementation's already-IK-resolved
// resources/start_end_points/bimanual_iiwa.txt for this same shelf scenario, in that
// file's line order (middle, bottom, top).
struct NamedState
{
    std::string label;
    Robot::ConfigurationArray array;
};

const std::vector<NamedState> kNamedStates = {
    {"middle",
     {{-0.5997312520566763F,  1.489780849654964F,   -1.4739679827359913F, 1.2905366081785483F,
       -0.04421061906813227F, -0.8793712572715165F, -1.1603461715511334F, 0.6478085507050495F,
       1.5420182953666222F,   1.4012033615433137F,  1.2931428651414443F,  0.14691416900781198F,
       -0.9348727702323719F,  1.864705746202578F}}},
    {"bottom",
     {{-0.6430910102907225F,  1.9156121024586796F, -1.7968254667817805F, 1.2945447141185198F,
       -0.023834531305537934F, -0.876966810663043F, -1.7041643160834519F, 0.7137057906077047F,
       1.9675104645685881F,   1.728621289619734F,   1.297295659493332F,   0.16350904416645204F,
       -0.9339939993465736F,  2.3860537824177745F}}},
    {"top",
     {{-0.1994994216078726F, 0.9140739951190965F,  -2.236618320862171F, 0.5238879195899456F,
       0.7998441913611017F,  -1.3575398006936048F, -1.0153092816310436F, 0.24160750214358093F,
       0.9022665362154509F,  2.2897413507442623F,  0.5286854254175202F, -0.8636815969096344F,
       -1.4123134496102092F, 1.7961899529805394F}}},
};

// Shelf cuboids, identical to bimanual_iiwa_leader_follower_shelf.cc's kShelfCuboids:
// each row is (x, y, z, dx, dy, dz) -- position + *full* extents (halved below to match
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

// Sanity-check a named state's already-resolved joint configuration directly against the
// environment (no IK, no task-space resolution -- these are real joint angles).
auto is_config_valid(const Robot::Configuration &q, const EnvironmentVector &environment_v) -> bool
{
    Robot::ConfigurationBlock<rake> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return vamp::planning::fkcc_block<Robot, rake>(environment_v, block);
}

// Right-hand (eef 1) pose expressed in the left hand's (eef 0) frame, as the (qw, qx, qy,
// qz, x, y, z) layout BimanualTaskSpaceConstraint/tsr_bimanual_error expect.
auto relative_pose(const Robot::ConfigurationArray &q) -> BimanualTSC::Transform
{
    const auto left = Robot::eefk(q, 0);
    const auto right = Robot::eefk(q, 1);
    const Eigen::Isometry3f rel = left.inverse() * right;
    const Eigen::Quaternionf quat(rel.rotation());
    const auto &t = rel.translation();
    return {quat.w(), quat.x(), quat.y(), quat.z(), t.x(), t.y(), t.z()};
}

// Per-(start, goal)-pair planning statistics, accumulated over every trial that drew that
// pair -- see bimanual_iiwa_leader_follower_shelf.cc's PairStats for the same shape and
// rationale (time/iteration stats over solved trials only; `total` over every trial).
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

// One waypoint per line, its Robot::dimension joint values comma-separated -- matches
// vamp_bimanual_example_iiwa.cc's trajectory.txt format.
template <typename PathT>
void write_path(const PathT &path, const std::filesystem::path &file)
{
    std::ofstream out(file);
    for (const auto &config : path)
    {
        const auto array = config.to_array();
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

auto main(int argc, char **argv) -> int
{
    const std::size_t n_trials = (argc > 1) ? static_cast<std::size_t>(std::stoul(argv[1])) : 100;
    const float range = (argc > 2) ? std::stof(argv[2]) : 2.0F;
    const std::filesystem::path trajectory_dir = (argc > 3) ? argv[3] : "trajectories/bimanual_iiwa_projection_shelf";
    std::filesystem::create_directories(trajectory_dir);
    std::cout << "Writing shortcut trajectories to: " << trajectory_dir << std::endl;

    std::cout << std::boolalpha;
    std::cout << "Robot::dimension (ambient/joint space): " << Robot::dimension << std::endl;

    const EnvironmentInput environment = build_environment();
    const EnvironmentVector environment_v(environment);
    std::cout << "Environment has " << environment_v.cuboids.size() << " cuboids (shelf)." << std::endl;

    std::cout << "\n--- Checking the three named problem states ---" << std::endl;
    std::vector<NamedState> valid_states;
    for (const auto &named : kNamedStates)
    {
        Robot::Configuration q(named.array);
        const bool valid = is_config_valid(q, environment_v);
        std::cout << named.label << ": valid = " << valid << std::endl;
        if (valid)
        {
            valid_states.push_back(named);
        }
    }

    if (valid_states.size() < 2)
    {
        std::cout << "Fewer than two named states are valid; nothing to plan." << std::endl;
        return 0;
    }

    // Closure target: the hand-to-hand relative pose read off the first valid named state.
    // All three named states share (up to numerical noise) the same rigid hand-to-hand
    // offset, so the constraint below enforces it directly instead of solving it via
    // closed-form IK.
    const auto lTr = relative_pose(valid_states.front().array);
    std::cout << "Closure target (right-in-left, qw qx qy qz x y z): ";
    for (const auto v : lTr)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    // Tight slab around the target: position/orientation are pinned, matching the closed-
    // form branch's rigid hand-to-hand offset.
    constexpr float kTol = 1e-3F;
    const BimanualTSC::Bound lower = {-kTol, -kTol, -kTol, -kTol, -kTol, -kTol};
    const BimanualTSC::Bound upper = {kTol, kTol, kTol, kTol, kTol, kTol};

    auto bimanual_constraint = std::make_shared<BimanualTSC>(lTr, lower, upper);

    // Best grid-search setting from the reference vamp_bimanual_example_iiwa.cc sweep:
    // "2.000, 1, 0, 0.750, 5, 0.200, 0, ..." in that file's (range, dynamic_domain,
    // proj_method, descend_rate, num_projection_iterations, std_dev_scaling_factor,
    // insert_all_to_tree) column order -- proj_method 0 = InnerLM,
    // insert_all_to_tree false maps to this ConstrainedLocalPlanner's emit_all_waypoints
    // (both mean "keep only the steer endpoint, not every intermediate projected
    // waypoint"). Convergence tolerance (squared error) is the reference project's actual
    // projection-loop threshold (task_space_constraint.hh's `dist.test_all_less_equal(
    // 0.0001F)`), not a guess.
    vamp::planning::constraint::ConstraintSettings constraint_settings;
    constraint_settings.method = ProjMethod::InnerLM;
    constraint_settings.descend_rate = 1.0F;
    constraint_settings.max_iterations = 25;
    constraint_settings.emit_all_waypoints = false;
    constraint_settings.perturbation_scale = 0.2F;
    constraint_settings.tolerance = 1e-4F;

    ConstraintSetT constraint_set(
        std::vector<ConstraintSetT::Ptr>{bimanual_constraint}, constraint_settings);
    ConstrainedLP local_planner(constraint_set);

    // Project the named states onto the constraint manifold: they should already sit on
    // (or extremely close to) it since lTr was read off one of them, but projecting
    // removes any residual FK numerical drift before they're used as RRTC endpoints.
    std::vector<Robot::ConfigurationArray> resolved_configs;
    resolved_configs.reserve(valid_states.size());
    for (const auto &named : valid_states)
    {
        Robot::Configuration q(named.array);
        if (not local_planner.project(q))
        {
            std::cout << "Warning: failed to project named state '" << named.label
                       << "' onto the closure manifold." << std::endl;
        }

        Robot::ConfigurationArray array{};
        const auto full = q.to_array();
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            array[j] = full[j];
        }

        resolved_configs.push_back(array);
    }

    vamp::planning::RRTCSettings settings;
    settings.range = range;
    settings.max_iterations = 1000000;
    settings.dynamic_domain = false;
    settings.radius = 4.0F;
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();


    std::mt19937 rng_engine(0);
    std::uniform_int_distribution<std::size_t> pick(0, valid_states.size() - 1);

    std::size_t solved_count = 0;
    std::map<std::string, PairStats> stats_by_pair;

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

        Robot::Configuration start_config(resolved_configs[start_idx]);
        Robot::Configuration goal_config(resolved_configs[goal_idx]);

        const auto t0 = std::chrono::steady_clock::now();
        auto result = RRTC::solve(start_config, goal_config, environment_v, settings, rng, local_planner);
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
            vamp::planning::SimplifySettings simplify_settings;
            simplify_settings.operations = {vamp::planning::SHORTCUT};
            auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, environment_v, simplify_settings, rng, local_planner);

            std::cout << ", cost " << cost_before << " -> " << shortcut_result.path.cost() << ", path size "
                       << shortcut_result.path.size();

            shortcut_result.path.interpolate_to_resolution(Robot::resolution);
            const std::string filename =
                "trial_" + std::to_string(trial) + "_" + start_named.label + "_to_" + goal_named.label + ".txt";
            write_path(shortcut_result.path, trajectory_dir / filename);
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
