// Usage:
//   vamp_fr3_maze_solver_benchmark [--use_smm] [--use_psi] [--use_fixed_order_smm]
//       [--use_deterministic_psi] [--results_csv <path>] [paths_json] [results_csv] [trajectory_dir]
//
// Port of iiwa_maze_solver_benchmark.cc to vamp::robots::FR3Marker (vamp/robots/fr3_marker.hh).
// The task-space parameterization is the same 8-dim StateArray (x, y, z, qx, qy, qz, qw, <free
// param>), but the free parameter (kept named "psi" below/in the CLI flags/CSV columns, for
// compatibility with the iiwa benchmark and scripts/compare_iiwa_maze_results.py) is FR3's joint
// 7 angle directly, and the branch triple `smm` is (case6_sel, case1_sel, q_actual_0) instead of
// iiwa's GC2/GC4/GC6 -- see fr3_parameterization.hh and fr3_maze_problem_generator.cc's header
// for the full rationale. Consequences for this file, relative to the iiwa version:
//   - case6_sel/case1_sel are read via a `> 0.5` threshold in resolve_block, so they live in
//     {0, 1}, not {-1, +1} -- kBranchOrder below enumerates the resulting 4 branches, not 8.
//   - q_actual_0 only matters in the (measure-zero) wrist-point-singular case, so it's redrawn
//     per-psi-candidate right alongside q7 (see try_resolve below) rather than being fixed for
//     an entire branch the way iiwa's GC2/GC4/GC6 triple is; --use_smm/--use_psi still load the
//     generator's exact saved (case6_sel, case1_sel, q_actual_0, q7) with no search at all.
//
// --results_csv <path> overrides <results_csv> above without needing to also pass
// <paths_json>/<trajectory_dir> positionally; if both are given, --results_csv wins.
//
// Every attempted problem (solved or not) gets one row in <results_csv> -- same schema as
// bimanual_iiwa_*_shelf.cc's results CSVs (method,trial,pair,solved,planning_time_ms,
// iterations,shortcut_time_ms,config_distance,eef_distance; "pair" is left blank, these
// are numbered maze problems, not named start/goal pairs), plus a trailing resolve_time_ms
// column (see find_valid_start_goal_on_branch) -- so scripts/plot_bimanual_results.py works
// unchanged on this file's output too (it reads columns by name and ignores the extra one).
// shortcut_time_ms/config_distance/eef_distance are only populated for solved problems and
// are the SHORTCUT path's values, not the raw RRTC path's; planning_time_ms/iterations/
// resolve_time_ms are recorded for every attempted (valid start/goal) problem, including ones
// RRTC failed to solve.
//
// Every solved problem's shortcut path is also dumped to
// <trajectory_dir>/problem_<n>.txt: one waypoint per line, its Robot::dimension joint values
// comma-separated -- same format bimanual_iiwa_*_shelf.cc's write_ambient_path uses (see
// write_ambient_path below), for offline playback/visualization. This is in addition to, not
// instead of, <paths_json>'s combined JSON dump of every solved problem's raw and shortcut
// paths (task-space and ambient).
//
// By default, each problem's start/goal eef poses are resolved to a valid ambient
// configuration by searching: the saved smm/psi from fr3_maze_problem_generator.cc are ignored
// entirely, and instead all 4 (case6_sel, case1_sel) branches ({0,0}, {0,1}, {1,0}, {1,1}) are
// tried in a freshly shuffled order per problem (see make_branch_search_order), sampling q7
// (psi) uniformly at random from [0, 2*pi) and q_actual_0 uniformly from FR3Marker's joint 1
// range within each branch (same random draws as the current generator), until one branch
// resolves both endpoints.
//
// --use_fixed_order_smm walks the 4 branches in their fixed order instead, always starting from
// (0,0) (only meaningful without --use_smm, which only ever has one branch to try). Ignored
// otherwise (i.e. combining it with --use_smm is a silent no-op, not an error).
// --use_deterministic_psi sweeps q7 deterministically from 0 upward in evenly-spaced steps
// instead of sampling it at random -- meaningful with or without --use_smm, since a q7 search
// happens in both modes unless --use_psi is also given. q_actual_0 is still sampled at random
// regardless (it only ever matters in the singular fallback, so there's no "sweep" for it to
// mean).
// --use_smm loads the generator's saved (case6_sel, case1_sel, q_actual_0) directly and only
// searches q7 on that one branch, skipping the branch search. --use_psi (only meaningful
// together with --use_smm; ignored on its own) additionally uses the generator's saved q7
// directly too, resolving each endpoint once with no search at all.
// --results_csv <path> overrides where results are written (see Usage above).
//
// The resolve_time_ms column is exactly what --use_smm/--use_psi are meant to speed up (and
// --use_fixed_order_smm/--use_deterministic_psi are meant to change the character of), so run
// the benchmark with each combination to compare.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/constraints/task_space_informed_sampler.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/fr3_marker.hh>

using Robot = vamp::robots::FR3Marker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution>;
using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot, ParameterizedSpace>;

// Fixed height / orientation for the maze task -- see fr3_maze_problem_generator.cc's kEefZ
// comment (kept numerically identical to the iiwa benchmark on the assumption of a shared
// maze/mount frame).
constexpr float kEefZ = 0.150519F;

// FR3's joint 1 range, used to sample q_actual_0 (see file header).
constexpr float kQActual0Min = Robot::lower_bound[0];
constexpr float kQActual0Max = Robot::upper_bound[0];

struct Problem
{
    std::array<float, Robot::dimension> problem_start;
    std::array<float, Robot::dimension> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
    // q7 the generator found valid for this problem's start/goal (field named "psi" for
    // compatibility -- see file header). Unused unless --use_psi is passed (which itself
    // requires --use_smm); otherwise re-resolved by random sampling in
    // find_valid_start_goal_on_branch.
    float start_psi;
    float goal_psi;
    // (case6_sel, case1_sel, q_actual_0) branch the generator resolved this problem's start/goal
    // on (see fr3_maze_problem_generator.cc's find_valid_start_goal_on_shared_branch); only used
    // when --use_smm is passed.
    std::array<float, 3> smm{1.0F, 1.0F, 0.0F};
    bool has_smm = false;
};

static bool load_cuboids_from_json(EnvironmentInput &environment, const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return false;
    }

    nlohmann::json j;
    try
    {
        ifs >> j;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return false;
    }

    if (!j.is_array())
    {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return false;
    }

    for (const auto &obj : j)
    {
        if (!obj.is_object())
        {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz"))
        {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try
        {
            float x = obj.at("x").get<float>() + 0.285*2;  // push it slightly forward, matches fr3_maze_problem_generator
            float y = obj.at("y").get<float>() + 0.0F; // push it slightly left, matches fr3_maze_problem_generator
            float z = obj.at("z").get<float>() + 0.0F; // push it slightly up, matches fr3_maze_problem_generator
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>(); // make it bigger

            float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
            if (obj.contains("roll")) roll = obj.at("roll").get<float>();
            if (obj.contains("pitch")) pitch = obj.at("pitch").get<float>();
            if (obj.contains("yaw")) yaw = obj.at("yaw").get<float>();

            std::array<float, 3> posf = {x, y, z};
            std::array<float, 3> rotf = {roll, pitch, yaw};
            std::array<float, 3> sizef = {dx / 2, dy / 2, dz / 2};
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return true;
}

static void load_problems_from_json(std::vector<Problem> &problems, const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return;
    }

    nlohmann::json j;
    try
    {
        ifs >> j;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return;
    }

    if (!j.is_array())
    {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return;
    }

    for (const auto &item : j)
    {
        try
        {
            Problem p;
            p.problem_start = item.at("problem_start").get<std::array<float, Robot::dimension>>();
            p.problem_end = item.at("problem_end").get<std::array<float, Robot::dimension>>();
            p.start_eef_pos = item.at("start_eef_pos").get<std::array<float, 3>>();
            p.goal_eef_pos = item.at("goal_eef_pos").get<std::array<float, 3>>();
            // set start psi to 0 if not present in the JSON, as it is optional and only used when --use_psi is passed
            p.start_psi = item.value("start_psi", 0.0F);
            p.goal_psi = item.value("goal_psi", 0.0F);
            if (item.contains("smm"))
            {
                p.smm = item.at("smm").get<std::array<float, 3>>();
                p.has_smm = true;
            }
            problems.push_back(p);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading problem fields: " << e.what() << " -- skipping problem" << std::endl;
            continue;
        }
    }
}

// Resolves a task-space pose through IK (ParameterizedSpace::resolve_block) and checks the
// resulting ambient configuration for collision. Returns false on either failure; does not
// print anything (this is the hot path inside the benchmark loop).
static auto resolve_and_check(
    const ParameterizedSpace::StateArray &pose_array,
    const EnvironmentVector &environment_v) -> bool
{
    ParameterizedSpace::State pose(pose_array.data());
    ParameterizedSpace::StateBlock<rake> pose_block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        pose_block[i] = pose.broadcast(i);
    }

    auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(pose_block);
    if (not param_valid)
    {
        return false;
    }

    return Robot::fkcc<rake>(environment_v, ambient_block);
}

// One waypoint per line, its Robot::dimension joint values comma-separated -- same format the
// bimanual_iiwa_*_shelf.cc benchmarks' write_ambient_path uses, for offline playback/
// visualization.
static void write_ambient_path(const std::vector<Robot::ConfigurationArray> &waypoints, const std::filesystem::path &file)
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

// (case6_sel, case1_sel) branches, each in {0, 1} -- see file header for why there are 4 here,
// not iiwa's 8. q_actual_0 (the third smm slot) isn't part of branch identity (it's redrawn
// per-psi-candidate in try_resolve below), so it's left at 0 here and overwritten before every
// ParameterizedSpace::set_smm call.
static constexpr std::array<std::array<float, 3>, 4> kBranchOrder = {{
    {1.0F, 0.0F, 0.0F},
    {1.0F, 1.0F, 0.0F},
    {1.0F, 0.0F, 0.0F},
    {1.0F, 1.0F, 0.0F},
}};

// Returns a freshly shuffled copy of kBranchOrder -- used by find_valid_start_goal_on_branch's
// branch search below when use_sampled_branch_order is true, so which branch gets tried first
// isn't always (0, 0). Shuffled, not independently re-sampled per branch slot, so it's still a
// search over all 4 branches exactly once, just in a randomized order.
static auto make_branch_search_order(std::mt19937 &rng) -> std::array<std::array<float, 3>, 4>
{
    auto order = kBranchOrder;
    std::shuffle(order.begin(), order.end(), rng);
    return order;
}

// Resolves start_eef_pos/goal_eef_pos to a valid (IK + collision-free) pose pair on a *shared*
// (case6_sel, case1_sel) branch -- a single RRTC run needs one consistent arm posture
// throughout, since ParameterizedSpace::resolve_block reads `smm` as global state, not
// something carried inside the State it resolves (same requirement
// fr3_maze_problem_generator.cc's find_valid_start_goal_on_shared_branch enforces at generation
// time). This also means start and goal are never tried on different branches mid-search:
// try_branch below resolves both endpoints against the one branch it was called with before the
// search moves on, whether the branches are visited in kBranchOrder's fixed order or
// make_branch_search_order's shuffled one.
//
// When `fixed_branch` holds a value (--use_smm), only that branch is tried. Otherwise all 4
// branches are tried -- in kBranchOrder's fixed order if `use_sampled_branch_order` is false
// (--use_fixed_order_smm), or a freshly shuffled order per problem otherwise (the default) --
// until one resolves both endpoints. Within whichever branch(es) are tried, q7 is either
// sampled uniformly at random from [0, 2*pi) (the default) or swept deterministically from 0
// upward in kNumPsiCandidates evenly-spaced steps (--use_deterministic_psi), up to
// kNumPsiCandidates attempts per endpoint either way, keeping the first candidate that resolves;
// q_actual_0 is always sampled uniformly at random from FR3Marker's joint 1 range per attempt
// (see file header -- it has no "sweep" to speak of).
//
// `fixed_psi`, when set (--use_smm plus --use_psi), skips that q7/q_actual_0 search entirely
// (regardless of `use_deterministic_psi`) and resolves each endpoint once with the generator's
// own saved (q7, q_actual_0) -- valid to do only alongside `fixed_branch`, since a saved smm was
// only ever verified valid on the generator's saved branch, not an arbitrary one. This is the
// fastest possible path through this function: one resolve_and_check call per endpoint, no
// search at all.
//
// Times the whole search (every branch/psi attempt, successful or not) with a steady_clock,
// since this is exactly the cost these flags are meant to cut down (or, for
// --use_deterministic_psi, change the character of) -- logged to resolve_time_ms in the results
// CSV so the modes can be compared directly.
static auto find_valid_start_goal_on_branch(
    const std::array<float, 3> &start_eef_pos,
    const std::array<float, 3> &goal_eef_pos,
    const EnvironmentVector &environment_v,
    std::mt19937 &psi_rng,
    bool use_sampled_branch_order,
    bool use_deterministic_psi,
    const std::optional<std::array<float, 3>> &fixed_branch,
    const std::optional<std::pair<float, float>> &fixed_psi = std::nullopt)
    -> std::tuple<
        bool,
        ParameterizedSpace::StateArray,
        ParameterizedSpace::StateArray,
        std::array<float, 3>,
        std::chrono::nanoseconds>
{
    constexpr int kNumPsiCandidates = 200;
    // psi_dist is not 0 to 2*pi because FR3's joint 7 has a limited range (see fr3_marker.hh's lower_bound[6]/upper_bound[6]), so the generator only ever samples within that range too. The generator's saved q7 is always within that range, so the search here doesn't need to go outside it either.
    std::uniform_real_distribution<float> psi_dist(Robot::lower_bound[6], Robot::upper_bound[6]);
    std::uniform_real_distribution<float> q_actual_0_dist(kQActual0Min, kQActual0Max);

    // Returns the resolved pose and the q_actual_0 used to resolve it (needed so `fixed_branch`
    // can be recorded with the q_actual_0 that actually worked, mirroring how the generator
    // records it) -- 0.0F when `forced_psi` short-circuits the search.
    auto try_resolve = [&](const std::array<float, 3> &eef_pos,
                            const std::array<float, 2> &branch,
                            const std::optional<float> &forced_psi)
        -> std::optional<std::pair<ParameterizedSpace::StateArray, float>>
    {
        if (forced_psi)
        {
            ParameterizedSpace::set_smm({branch[0], branch[1], 0.0F});
            const ParameterizedSpace::StateArray pose_array = {
                {eef_pos[0], eef_pos[1], kEefZ, 0.0F, -1.0F, 0.0F, 0.0F, *forced_psi}};
            if (resolve_and_check(pose_array, environment_v))
            {
                return std::make_pair(pose_array, 0.0F);
            }

            std::cout << "Failed to resolve eef_pos (" << eef_pos[0] << ", " << eef_pos[1] << ", " << eef_pos[2]
                      << ") with saved psi " << *forced_psi << " on the saved branch" << std::endl;
            return std::nullopt;
        }

        for (int attempt = 0; attempt < kNumPsiCandidates; ++attempt)
        {
            // psi is FR3's joint 7 angle directly, and cannot be sampled from [0, 2*pi) -- see fr3_marker.hh's lower_bound[6]/upper_bound[6] for the actual joint limits.
            
            const float psi = use_deterministic_psi
                ? static_cast<float>(attempt) / static_cast<float>(kNumPsiCandidates) * (Robot::upper_bound[6] - Robot::lower_bound[6]) + Robot::lower_bound[6]
                : psi_dist(psi_rng);
            const float q_actual_0 = q_actual_0_dist(psi_rng);
            ParameterizedSpace::set_smm({branch[0], branch[1], q_actual_0});
            const ParameterizedSpace::StateArray pose_array = {
                {eef_pos[0], eef_pos[1], kEefZ, 0.0F, -1.0F, 0.0F, 0.0F, psi}};
            if (resolve_and_check(pose_array, environment_v))
            {
                return std::make_pair(pose_array, q_actual_0);
            }
        }
        std::cout << "Failed to resolve eef_pos (" << eef_pos[0] << ", " << eef_pos[1] << ", " << eef_pos[2]
                  << ") on this branch after " << kNumPsiCandidates
                  << (use_deterministic_psi ? " swept psi candidates" : " random psi attempts") << std::endl;

        return std::nullopt;
    };

    auto try_branch = [&](const std::array<float, 3> &branch)
        -> std::optional<std::tuple<ParameterizedSpace::StateArray, ParameterizedSpace::StateArray, float>>
    {
        const std::array<float, 2> case_sel = {branch[0], branch[1]};

        auto start_result = try_resolve(
            start_eef_pos, case_sel, fixed_psi ? std::optional<float>(fixed_psi->first) : std::nullopt);
        if (!start_result)
        {
            return std::nullopt;
        }

        auto goal_result = try_resolve(
            goal_eef_pos, case_sel, fixed_psi ? std::optional<float>(fixed_psi->second) : std::nullopt);
        if (!goal_result)
        {
            return std::nullopt;
        }

        // Report the goal's q_actual_0 as "the" branch q_actual_0 -- see
        // fr3_maze_problem_generator.cc's find_valid_start_goal_on_shared_branch for why it
        // doesn't matter which endpoint's draw is kept (q_actual_0 is a singular-case tie-break,
        // not a posture selector, so start/goal don't need to share it the way they share
        // case6_sel/case1_sel).
        return std::make_tuple(start_result->first, goal_result->first, goal_result->second);
    };

    const auto search_start = std::chrono::steady_clock::now();

    if (fixed_branch)
    {
        if (auto poses = try_branch(*fixed_branch))
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - search_start);
            const auto &[start_pose, goal_pose, q_actual_0] = *poses;
            return {true, start_pose, goal_pose, {(*fixed_branch)[0], (*fixed_branch)[1], q_actual_0}, elapsed};
        }
    }
    else
    {
        const auto branch_order = use_sampled_branch_order ? make_branch_search_order(psi_rng) : kBranchOrder;
        for (const auto &branch : branch_order)
        {
            if (auto poses = try_branch(branch))
            {
                const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - search_start);
                const auto &[start_pose, goal_pose, q_actual_0] = *poses;
                return {true, start_pose, goal_pose, {branch[0], branch[1], q_actual_0}, elapsed};
            }
        }
    }

    const auto elapsed =
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - search_start);
    return {false, {}, {}, {}, elapsed};
}

auto main(int argc, char **argv) -> int
{
    bool use_smm = false;
    bool use_psi_requested = false;
    // --use_fixed_order_smm only means anything when --use_smm is *not* passed (with --use_smm
    // there's only ever one branch to try, so there's no order to speak of) -- unlike --use_psi,
    // it's not an error or no-op to combine with --use_smm, it just has nothing to affect.
    bool use_fixed_order_smm = false;
    bool use_deterministic_psi = false;
    std::string results_csv_override;
    std::vector<std::string> positional_args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--use_smm")
        {
            use_smm = true;
        }
        else if (arg == "--use_psi")
        {
            use_psi_requested = true;
        }
        else if (arg == "--use_fixed_order_smm")
        {
            use_fixed_order_smm = true;
        }
        else if (arg == "--use_deterministic_psi")
        {
            use_deterministic_psi = true;
        }
        else if (arg == "--results_csv")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "--results_csv requires a path argument" << std::endl;
                return 1;
            }
            results_csv_override = argv[++i];
        }
        else
        {
            positional_args.push_back(argv[i]);
        }
    }

    // --use_psi only means anything alongside --use_smm (a saved psi was only ever verified
    // valid on the saved branch, not an arbitrary one -- see find_valid_start_goal_on_branch),
    // so it's silently ignored on its own rather than treated as an error.
    const bool use_psi = use_psi_requested and use_smm;
    if (use_psi_requested and not use_smm)
    {
        std::cout << "--use_psi has no effect without --use_smm; ignoring it." << std::endl;
    }

    // See find_valid_start_goal_on_branch: false (default) shuffles the 4-branch search order
    // per problem; --use_fixed_order_smm instead always starts from (0, 0) and walks
    // kBranchOrder in its fixed order.
    const bool use_sampled_branch_order = not use_fixed_order_smm;

    EnvironmentInput environment;

    const std::vector<std::string> candidate_paths = {
        "resources/environments/maze_cuboids.json",
    };

    bool loaded = false;
    for (const auto &p : candidate_paths)
    {
        if (load_cuboids_from_json(environment, p))
        {
            std::cout << "Loaded cuboids from: " << p << std::endl;
            loaded = true;
            break;
        }
    }
    if (!loaded)
    {
        std::cerr << "Failed to load cuboids JSON from any candidate path. Exiting." << std::endl;
        return 1;
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // --- Task Space Region: tool facing down (with only slack for numerical tilt), free yaw
    // about the down axis, and pinned to the z=0 plane (xy free within the maze footprint).
    // Matches the iiwa benchmark's working bounds -- a narrower box here starves the RRT's
    // informed sampler of the space it needs to route around obstacles, even when the
    // individual start/goal poses are themselves IK-valid.
    TaskSampler::Transform world_to_reference = {0.0F, 0.0F, kEefZ, 0.0F, 1.0F, 0.0F, 0.0F};
    TaskSampler::Transform eef_to_offset = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};

    TaskSampler::Bound tsr_lower = {-0.85F, -0.7F, -0.0F, -0.000F, -0.000F, -3.14159265358979F};
    TaskSampler::Bound tsr_upper = {0.0F, 0.7F, 0.0F, 0.00F, 0.000F, 3.14159265358979F};

    auto task_sampler = vamp::planning::make_task_space_informed_sampler<Robot, ParameterizedSpace>(
        eef_to_offset,
        world_to_reference,
        tsr_lower,
        tsr_upper,
        environment,
        std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>());

    std::cout << "\n--- TaskSpaceInformedSampler samples ---" << std::endl;
    for (int i = 0; i < 5; ++i)
    {
        ParameterizedSpace::State sample_state = task_sampler->next();
        const auto sample_array = sample_state.to_array();

        std::cout << "Sample " << i << ": ";
        for (std::size_t j = 0; j < ParameterizedSpace::dimension; ++j)
        {
            std::cout << sample_array[j] << (j < ParameterizedSpace::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        ParameterizedSpace::StateBlock<rake> sample_block;
        for (std::size_t j = 0; j < ParameterizedSpace::dimension; ++j)
        {
            sample_block[j] = sample_state.broadcast(j);
        }

        auto [sample_valid, sample_ambient_block] = ParameterizedSpace::resolve_block<rake>(sample_block);
        std::cout << "  resolve_block valid: " << std::boolalpha << sample_valid << std::endl;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            std::cout << sample_ambient_block[{j, 0}] << (j < Robot::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;
    }
    std::cout << "--- end TaskSpaceInformedSampler samples ---\n" << std::endl;

    // Imaginary maze entry/exit poses: tool pointing straight down (qx=1,qy=0,qz=0,qw=0), on
    // the z=0 plane, q7 arbitrary at 1.45. Same numeric positions as the iiwa benchmark's demo
    // block -- purely illustrative, not tied to any particular problem file.
    ParameterizedSpace::set_smm({1.0F, 1.0F, 0.0F});
    ParameterizedSpace::StateArray start_pose_array = {
        {0.6155468821525574F + 0.05F - 0.15F, -0.62754705131053925F, kEefZ, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};
    ParameterizedSpace::StateArray goal_pose_array = {
        {0.5836458206176758F + 0.05F - 0.2F, 0.4369207215309143F, kEefZ, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};

    auto rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.42F;
    rrtc_settings.max_iterations = 100000;
    rrtc_settings.max_samples = 100000;
    rrtc_settings.dynamic_domain = false;
    rrtc_settings.radius = 1.0F;

    const TaskLocalPlanner task_local_planner;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};

    // Dump each successful problem's raw and shortcut paths for offline "distance" analysis in
    // python. Configurations here are this robot's task-space parameterization (x, y, z, qx,
    // qy, qz, qw, q7), not joint angles.
    auto path_to_json = [](const vamp::planning::Path<Robot, ParameterizedSpace> &path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &state : path)
        {
            // to_array() pads out to num_scalars_rounded (SIMD width); only the first
            // ParameterizedSpace::dimension entries are meaningful.
            const auto full = state.to_array();
            arr.push_back(std::vector<float>(full.begin(), full.begin() + ParameterizedSpace::dimension));
        }

        return arr;
    };

    // Resolve each task-space pose through resolve_block to the ambient (joint-space)
    // configuration -- this is what's actually physically reachable, so it's the
    // representation "distance" analysis should really care about. Split from the json/
    // distance helpers below so the (relatively expensive) IK resolution happens once per
    // waypoint, not once per consumer.
    auto resolve_ambient_path = [](const vamp::planning::Path<Robot, ParameterizedSpace> &path)
    {
        std::vector<Robot::ConfigurationArray> ambient_path;
        ambient_path.reserve(path.size());
        for (const auto &state : path)
        {
            ParameterizedSpace::StateBlock<rake> pose_block;
            for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
            {
                pose_block[i] = state.broadcast(i);
            }

            auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(pose_block);
            static_cast<void>(param_valid);

            Robot::ConfigurationArray ambient_array;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                ambient_array[i] = ambient_block[{i, 0}];
            }

            ambient_path.push_back(ambient_array);
        }

        return ambient_path;
    };

    auto ambient_path_to_json = [](const std::vector<Robot::ConfigurationArray> &ambient_path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &q : ambient_path)
        {
            arr.push_back(std::vector<float>(q.begin(), q.end()));
        }

        return arr;
    };

    // Hand-rolled Euclidean distance summed over consecutive ambient (joint-space) waypoints.
    auto ambient_path_distance = [](const std::vector<Robot::ConfigurationArray> &ambient_path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < ambient_path.size(); ++i)
        {
            float squared = 0.0F;
            for (std::size_t j = 0; j < Robot::dimension; ++j)
            {
                const float diff = ambient_path[i][j] - ambient_path[i + 1][j];
                squared += diff * diff;
            }

            total += std::sqrt(squared);
        }

        return total;
    };

    // Hand-rolled SE3 distance (translation distance and quaternion angle, combined in
    // quadrature) between two eef poses. Kept identical to the iiwa/mcvamp benchmarks' version
    // so the "eef distance" numbers are directly comparable.
    auto se3_distance = [](const Eigen::Vector3f &ta,
                            const Eigen::Quaternionf &qa,
                            const Eigen::Vector3f &tb,
                            const Eigen::Quaternionf &qb)
    {
        const float translation_distance = (tb - ta).norm();
        float dot = std::abs(static_cast<float>(qa.dot(qb)));
        dot = std::min(1.0F, dot);
        const float rotation_distance = 2.0F * std::acos(dot);
        return std::sqrt(translation_distance * translation_distance + rotation_distance * rotation_distance);
    };

    // Total SE3 distance along a task-space pose path. The State already *is* the eef pose
    // (x, y, z, qx, qy, qz, qw, q7) -- no FK needed -- and index 7 (q7, the free/self-motion
    // joint angle) is simply never read here, since it isn't part of the eef pose.
    auto path_se3_distance = [&](const vamp::planning::Path<Robot, ParameterizedSpace> &path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            const auto a = path[i].to_array();
            const auto b = path[i + 1].to_array();
            const Eigen::Vector3f ta(a[0], a[1], a[2]);
            const Eigen::Quaternionf qa(a[6], a[3], a[4], a[5]);
            const Eigen::Vector3f tb(b[0], b[1], b[2]);
            const Eigen::Quaternionf qb(b[6], b[3], b[4], b[5]);
            total += se3_distance(ta, qa, tb, qb);
        }

        return total;
    };

    nlohmann::json all_paths = nlohmann::json::array();
    const std::string paths_output_path =
        (positional_args.size() > 0) ? positional_args[0] : "resources/fr3_marker/maze_solver_benchmark_paths.json";
    // --results_csv takes priority over the positional slot if both are somehow given.
    const std::filesystem::path results_csv_path = (not results_csv_override.empty()) ? results_csv_override
                                                    : (positional_args.size() > 1)     ? positional_args[1]
                                                                                        : "results/fr3_maze_solver_benchmark.csv";
    const std::filesystem::path trajectory_dir =
        (positional_args.size() > 2) ? positional_args[2] : "trajectories/fr3_maze_solver_benchmark";
    std::filesystem::create_directories(results_csv_path.parent_path());
    std::filesystem::create_directories(trajectory_dir);
    std::ofstream results_csv(results_csv_path);
    if (not results_csv)
    {
        std::cerr << "Failed to open results CSV for writing: " << results_csv_path << std::endl;
        return 1;
    }
    std::cout << "Writing per-problem results to: " << results_csv_path << std::endl;
    std::cout << "Writing shortcut trajectories to: " << trajectory_dir << std::endl;

    // Same schema as bimanual_iiwa_*_shelf.cc's results CSVs (see
    // scripts/plot_bimanual_results.py) so the same plotting script works unchanged here --
    // "pair" doesn't apply to these numbered maze problems, so it's left blank. Plus a trailing
    // resolve_time_ms column (see find_valid_start_goal_on_branch), not part of that shared
    // schema but harmless since plot_bimanual_results.py reads columns by name.
    // shortcut_time_ms/config_distance/eef_distance are only populated for solved problems
    // and are the SHORTCUT path's values, not the raw RRTC path's; planning_time_ms,
    // iterations, and resolve_time_ms are recorded for every attempted problem, solved or not.
    results_csv << "method,trial,pair,solved,planning_time_ms,iterations,shortcut_time_ms,config_distance,"
                   "eef_distance,resolve_time_ms\n";
    results_csv.flush();

    {
        ParameterizedSpace::set_smm({1.0F, 1.0F, 0.0F});
        const ParameterizedSpace::State start_state(start_pose_array.data());
        const ParameterizedSpace::State goal_state(goal_pose_array.data());

        auto result = TaskRRTC::solve(
            start_state,
            goal_state,
            env_v,
            rrtc_settings,
            task_sampler,
            task_local_planner);

        std::cout << "RRTC path size: " << result.path.size() << ", iterations: " << result.iterations
                  << ", microseconds: " << result.nanoseconds / 1000.0F << ", with tree sizes: " << result.size[0]
                  << ", " << result.size[1] << std::endl;
    }

    std::vector<Problem> problems;
    const std::string problem_json_path = "resources/fr3_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);

    if (use_smm and use_psi)
    {
        std::cout << "--use_smm --use_psi: resolving each problem with its saved branch and psi directly, "
                      "no search"
                   << std::endl;
    }
    else if (use_smm)
    {
        std::cout << "--use_smm: resolving each problem's saved branch only, "
                   << (use_deterministic_psi ? "sweeping psi" : "searching psi") << std::endl;
    }
    else
    {
        std::cout << "Resolving each problem by searching all branches ("
                   << (use_sampled_branch_order ? "shuffled order" : "fixed order from (0,0)") << "), "
                   << (use_deterministic_psi ? "sweeping psi" : "searching psi") << std::endl;
    }

    // Seeded from std::random_device like iiwa_branch_selector.cc's branch sweep -- also used to
    // shuffle the branch search order (make_branch_search_order) when use_sampled_branch_order
    // is set, and to sample q_actual_0, so this is shared across every problem's resolution
    // search.
    std::mt19937 psi_rng(std::random_device{}());

    std::size_t total_num_problems = 0;
    std::size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;
    std::size_t valid_problems = 0;
    std::size_t unresolvable_problems = 0;

    // Per-branch {attempted, solved} counts, keyed by the (case6_sel, case1_sel, q_actual_0)
    // find_valid_start_goal_on_branch resolved the problem on -- attempted here means "resolved"
    // (i.e. the RRTC solve was actually attempted on this branch), matching valid_problems'
    // definition. Unresolvable problems (no branch found at all) have no branch to attribute to,
    // so they're excluded (see unresolvable_problems above) rather than binned separately.
    // Keyed on the full 3-tuple (not just case6_sel/case1_sel) since q_actual_0 varies per
    // problem even on the same discrete branch -- this mostly just means each bucket ends up
    // with attempted=solved=1, unlike the iiwa version's genuinely repeated 8-branch keys; see
    // the printout below for the same information collapsed onto (case6_sel, case1_sel).
    struct BranchStats
    {
        std::size_t attempted = 0;
        std::size_t solved = 0;
    };
    std::map<std::array<float, 3>, BranchStats> stats_by_branch;
    std::map<std::array<float, 2>, BranchStats> stats_by_case_branch;

    // How long find_valid_start_goal_on_branch took per problem -- recorded for every attempt,
    // regardless of whether a branch was ultimately found, since a failed search's cost (e.g.
    // exhausting all 4 branches under the default mode) is itself part of what --use_smm/
    // --use_psi are meant to avoid.
    std::vector<std::size_t> resolve_nanoseconds_per_problem;

    // "Configuration distance": ambient (joint-space) path length -- what's actually
    // physically reachable, so the representation distance analysis should really care
    // about. "EEF distance": path_se3_distance's SE3 metric over the task-space poses
    // themselves, skipping q7 (index 7) -- Space::distance mixes q7 into its metric (it's a
    // self-motion joint, not part of the eef pose), so it is not a faithful eef-space distance
    // and can't be reused here.
    std::vector<float> configuration_distance_per_problem;
    std::vector<float> shortcut_configuration_distance_per_problem;
    std::vector<float> eef_distance_per_problem;
    std::vector<float> shortcut_eef_distance_per_problem;

    // Time/iterations RRTC actually spent on problems it gave up on (ran out of
    // max_iterations/max_samples with no path) -- tracked separately from the successful
    // stats above since a timed-out search's cost profile is not comparable to a solved one.
    std::vector<std::size_t> failed_nanoseconds_per_problem;
    std::vector<std::size_t> failed_iterations_per_problem;

    for (const auto &problem : problems)
    {
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;
        // Restart the Halton sequence for each problem so results are reproducible per-problem
        // and independent of how many samples earlier problems in this run consumed.
        task_sampler->reset();

        if (use_smm and not problem.has_smm)
        {
            std::cout << "Skipping problem: --use_smm given but problem file has no saved smm "
                          "(regenerate it with the current fr3_maze_problem_generator.cc)."
                       << std::endl;
            continue;
        }

        const std::optional<std::array<float, 3>> fixed_branch =
            use_smm ? std::optional<std::array<float, 3>>(problem.smm) : std::nullopt;
        // --use_psi (only meaningful alongside --use_smm -- see the flag parsing above) also
        // uses the generator's saved q7 directly, skipping the random q7/q_actual_0 search
        // entirely for the fastest possible resolve path.
        const std::optional<std::pair<float, float>> fixed_psi =
            use_psi ? std::optional<std::pair<float, float>>(std::make_pair(problem.start_psi, problem.goal_psi))
                    : std::nullopt;

        const auto [resolved, start_pose_array, goal_pose_array, branch, resolve_ns] = find_valid_start_goal_on_branch(
            problem.start_eef_pos,
            problem.goal_eef_pos,
            env_v,
            psi_rng,
            use_sampled_branch_order,
            use_deterministic_psi,
            fixed_branch,
            fixed_psi);

        resolve_nanoseconds_per_problem.push_back(static_cast<std::size_t>(resolve_ns.count()));

        if (not resolved)
        {
            unresolvable_problems++;
            std::cout << "Unable to resolve problem's start/goal configuration on any branch after "
                       << (resolve_ns.count() / 1.0e6) << " ms. Skipping problem." << std::endl;
            results_csv << "fr3_maze_solver," << (total_num_problems - 1) << ",,0,,,,,,"
                        << (resolve_ns.count() / 1.0e6) << "\n";
            results_csv.flush();
            continue;
        }

        valid_problems++;
        stats_by_branch[branch].attempted++;
        stats_by_case_branch[{branch[0], branch[1]}].attempted++;

        // find_valid_start_goal_on_branch already left `smm` set to `branch` via its internal
        // try_resolve calls, but the last call it made was for the goal endpoint of whichever
        // branch attempt failed *before* the one that succeeded, in the non-fixed-branch search
        // case -- re-set it explicitly here so the RRTC solve below (and every resolve_block
        // call TaskRRTC/TaskLocalPlanner make while extending the tree) definitely uses the
        // branch this problem was actually resolved on.
        ParameterizedSpace::set_smm(branch);

        const ParameterizedSpace::State start_state(start_pose_array.data());
        const ParameterizedSpace::State goal_state(goal_pose_array.data());

        auto result = TaskRRTC::solve(
            start_state,
            goal_state,
            env_v,
            rrtc_settings,
            task_sampler,
            task_local_planner);

        if (result.path.size() > 0)
        {
            successful_problems++;
            stats_by_branch[branch].solved++;
            stats_by_case_branch[{branch[0], branch[1]}].solved++;
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);

            auto shortcut_result =
                vamp::planning::simplify<Robot, rake, Robot::resolution, TaskLocalPlanner, ParameterizedSpace>(
                    result.path, env_v, simplify_settings, rng, task_local_planner);
            shortcut_nanoseconds_per_problem.push_back(shortcut_result.nanoseconds);
            path_size_before_shortcut.push_back(result.path.size());
            path_size_after_shortcut.push_back(shortcut_result.path.size());
            shortcut_result.path.interpolate_to_resolution(Robot::resolution);

            const auto ambient_path = resolve_ambient_path(result.path);
            const auto shortcut_ambient_path = resolve_ambient_path(shortcut_result.path);

            const float configuration_distance = ambient_path_distance(ambient_path);
            const float shortcut_configuration_distance = ambient_path_distance(shortcut_ambient_path);
            const float eef_distance = path_se3_distance(result.path);
            const float shortcut_eef_distance = path_se3_distance(shortcut_result.path);

            configuration_distance_per_problem.push_back(configuration_distance);
            shortcut_configuration_distance_per_problem.push_back(shortcut_configuration_distance);
            eef_distance_per_problem.push_back(eef_distance);
            shortcut_eef_distance_per_problem.push_back(shortcut_eef_distance);

            nlohmann::json path_entry;
            path_entry["problem_index"] = total_num_problems - 1;
            path_entry["start_eef_pos"] = problem.start_eef_pos;
            path_entry["goal_eef_pos"] = problem.goal_eef_pos;
            path_entry["smm"] = branch;
            path_entry["resolve_nanoseconds"] = resolve_ns.count();
            path_entry["nanoseconds"] = result.nanoseconds;
            path_entry["path"] = path_to_json(result.path);
            path_entry["ambient_path"] = ambient_path_to_json(ambient_path);
            path_entry["path_ambient_distance"] = configuration_distance;
            path_entry["path_se3_distance"] = eef_distance;
            path_entry["shortcut_nanoseconds"] = shortcut_result.nanoseconds;
            path_entry["shortcut_path"] = path_to_json(shortcut_result.path);
            path_entry["shortcut_ambient_path"] = ambient_path_to_json(shortcut_ambient_path);
            path_entry["shortcut_path_ambient_distance"] = shortcut_configuration_distance;
            path_entry["shortcut_path_se3_distance"] = shortcut_eef_distance;
            path_entry["iterations"] = result.iterations;
            all_paths.push_back(path_entry);

            std::ofstream paths_file(paths_output_path);
            if (paths_file.is_open())
            {
                paths_file << all_paths.dump(4);
            }

            const std::string trajectory_filename = "problem_" + std::to_string(total_num_problems - 1) + ".txt";
            write_ambient_path(shortcut_ambient_path, trajectory_dir / trajectory_filename);

            results_csv << "fr3_maze_solver," << (total_num_problems - 1) << ",,1,"
                        << (result.nanoseconds / 1.0e6) << "," << result.iterations << ","
                        << (shortcut_result.nanoseconds / 1.0e6) << "," << shortcut_configuration_distance << ","
                        << shortcut_eef_distance << "," << (resolve_ns.count() / 1.0e6) << "\n";
        }
        else
        {
            failed_nanoseconds_per_problem.push_back(result.nanoseconds);
            failed_iterations_per_problem.push_back(result.iterations);
            std::cout << "Unable to solve problem with start and goal configs after " << result.iterations
                      << " iterations, " << result.nanoseconds / 1000000.0 << " ms. " << result.size[0] << ", " << result.size[1] << std::endl;

            results_csv << "fr3_maze_solver," << (total_num_problems - 1) << ",,0,"
                        << (result.nanoseconds / 1.0e6) << "," << result.iterations << ",,,,"
                        << (resolve_ns.count() / 1.0e6) << "\n";
        }

        results_csv.flush();
    }

    std::cout << "Saved " << all_paths.size() << " problem paths to " << paths_output_path << std::endl;
    std::cout << "Saved per-problem results to " << results_csv_path << std::endl;
    std::cout << "Saved " << all_paths.size() << " shortcut trajectory files to " << trajectory_dir << std::endl;

    const std::size_t failed_problems = valid_problems - successful_problems;
    std::cout << "Total problems: " << total_num_problems << std::endl
              << "Unresolvable problems (no branch found): " << unresolvable_problems << std::endl
              << "Valid problems: " << valid_problems << std::endl
              << "Successful problems: " << successful_problems << std::endl
              << "Failed problems: " << failed_problems << std::endl
              << "Success rate: " << (static_cast<float>(successful_problems) / static_cast<float>(valid_problems)) * 100.0F
              << "%" << std::endl;

    // Success rate broken down by (case6_sel, case1_sel) -- collapsing out q_actual_0, which
    // varies per problem even on the same discrete branch (see stats_by_branch's comment above)
    // -- so this shows whether either of FR3's two IK-formula selectors is systematically harder
    // for RRTC to route through the maze on than the other.
    if (!stats_by_case_branch.empty())
    {
        std::cout << "\n--- Success rate by branch (case6_sel, case1_sel) ---" << std::endl;
        for (const auto &[branch_key, branch_stats] : stats_by_case_branch)
        {
            const float branch_success_rate =
                (branch_stats.attempted > 0)
                    ? (static_cast<float>(branch_stats.solved) / static_cast<float>(branch_stats.attempted)) * 100.0F
                    : 0.0F;
            std::cout << "(" << branch_key[0] << ", " << branch_key[1] << "): " << branch_stats.solved << " / "
                      << branch_stats.attempted << " solved (" << branch_success_rate << "%)" << std::endl;
        }
    }

    // How long find_valid_start_goal_on_branch took, over every attempted problem (resolved or
    // not) -- this is the number --use_smm/--use_psi are meant to shrink, by skipping the
    // branch sweep and/or the psi search.
    if (!resolve_nanoseconds_per_problem.empty())
    {
        std::vector<float> resolve_ms_per_problem;
        resolve_ms_per_problem.reserve(resolve_nanoseconds_per_problem.size());
        for (const auto ns : resolve_nanoseconds_per_problem)
        {
            resolve_ms_per_problem.push_back(static_cast<float>(ns) / 1.0e6F);
        }

        std::sort(resolve_ms_per_problem.begin(), resolve_ms_per_problem.end());
        const std::size_t n = resolve_ms_per_problem.size();
        const float sum = std::accumulate(resolve_ms_per_problem.begin(), resolve_ms_per_problem.end(), 0.0F);
        std::string resolve_mode_label =
            (use_smm and use_psi) ? "--use_smm --use_psi" : (use_smm ? "--use_smm" : "branch search");
        if (not (use_smm and use_psi))
        {
            resolve_mode_label += use_deterministic_psi ? " + --use_deterministic_psi" : " + random psi";
        }
        if (not use_smm)
        {
            resolve_mode_label += use_sampled_branch_order ? " + shuffled branch order" : " + --use_fixed_order_smm";
        }
        std::cout << "\n--- Resolve time (find_valid_start_goal_on_branch), " << resolve_mode_label << " ---"
                  << std::endl;
        std::cout << "Average resolve time (ms): " << sum / static_cast<float>(n) << std::endl;
        std::cout << "Median resolve time (ms): " << resolve_ms_per_problem[n / 2] << std::endl;
        std::cout << "Min resolve time (ms): " << resolve_ms_per_problem[0] << std::endl;
        std::cout << "Max resolve time (ms): " << resolve_ms_per_problem[n - 1] << std::endl;
    }

    // Shared by both the successful- and failed-problem time/iteration breakdowns below.
    auto print_time_stats =
        [](const std::string &label, std::vector<std::size_t> nanoseconds, std::vector<std::size_t> iterations)
    {
        const std::size_t n = nanoseconds.size();
        const std::size_t total_nanoseconds = std::accumulate(nanoseconds.begin(), nanoseconds.end(), std::size_t{0});
        const std::size_t total_iterations = std::accumulate(iterations.begin(), iterations.end(), std::size_t{0});
        std::cout << "Average time (ms) for " << label << ": " << (total_nanoseconds / n) / 1000000.0 << std::endl;
        std::cout << "Average iterations for " << label << ": " << total_iterations / n << std::endl;

        std::sort(nanoseconds.begin(), nanoseconds.end());
        std::sort(iterations.begin(), iterations.end());
        std::cout << "Median time (ms) for " << label << ": " << (nanoseconds[n / 2]) / 1000000.0 << std::endl;
        std::cout << "Median iterations for " << label << ": " << iterations[n / 2] << std::endl;
        std::cout << "Minimum time (ms) for " << label << ": " << (nanoseconds[0]) / 1000000.0 << std::endl;
        std::cout << "Minimum iterations for " << label << ": " << iterations[0] << std::endl;
        std::cout << "Maximum time (ms) for " << label << ": " << (nanoseconds[n - 1]) / 1000000.0 << std::endl;
        std::cout << "Maximum iterations for " << label << ": " << iterations[n - 1] << std::endl;
        std::cout << "Q1 time (ms) for " << label << ": " << (nanoseconds[n / 4]) / 1000000.0 << std::endl;
        std::cout << "Q1 iterations for " << label << ": " << iterations[n / 4] << std::endl;
        std::cout << "Q3 time (ms) for " << label << ": " << (nanoseconds[3 * n / 4]) / 1000000.0 << std::endl;
        std::cout << "Q3 iterations for " << label << ": " << iterations[3 * n / 4] << std::endl;
        std::cout << "95th percentile time (ms) for " << label << ": " << (nanoseconds[95 * n / 100]) / 1000000.0
                  << std::endl;
        std::cout << "95th percentile iterations for " << label << ": " << iterations[95 * n / 100] << std::endl;
    };

    if (failed_problems > 0)
    {
        print_time_stats("failed problems", failed_nanoseconds_per_problem, failed_iterations_per_problem);
    }

    if (successful_problems > 0)
    {
        std::size_t total_nanoseconds = 0;
        std::size_t total_iterations = 0;
        for (std::size_t i = 0; i < successful_problems; i++)
        {
            total_nanoseconds += nanoseconds_per_problem[i];
            total_iterations += iterations_per_problem[i];
        }
        std::cout << "Average time (ms) for successful problems: "
                  << (total_nanoseconds / successful_problems) / 1000000.0 << std::endl;
        std::cout << "Average iterations for successful problems: " << total_iterations / successful_problems
                  << std::endl;

        std::sort(nanoseconds_per_problem.begin(), nanoseconds_per_problem.end());
        std::sort(iterations_per_problem.begin(), iterations_per_problem.end());
        std::cout << "Median time (ms) for successful problems: "
                  << (nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Median iterations for successful problems: "
                  << iterations_per_problem[successful_problems / 2] << std::endl;
        std::cout << "Minimum time (ms) for successful problems: " << (nanoseconds_per_problem[0]) / 1000000.0
                  << std::endl;
        std::cout << "Minimum iterations for successful problems: " << iterations_per_problem[0] << std::endl;
        std::cout << "Maximum time (ms) for successful problems: "
                  << (nanoseconds_per_problem[successful_problems - 1]) / 1000000.0 << std::endl;
        std::cout << "Maximum iterations for successful problems: "
                  << iterations_per_problem[successful_problems - 1] << std::endl;

        std::cout << "Q1 time (ms) for successful problems: "
                  << (nanoseconds_per_problem[successful_problems / 4]) / 1000000.0 << std::endl;
        std::cout << "Q1 iterations for successful problems: " << iterations_per_problem[successful_problems / 4]
                  << std::endl;
        std::cout << "Q3 time (ms) for successful problems: "
                  << (nanoseconds_per_problem[3 * successful_problems / 4]) / 1000000.0 << std::endl;
        std::cout << "Q3 iterations for successful problems: "
                  << iterations_per_problem[3 * successful_problems / 4] << std::endl;
        std::cout << "95th percentile time (ms) for successful problems: "
                  << (nanoseconds_per_problem[95 * successful_problems / 100]) / 1000000.0 << std::endl;
        std::cout << "95th percentile iterations for successful problems: "
                  << iterations_per_problem[95 * successful_problems / 100] << std::endl;

        std::size_t total_shortcut_nanoseconds = 0;
        std::size_t total_size_before = 0;
        std::size_t total_size_after = 0;
        for (std::size_t i = 0; i < successful_problems; i++)
        {
            total_shortcut_nanoseconds += shortcut_nanoseconds_per_problem[i];
            total_size_before += path_size_before_shortcut[i];
            total_size_after += path_size_after_shortcut[i];
        }
        std::sort(shortcut_nanoseconds_per_problem.begin(), shortcut_nanoseconds_per_problem.end());
        std::cout << "Average shortcut time (ms): " << (total_shortcut_nanoseconds / successful_problems) / 1000000.0
                  << std::endl;
        std::cout << "Median shortcut time (ms): "
                  << (shortcut_nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Average path size before shortcutting: "
                  << static_cast<float>(total_size_before) / static_cast<float>(successful_problems) << std::endl;
        std::cout << "Average path size after shortcutting: "
                  << static_cast<float>(total_size_after) / static_cast<float>(successful_problems) << std::endl;

        auto print_distance_stats = [](const std::string &label, std::vector<float> values)
        {
            std::sort(values.begin(), values.end());
            const std::size_t n = values.size();
            const float sum = std::accumulate(values.begin(), values.end(), 0.0F);
            std::cout << "Average " << label << ": " << sum / static_cast<float>(n) << std::endl;
            std::cout << "Median " << label << ": " << values[n / 2] << std::endl;
            std::cout << "Min " << label << ": " << values[0] << std::endl;
            std::cout << "Max " << label << ": " << values[n - 1] << std::endl;
        };

        print_distance_stats("configuration distance", configuration_distance_per_problem);
        print_distance_stats("shortcut configuration distance", shortcut_configuration_distance_per_problem);
        print_distance_stats("EEF distance", eef_distance_per_problem);
        print_distance_stats("shortcut EEF distance", shortcut_eef_distance_per_problem);
    }
    return 0;
}
