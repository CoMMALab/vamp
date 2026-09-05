// Port of iiwa_maze_tsr_benchmark.cc to vamp::robots::FR3Marker (vamp/robots/fr3_marker.hh).
// Settings, TSR bounds, constraint settings, and the core json output schema are kept identical
// to the iiwa version so results are directly comparable; only the include, robot alias, and
// resource/output paths changed.
//
// Plans directly over FR3Marker's ambient joint-space Configuration with a generic
// vamp::planning::constraint::TaskSpaceConstraint -- CBiRRT2-style projection (Berenson et al.,
// "Task Space Regions") that resolves the numeric tsr_error/solve_tsr_error_lm_inner/
// _lm_outer/_gradient_descent kernels, the same generic machinery
// scripts/mcflask_line_benchmark.py's "geo" baseline uses for panda via
// pa.TaskSpaceConstraint(...)/pa.rrtc(..., constraints) -- as opposed to
// fr3_maze_solver_benchmark.cc's closed-form analytic IK (ParameterizedSpace::resolve_block).
// Unlike that file, this one never reads/sets ParameterizedSpace::smm at all: FR3Marker's own
// ambient Configuration and fkcc are all this planner needs, and Problem::smm below is kept
// only to break success rate down by whichever (case6_sel, case1_sel, q_actual_0) branch
// fr3_maze_problem_generator.cc happened to resolve each problem's start/goal on.
//
// Usage:
//   vamp_fr3_maze_tsr_benchmark [paths_json] [results_csv] [trajectory_dir]
//
// Every attempted problem (solved or not) gets one row in <results_csv> -- same schema as
// bimanual_iiwa_*_shelf.cc's results CSVs (method,trial,pair,solved,planning_time_ms,
// iterations,shortcut_time_ms,config_distance,eef_distance; "pair" is left blank, these
// are numbered maze problems, not named start/goal pairs) -- so
// scripts/plot_bimanual_results.py works unchanged on this file's output too.
// shortcut_time_ms/config_distance/eef_distance are only populated for solved problems and
// are the SHORTCUT path's values, not the raw RRTC path's; planning_time_ms/iterations are
// recorded for every attempted (valid start/goal) problem, including ones RRTC failed to
// solve.
//
// Every solved problem's shortcut path is also dumped to <trajectory_dir>/problem_<n>.txt: one
// waypoint per line, its Robot::dimension joint values comma-separated -- same format
// fr3_maze_solver_benchmark.cc's write_ambient_path uses, for offline playback/visualization.
// This is in addition to, not instead of, <paths_json>'s combined JSON dump of every solved
// problem's raw and shortcut paths.

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/fr3_marker.hh>
#include <vamp/utils/profiling.hh>

using Robot = vamp::robots::FR3Marker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

using ConstraintT = vamp::planning::constraint::Constraint<Robot, rake>;
using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstrainedLP = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
using TSC = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using ProjMethod = vamp::planning::constraint::ProjMethod;

using Configuration = typename Robot::Configuration;
using RNG = typename vamp::rng::RNG<Robot>;
using EefPose = std::pair<Eigen::Vector3f, Eigen::Quaternionf>;

// problem_start/problem_end are the ambient (joint-space) configuration that
// fr3_maze_problem_generator.cc resolves via parameterized IK and writes directly -- for this
// 7-dof joint-space robot that's exactly Robot::dimension floats, so we read them straight in
// as the start/goal configuration, no per-problem IK/projection needed here.
struct Problem
{
    std::array<float, Robot::dimension> problem_start;
    std::array<float, Robot::dimension> problem_end;
    // (case6_sel, case1_sel, q_actual_0) branch the generator resolved this problem's start/goal
    // on (see fr3_maze_problem_generator.cc's find_valid_start_goal_on_shared_branch). This
    // solver never selects or depends on a branch itself (it plans directly in ambient space via
    // a generic TaskSpaceConstraint projection, not the closed-form ParameterizedSpace
    // resolve_block fr3_maze_solver_benchmark.cc uses) -- kept purely to break success rate down
    // by branch, to see whether either of FR3's two IK-formula selectors is harder for this
    // planner too. Only present in problem files produced after
    // find_valid_start_goal_on_shared_branch was added.
    std::array<float, 3> smm{1.0F, 1.0F, 0.0F};
    bool has_smm = false;
};

void load_problems_from_json(std::vector<Problem> &problems, const std::string &path)
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
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
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

// One waypoint per line, its Robot::dimension joint values comma-separated -- same format
// fr3_maze_solver_benchmark.cc's write_ambient_path uses (which in turn matches
// bimanual_iiwa_*_shelf.cc's), for offline playback/visualization. Takes the raw Configuration
// path directly (unlike that file's write_ambient_path, no resolve_block needed here -- see
// this file's header, path is already ambient joint space).
static void write_ambient_path(const std::vector<Configuration> &path, const std::filesystem::path &file)
{
    std::ofstream out(file);
    for (const auto &config : path)
    {
        const auto full = config.to_array();
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            if (i != 0)
            {
                out << ",";
            }

            out << full[i];
        }

        out << "\n";
    }
}

auto main(int argc, char **argv) -> int
{
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

    // Fixed height / orientation for the maze task, matching fr3_maze_solver_benchmark.cc's
    // kEefZ (see that file's comment on the shared-maze-frame assumption). TaskSpaceConstraint's
    // transforms are (qw, qx, qy, qz, x, y, z); the (qx, qy, qz, qw) = (0, -1, 0, 0) tool-down
    // orientation used elsewhere becomes (qw, qx, qy, qz) = (0, 0, -1, 0) here.
    constexpr float kEefZ = 0.150519F;
    const TSC::Transform kIdentityOffset = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    const TSC::Transform kToolDownOrientation = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, kEefZ};

    // Region-wide TSR used while planning: z and tilt (rx, ry) pinned tight to the tool-down
    // plane, xy and yaw (rz) free -- the maze lives in that plane, and its footprint is
    // enforced by collision-checking against the maze walls, not by this bound. Bound order
    // is (dx, dy, dz, rx, ry, rz).
    const TSC::Bound region_lower = {-10.01F, -10.01F, -0.001F, -0.001F, -0.001F, -10.01F};
    const TSC::Bound region_upper = {10.01F, 10.01F, 0.001F, 0.001F, 0.001F, 10.01F};

    auto region_constraint = std::make_shared<TSC>(
        std::array<TSC::Transform, Robot::n_eef>{kIdentityOffset},
        std::array<TSC::Transform, Robot::n_eef>{kToolDownOrientation},
        std::array<TSC::Bound, Robot::n_eef>{region_lower},
        std::array<TSC::Bound, Robot::n_eef>{region_upper});

    vamp::planning::constraint::ConstraintSettings region_settings;
    region_settings.method = ProjMethod::InnerLM;
    region_settings.descend_rate = 1.0F;
    region_settings.max_iterations = 10;
    region_settings.emit_all_waypoints = false;
    // region_settings.tolerance = 1e-3F;

    ConstraintSetT region_set(std::vector<ConstraintSetT::Ptr>{region_constraint}, region_settings);
    ConstrainedLP region_lp(region_set);

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // Sanity check on the configurations read from the problem file: broadcast and
    // collision-check them directly, since they're already real joint configurations.
    auto is_config_valid = [&](const Configuration &q)
    {
        Robot::template ConfigurationBlock<rake> block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            block[i] = q.broadcast(i);
        }

        return vamp::planning::fkcc_block<Robot, rake>(env_v, block);
    };

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.5F;
    rrtc_settings.max_iterations = 200000;
    rrtc_settings.max_samples = 200000;
    rrtc_settings.dynamic_domain = false;
    rrtc_settings.radius = 1.0;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};

    // Dump each successful problem's raw and shortcut paths for offline "distance" analysis
    // in python. Configurations here are real joint angles (Robot::dimension of them).
    auto path_to_json = [](const std::vector<Configuration> &path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &config : path)
        {
            // to_array() pads out to num_scalars_rounded (SIMD width); only the first
            // Robot::dimension entries are meaningful.
            const auto full = config.to_array();
            arr.push_back(std::vector<float>(full.begin(), full.begin() + Robot::dimension));
        }

        return arr;
    };

    // Total Euclidean distance summed over consecutive joint-space waypoints. Configuration
    // is FloatVector<Robot::dimension>, which already provides distance() (see
    // vamp/vector/interface.hh) -- no need to hand-roll this one.
    auto path_joint_distance = [](const std::vector<Configuration> &path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            total += path[i].distance(path[i + 1]);
        }

        return total;
    };

    // Resolve the eef pose (translation, quaternion) at each waypoint via forward kinematics
    // once -- reused for the saved eef_path json, the eef SE3 distance below, and the
    // per-problem start/goal eef positions (there's no separate task-space pose to read
    // those from here, unlike fr3_maze_solver_benchmark.cc's problem file).
    auto resolve_eef_path = [](const std::vector<Configuration> &path)
    {
        std::vector<EefPose> eef_path;
        eef_path.reserve(path.size());
        for (const auto &config : path)
        {
            const auto full = config.to_array();
            Robot::ConfigurationArray config_array;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                config_array[i] = full[i];
            }

            const auto eefk = Robot::eefk(config_array);
            eef_path.emplace_back(eefk.translation(), Eigen::Quaternionf(eefk.rotation()));
        }

        return eef_path;
    };

    auto eef_path_to_json = [](const std::vector<EefPose> &eef_path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &[translation, rotation] : eef_path)
        {
            arr.push_back(std::vector<float>{
                translation.x(),
                translation.y(),
                translation.z(),
                rotation.x(),
                rotation.y(),
                rotation.z(),
                rotation.w()});
        }

        return arr;
    };

    // Hand-rolled SE3 distance (translation distance and quaternion angle, combined in
    // quadrature) between two eef poses. Kept identical to fr3_maze_solver_benchmark.cc's
    // (also hand-rolled) version so the two files' "eef distance" numbers are directly
    // comparable.
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

    auto eef_path_distance = [&](const std::vector<EefPose> &eef_path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < eef_path.size(); ++i)
        {
            total += se3_distance(eef_path[i].first, eef_path[i].second, eef_path[i + 1].first, eef_path[i + 1].second);
        }

        return total;
    };

    nlohmann::json all_paths = nlohmann::json::array();
    const std::string paths_output_path =
        (argc > 1) ? argv[1] : "resources/fr3_marker/maze_solver_tsr_benchmark_paths.json";
    const std::filesystem::path results_csv_path =
        (argc > 2) ? argv[2] : "results/fr3_maze_tsr_benchmark.csv";
    const std::filesystem::path trajectory_dir =
        (argc > 3) ? argv[3] : "trajectories/fr3_maze_tsr_benchmark";
    std::filesystem::create_directories(results_csv_path.parent_path());
    std::filesystem::create_directories(trajectory_dir);
    std::ofstream results_csv(results_csv_path);
    if (not results_csv)
    {
        std::cerr << "Failed to open results CSV for writing: " << results_csv_path << std::endl;
        return 1;
    }
    std::cout << "Writing shortcut trajectories to: " << trajectory_dir << std::endl;

    // Same schema as bimanual_iiwa_*_shelf.cc's results CSVs (see
    // scripts/plot_bimanual_results.py) so the same plotting script works unchanged here --
    // "pair" doesn't apply to these numbered maze problems, so it's left blank.
    results_csv << "method,trial,pair,solved,planning_time_ms,iterations,shortcut_time_ms,config_distance,"
                   "eef_distance\n";
    results_csv.flush();

    std::vector<Problem> problems;
    std::string problem_json_path = "resources/fr3_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);

    std::size_t total_num_problems = 0;
    std::size_t valid_problems = 0;
    std::size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;

    // Per-branch {attempted, solved} counts, keyed by the generator's saved (case6_sel,
    // case1_sel) -- q_actual_0 is dropped from the key since it varies per problem even on the
    // same discrete branch (see fr3_maze_solver_benchmark.cc's stats_by_branch comment) and
    // isn't a posture selector this planner could plausibly be sensitive to. "attempted" here
    // means the problem's start/goal passed is_config_valid and RRTC was actually run, matching
    // valid_problems' definition. Problems from a problem file with no saved smm (has_smm false)
    // have no branch to attribute to, so they're excluded rather than binned separately.
    struct BranchStats
    {
        std::size_t attempted = 0;
        std::size_t solved = 0;
    };
    std::map<std::array<float, 2>, BranchStats> stats_by_branch;

    // "Configuration distance": ambient (joint-space) path length. "EEF distance":
    // eef_path_distance's SE3 metric over the FK-resolved eef poses. Named to match
    // fr3_maze_solver_benchmark.cc's equivalent vectors/json keys.
    std::vector<float> configuration_distance_per_problem;
    std::vector<float> shortcut_configuration_distance_per_problem;
    std::vector<float> eef_distance_per_problem;
    std::vector<float> shortcut_eef_distance_per_problem;

    // Time/iterations RRTC actually spent on problems it gave up on -- tracked separately
    // from the successful stats above since a timed-out search's cost profile isn't
    // comparable to a solved one.
    std::vector<std::size_t> failed_nanoseconds_per_problem;
    std::vector<std::size_t> failed_iterations_per_problem;

    for (const auto &problem : problems)
    {
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;

        Configuration start_config(problem.problem_start);
        Configuration goal_config(problem.problem_end);

        if (!is_config_valid(start_config) || !is_config_valid(goal_config))
        {
            std::cout << "Skipping problem: start or goal configuration is in collision." << std::endl;
            continue;
        }

        valid_problems++;
        const std::array<float, 2> case_branch = {problem.smm[0], problem.smm[1]};
        if (problem.has_smm)
        {
            stats_by_branch[case_branch].attempted++;
        }

        auto result = RRTC::solve(start_config, goal_config, env_v, rrtc_settings, rng, region_lp);

        if (result.path.size() > 0)
        {
            successful_problems++;
            if (problem.has_smm)
            {
                stats_by_branch[case_branch].solved++;
            }
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);

            auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, env_v, simplify_settings, rng, region_lp);
            shortcut_nanoseconds_per_problem.push_back(shortcut_result.nanoseconds);
            path_size_before_shortcut.push_back(result.path.size());
            path_size_after_shortcut.push_back(shortcut_result.path.size());

            const auto eef_path = resolve_eef_path(result.path);
            const auto shortcut_eef_path = resolve_eef_path(shortcut_result.path);

            const float configuration_distance = path_joint_distance(result.path);
            const float shortcut_configuration_distance = path_joint_distance(shortcut_result.path);
            const float eef_distance = eef_path_distance(eef_path);
            const float shortcut_eef_distance = eef_path_distance(shortcut_eef_path);

            configuration_distance_per_problem.push_back(configuration_distance);
            shortcut_configuration_distance_per_problem.push_back(shortcut_configuration_distance);
            eef_distance_per_problem.push_back(eef_distance);
            shortcut_eef_distance_per_problem.push_back(shortcut_eef_distance);

            // FK on the planned start/goal configs -- the closest analogue here to
            // fr3_maze_solver_benchmark.cc's start_eef_pos/goal_eef_pos, which it reads
            // straight from the problem file's task-space pose instead.
            const auto &start_pose = eef_path.front().first;
            const auto &goal_pose = eef_path.back().first;

            nlohmann::json path_entry;
            path_entry["problem_index"] = total_num_problems - 1;
            path_entry["start_eef_pos"] = std::vector<float>{start_pose.x(), start_pose.y(), start_pose.z()};
            path_entry["goal_eef_pos"] = std::vector<float>{goal_pose.x(), goal_pose.y(), goal_pose.z()};
            path_entry["nanoseconds"] = result.nanoseconds;
            path_entry["iterations"] = result.iterations;
            path_entry["path"] = path_to_json(result.path);
            path_entry["eef_path"] = eef_path_to_json(eef_path);
            path_entry["path_ambient_distance"] = configuration_distance;
            path_entry["path_se3_distance"] = eef_distance;
            path_entry["shortcut_nanoseconds"] = shortcut_result.nanoseconds;
            path_entry["shortcut_path"] = path_to_json(shortcut_result.path);
            path_entry["shortcut_eef_path"] = eef_path_to_json(shortcut_eef_path);
            path_entry["shortcut_path_ambient_distance"] = shortcut_configuration_distance;
            path_entry["shortcut_path_se3_distance"] = shortcut_eef_distance;
            all_paths.push_back(path_entry);

            std::ofstream paths_file(paths_output_path);
            if (paths_file.is_open())
            {
                paths_file << all_paths.dump(4);
            }

            const std::string trajectory_filename = "problem_" + std::to_string(total_num_problems - 1) + ".txt";
            shortcut_result.path.interpolate_to_resolution(256);
            write_ambient_path(shortcut_result.path, trajectory_dir / trajectory_filename);

            results_csv << "fr3_maze_tsr," << (total_num_problems - 1) << ",,1," << (result.nanoseconds / 1.0e6)
                        << "," << result.iterations << "," << (shortcut_result.nanoseconds / 1.0e6) << ","
                        << shortcut_configuration_distance << "," << shortcut_eef_distance << "\n";
        }
        else
        {
            failed_nanoseconds_per_problem.push_back(result.nanoseconds);
            failed_iterations_per_problem.push_back(result.iterations);
            std::cout << "Unable to solve problem with start and goal configs after " << result.iterations
                      << " iterations, " << result.nanoseconds / 1000000.0 << " ms." << std::endl;

            results_csv << "fr3_maze_tsr," << (total_num_problems - 1) << ",,0," << (result.nanoseconds / 1.0e6)
                        << "," << result.iterations << ",,,\n";
        }

        results_csv.flush();
    }

    std::cout << "Saved " << all_paths.size() << " problem paths to " << paths_output_path << std::endl;
    std::cout << "Saved per-problem results to " << results_csv_path << std::endl;
    std::cout << "Saved " << all_paths.size() << " shortcut trajectory files to " << trajectory_dir << std::endl;

    const std::size_t failed_problems = valid_problems - successful_problems;
    std::cout << "Total problems: " << total_num_problems << std::endl
              << "Valid problems: " << valid_problems << std::endl
              << "Successful problems: " << successful_problems << std::endl
              << "Failed problems: " << failed_problems << std::endl
              << "Success rate: " << (static_cast<float>(successful_problems) / static_cast<float>(valid_problems)) * 100.0F
              << "%" << std::endl;

    // Success rate broken down by (case6_sel, case1_sel) (see Problem::smm) -- this planner
    // doesn't select or depend on a branch itself, so this shows whether the branch the
    // *generator* happened to resolve a problem on correlates with how hard this
    // TSR-projection planner finds it, independent of fr3_maze_solver_benchmark.cc's own
    // branch-search behavior.
    if (!stats_by_branch.empty())
    {
        std::cout << "\n--- Success rate by branch (case6_sel, case1_sel) ---" << std::endl;
        for (const auto &[branch_key, branch_stats] : stats_by_branch)
        {
            const float branch_success_rate =
                (branch_stats.attempted > 0)
                    ? (static_cast<float>(branch_stats.solved) / static_cast<float>(branch_stats.attempted)) * 100.0F
                    : 0.0F;
            std::cout << "(" << branch_key[0] << ", " << branch_key[1] << "): " << branch_stats.solved << " / "
                      << branch_stats.attempted << " solved (" << branch_success_rate << "%)" << std::endl;
        }
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
        print_time_stats("successful problems", nanoseconds_per_problem, iterations_per_problem);

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
#ifdef VAMP_PROFILING
    std::cout << "\n--- Kernel profiling (aggregated over all problems) ---" << std::endl;
    vamp::utils::profiling::report(std::cout);
#endif

    return 0;
}
