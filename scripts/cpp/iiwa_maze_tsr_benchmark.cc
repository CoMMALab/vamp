// Faithful port of ik_parameterized_planner's scripts/cpp/iiwa_maze_solver_mcvamp_benchmark.cc
// to this branch's single vamp/robots/iiwa_marker.hh (that branch split the closed-form
// task-space-parameterized robot and the plain 7-dof joint-space robot into two headers --
// iiwamarker.hh and iiwamarker7.hh, both defining vamp::robots::IiwaMarker, so only one
// could ever be included per translation unit; this branch instead carries the TSR/
// tsr_error kernels needed for the latter directly on the same IiwaMarker as the former,
// so this file just doesn't touch ParameterizedSpace). Settings, TSR bounds, constraint
// settings, and the core json output schema are kept identical to the original so results
// are directly comparable to that reference run; only the include and the now-unnecessary
// "two headers" comment changed.
//
// The stats/output surface (failed-problem timing, distance-stat summaries, per-path
// iterations/start_eef_pos/goal_eef_pos, VAMP_PROFILING report) is additionally widened to
// match iiwa_maze_solver_benchmark.cc's -- the closed-form ParameterizedSpace benchmark this
// is meant to be compared against -- including its exact json distance-field names
// (path_ambient_distance/path_se3_distance and shortcut_* counterparts) so
// scripts/compare_iiwa_maze_results.py can read either file. There's no separate task-space
// pose to read start/goal eef positions from here (the planned path already lives in
// ambient joint space), so those, like the rest of the eef path, come from forward
// kinematics (Robot::eefk) on the planned configurations rather than from the problem file.
//
// Plans directly over IiwaMarker's ambient joint-space Configuration with a generic
// vamp::planning::constraint::TaskSpaceConstraint -- CBiRRT2-style projection (Berenson et
// al., "Task Space Regions") that resolves the numeric tsr_error/solve_tsr_error_lm_inner/
// _lm_outer/_gradient_descent kernels, the same generic machinery
// scripts/mcflask_line_benchmark.py's "geo" baseline uses for panda via
// pa.TaskSpaceConstraint(...)/pa.rrtc(..., constraints) -- as opposed to
// iiwa_maze_solver_benchmark.cc's closed-form analytic IK (ParameterizedSpace::resolve_block).

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
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
#include <vamp/robots/iiwa_marker.hh>
#include <vamp/utils/profiling.hh>

using Robot = vamp::robots::IiwaMarker;
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
// iiwa_maze_problem_generator.cc resolves via parameterized_ik and writes directly -- for
// this 7-dof joint-space robot that's exactly Robot::dimension floats, so we read them
// straight in as the start/goal configuration, no per-problem IK/projection needed here.
struct Problem
{
    std::array<float, Robot::dimension> problem_start;
    std::array<float, Robot::dimension> problem_end;
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
            float x = obj.at("x").get<float>() + 0.05F;  // push it slightly forward
            float y = obj.at("y").get<float>();
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

auto main(int, char **) -> int
{
    EnvironmentInput environment;

    const std::vector<std::string> candidate_paths = {
        "resources/environments/real_maze.json",
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

    // Fixed height / orientation for the maze task, matching iiwa_maze_solver.cc: tool
    // pointing straight down, on the z = kEefZ plane. TaskSpaceConstraint's transforms are
    // (qw, qx, qy, qz, x, y, z); the old (qx, qy, qz, qw) = (0, -1, 0, 0) tool-down
    // orientation used by iiwa_maze_solver.cc becomes (qw, qx, qy, qz) = (0, 0, -1, 0) here.
    constexpr float kEefZ = 0.22607783F;
    const TSC::Transform kIdentityOffset = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    const TSC::Transform kToolDownOrientation = {0.0F, 0.0F, -1.0F, 0.0F, 0.0F, 0.0F, kEefZ};

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
    region_settings.method = ProjMethod::OuterLM;
    region_settings.descend_rate = 0.75F;
    region_settings.max_iterations = 10;
    region_settings.emit_all_waypoints = false;

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
    rrtc_settings.range = 0.4F;
    rrtc_settings.max_iterations = 200000;
    rrtc_settings.max_samples = 200000;
    rrtc_settings.dynamic_domain = false;

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
    // those from here, unlike iiwa_maze_solver_benchmark.cc's problem file).
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
    // quadrature) between two eef poses. Kept identical to the (also hand-rolled) version in
    // iiwa_maze_solver_benchmark.cc so the two files' "eef distance" numbers are directly
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
    const char *paths_output_path = "resources/iiwa_marker/maze_solver_mcvamp_benchmark_paths.json";

    std::vector<Problem> problems;
    std::string problem_json_path = "resources/iiwa_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);

    std::size_t total_num_problems = 0;
    std::size_t valid_problems = 0;
    std::size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;

    // "Configuration distance": ambient (joint-space) path length. "EEF distance":
    // eef_path_distance's SE3 metric over the FK-resolved eef poses. Named to match
    // iiwa_maze_solver_benchmark.cc's equivalent vectors/json keys.
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

        auto result = RRTC::solve(start_config, goal_config, env_v, rrtc_settings, rng, region_lp);

        if (result.path.size() > 0)
        {
            successful_problems++;
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
            // iiwa_maze_solver_benchmark.cc's start_eef_pos/goal_eef_pos, which it reads
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
        }
        else
        {
            failed_nanoseconds_per_problem.push_back(result.nanoseconds);
            failed_iterations_per_problem.push_back(result.iterations);
            std::cout << "Unable to solve problem with start and goal configs after " << result.iterations
                      << " iterations, " << result.nanoseconds / 1000000.0 << " ms." << std::endl;
        }
    }

    std::cout << "Saved " << all_paths.size() << " problem paths to " << paths_output_path << std::endl;

    const std::size_t failed_problems = valid_problems - successful_problems;
    std::cout << "Total problems: " << total_num_problems << std::endl
              << "Valid problems: " << valid_problems << std::endl
              << "Successful problems: " << successful_problems << std::endl
              << "Failed problems: " << failed_problems << std::endl
              << "Success rate: " << (static_cast<float>(successful_problems) / static_cast<float>(valid_problems)) * 100.0F
              << "%" << std::endl;

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
