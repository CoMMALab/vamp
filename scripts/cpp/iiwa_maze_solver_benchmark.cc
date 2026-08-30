#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
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
#include <vamp/robots/iiwa_marker.hh>
#include <vamp/utils/profiling.hh>

using Robot = vamp::robots::IiwaMarker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution>;
using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot, ParameterizedSpace>;

struct Problem
{
    std::array<float, 7> problem_start;
    std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
    // psi the generator found valid for this problem's start/goal -- see
    // iiwa_maze_problem_generator.cc's find_valid_psi_pose for why this can't just be a shared
    // constant across problems.
    float start_psi;
    float goal_psi;
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
            float x = obj.at("x").get<float>() + 0.05F;  // push it slightly forward
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

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
            p.problem_start = item.at("problem_start").get<std::array<float, 7>>();
            p.problem_end = item.at("problem_end").get<std::array<float, 7>>();
            p.start_eef_pos = item.at("start_eef_pos").get<std::array<float, 3>>();
            p.goal_eef_pos = item.at("goal_eef_pos").get<std::array<float, 3>>();
            p.start_psi = item.at("start_psi").get<float>();
            p.goal_psi = item.at("goal_psi").get<float>();
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
// print anything (this is the hot path inside the benchmark loop, unlike iiwa_maze_solver.cc's
// verbose resolve_and_check).
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

    // --- Task Space Region: tool facing down (with only slack for numerical tilt), free yaw
    // about the down axis, and pinned to the z=0 plane (xy free within the maze footprint).
    // Matches iiwa_maze_solver.cc's working bounds -- a narrower box here starves the RRT's
    // informed sampler of the space it needs to route around obstacles, even when the
    // individual start/goal poses are themselves IK-valid.
    TaskSampler::Transform world_to_reference = {0.0F, 0.0F, 0.22607783F, 0.0F, 1.0F, 0.0F, 0.0F};
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

    // psi (index 7) doesn't change the eef pose -- only which arm configuration reaches it --
    // so instead of hardcoding one shared value, sweep a handful of candidates and keep the
    // first that resolves within joint limits and collision-free. See find_valid_psi_pose in
    // iiwa_maze_problem_generator.cc for the full rationale.
    constexpr int kNumPsiCandidates = 16;
    auto find_valid_psi_pose =
        [&](const std::array<float, 3> &eef_pos) -> std::pair<bool, ParameterizedSpace::StateArray>
    {
        for (int k = 0; k < kNumPsiCandidates; ++k)
        {
            const float psi =
                2.0F * static_cast<float>(M_PI) * static_cast<float>(k) / static_cast<float>(kNumPsiCandidates);
            ParameterizedSpace::StateArray pose_array = {
                {eef_pos[0], eef_pos[1], 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, psi}};
            if (resolve_and_check(pose_array, env_v))
            {
                return {true, pose_array};
            }
        }

        return {false, {}};
    };

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
    // the z=0 plane, redundancy parameter (psi) arbitrary at 1.45.
    ParameterizedSpace::StateArray start_pose_array = {
        {0.6155468821525574F + 0.05F - 0.15F, -0.62754705131053925F, 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};
    ParameterizedSpace::StateArray goal_pose_array = {
        {0.5836458206176758F + 0.05F - 0.2F, 0.4369207215309143F, 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};

    auto rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75F;
    rrtc_settings.max_iterations = 500000;
    rrtc_settings.max_samples = 500000;
    rrtc_settings.dynamic_domain = false;

    const TaskLocalPlanner task_local_planner;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};

    // Dump each successful problem's raw and shortcut paths for offline "distance" analysis in
    // python. Configurations here are this robot's task-space parameterization (x, y, z, qx,
    // qy, qz, qw, psi), not joint angles.
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
    // quadrature) between two eef poses. Kept identical to the mcvamp benchmark's version so
    // the two files' "eef distance" numbers are directly comparable.
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
    // (x, y, z, qx, qy, qz, qw, psi) -- no FK needed -- and index 7 (psi, the self-motion-
    // manifold redundancy parameter) is simply never read here, since it isn't part of the eef
    // pose.
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
    const char *paths_output_path = "resources/iiwa_marker/maze_solver_benchmark_paths.json";

    {
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
    const std::string problem_json_path = "resources/iiwa_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);

    std::size_t total_num_problems = 0;
    std::size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;
    std::size_t valid_problems = 0;

    for (const auto &problem : problems)
    {
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;
        // Restart the Halton sequence for each problem so results are reproducible per-problem
        // and independent of how many samples earlier problems in this run consumed.
        task_sampler->reset();

        auto make_pose_array = [](const std::array<float, 3> &eef_pos, float psi)
        {
            ParameterizedSpace::StateArray pose_array{};
            pose_array[0] = eef_pos[0];
            pose_array[1] = eef_pos[1];
            pose_array[2] = 0.22607783F;
            pose_array[3] = 0.0F;
            pose_array[4] = -1.0F;
            pose_array[5] = 0.0F;
            pose_array[6] = 0.0F;
            pose_array[7] = psi;
            return pose_array;
        };
        const ParameterizedSpace::StateArray start_pose_array = make_pose_array(problem.start_eef_pos, problem.start_psi);
        const ParameterizedSpace::StateArray goal_pose_array = make_pose_array(problem.goal_eef_pos, problem.goal_psi);

        const bool start_valid = resolve_and_check(start_pose_array, env_v);
        const bool goal_valid = resolve_and_check(goal_pose_array, env_v);

        if (not start_valid or not goal_valid)
        {
            std::cout << "Skipping problem due to invalid start or goal configuration." << std::endl;
            continue;
        }

        valid_problems++;

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
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);

            auto shortcut_result =
                vamp::planning::simplify<Robot, rake, Robot::resolution, TaskLocalPlanner, ParameterizedSpace>(
                    result.path, env_v, simplify_settings, rng, task_local_planner);
            shortcut_nanoseconds_per_problem.push_back(shortcut_result.nanoseconds);
            path_size_before_shortcut.push_back(result.path.size());
            path_size_after_shortcut.push_back(shortcut_result.path.size());

            const auto ambient_path = resolve_ambient_path(result.path);
            const auto shortcut_ambient_path = resolve_ambient_path(shortcut_result.path);

            nlohmann::json path_entry;
            path_entry["problem_index"] = total_num_problems - 1;
            path_entry["start_eef_pos"] = problem.start_eef_pos;
            path_entry["goal_eef_pos"] = problem.goal_eef_pos;
            path_entry["nanoseconds"] = result.nanoseconds;
            path_entry["path"] = path_to_json(result.path);
            path_entry["ambient_path"] = ambient_path_to_json(ambient_path);
            path_entry["path_ambient_distance"] = ambient_path_distance(ambient_path);
            path_entry["path_se3_distance"] = path_se3_distance(result.path);
            path_entry["shortcut_nanoseconds"] = shortcut_result.nanoseconds;
            path_entry["shortcut_path"] = path_to_json(shortcut_result.path);
            path_entry["shortcut_ambient_path"] = ambient_path_to_json(shortcut_ambient_path);
            path_entry["shortcut_path_ambient_distance"] = ambient_path_distance(shortcut_ambient_path);
            path_entry["shortcut_path_se3_distance"] = path_se3_distance(shortcut_result.path);
            path_entry["iterations"] = result.iterations;
            all_paths.push_back(path_entry);

            std::ofstream paths_file(paths_output_path);
            if (paths_file.is_open())
            {
                paths_file << all_paths.dump(4);
            }
        }
        else
        {
            std::cout << "Unable to solve problem with start and goal configs." << std::endl;
        }
    }

    std::cout << "Saved " << all_paths.size() << " problem paths to " << paths_output_path << std::endl;

    std::cout << "Total problems: " << total_num_problems << std::endl
              << "Valid problems: " << valid_problems << std::endl
              << "Successful problems: " << successful_problems << std::endl
              << "Success rate: " << (static_cast<float>(successful_problems) / static_cast<float>(valid_problems)) * 100.0F
              << "%" << std::endl;

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
    }
#ifdef VAMP_PROFILING
    std::cout << "\n--- Kernel profiling (aggregated over all problems) ---" << std::endl;
    vamp::utils::profiling::report(std::cout);
#endif

    return 0;
}
