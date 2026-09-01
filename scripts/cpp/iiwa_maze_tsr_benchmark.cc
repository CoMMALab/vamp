// TSR-projection variant of iiwa_maze_solver_benchmark.cc: instead of planning over
// ParameterizedSpace (samples resolved to the ambient configuration by IiwaMarker's
// closed-form analytic IK, via ParameterizedSpace::resolve_block), this plans directly
// over IiwaMarker's own ambient Configuration space with a generic
// vamp::planning::constraint::TaskSpaceConstraint -- CBiRRT2-style projection (Berenson
// et al., "Task Space Regions") that iteratively resolves the numeric TSR error/Jacobian
// kernels (Robot::tsr_error + Robot::solve_tsr_error_lm_inner/outer/gradient_descent)
// brought into iiwa_marker.hh, the same generic machinery scripts/mcflask_line_benchmark.py's
// "geo" (geometric constrained RRTC) baseline uses for panda via
// pa.TaskSpaceConstraint(...)/pa.rrtc(..., constraints).
//
// Same maze environment, same TSR box, and the same problem set (resources/iiwa_marker/
// maze_problems_checked_ik.json) as iiwa_maze_solver_benchmark.cc, so results are directly
// comparable: this is the "does the generic numeric TSR-projection local planner match the
// closed-form analytic-IK task-space planner's success rate / speed / path quality on the
// same problems" benchmark.

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/iiwa_marker.hh>
#include <vamp/utils/profiling.hh>

using Robot = vamp::robots::IiwaMarker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TSRConstraint = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using ConstraintT = vamp::planning::constraint::Constraint<Robot, rake>;
using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstrainedLP = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

struct Problem
{
    std::array<float, 7> problem_start;
    std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
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

    // --- Task Space Region, same box as iiwa_maze_solver_benchmark.cc's TaskSampler
    // Transform/Bound (tool facing down, xy free within the maze footprint, pinned to
    // z = 0.22607783). TaskSpaceConstraint::Transform is (qw, qx, qy, qz, x, y, z) --
    // matching Robot::tsr_error's generated input layout -- unlike
    // TaskSpaceInformedSampler::Transform's (x, y, z, qx, qy, qz, qw), so the transforms
    // below are the same physical frames as the other benchmark's, just reordered.
    const std::array<TSRConstraint::Transform, Robot::n_eef> eef_to_offset = {
        TSRConstraint::Transform{1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
    const std::array<TSRConstraint::Transform, Robot::n_eef> world_to_reference = {
        TSRConstraint::Transform{0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.22607783F}};
    const std::array<TSRConstraint::Bound, Robot::n_eef> tsr_lower = {
        TSRConstraint::Bound{-0.85F, -0.7F, -0.0F, -0.000F, -0.000F, -3.14159265358979F}};
    const std::array<TSRConstraint::Bound, Robot::n_eef> tsr_upper = {
        TSRConstraint::Bound{0.0F, 0.7F, 0.0F, 0.00F, 0.000F, 3.14159265358979F}};

    auto make_lp = [&]() -> ConstrainedLP
    {
        auto tsr = std::make_shared<TSRConstraint>(eef_to_offset, world_to_reference, tsr_lower, tsr_upper);
        std::vector<std::shared_ptr<const ConstraintT>> constraints = {tsr};
        return ConstrainedLP(ConstraintSetT(std::move(constraints)));
    };

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75F;
    rrtc_settings.max_iterations = 500000;
    rrtc_settings.max_samples = 500000;
    rrtc_settings.dynamic_domain = false;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};

    auto path_to_json = [](const vamp::planning::Path<Robot, Robot> &path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &state : path)
        {
            const auto full = state.to_array();
            arr.push_back(std::vector<float>(full.begin(), full.begin() + Robot::dimension));
        }

        return arr;
    };

    auto path_distance = [](const vamp::planning::Path<Robot, Robot> &path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            const auto a = path[i].to_array();
            const auto b = path[i + 1].to_array();
            float squared = 0.0F;
            for (std::size_t j = 0; j < Robot::dimension; ++j)
            {
                const float diff = a[j] - b[j];
                squared += diff * diff;
            }

            total += std::sqrt(squared);
        }

        return total;
    };

    // Hand-rolled SE3 distance -- kept identical to iiwa_maze_solver_benchmark.cc's version
    // so the two files' "eef distance" numbers are directly comparable, computed here via
    // IiwaMarker::eefk (forward kinematics) since this benchmark's path is already the
    // ambient joint-space path, not a task-space pose sequence.
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

    auto path_se3_distance = [&](const vamp::planning::Path<Robot, Robot> &path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < path.size(); ++i)
        {
            const auto a = path[i].to_array();
            const auto b = path[i + 1].to_array();
            std::array<float, 7> qa{}, qb{};
            std::copy(a.begin(), a.begin() + 7, qa.begin());
            std::copy(b.begin(), b.begin() + 7, qb.begin());
            const auto pose_a = Robot::eefk(qa);
            const auto pose_b = Robot::eefk(qb);
            total += se3_distance(
                pose_a.translation(),
                Eigen::Quaternionf(pose_a.rotation()),
                pose_b.translation(),
                Eigen::Quaternionf(pose_b.rotation()));
        }

        return total;
    };

    nlohmann::json all_paths = nlohmann::json::array();
    const char *paths_output_path = "resources/iiwa_marker/maze_solver_tsr_benchmark_paths.json";

    std::vector<Problem> problems;
    const std::string problem_json_path = "resources/iiwa_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);

    std::size_t total_num_problems = 0;
    std::size_t valid_problems = 0;
    std::size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;

    std::vector<float> configuration_distance_per_problem;
    std::vector<float> shortcut_configuration_distance_per_problem;
    std::vector<float> eef_distance_per_problem;
    std::vector<float> shortcut_eef_distance_per_problem;

    std::vector<std::size_t> failed_nanoseconds_per_problem;
    std::vector<std::size_t> failed_iterations_per_problem;

    for (const auto &problem : problems)
    {
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;

        // A fresh local planner (and thus a fresh, unshared constraint-evaluation cache and
        // constraint instance) per problem -- ConstrainedLocalPlanner/Constraint are not
        // thread-safe/reentrant to share across problems.
        auto lp = make_lp();

        // By-value std::array construction (not `.data()`): the by-pointer FloatVector
        // constructor assumes an aligned, SIMD-width-padded buffer (see interface.hh's
        // load_vector), which problem.problem_start/problem_end -- plain, unaligned,
        // 7-wide std::arrays -- are not; that mismatch segfaults on an unaligned "aligned"
        // load. The by-value overload copies into its own padded/aligned buffer instead,
        // matching how iiwa_parameterized_ik_planner's iiwa_try.cc constructs
        // Robot::Configuration from a plain array.
        Robot::Configuration start_config(problem.problem_start);
        Robot::Configuration goal_config(problem.problem_end);

        // Snap onto the TSR manifold before checking feasibility/planning: the loaded
        // ambient configs came from the closed-form IK benchmark's problem generator, so
        // they satisfy the TSR up to that solver's numerical precision, not necessarily
        // this constraint's own tolerance.
        const bool start_ok = lp.project(start_config) and lp.satisfied(start_config);
        const bool goal_ok = lp.project(goal_config) and lp.satisfied(goal_config);

        if (not start_ok or not goal_ok)
        {
            std::cout << "Skipping problem due to invalid start or goal configuration." << std::endl;
            continue;
        }

        valid_problems++;

        auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

        auto result = TaskRRTC::solve<ConstrainedLP>(
            start_config, goal_config, env_v, rrtc_settings, rng, lp);

        if (result.path.size() > 0)
        {
            successful_problems++;
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);

            auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution, ConstrainedLP>(
                result.path, env_v, simplify_settings, rng, lp);
            shortcut_nanoseconds_per_problem.push_back(shortcut_result.nanoseconds);
            path_size_before_shortcut.push_back(result.path.size());
            path_size_after_shortcut.push_back(shortcut_result.path.size());

            const float configuration_distance = path_distance(result.path);
            const float shortcut_configuration_distance = path_distance(shortcut_result.path);
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
            path_entry["nanoseconds"] = result.nanoseconds;
            path_entry["path"] = path_to_json(result.path);
            path_entry["path_ambient_distance"] = configuration_distance;
            path_entry["path_se3_distance"] = eef_distance;
            path_entry["shortcut_nanoseconds"] = shortcut_result.nanoseconds;
            path_entry["shortcut_path"] = path_to_json(shortcut_result.path);
            path_entry["shortcut_path_ambient_distance"] = shortcut_configuration_distance;
            path_entry["shortcut_path_se3_distance"] = shortcut_eef_distance;
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
