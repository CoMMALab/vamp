#include <vector>
#include <array>
#include <utility>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <nlohmann/json.hpp>

#include <Eigen/Geometry>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/constraints/ik_parameterized_local_planner.hh>
#include <vamp/planning/constraints/task_space_informed_sampler.hh>
#include <vamp/robots/iiwamarker.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::IiwaMarker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using IKLocalPlanner = vamp::planning::IKParameterizedLocalPlanner<Robot, rake, Robot::resolution>;

// Define your problem structure
struct Problem {
	std::array<float, 7> problem_start;
	std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
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
            float x = obj.at("x").get<float>() + 0.05; // push it slightly forward
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

void load_problems_from_json(std::vector<Problem> &problems, const std::string &path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return;
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return;
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return;
    }

    for (const auto &item : j) {
        try {
            Problem p;
            p.problem_start = item.at("problem_start").get<std::array<float, 7>>();
            p.problem_end = item.at("problem_end").get<std::array<float, 7>>();
            p.start_eef_pos = item.at("start_eef_pos").get<std::array<float, 3>>();
            p.goal_eef_pos = item.at("goal_eef_pos").get<std::array<float, 3>>();
            problems.push_back(p);
        } catch (const std::exception &e) {
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

    // --- Task Space Region: tool facing down (with only slack for numerical tilt), free
    // yaw about the down axis, and pinned to the z=0 plane (xy free within +-1m).
    //
    // Transform layout consumed by TaskSpaceInformedSampler::to_isometry is
    // (x, y, z, qx, qy, qz, qw) -- translation first, quaternion scalar-last.
    //
    // eef_to_offset is the hardcoded tool-to-eef transform: a 180 degree rotation about X
    // (qx=1, qy=0, qz=0, qw=0) with the physical tool-tip offset from the flange.
    using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot>;

    // TaskSampler::Transform world_to_reference = {0.29276255F, -0.55347496F, 0.20607783F, 1.0F, 0.0F, 0.0F, 0.0F};
    // TaskSampler::Transform eef_to_offset = {-0.003750F, 0.000125F, 0.211500F, -0.707106F, 0.000000F, 0.000000F, 0.707107F};


    TaskSampler::Transform world_to_reference = {0.0, 0.0F, 0.22607783F, 0.0F, 1.0F, 0.0F, 0.0F};
    TaskSampler::Transform eef_to_offset = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};

    // Bound order is (dx, dy, dz, rx, ry, rz): translation box + so(3) log-map rotation box.
    // z pinned to the xy plane, xy free within +-1m; rx/ry (tilt away from facing down) held
    // to a tight numerical tolerance, rz (yaw about the down axis) free over a full turn.
    TaskSampler::Bound tsr_lower = {-0.76F, -0.70F, -0.0F, -0.0F, -0.0F, -0.0F};
    TaskSampler::Bound tsr_upper = {-0.19F, 0.70F, 0.0F, 0.0F, 0.0F, 0.0F};

    auto task_sampler = vamp::planning::make_task_space_informed_sampler<Robot>(
        eef_to_offset,
        world_to_reference,
        tsr_lower,
        tsr_upper,
        environment,
        std::make_shared<vamp::rng::Halton<Robot>>());

    // Resolve a task-space pose through parameterized IK, then collision-check the resulting
    // ambient (joint-space) configuration; on collision, dump the fkcc_debug breakdown.
    auto resolve_and_check = [&](const Robot::ConfigurationArray &pose_array, const std::string &label)
    {
        Robot::Configuration pose(pose_array);
        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = pose.broadcast(i);
        }

        auto [param_valid, ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
        // std::cout << label << " parameterized IK valid: " << std::boolalpha << param_valid << std::endl;

        if (not param_valid)
        {
            return false;
        }

        // print the ambient_block as a comma-separated list
        // for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        // {
        //     std::cout << ambient_block[{i, 0}] << (i < Robot::ambient_dimension - 1 ? ", " : "");
        // }
        // std::cout << std::endl;

        auto is_in_coll = Robot::template fkcc<rake>(env_v, ambient_block);
        // std::cout << label << " configuration is collision free: " << std::boolalpha << is_in_coll << std::endl;

        if (not is_in_coll)
        {
            return false;
            auto [env_cc, self_cc] = Robot::template fkcc_debug<rake>(env_v, ambient_block);
            std::cout << env_cc.size() << " environment collisions, " << self_cc.size() << " self-collisions"
                      << std::endl;

            for (const auto &pair : self_cc)
            {
                std::cout << "Self-collision between links " << pair.first << " and " << pair.second << std::endl;
            }

            for (std::size_t link_sphere_idx = 0; link_sphere_idx < env_cc.size(); ++link_sphere_idx)
            {
                const auto &cc = env_cc[link_sphere_idx];
                // cc is a std::vector<std::string> if not empty print its contents
                if (!cc.empty())
                {
                    std::cout << "Environment collision: ";
                    for (std::size_t i = 0; i < cc.size(); ++i)
                    {
                        std::cout << "(" << link_sphere_idx << ", " << i << ")" << (i < cc.size() - 1 ? ", " : "");
                    }
                    std::cout << std::endl;
                }
                
            }

            // also compute the forward kinematics of the ambient_block and compare it to the original pose_array
            Robot::AmbientConfigurationArray result_ambient_config_array;
            for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
            {
                result_ambient_config_array[i] = ambient_block[{i, 0}];
            }
            // run eefk
            auto eefk = Robot::eefk(result_ambient_config_array);
            const Eigen::Vector3f translation = eefk.translation();
            const Eigen::Quaternionf rotation(eefk.rotation());
            std::cout << label << " recomputed end effector pose from IK result: " << std::fixed << std::setprecision(6)
                      << translation.x() << ", " << translation.y() << ", " << translation.z() << ", "
                      << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ", "
                      << rotation.w() << std::endl;
        }

        return true;
    };


    std::cout << "\n--- TaskSpaceInformedSampler samples ---" << std::endl;
    for (int i = 0; i < 5; ++i)
    {
        Robot::Configuration sample_config(task_sampler->next());
        const auto sample_array = sample_config.to_array();

        std::cout << "Sample " << i << ": ";
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            std::cout << sample_array[j] << (j < Robot::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        Robot::template ConfigurationBlock<rake> sample_block;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            sample_block[j] = sample_config.broadcast(j);
        }

        auto [sample_valid, sample_ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(sample_block);
        std::cout << "  parameterized IK valid: " << std::boolalpha << sample_valid << std::endl;
        Robot::AmbientConfigurationArray sample_ambient_array;
        for (std::size_t j = 0; j < Robot::ambient_dimension; ++j)
        {
            sample_ambient_array[j] = sample_ambient_block[{j, 0}];
            std::cout << sample_ambient_array[j] << (j < Robot::ambient_dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        if (sample_valid)
        {

            auto sample_eefk = Robot::eefk(sample_ambient_array);
            const Eigen::Vector3f sample_translation = sample_eefk.translation();
            const Eigen::Quaternionf sample_rotation(sample_eefk.rotation());
            std::cout << "  recomputed eef pose: " << sample_translation.x() << ", " << sample_translation.y()
                      << ", " << sample_translation.z() << ", " << sample_rotation.x() << ", "
                      << sample_rotation.y() << ", " << sample_rotation.z() << ", " << sample_rotation.w()
                      << std::endl;
        }
    }
    std::cout << "--- end TaskSpaceInformedSampler samples ---\n" << std::endl;


    // Imaginary maze entry/exit poses: tool pointing straight down (qx=1,qy=0,qz=0,qw=0),
    // on the z=0 plane, redundancy parameter (psi) arbitrary at 0.0.
    Robot::ConfigurationArray start_pose_array = {0.6155468821525574 + 0.05 - 0.15, -0.62754705131053925, 0.22607783F, 0.0, -1.0F, 0.0F, 0.0F, 1.45};
    Robot::ConfigurationArray goal_pose_array = {0.5836458206176758 + 0.05 - 0.2, 0.4369207215309143, 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, 1.45};

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75;
    rrtc_settings.max_iterations = 1000000;
    rrtc_settings.max_samples = 1000000;
    rrtc_settings.dynamic_domain = false;

    IKLocalPlanner ik_local_planner;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};

    // Dump each successful problem's raw and shortcut paths for offline "distance" analysis
    // in python. Configurations here are this robot's task-space parameterization (x, y, z,
    // qx, qy, qz, qw, psi), not joint angles.
    auto path_to_json = [](const std::vector<Robot::Configuration> &path)
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

    // Resolve each task-space pose through parameterized_ik to the ambient (joint-space)
    // configuration -- this is what's actually physically reachable, so it's the
    // representation "distance" analysis should really care about. Split from the json/
    // distance helpers below so the (relatively expensive) IK resolution happens once per
    // waypoint, not once per consumer.
    auto resolve_ambient_path = [](const std::vector<Robot::Configuration> &path)
    {
        std::vector<Robot::AmbientConfigurationArray> ambient_path;
        ambient_path.reserve(path.size());
        for (const auto &config : path)
        {
            Robot::template ConfigurationBlock<rake> pose_block;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                pose_block[i] = config.broadcast(i);
            }

            auto [param_valid, ambient_block] =
                Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
            static_cast<void>(param_valid);

            Robot::AmbientConfigurationArray ambient_array;
            for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
            {
                ambient_array[i] = ambient_block[{i, 0}];
            }

            ambient_path.push_back(ambient_array);
        }

        return ambient_path;
    };

    auto ambient_path_to_json = [](const std::vector<Robot::AmbientConfigurationArray> &ambient_path)
    {
        nlohmann::json arr = nlohmann::json::array();
        for (const auto &q : ambient_path)
        {
            arr.push_back(std::vector<float>(q.begin(), q.end()));
        }

        return arr;
    };

    // Hand-rolled Euclidean distance summed over consecutive ambient (joint-space) waypoints.
    auto ambient_path_distance = [](const std::vector<Robot::AmbientConfigurationArray> &ambient_path)
    {
        float total = 0.0F;
        for (std::size_t i = 0; i + 1 < ambient_path.size(); ++i)
        {
            float squared = 0.0F;
            for (std::size_t j = 0; j < Robot::ambient_dimension; ++j)
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

    // Total SE3 distance along a task-space pose path. The Configuration already *is* the eef
    // pose (x, y, z, qx, qy, qz, qw, psi) -- no FK needed -- and index 7 (psi, the
    // self-motion-manifold redundancy parameter) is simply never read here, since it isn't
    // part of the eef pose.
    auto path_se3_distance = [&](const std::vector<Robot::Configuration> &path)
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

    auto result = RRTC::solve(
        Robot::Configuration(start_pose_array),
        Robot::Configuration(goal_pose_array),
        env_v,
        rrtc_settings,
        task_sampler,
        ik_local_planner);

    std::cout << "RRTC path size: " << result.path.size() << ", iterations: " << result.iterations
              << ", microseconds: " << result.nanoseconds / 1000.0F << ", with tree sizes: " << result.size[0] << ", "
              << result.size[1] << std::endl;


    std::vector<Problem> problems;
    // load the json file of problems from the specified path
    std::string problem_json_path = "resources/iiwa_marker/maze_problems_checked_ik.json";
    load_problems_from_json(problems, problem_json_path);



    size_t total_num_problems = 0;
    size_t successful_problems = 0;
    std::vector<std::size_t> nanoseconds_per_problem;
    std::vector<std::size_t> iterations_per_problem;
    std::vector<std::size_t> shortcut_nanoseconds_per_problem;
    std::vector<std::size_t> path_size_before_shortcut;
    std::vector<std::size_t> path_size_after_shortcut;
    size_t valid_problems = 0;


    for(const auto &problem: problems){
        std::cout << "Planning problem " << total_num_problems + 1 << " / " << problems.size() << std::endl;
        total_num_problems++;

        // Configuration start = Configuration(problem.problem_start);
        // Configuration goal = Configuration(problem.problem_end);
        // convert problem start_eef_pos and goal_eef_pos to 8 elements, fill 3:7 with 0.0, -1.0F, 0.0F, 0.0F and 8 with 1.45

        // write a lambda
        auto make_pose_array = [](const std::array<float, 3> &eef_pos) {
            Robot::ConfigurationArray pose_array;
            pose_array[0] = eef_pos[0];
            pose_array[1] = eef_pos[1];
            pose_array[2] = 0.22607783F;
            pose_array[3] = 0.0F;
            pose_array[4] = -1.0F;
            pose_array[5] = 0.0F;
            pose_array[6] = 0.0F;
            pose_array[7] = 1.45F;
            return pose_array;
        };
        const Robot::ConfigurationArray start_pose_array = make_pose_array(problem.start_eef_pos);
        const Robot::ConfigurationArray goal_pose_array = make_pose_array(problem.goal_eef_pos);

        auto start_valid = resolve_and_check(start_pose_array, "Start");
        auto goal_valid = resolve_and_check(goal_pose_array, "Goal");

        if (not start_valid || not goal_valid)
        {
            std::cout << "Skipping problem due to invalid start or goal configuration." << std::endl;
            continue;
        }

        valid_problems++;


        auto result = RRTC::solve(
            Robot::Configuration(start_pose_array),
            Robot::Configuration(goal_pose_array),
            env_v,
            rrtc_settings,
            task_sampler,
            ik_local_planner);


        if(result.path.size() > 0)
        {
            successful_problems++;
            nanoseconds_per_problem.push_back(result.nanoseconds);
            iterations_per_problem.push_back(result.iterations);

            auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, env_v, simplify_settings, rng, ik_local_planner);
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
            all_paths.push_back(path_entry);

            std::ofstream paths_file(paths_output_path);
            if (paths_file.is_open())
            {
                paths_file << all_paths.dump(4);
            }
        }
        else {
            std::cout << "Unable to solve problem with start and goal configs : " << Robot::Configuration(start_pose_array) << " and " << Robot::Configuration(goal_pose_array) << std::endl;
        }
    }

    std::cout << "Saved " << all_paths.size() << " problem paths to " << paths_output_path << std::endl;

    // print summary of results
    std::cout << "Total problems: " << total_num_problems << std::endl
                << "Valid problems: " << valid_problems << std::endl
                << "Successful problems: " << successful_problems << std::endl
                << "Success rate: " << (static_cast<float>(successful_problems) / valid_problems) * 100.0f << "%" << std::endl;

    if(successful_problems > 0){
        std::size_t total_nanoseconds = 0;
        std::size_t total_iterations = 0;
        for(size_t i = 0; i < successful_problems; i++){
            total_nanoseconds += nanoseconds_per_problem[i];
            total_iterations += iterations_per_problem[i];
        }
        std::cout << "Average time (ms) for successful problems: " << (total_nanoseconds / successful_problems) / 1000000.0 << std::endl;
        std::cout << "Average iterations for successful problems: " << total_iterations / successful_problems << std::endl;
        // compute median for time and iterations
        std::sort(nanoseconds_per_problem.begin(), nanoseconds_per_problem.end());
        std::sort(iterations_per_problem.begin(), iterations_per_problem.end());
        std::cout << "Median time (ms) for successful problems: " << (nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Median iterations for successful problems: " << iterations_per_problem[successful_problems / 2] << std::endl;
        std::cout << "Minimum time (ms) for successful problems: " << (nanoseconds_per_problem[0]) / 1000000.0 << std::endl;
        std::cout << "Minimum iterations for successful problems: " << iterations_per_problem[0] << std::endl;
        std::cout << "Maximum time (ms) for successful problems: " << (nanoseconds_per_problem[successful_problems - 1]) / 1000000.0 << std::endl;
        std::cout << "Maximum iterations for successful problems: " << iterations_per_problem[successful_problems - 1] << std::endl;

        // also get q1, q3 and 95% results here
        std::cout << "Q1 time (ms) for successful problems: " << (nanoseconds_per_problem[successful_problems / 4]) / 1000000.0 << std::endl;
        std::cout << "Q1 iterations for successful problems: " << iterations_per_problem[successful_problems / 4] << std::endl;
        std::cout << "Q3 time (ms) for successful problems: " << (nanoseconds_per_problem[3 * successful_problems / 4]) / 1000000.0 << std::endl;
        std::cout << "Q3 iterations for successful problems: " << iterations_per_problem[3 * successful_problems / 4] << std::endl;
        std::cout << "95th percentile time (ms) for successful problems: " << (nanoseconds_per_problem[95 * successful_problems / 100]) / 1000000.0 << std::endl;
        std::cout << "95th percentile iterations for successful problems: " << iterations_per_problem[95 * successful_problems / 100] << std::endl;

        // shortcutting stats
        std::size_t total_shortcut_nanoseconds = 0;
        std::size_t total_size_before = 0;
        std::size_t total_size_after = 0;
        for (size_t i = 0; i < successful_problems; i++)
        {
            total_shortcut_nanoseconds += shortcut_nanoseconds_per_problem[i];
            total_size_before += path_size_before_shortcut[i];
            total_size_after += path_size_after_shortcut[i];
        }
        std::sort(shortcut_nanoseconds_per_problem.begin(), shortcut_nanoseconds_per_problem.end());
        std::cout << "Average shortcut time (ms): " << (total_shortcut_nanoseconds / successful_problems) / 1000000.0 << std::endl;
        std::cout << "Median shortcut time (ms): " << (shortcut_nanoseconds_per_problem[successful_problems / 2]) / 1000000.0 << std::endl;
        std::cout << "Average path size before shortcutting: " << static_cast<float>(total_size_before) / successful_problems << std::endl;
        std::cout << "Average path size after shortcutting: " << static_cast<float>(total_size_after) / successful_problems << std::endl;
    }
    return 0;

}
