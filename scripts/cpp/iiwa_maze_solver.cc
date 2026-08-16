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
    TaskSampler::Bound tsr_lower = {-0.85F, -0.7F, -0.0F, -0.000F, -0.000F, -3.14159265358979F};
    TaskSampler::Bound tsr_upper = {0.0F, 0.7F, 0.0F, 0.00F, 0.000F, 3.14159265358979F};

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

        // print the ambient_block as a comma-separated list
        for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        {
            std::cout << ambient_block[{i, 0}] << (i < Robot::ambient_dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        auto is_in_coll = Robot::template fkcc<rake>(env_v, ambient_block);
        // std::cout << label << " configuration is collision free: " << std::boolalpha << is_in_coll << std::endl;

        if (not is_in_coll)
        {
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

        return ambient_block;
    };

    // psi (index 7) doesn't change the eef pose -- only which arm configuration reaches it --
    // so instead of hardcoding one shared value (which pushes the wrist bend, joint 6, near its
    // singularity for some eef positions and not others), sweep a handful of candidates and
    // keep the first that resolves within joint limits and collision-free. See
    // find_valid_psi_pose in iiwa_maze_problem_generator.cc for the full rationale.
    constexpr int kNumPsiCandidates = 16;
    auto find_valid_psi_pose = [&](const std::array<float, 3> &eef_pos) -> std::pair<bool, Robot::ConfigurationArray>
    {
        for (int k = 0; k < kNumPsiCandidates; ++k)
        {
            const float psi =
                2.0F * static_cast<float>(M_PI) * static_cast<float>(k) / static_cast<float>(kNumPsiCandidates);
            Robot::ConfigurationArray pose_array = {eef_pos[0], eef_pos[1], 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, psi};

            Robot::Configuration pose(pose_array);
            Robot::template ConfigurationBlock<rake> pose_block;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                pose_block[i] = pose.broadcast(i);
            }

            auto [param_valid, ambient_block] =
                Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
            if (param_valid and Robot::template fkcc<rake>(env_v, ambient_block))
            {
                return {true, pose_array};
            }
        }

        return {false, {}};
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


    // Imaginary maze entry/exit poses: tool pointing straight down (qx=1,qy=0,qz=0,qw=0), on
    // the z=0 plane. psi is searched per-endpoint rather than hardcoded (see
    // find_valid_psi_pose above).
    auto [start_psi_found, start_pose_array] =
        find_valid_psi_pose({ 0.5523142218589783F, 0.033929165452718735F, 0.22607783F});
    auto [goal_psi_found, goal_pose_array] =
        find_valid_psi_pose({0.5383931994438171F, -0.2849438190460205F, 0.22607783F});
    if (!start_psi_found || !goal_psi_found)
    {
        std::cerr << "Failed to find a valid psi for the start/goal poses." << std::endl;
        return 1;
    }

    auto start_ambient_block = resolve_and_check(start_pose_array, "Start");
    auto goal_ambient_block = resolve_and_check(goal_pose_array, "Goal");

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75;
    rrtc_settings.max_iterations = 1000000;
    rrtc_settings.max_samples = 1000000;
    rrtc_settings.dynamic_domain = false;

    IKLocalPlanner ik_local_planner;

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

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};
    auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
        result.path, env_v, simplify_settings, rng, ik_local_planner);

    std::cout << "Shortcut path size: " << shortcut_result.path.size() << " (from " << result.path.size()
              << "), nanoseconds: " << shortcut_result.nanoseconds << std::endl;

    for (const auto &config : result.path)
    {
        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = config.broadcast(i);
        }

        auto [param_valid, ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);

        for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        {
            std::cout << ambient_block[{i, 0}] << (i < Robot::ambient_dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;
    }

    return 0;
}
