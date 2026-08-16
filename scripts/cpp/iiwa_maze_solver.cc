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

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/constraints/task_space_informed_sampler.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/iiwa_marker.hh>

using Robot = vamp::robots::IiwaMarker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution>;
using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot, ParameterizedSpace>;

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

// Resolves a task-space State through IK (ParameterizedSpace::resolve_block) and, on success,
// collision-checks the resulting ambient (joint-space) configuration; on collision, prints the
// fkcc_debug breakdown. Mirrors rby1_task_space_planner.cc's resolve_and_report /
// check_collision_free pair.
static auto resolve_and_check(
    const ParameterizedSpace::State &state,
    const std::string &label,
    const EnvironmentVector &environment_v,
    const bool debug_print = false
) -> Robot::ConfigurationBlock<rake>
{
    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
    std::cout << label << " resolve_block valid: " << std::boolalpha << param_valid << std::endl;
    if (debug_print)
    {
        std::cout << label << " ambient configuration: ";
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            std::cout << ambient_block[{i, 0}] << (i < Robot::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;
    }

    if (not param_valid)
    {
        return ambient_block;
    }

    const bool collision_free = Robot::fkcc<rake>(environment_v, ambient_block);
    if (not collision_free)
    {
        auto [env_cc, self_cc] = Robot::fkcc_debug<rake>(environment_v, ambient_block);
        std::cout << env_cc.size() << " environment collisions, " << self_cc.size() << " self-collisions"
                  << std::endl;

        for (const auto &pair : self_cc)
        {
            std::cout << "Self-collision between links " << pair.first << " and " << pair.second << std::endl;
        }

        for (std::size_t link_sphere_idx = 0; link_sphere_idx < env_cc.size(); ++link_sphere_idx)
        {
            const auto &cc = env_cc[link_sphere_idx];
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
    }

    return ambient_block;
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
    //
    // Transform layout consumed by TaskSpaceInformedSampler is (x, y, z, qx, qy, qz, qw) --
    // translation first, quaternion scalar-last. eef_to_offset is the hardcoded tool-to-eef
    // transform: identity here (the marker tip pose IS the eef pose).
    TaskSampler::Transform world_to_reference = {0.0F, 0.0F, 0.22607783F, 0.0F, 1.0F, 0.0F, 0.0F};
    TaskSampler::Transform eef_to_offset = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F};

    // Bound order is (dx, dy, dz, rx, ry, rz): translation box + so(3) log-map rotation box.
    // z pinned to the xy plane, xy free within the maze footprint; rx/ry (tilt away from facing
    // down) held to a tight numerical tolerance, rz (yaw about the down axis) free over a full
    // turn.
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
    // so instead of hardcoding one shared value (which pushes the wrist bend, joint 6, near its
    // singularity for some eef positions and not others), sweep a handful of candidates and
    // keep the first that resolves within joint limits and collision-free.
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

            ParameterizedSpace::State pose(pose_array.data());
            ParameterizedSpace::StateBlock<rake> pose_block;
            for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
            {
                pose_block[i] = pose.broadcast(i);
            }

            auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(pose_block);
            if (param_valid and Robot::fkcc<rake>(env_v, ambient_block))
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
        Robot::ConfigurationArray sample_ambient_array;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            sample_ambient_array[j] = sample_ambient_block[{j, 0}];
            std::cout << sample_ambient_array[j] << (j < Robot::dimension - 1 ? ", " : "");
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
    auto [start_psi_found, start_state_array] =
        find_valid_psi_pose({ 
            0.5383931994438171,
            -0.2849438190460205,
            0.2260778248310089
        });
    auto [goal_psi_found, goal_state_array] =
        find_valid_psi_pose({
            0.5523142218589783,
            0.033929165452718735,
            0.2260778248310089
        });
    if (!start_psi_found || !goal_psi_found)
    {
        std::cerr << "Failed to find a valid psi for the start/goal poses." << std::endl;
        return 1;
    }

    ParameterizedSpace::State start_state(start_state_array.data());
    ParameterizedSpace::State goal_state(goal_state_array.data());

    resolve_and_check(start_state, "Start", env_v, true);
    resolve_and_check(goal_state, "Goal", env_v, true);

    // --- DEBUG: eefs_collision_free sanity check ---
    // start_state is already known-IK-valid and collision-free (per resolve_and_check above,
    // which runs full fkcc on the resolved ambient config). Feed the same task-space state
    // through the eef-only prefilter directly, with the RRT/shortcut path below disabled, so we
    // can inspect exactly what world-space sphere locations eefs_collision_free is testing.
    {
        ParameterizedSpace::StateBlock<rake> debug_block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            debug_block[i] = start_state.broadcast(i);
        }

        std::cout << "\n--- eefs_collision_free debug (start_state) ---" << std::endl;
        const bool eef_free = ParameterizedSpace::eefs_collision_free<rake>(env_v, debug_block);
        std::cout << "eefs_collision_free result: " << std::boolalpha << eef_free << std::endl;
        std::cout << "--- end eefs_collision_free debug ---\n" << std::endl;
    }

    // call distance between start and goal
    float start_goal_distance = ParameterizedSpace::distance(start_state, goal_state);
    std::cout << "Start to goal distance: " << start_goal_distance << std::fixed << std::setprecision(6) << start_goal_distance << std::endl;

    // ----- call steer function from start to goal and check if the path is valid -----
    TaskLocalPlanner ik_local_planner;
    const auto steer_extension = ik_local_planner.steer(
        start_state, 
        goal_state,
        start_goal_distance,
        0.75F,
        true,
        env_v);
    const bool steer_valid = steer_extension.status == vamp::planning::SteerStatus::Reached;
    std::cout << "Steer from start to goal valid: " << std::boolalpha << steer_valid
               << ", path size: " << steer_extension.waypoints.size() << std::endl;


    // /* RRT/shortcut path disabled while debugging eefs_collision_free above.
    auto rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();
    // Restart the Halton sequence for each problem so results are reproducible per-problem
    // and independent of how many samples earlier problems in this run consumed.
    task_sampler->reset();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.75;
    rrtc_settings.max_iterations = 1000000;
    rrtc_settings.max_samples = 1000000;
    rrtc_settings.dynamic_domain = false;

    const TaskLocalPlanner task_local_planner;

    auto result = TaskRRTC::solve(
        start_state,
        goal_state,
        env_v,
        rrtc_settings,
        task_sampler,
        task_local_planner);

    std::cout << "RRTC path size: " << result.path.size() << ", iterations: " << result.iterations
              << ", microseconds: " << result.nanoseconds / 1000.0F << ", with tree sizes: " << result.size[0] << ", "
              << result.size[1] << std::endl;

    vamp::planning::SimplifySettings simplify_settings;
    simplify_settings.operations = {vamp::planning::SHORTCUT};
    auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution, TaskLocalPlanner, ParameterizedSpace>(
        result.path, env_v, simplify_settings, rng, task_local_planner);

    std::cout << "Shortcut path size: " << shortcut_result.path.size() << " (from " << result.path.size()
              << "), nanoseconds: " << shortcut_result.nanoseconds << std::endl;

    for (const auto &state : shortcut_result.path)
    {
        ParameterizedSpace::StateBlock<rake> block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            block[i] = state.broadcast(i);
        }

        auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);

        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            std::cout << ambient_block[{i, 0}] << (i < Robot::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;
    }
}
