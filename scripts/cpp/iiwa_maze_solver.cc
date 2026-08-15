#include <array>
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
    const EnvironmentVector &environment_v) -> Robot::ConfigurationBlock<rake>
{
    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
    std::cout << label << " resolve_block valid: " << std::boolalpha << param_valid << std::endl;

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
    TaskSampler::Bound tsr_lower = {-0.85F, -0.7F, -0.0F, -0.000F, -0.000F, -0.000F};
    TaskSampler::Bound tsr_upper = {0.0F, 0.7F, 0.0F, 0.00F, 0.000F, 0.000F};

    auto task_sampler = vamp::planning::make_task_space_informed_sampler<Robot, ParameterizedSpace>(
        eef_to_offset,
        world_to_reference,
        tsr_lower,
        tsr_upper,
        environment,
        std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>());

    // Imaginary maze entry/exit poses: tool pointing straight down (qx=1,qy=0,qz=0,qw=0), on
    // the z=0 plane, redundancy parameter (psi) arbitrary at 1.45.
    ParameterizedSpace::StateArray start_state_array = {
        {0.6155468821525574F + 0.05F - 0.15F, -0.62754705131053925F, 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};
    ParameterizedSpace::StateArray goal_state_array = {
        {0.5836458206176758F + 0.05F - 0.2F, 0.4369207215309143F, 0.22607783F, 0.0F, -1.0F, 0.0F, 0.0F, 1.45F}};

    ParameterizedSpace::State start_state(start_state_array.data());
    ParameterizedSpace::State goal_state(goal_state_array.data());

    resolve_and_check(start_state, "Start", env_v);
    resolve_and_check(goal_state, "Goal", env_v);

    // --- Diagnostic: draw raw samples from task_sampler (same source RRTC will use) and check
    // each one individually (broadcast into all lanes, not packed via interpolate_block) to
    // isolate whether resolve_block or fkcc is rejecting nearly everything.
    {
        constexpr int n_diag_samples = 50;
        int resolve_valid_count = 0;
        int collision_free_count = 0;
        for (int i = 0; i < n_diag_samples; ++i)
        {
            ParameterizedSpace::State sample_state = task_sampler->next();
            ParameterizedSpace::StateBlock<rake> block;
            for (std::size_t j = 0; j < ParameterizedSpace::dimension; ++j)
            {
                block[j] = sample_state.broadcast(j);
            }

            auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
            if (!param_valid)
            {
                continue;
            }
            resolve_valid_count++;

            const bool collision_free = Robot::fkcc<rake>(env_v, ambient_block);
            if (collision_free)
            {
                collision_free_count++;
            }
        }
        std::cout << "Diagnostic: " << n_diag_samples << " samples, " << resolve_valid_count
                  << " resolve_block valid, " << collision_free_count << " collision-free" << std::endl;
    }

    auto rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0F;
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

    return 0;
}
