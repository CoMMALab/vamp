// Port of iiwa_maze_solver.cc to vamp::robots::FR3Marker (vamp/robots/fr3_marker.hh): given a
// single hardcoded start/goal eef pose (edit start_eef_pos/goal_eef_pos below), resolves both to
// an ambient configuration and runs one RRTC + shortcut solve, printing debug info along the way.
//
// FR3's ParameterizedSpace free parameter (index 7 of StateArray) is joint 7 directly, kept
// named "psi" below for continuity with the iiwa file; its branch triple `smm` is (case6_sel,
// case1_sel, q_actual_0) instead of iiwa's GC2/GC4/GC6 -- see fr3_parameterization.hh and
// fr3_maze_problem_generator.cc's header for the full rationale. Unlike the iiwa file (which
// only searches psi, always on ParameterizedSpace's default (1,1,1) branch), find_valid_pose
// below additionally searches the branch: for each eef pose it tries, in order, (case6_sel,
// case1_sel) = (0,1), (1,0), (0,0), (1,1), sampling psi (and q_actual_0, which only matters in
// the measure-zero wrist-point-singular case) at random within each branch, keeping the first
// (branch, psi, q_actual_0) that resolves to a valid, in-limits, collision-free configuration.
// Start and goal are resolved on a *shared* branch (see find_valid_start_goal below) since a
// single RRTC/steer call needs one consistent arm posture throughout -- ParameterizedSpace's
// resolve_block reads `smm` as global state, not something carried inside the State itself.

#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
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
#include <vamp/utils/profiling.hh>
#include <vamp/robots/fr3_marker.hh>

using Robot = vamp::robots::FR3Marker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution>;
using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot, ParameterizedSpace>;

// Fixed height for the maze task -- see fr3_maze_problem_generator.cc's kEefZ comment (kept
// numerically identical to the iiwa file on the assumption of a shared maze/mount frame).
constexpr float kEefZ = 0.150519F;

// (case6_sel, case1_sel) branches to try, in the order requested: (0,1), (1,0), (0,0), (1,1).
// q_actual_0 (the third smm slot) isn't part of branch identity -- it's resampled per psi
// candidate in find_valid_pose below -- so it's left at 0 here and overwritten before use.
static constexpr std::array<std::array<float, 3>, 4> kBranchOrder = {{
    {0.0F, 1.0F, 0.0F},
    {1.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 0.0F},
    {1.0F, 1.0F, 0.0F},
}};

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
// fkcc_debug breakdown. Mirrors iiwa_maze_solver.cc's resolve_and_check.
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

// Resolves a single eef_pos to a valid (IK + collision-free) task-space pose by searching psi
// (q7) at random within whichever branch is currently set via ParameterizedSpace::set_smm --
// branch selection itself lives one level up, in find_valid_start_goal, since start and goal
// must resolve on the *same* branch. q_actual_0 (see file header) is resampled per psi candidate
// alongside psi -- it only matters in the measure-zero wrist-point-singular case, so there's no
// cost to redrawing it rather than fixing it for the whole branch.
//
// Higher than the 16 candidates used elsewhere (fr3_maze_problem_generator.cc/
// fr3_maze_solver_benchmark.cc): those resample a fresh random eef pose whenever a candidate
// fails to resolve, whereas this file solves one fixed, user-provided pose pair, so it needs
// enough attempts to reliably clear whatever single-pose resolve rate that pair happens to have
// (this file's own default start/goal resolve at roughly 0.5-2% per psi/q_actual_0 draw).
constexpr int kNumPsiCandidates = 512;
static auto find_valid_psi_pose(
    const std::array<float, 3> &eef_pos,
    const std::array<float, 2> &case_branch,
    const EnvironmentVector &env_v) -> std::pair<bool, ParameterizedSpace::StateArray>
{
    for (int k = 0; k < kNumPsiCandidates; ++k)
    {
        const float psi = static_cast<float>(rand()) / RAND_MAX * 2.0F * static_cast<float>(M_PI);
        const float q_actual_0 =
            Robot::lower_bound[0] +
            static_cast<float>(rand()) / RAND_MAX * (Robot::upper_bound[0] - Robot::lower_bound[0]);
        ParameterizedSpace::set_smm({case_branch[0], case_branch[1], q_actual_0});

        ParameterizedSpace::StateArray pose_array = {
            {eef_pos[0], eef_pos[1], kEefZ, 0.0F, 1.0F, 0.0F, 0.0F, psi}};

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
}

// Resolves start_eef_pos/goal_eef_pos to a valid pose pair on a *shared* (case6_sel, case1_sel)
// branch -- tries kBranchOrder in order (0,1), (1,0), (0,0), (1,1), resolving both endpoints'
// psi (find_valid_psi_pose above) against each branch in turn before moving to the next, so
// start and goal are never accepted on different branches. Returns the branch that worked
// alongside both resolved poses; leaves ParameterizedSpace::smm set to that branch on success.
static auto find_valid_start_goal(
    const std::array<float, 3> &start_eef_pos,
    const std::array<float, 3> &goal_eef_pos,
    const EnvironmentVector &env_v)
    -> std::tuple<bool, ParameterizedSpace::StateArray, ParameterizedSpace::StateArray, std::array<float, 3>>
{
    for (const auto &branch : kBranchOrder)
    {
        const std::array<float, 2> case_branch = {branch[0], branch[1]};

        auto [start_valid, start_pose] = find_valid_psi_pose(start_eef_pos, case_branch, env_v);
        if (!start_valid)
        {
            std::cout << "Branch (" << case_branch[0] << ", " << case_branch[1]
                      << "): failed to resolve start pose after " << kNumPsiCandidates << " psi attempts"
                      << std::endl;
            continue;
        }

        auto [goal_valid, goal_pose] = find_valid_psi_pose(goal_eef_pos, case_branch, env_v);
        if (!goal_valid)
        {
            std::cout << "Branch (" << case_branch[0] << ", " << case_branch[1]
                      << "): resolved start but failed to resolve goal pose after " << kNumPsiCandidates
                      << " psi attempts" << std::endl;
            continue;
        }

        std::cout << "Resolved start/goal on branch (case6_sel=" << case_branch[0]
                  << ", case1_sel=" << case_branch[1] << "), start psi=" << start_pose[7]
                  << ", goal psi=" << goal_pose[7] << std::endl;
        return {true, start_pose, goal_pose, branch};
    }

    return {false, {}, {}, {}};
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

    std::array<float, 7> example_config = {-2.04490065574646,
            0.4090642035007477,
            0.9035356640815735,
            -2.0449202060699463,
            -0.42098408937454224,
            2.261169195175171,
            1.3182013034820557
};
    // compute fk from this
    auto example_eefk = Robot::eefk(example_config);
    std::cout << example_eefk.matrix() << std::endl;
    const Eigen::Vector3f example_translation = example_eefk.translation();
    const Eigen::Quaternionf example_rotation(example_eefk.rotation());
    std::cout << "Example configuration: ";
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        std::cout << example_config[i] << (i < Robot::dimension - 1 ? ", " : "");
    }
    std::cout << std::endl;
    std::cout << "Example eef pose: " << example_translation.x() << ", " << example_translation.y() << ", " << example_translation.z() << ", "
              << example_rotation.x() << ", " << example_rotation.y() << ", " << example_rotation.z() << ", " << example_rotation.w() << std::endl;
    
    return 0;
    // --- Task Space Region: tool facing down (with only slack for numerical tilt), free yaw
    // about the down axis, and pinned to the z=0 plane (xy free within the maze footprint).
    //
    // Transform layout consumed by TaskSpaceInformedSampler is (x, y, z, qx, qy, qz, qw) --
    // translation first, quaternion scalar-last. eef_to_offset is the hardcoded tool-to-eef
    // transform: identity here (the marker tip pose IS the eef pose).
    TaskSampler::Transform world_to_reference = {0.0F, 0.0F, kEefZ, 0.0F, 1.0F, 0.0F, 0.0F};
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

    // --- Manual start/goal eef positions: edit these to solve a different problem. Tool
    // pointing straight down (qx=1,qy=0,qz=0,qw=0), on the z=kEefZ plane. NOT the same demo
    // positions as iiwa_maze_solver.cc's -- those are outside FR3's reachable workspace at this
    // height/orientation (confirmed by exhaustive sampling: 0 resolves in 2M attempts across all
    // 4 branches). At kEefZ, FR3 in this mount/frame can only reach roughly x in [0.2, 0.5],
    // y in [-0.68, -0.18] (the maze's nominal footprint is x in [0.196, 0.755], y in
    // [-0.7, 0.7]) -- these two points were chosen from within that reachable region instead.
    const std::array<float, 3> start_eef_pos = {0.22F, -0.62F, kEefZ};
    const std::array<float, 3> goal_eef_pos = {0.45F, -0.22F, kEefZ};

    auto [resolved, start_state_array, goal_state_array, branch] =
        find_valid_start_goal(start_eef_pos, goal_eef_pos, env_v);
    if (!resolved)
    {
        std::cerr << "Failed to find a shared branch/psi for the start/goal poses." << std::endl;
        return 1;
    }

    // find_valid_start_goal leaves ParameterizedSpace::smm set to `branch` already, but set it
    // explicitly here too so it's unambiguous for every resolve_block call below (the
    // TaskSpaceInformedSampler debug block above, and any future edits between here and there,
    // could otherwise leave stale state).
    ParameterizedSpace::set_smm(branch);

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
#ifdef VAMP_PROFILING
    std::cout << "\n--- Kernel profiling (aggregated over all problems) ---" << std::endl;
    vamp::utils::profiling::report(std::cout);
#endif

}
