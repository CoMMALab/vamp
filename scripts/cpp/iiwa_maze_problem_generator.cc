#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cstring>
#include <string>
#include <fstream>
#include <limits>
#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/iiwamarker.hh>
#include <vamp/random/halton.hh>

using json = nlohmann::json;

using Robot = vamp::robots::IiwaMarker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using Configuration = typename Robot::Configuration;
using ConfigurationArray = typename Robot::ConfigurationArray;
static constexpr auto dimension = Robot::dimension;
using RNG = typename vamp::rng::RNG<Robot>;

// Define your problem structure. problem_start/problem_end are the ambient (joint-space)
// configuration resolved by parameterized_ik for the eef pose, not the task-space pose array
// itself.
struct Problem
{
    std::array<float, Robot::ambient_dimension> problem_start;
    std::array<float, Robot::ambient_dimension> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
};

// This helper function allows nlohmann::json to "just work" with your struct
void to_json(json &j, const Problem &p)
{
    j = json{
        {"problem_start", p.problem_start},
        {"problem_end", p.problem_end},
        {"start_eef_pos", p.start_eef_pos},
        {"goal_eef_pos", p.goal_eef_pos}};
}

static auto load_cuboids_from_json(EnvironmentInput &environment, const std::string &path)
{
    std::array<float, 2> min_bound = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    std::array<float, 2> max_bound = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    nlohmann::json j;
    try
    {
        ifs >> j;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    if (!j.is_array())
    {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    for (const auto &obj : j)
    {
        if (!obj.is_object())
        {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz"))
        {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try
        {
            float x = obj.at("x").get<float>() + 0.05F;  // push it slightly forward, matches iiwa_maze_solver
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

            // Update bounds
            min_bound[0] = std::min(min_bound[0], posf[0] - sizef[0]);
            min_bound[1] = std::min(min_bound[1], posf[1] - sizef[1]);
            max_bound[0] = std::max(max_bound[0], posf[0] + sizef[0]);
            max_bound[1] = std::max(max_bound[1], posf[1] + sizef[1]);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return std::make_pair(min_bound, max_bound);
}

int main(int argc, char **argv)
{
    bool check_ik = false;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--check_ik") == 0)
        {
            check_ik = true;
        }
    }

    std::vector<Problem> problems;
    const char *output_path = "resources/iiwa_marker/maze_problems_checked_ik.json";
    std::ofstream file(output_path);

    EnvironmentInput environment;

    auto bounds = load_cuboids_from_json(environment, "resources/environments/real_maze.json");
    std::cout << "Loaded environment bounds: x[" << bounds.first[0] << ", " << bounds.second[0] << "], y["
              << bounds.first[1] << ", " << bounds.second[1] << "]" << std::endl;

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Fixed height / orientation for the end effector, matching iiwa_maze_solver.cc: tool
    // pointing straight down (qx=1,qy=0,qz=0,qw=0 -- i.e. (0, -1, 0, 0) in this parameterization),
    // redundancy parameter (psi) fixed at 1.45.
    constexpr float kEefZ = 0.22607783F;
    constexpr float kPsi = 1.45F;

    auto make_pose_array = [&](const std::array<float, 3> &eef_pos)
    {
        Robot::ConfigurationArray pose_array;
        pose_array[0] = eef_pos[0];
        pose_array[1] = eef_pos[1];
        pose_array[2] = kEefZ;
        pose_array[3] = 0.0F;
        pose_array[4] = -1.0F;
        pose_array[5] = 0.0F;
        pose_array[6] = 0.0F;
        pose_array[7] = kPsi;
        return pose_array;
    };

    // Cheap early-reject check: broadcast the (single) candidate pose across all lanes and ask
    // whether the end-effector spheres implied by that pose are collision free, without solving
    // IK for the rest of the arm.
    auto is_eef_pose_valid = [&](const Robot::ConfigurationArray &pose_array)
    {
        Robot::Configuration pose(pose_array);
        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = pose.broadcast(i);
        }

        return Robot::template check_if_ik_valid_block<rake>(env_v, pose_block);
    };

    // Solve the pose to an ambient (joint-space) configuration via parameterized IK and
    // collision-check it. This is what actually gets written to problem_start/problem_end --
    // the task-space pose array is only ever an intermediate used to reach it.
    auto resolve_ambient_config = [&](const Robot::ConfigurationArray &pose_array)
        -> std::pair<bool, Robot::AmbientConfigurationArray>
    {
        Robot::Configuration pose(pose_array);
        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = pose.broadcast(i);
        }

        auto [param_valid, ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);

        if (not param_valid or not Robot::template fkcc<rake>(env_v, ambient_block))
        {
            return {false, {}};
        }

        Robot::AmbientConfigurationArray ambient_array;
        for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        {
            ambient_array[i] = ambient_block[{i, 0}];
        }

        return {true, ambient_array};
    };

    // Stricter check used with --check_ik: same as above, bool-only, for the straight-line
    // triviality sweep.
    auto is_eef_pose_ik_valid = [&](const Robot::ConfigurationArray &pose_array)
    { return resolve_ambient_config(pose_array).first; };

    // Reject problems that a straight line in eef-space already solves: sample points along
    // the segment from start to goal and check each with whichever validity check the endpoints
    // were accepted with. If every sample is collision free, the problem is trivial.
    constexpr int kNumStraightLineSamples = 100;
    auto is_straight_line_trivial = [&](const Robot::ConfigurationArray &start_pose_array,
                                         const Robot::ConfigurationArray &goal_pose_array)
    {
        for (int i = 0; i < kNumStraightLineSamples; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(kNumStraightLineSamples - 1);

            Robot::ConfigurationArray interp_pose_array;
            for (std::size_t d = 0; d < Robot::dimension; ++d)
            {
                interp_pose_array[d] = start_pose_array[d] + t * (goal_pose_array[d] - start_pose_array[d]);
            }

            const bool valid = check_ik ? is_eef_pose_ik_valid(interp_pose_array) : is_eef_pose_valid(interp_pose_array);
            if (!valid)
            {
                return false;
            }
        }

        return true;
    };

    while (problems.size() < 200)
    {
        // sample random position for the problem within the bounds of the environment
        std::array<float, 3> random_start_position = {
            bounds.first[0] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[0] - bounds.first[0]),
            bounds.first[1] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[1] - bounds.first[1]),
            kEefZ};
        std::array<float, 3> random_goal_position = {
            bounds.first[0] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[0] - bounds.first[0]),
            bounds.first[1] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[1] - bounds.first[1]),
            kEefZ};

        // if the points are too close to each other, resample
        if ((random_start_position[0] - random_goal_position[0]) * (random_start_position[0] - random_goal_position[0]) +
                (random_start_position[1] - random_goal_position[1]) * (random_start_position[1] - random_goal_position[1]) <
            0.1)
            continue;

        const auto start_pose_array = make_pose_array(random_start_position);
        const auto goal_pose_array = make_pose_array(random_goal_position);

        if (!is_eef_pose_valid(start_pose_array) || !is_eef_pose_valid(goal_pose_array))
        {
            continue;
        }

        if (check_ik)
        {
            if (!is_eef_pose_ik_valid(start_pose_array) || !is_eef_pose_ik_valid(goal_pose_array))
            {
                continue;
            }
        }

        if (is_straight_line_trivial(start_pose_array, goal_pose_array))
        {
            continue;
        }

        // Resolve start/goal to the ambient (joint-space) configuration to actually store --
        // required regardless of --check_ik, since that's what problem_start/problem_end hold.
        auto [start_ik_valid, start_ambient] = resolve_ambient_config(start_pose_array);
        auto [goal_ik_valid, goal_ambient] = resolve_ambient_config(goal_pose_array);

        if (!start_ik_valid || !goal_ik_valid)
        {
            continue;
        }

        Problem p;
        p.problem_start = start_ambient;
        p.problem_end = goal_ambient;
        p.start_eef_pos = random_start_position;
        p.goal_eef_pos = random_goal_position;
        std::cout << "Adding start/goal to problem set (" << problems.size() + 1 << " / 100)" << std::endl;
        problems.push_back(p);

        if (file.is_open())
        {
            file.seekp(0);
            file << json(problems).dump(4);
            file.flush();
        }
    }

    json j_all_problems = problems;
    if (file.is_open())
    {
        file.seekp(0);
        file << j_all_problems.dump(4);
        std::cout << "Successfully saved to " << output_path << std::endl;
    }

    return 0;
}
