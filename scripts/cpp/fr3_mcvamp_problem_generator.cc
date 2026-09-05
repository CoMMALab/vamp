// mcvamp problem generator: generates FR3Marker (vamp/robots/fr3_marker.hh) maze start/goal
// problems by *projecting* random ambient samples onto a task-space constraint (mcvamp's
// generic vamp::planning::constraint::TaskSpaceConstraint / ConstraintSet machinery), instead
// of fr3_maze_problem_generator.cc's closed-form analytic IK (ParameterizedSpace::resolve_block).
//
// Sampling, environment-bounds handling, and the straight-line-triviality rejection are kept
// structurally the same as fr3_maze_problem_generator.cc so the two problem sets are directly
// comparable; only how an eef position gets turned into an ambient joint configuration differs.
//
// The maze-plane manifold is expressed with the same constants fr3_maze_tsr_benchmark.cc uses
// for its region-wide TSR while planning:
//   - kEefZ / kIdentityOffset / kToolDownOrientation: tool-down orientation at a fixed height.
//   - region_lower / region_upper: (dx, dy, dz, rx, ry, rz) bound with z and tilt (rx, ry)
//     pinned tight and xy/yaw (rz) free -- the region-wide constraint used while *planning*
//     within the maze plane.
// For *generating* a single start/goal point, xy also needs to be pinned (to the sampled eef
// position), so each problem endpoint gets its own point constraint: same tool-down orientation
// translated to that point, same tight z/tilt tolerance as region_lower/region_upper, but with
// dx/dy tightened to that same tolerance too (rz stays free -- the maze doesn't constrain
// end-effector yaw). A random Halton ambient sample is then projected onto that point constraint
// via ConstraintSet::project, and kept if it converges and is collision free.

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/fr3_marker.hh>

using json = nlohmann::json;

using Robot = vamp::robots::FR3Marker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using TSC = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using ProjMethod = vamp::planning::constraint::ProjMethod;

using Configuration = typename Robot::Configuration;
using RNG = typename vamp::rng::RNG<Robot>;

// Define your problem structure. problem_start/problem_end are the ambient (joint-space)
// configuration that ConstraintSet::project resolved for the eef pose, not the task-space
// pose itself.
struct Problem
{
    std::array<float, Robot::dimension> problem_start;
    std::array<float, Robot::dimension> problem_end;
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
            float x = obj.at("x").get<float>() + 0.285*2;  // push it slightly forward, matches fr3_maze_problem_generator
            float y = obj.at("y").get<float>() + 0.0F; // push it slightly left, matches fr3_maze_problem_generator
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
    static_cast<void>(argc);
    static_cast<void>(argv);

    std::vector<Problem> problems;
    const char *output_path = "resources/fr3_marker/maze_problems_checked_ik.json";
    std::ofstream file(output_path);

    EnvironmentInput environment;

    auto bounds = load_cuboids_from_json(environment, "resources/environments/maze_cuboids.json");
    std::cout << "Loaded environment bounds: x[" << bounds.first[0] << ", " << bounds.second[0] << "], y["
              << bounds.first[1] << ", " << bounds.second[1] << "]" << std::endl;

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // Region-wide TSR constants, identical to fr3_maze_tsr_benchmark.cc's -- see file header.
    constexpr float kEefZ = 0.150519F;
    const TSC::Transform kIdentityOffset = {1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    const TSC::Transform kToolDownOrientation = {0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, kEefZ};

    // Region-wide TSR used while planning: z and tilt (rx, ry) pinned tight to the tool-down
    // plane, xy and yaw (rz) free -- the maze lives in that plane, and its footprint is
    // enforced by collision-checking against the maze walls, not by this bound. Bound order
    // is (dx, dy, dz, rx, ry, rz).
    const TSC::Bound region_lower = {-10.01F, -10.01F, -0.001F, -0.001F, -0.001F, -10.01F};
    const TSC::Bound region_upper = {10.01F, 10.01F, 0.001F, 0.001F, 0.001F, 10.01F};

    // Per-point TSR used while *generating*: same tight z/tilt tolerance as region_lower/
    // region_upper, reused for dx/dy too, so a specific eef position is pinned exactly (yaw,
    // rz, stays free, same as the region constraint).
    const TSC::Bound point_lower = {region_lower[2], region_lower[2], region_lower[2], region_lower[3], region_lower[4], region_lower[5]};
    const TSC::Bound point_upper = {region_upper[2], region_upper[2], region_upper[2], region_upper[3], region_upper[4], region_upper[5]};

    auto make_point_transform = [&](const std::array<float, 3> &eef_pos)
    {
        TSC::Transform t = kToolDownOrientation;
        t[4] = eef_pos[0];
        t[5] = eef_pos[1];
        t[6] = kEefZ;
        return t;
    };

    auto make_point_constraint = [&](const std::array<float, 3> &eef_pos)
    {
        return std::make_shared<TSC>(
            std::array<TSC::Transform, Robot::n_eef>{kIdentityOffset},
            std::array<TSC::Transform, Robot::n_eef>{make_point_transform(eef_pos)},
            std::array<TSC::Bound, Robot::n_eef>{point_lower},
            std::array<TSC::Bound, Robot::n_eef>{point_upper});
    };

    vamp::planning::constraint::ConstraintSettings point_settings;
    point_settings.method = ProjMethod::OuterLM;
    point_settings.descend_rate = 1.0F;
    point_settings.max_iterations = 100;

    // Sanity/collision check on a resolved ambient configuration: broadcast and collision-check
    // it directly, since it's already a real joint configuration.
    auto is_config_valid = [&](const Configuration &q)
    {
        Robot::template ConfigurationBlock<rake> block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            block[i] = q.broadcast(i);
        }

        return vamp::planning::fkcc_block<Robot, rake>(env_v, block);
    };

    // Project random Halton ambient samples onto the point constraint for eef_pos until one
    // converges and is collision free, or attempts run out.
    constexpr int kNumProjectionAttempts = 500;
    auto find_valid_config_for_point = [&](const std::array<float, 3> &eef_pos) -> std::pair<bool, Configuration>
    {
        auto constraint = make_point_constraint(eef_pos);
        ConstraintSetT constraint_set(std::vector<ConstraintSetT::Ptr>{constraint}, point_settings);

        for (int attempt = 0; attempt < kNumProjectionAttempts; ++attempt)
        {
            Configuration q = rng->next();
            if (!constraint_set.project(q))
            {
                continue;
            }

            if (!is_config_valid(q))
            {
                continue;
            }

            return {true, q};
        }

        return {false, {}};
    };

    // Reject problems that a straight line in eef-space already solves: sample points along the
    // segment from start to goal (fixed z/orientation, same as the endpoints) and try to resolve
    // each with a handful of projection attempts. If every sample resolves, the problem is
    // trivial. A sample that fails to resolve within its attempt budget is *not* proof of a
    // collision (it might just be an unlucky set of Halton draws), so it counts as "can't
    // certify trivial" and the problem is kept -- same conservative rule
    // fr3_maze_problem_generator.cc's is_straight_line_trivial uses.
    constexpr int kNumStraightLineSamples = 30;
    constexpr int kNumStraightLineAttempts = 20;
    auto is_straight_line_trivial = [&](const std::array<float, 3> &start_eef_pos, const std::array<float, 3> &goal_eef_pos)
    {
        for (int i = 0; i < kNumStraightLineSamples; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(kNumStraightLineSamples - 1);

            std::array<float, 3> interp_eef_pos;
            for (std::size_t d = 0; d < 3; ++d)
            {
                interp_eef_pos[d] = start_eef_pos[d] + t * (goal_eef_pos[d] - start_eef_pos[d]);
            }

            auto constraint = make_point_constraint(interp_eef_pos);
            ConstraintSetT constraint_set(std::vector<ConstraintSetT::Ptr>{constraint}, point_settings);

            bool resolved = false;
            for (int attempt = 0; attempt < kNumStraightLineAttempts; ++attempt)
            {
                Configuration q = rng->next();
                if (constraint_set.project(q) && is_config_valid(q))
                {
                    resolved = true;
                    break;
                }
            }

            if (!resolved)
            {
                return false;
            }
        }

        return true;
    };

    constexpr std::size_t kNumProblems = 200;
    while (problems.size() < kNumProblems)
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
            0.5F * 0.5F)
            continue;

        auto [start_valid, start_config] = find_valid_config_for_point(random_start_position);
        if (!start_valid)
        {
            continue;
        }

        auto [goal_valid, goal_config] = find_valid_config_for_point(random_goal_position);
        if (!goal_valid)
        {
            continue;
        }

        if (is_straight_line_trivial(random_start_position, random_goal_position))
        {
            continue;
        }

        Problem p;
        const auto start_full = start_config.to_array();
        const auto goal_full = goal_config.to_array();
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            p.problem_start[i] = start_full[i];
            p.problem_end[i] = goal_full[i];
        }
        p.start_eef_pos = random_start_position;
        p.goal_eef_pos = random_goal_position;

        std::cout << "Adding start/goal to problem set (" << problems.size() + 1 << " / " << kNumProblems << ")" << std::endl;
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
