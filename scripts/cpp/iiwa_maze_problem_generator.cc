#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/iiwa_marker.hh>

using json = nlohmann::json;

using Robot = vamp::robots::IiwaMarker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using RNG = typename vamp::rng::RNG<Robot>;

// Define your problem structure. problem_start/problem_end are the ambient (joint-space)
// configuration resolved by ParameterizedSpace::resolve_block for the eef pose, not the
// task-space pose array itself.
struct Problem
{
    std::array<float, Robot::dimension> problem_start;
    std::array<float, Robot::dimension> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
    // psi (redundancy parameter) that resolved to problem_start/problem_end -- see
    // find_valid_psi_pose below for why this is chosen per-problem instead of a fixed constant.
    float start_psi;
    float goal_psi;
    // Self-motion-manifold (GC2/GC4/GC6, i.e. elbow_sel/shoulder_sel/wrist_sel) branch, each
    // axis in {-1, +1}, that both problem_start and problem_end were resolved on -- see
    // find_valid_start_goal_on_shared_branch for why this is sampled per-problem (shared between
    // start and goal) instead of always using the default (1, 1, 1) branch.
    std::array<float, 3> smm;
};

// This helper function allows nlohmann::json to "just work" with your struct
void to_json(json &j, const Problem &p)
{
    j = json{
        {"problem_start", p.problem_start},
        {"problem_end", p.problem_end},
        {"start_eef_pos", p.start_eef_pos},
        {"goal_eef_pos", p.goal_eef_pos},
        {"start_psi", p.start_psi},
        {"goal_psi", p.goal_psi},
        {"smm", p.smm}};
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
    // pointing straight down (qx=1,qy=0,qz=0,qw=0 -- i.e. (0, -1, 0, 0) in this parameterization).
    constexpr float kEefZ = 0.22607783F;

    auto make_pose_array = [&](const std::array<float, 3> &eef_pos, float psi)
    {
        ParameterizedSpace::StateArray pose_array;
        pose_array[0] = eef_pos[0];
        pose_array[1] = eef_pos[1];
        pose_array[2] = kEefZ;
        pose_array[3] = 0.0F;
        pose_array[4] = -1.0F;
        pose_array[5] = 0.0F;
        pose_array[6] = 0.0F;
        pose_array[7] = psi;
        return pose_array;
    };

    auto to_state_block = [&](const ParameterizedSpace::StateArray &pose_array)
    {
        ParameterizedSpace::State pose(pose_array.data());
        ParameterizedSpace::StateBlock<rake> pose_block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            pose_block[i] = pose.broadcast(i);
        }
        return pose_block;
    };

    // Cheap early-reject check: broadcast the (single) candidate pose across all lanes and ask
    // whether the end-effector spheres implied by that pose are collision free, without solving
    // IK for the rest of the arm.
    auto is_eef_pose_valid = [&](const ParameterizedSpace::StateArray &pose_array)
    { return ParameterizedSpace::eefs_collision_free<rake>(env_v, to_state_block(pose_array)); };

    // Solve the pose to an ambient (joint-space) configuration via ParameterizedSpace::resolve_block
    // and collision-check it. This is what actually gets written to problem_start/problem_end --
    // the task-space pose array is only ever an intermediate used to reach it.
    auto resolve_ambient_config = [&](const ParameterizedSpace::StateArray &pose_array)
        -> std::pair<bool, Robot::ConfigurationArray>
    {
        auto [param_valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(to_state_block(pose_array));

        if (not param_valid or not Robot::fkcc<rake>(env_v, ambient_block))
        {
            return {false, {}};
        }

        Robot::ConfigurationArray ambient_array;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            ambient_array[i] = ambient_block[{i, 0}];
        }

        return {true, ambient_array};
    };

    // Stricter check used with --check_ik: same as above, bool-only, for the straight-line
    // triviality sweep.
    auto is_eef_pose_ik_valid = [&](const ParameterizedSpace::StateArray &pose_array)
    { return resolve_ambient_config(pose_array).first; };

    // psi (index 7) doesn't change the eef pose -- only which arm configuration reaches it --
    // so instead of hardcoding one value for every problem (which pushed the wrist bend, joint
    // 6, near its singularity for some eef positions and not others, and thus made enforcing
    // the joint-7 limit reject a large, position-dependent fraction of problems), try a handful
    // of candidates per endpoint and keep the first that resolves within joint limits and
    // collision-free.
    constexpr int kNumPsiCandidates = 16;

    // Tries psi (see above) for a single endpoint on whichever GC branch is currently set via
    // ParameterizedSpace::set_smm -- branch selection itself lives one level up, in
    // find_valid_start_goal_on_shared_branch, since start and goal must resolve on the *same*
    // branch (a physically consistent arm posture can't jump branches mid-problem). Candidates
    // are drawn uniformly at random from [0, 2*pi) rather than swept, per current experiment
    // (comparing against iiwa_maze_solver_benchmark.cc, which is being changed to sample psi the
    // same way, instead of both sides disagreeing about sweep-vs-random).
    auto find_valid_psi_pose = [&](const std::array<float, 3> &eef_pos)
        -> std::pair<bool, std::pair<ParameterizedSpace::StateArray, Robot::ConfigurationArray>>
    {
        for (int k = 0; k < kNumPsiCandidates; ++k)
        {
            const float psi = static_cast<float>(rand()) / RAND_MAX * 2.0F * static_cast<float>(M_PI);
            const auto pose_array = make_pose_array(eef_pos, psi);
            auto [ik_valid, ambient_array] = resolve_ambient_config(pose_array);
            if (ik_valid)
            {
                return {true, {pose_array, ambient_array}};
            }
        }

        return {false, {}};
    };

    // The self-motion-manifold (GC2/GC4/GC6, i.e. elbow_sel/shoulder_sel/wrist_sel) branch --
    // each axis in {-1, +1} -- picks which of the 8 arm postures reaches a given eef pose (the
    // param_ik_code formulas use smm[i] as a sign flip, e.g. y[1] = smm[0] * acos(v[22]), which
    // only makes sense for +-1, not {0, 1}). ParameterizedSpace defaults to (1, 1, 1) and every
    // problem used to be resolved on that one branch. Sample a random branch instead, and give
    // each sampled branch kNumBranchAttempts tries at resolving *both* the start and goal eef
    // poses on it -- start and goal must share a branch, since resolve_block's `smm` selects one
    // arm posture per pose and a problem's start/goal configurations both need to belong to the
    // same posture family for the branch to mean anything as "the" arm configuration.
    constexpr int kNumBranchAttempts = 8;
    auto random_branch = []() -> std::array<float, 3>
    {
        return {
            (rand() % 2) ? 1.0F : -1.0F,
            (rand() % 2) ? 1.0F : -1.0F,
            (rand() % 2) ? 1.0F : -1.0F};
    };

    auto find_valid_start_goal_on_shared_branch = [&](const std::array<float, 3> &start_eef_pos,
                                                        const std::array<float, 3> &goal_eef_pos)
        -> std::tuple<
            bool,
            ParameterizedSpace::StateArray,
            Robot::ConfigurationArray,
            ParameterizedSpace::StateArray,
            Robot::ConfigurationArray,
            std::array<float, 3>>
    {
        for (int b = 0; b < kNumBranchAttempts; ++b)
        {
            const auto branch = random_branch();
            ParameterizedSpace::set_smm(branch);

            auto [start_valid, start] = find_valid_psi_pose(start_eef_pos);
            if (!start_valid)
            {
                continue;
            }

            auto [goal_valid, goal] = find_valid_psi_pose(goal_eef_pos);
            if (!goal_valid)
            {
                continue;
            }

            return {true, start.first, start.second, goal.first, goal.second, branch};
        }

        return {false, {}, {}, {}, {}, {}};
    };

    // Reject problems that a straight line in eef-space already solves: sample points along
    // the segment from start to goal and check each with whichever validity check the endpoints
    // were accepted with. If every sample is collision free, the problem is trivial.
    constexpr int kNumStraightLineSamples = 200;
    auto is_straight_line_trivial = [&](const ParameterizedSpace::StateArray &start_pose_array,
                                         const ParameterizedSpace::StateArray &goal_pose_array)
    {
        for (int i = 0; i < kNumStraightLineSamples; ++i)
        {
            const float t = static_cast<float>(i) / static_cast<float>(kNumStraightLineSamples - 1);

            ParameterizedSpace::StateArray interp_pose_array;
            for (std::size_t d = 0; d < ParameterizedSpace::dimension; ++d)
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
            0.5f * 0.5f)
            continue;

        // Cheap eef-collision prefilter (psi-independent, so any psi value works here).
        if (!is_eef_pose_valid(make_pose_array(random_start_position, 0.0F)) ||
            !is_eef_pose_valid(make_pose_array(random_goal_position, 0.0F)))
        {
            continue;
        }

        // Search for a shared GC branch on which a psi resolves both endpoints to a valid,
        // in-limits, collision-free ambient configuration -- this also gives us
        // problem_start/problem_end, so it happens regardless of --check_ik (unlike the plain
        // is_eef_pose_valid check above).
        auto [ik_valid, start_pose_array, start_ambient, goal_pose_array, goal_ambient, branch] =
            find_valid_start_goal_on_shared_branch(random_start_position, random_goal_position);
        if (!ik_valid)
        {
            continue;
        }

        // find_valid_start_goal_on_shared_branch leaves `smm` set to `branch`, which is exactly
        // what is_straight_line_trivial's --check_ik path needs (start and goal, and therefore
        // every point interpolated between them, all resolve on the same branch).
        if (is_straight_line_trivial(start_pose_array, goal_pose_array))
        {
            continue;
        }

        Problem p;
        p.problem_start = start_ambient;
        p.problem_end = goal_ambient;
        p.start_eef_pos = random_start_position;
        p.goal_eef_pos = random_goal_position;
        p.start_psi = start_pose_array[7];
        p.goal_psi = goal_pose_array[7];
        p.smm = branch;
        std::cout << "Adding start/goal to problem set (" << problems.size() + 1 << " / 200)" << std::endl;
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
