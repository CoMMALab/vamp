// Port of iiwa_maze_problem_generator.cc to vamp::robots::FR3Marker (vamp/robots/fr3_marker.hh).
//
// The task-space parameterization is structurally the same 8-dim StateArray (x, y, z, qx, qy,
// qz, qw, <free param>) as iiwa_marker.hh's ParameterizedSpace, but the free parameter and the
// branch selector triple mean something different here -- see fr3_parameterization.hh:
//   - index 7 ("psi" below, kept for schema/field-name compatibility with the iiwa problem
//     files and scripts/compare_iiwa_maze_results.py) is FR3's joint 7 angle directly, not an
//     abstract shoulder/elbow/wrist redundancy parameter.
//   - `smm` is (case6_sel, case1_sel, q_actual_0): case6_sel/case1_sel each select one of two
//     whole alternate IK formulas and are read via a `> 0.5` threshold, so they belong in {0, 1}
//     (not iiwa's {-1, +1} GC2/GC4/GC6 sign flips); q_actual_0 is a continuous tie-break seed
//     used only in the (measure-zero) wrist-point-singular case, substituting directly for the
//     otherwise-undetermined joint 1 angle -- sampled here from FR3Marker's own joint 1 range
//     (lower_bound[0]/upper_bound[0]) so a singular draw is still in-limits.
// Consequently there are only 4 meaningful discrete branches here (case6_sel x case1_sel), not
// iiwa's 8, and each is paired with a freshly-sampled q_actual_0 per resolve attempt (folded
// into the same per-candidate loop as psi/q7, alongside it below) rather than being fixed for
// an entire branch the way iiwa's GC2/GC4/GC6 triple is.

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
#include <vamp/robots/fr3_marker.hh>

using json = nlohmann::json;

using Robot = vamp::robots::FR3Marker;
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
    // psi (really FR3 joint 7, "q7" -- see file header; field kept named "psi" for schema
    // compatibility with the iiwa problem files/scripts) that resolved to
    // problem_start/problem_end.
    float start_psi;
    float goal_psi;
    // (case6_sel, case1_sel, q_actual_0) branch that both problem_start and problem_end were
    // resolved on -- see file header and find_valid_start_goal_on_shared_branch below.
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
            float x = obj.at("x").get<float>() + 0.285*2;  // push it slightly forward, matches fr3_maze_problem_generator
            float y = obj.at("y").get<float>() + 0.0F; // push it slightly left, matches fr3_maze_problem_generator
            float z = obj.at("z").get<float>() + 0.0F; // push it slightly up, matches fr3_maze_problem_generator
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>(); // make it bigger

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
    const char *output_path = "resources/fr3_marker/maze_problems_checked_ik.json";
    std::ofstream file(output_path);

    EnvironmentInput environment;

    auto bounds = load_cuboids_from_json(environment, "resources/environments/maze_cuboids.json");
    std::cout << "Loaded environment bounds: x[" << bounds.first[0] << ", " << bounds.second[0] << "], y["
              << bounds.first[1] << ", " << bounds.second[1] << "]" << std::endl;

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Fixed height / orientation for the end effector, matching iiwa_maze_problem_generator.cc:
    // tool pointing straight down (qx=1,qy=0,qz=0,qw=0 -- i.e. (0, -1, 0, 0) in this
    // parameterization). Kept numerically identical on the assumption the maze/mount frame is
    // shared between the iiwa and fr3 rigs; there's no fr3-specific calibration to derive this
    // from otherwise.
    constexpr float kEefZ = 0.150519F;

    // FR3's joint 1 range (lower_bound[0]/upper_bound[0]) -- used to sample q_actual_0 (see
    // file header) so a singular-case draw is still in-limits.
    constexpr float kQActual0Min = Robot::lower_bound[0];
    constexpr float kQActual0Max = Robot::upper_bound[0];

    auto make_pose_array = [&](const std::array<float, 3> &eef_pos, float q7)
    {
        ParameterizedSpace::StateArray pose_array;
        pose_array[0] = eef_pos[0];
        pose_array[1] = eef_pos[1];
        pose_array[2] = kEefZ;
        pose_array[3] = 0.0F;
        pose_array[4] = -1.0F;
        pose_array[5] = 0.0F;
        pose_array[6] = 0.0F;
        pose_array[7] = q7;
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

    // q7 (index 7) doesn't change the eef pose -- only which arm configuration reaches it --
    // so instead of hardcoding one value for every problem, try a handful of candidates per
    // endpoint and keep the first that resolves within joint limits and collision-free. Each
    // candidate also draws a fresh q_actual_0 (see file header): it only matters in the
    // measure-zero wrist-point-singular case, so redrawing it alongside q7 rather than fixing
    // it per-branch costs nothing and still keeps a valid in-limits fallback available.
    constexpr int kNumPsiCandidates = 256;

    // Tries q7/q_actual_0 (see above) for a single endpoint on whichever (case6_sel, case1_sel)
    // branch is currently set via ParameterizedSpace::set_smm -- branch selection itself lives
    // one level up, in find_valid_start_goal_on_shared_branch, since start and goal must resolve
    // on the *same* branch (a physically consistent arm posture can't jump branches mid-problem).
    auto find_valid_psi_pose = [&](const std::array<float, 3> &eef_pos,
                                    const std::array<float, 2> &branch)
        -> std::tuple<bool, ParameterizedSpace::StateArray, Robot::ConfigurationArray, float>
    {
        for (int k = 0; k < kNumPsiCandidates; ++k)
        {
            // const float q7 = static_cast<float>(rand()) / RAND_MAX * 2.0F * static_cast<float>(M_PI);
            // this needs to be from q7's joint limits, not [0, 2pi] -- see fr3_marker.hh's lower_bound[6]/upper_bound[6]
            const float q7 = Robot::lower_bound[6] + static_cast<float>(rand()) / RAND_MAX * (Robot::upper_bound[6] - Robot::lower_bound[6]);

            const float q_actual_0 =
                kQActual0Min + static_cast<float>(rand()) / RAND_MAX * (kQActual0Max - kQActual0Min);
            ParameterizedSpace::set_smm({branch[0], branch[1], q_actual_0});

            const auto pose_array = make_pose_array(eef_pos, q7);
            auto [ik_valid, ambient_array] = resolve_ambient_config(pose_array);
            if (ik_valid)
            {
                return {true, pose_array, ambient_array, q_actual_0};
            }
        }

        return {false, {}, {}, 0.0F};
    };

    // The (case6_sel, case1_sel) branch -- each in {0, 1} (see file header) -- picks which of
    // the 4 alternate IK formulas resolves a given eef pose. ParameterizedSpace defaults to
    // (1, 1, 1) and every problem used to be resolved on that one branch (equivalently here,
    // case6_sel=1/case1_sel=1). Sample a random branch instead, and give each sampled branch
    // kNumBranchAttempts tries at resolving *both* the start and goal eef poses on it -- start
    // and goal must share a branch, since resolve_block's `smm` selects one arm posture per pose
    // and a problem's start/goal configurations both need to belong to the same posture family
    // for the branch to mean anything as "the" arm configuration.
    constexpr int kNumBranchAttempts = 8;
    auto random_branch = []() -> std::array<float, 2>
    {
        return {
            (rand() % 2) ? 1.0F : 0.0F,
            (rand() % 2) ? 1.0F : 0.0F};
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

            auto [start_valid, start_pose, start_ambient, start_q_actual_0] =
                find_valid_psi_pose(start_eef_pos, branch);
            if (!start_valid)
            {
                continue;
            }

            // find_valid_psi_pose redraws q_actual_0 per-candidate independently for the goal
            // pose, so both endpoints end up sharing only (case6_sel, case1_sel), exactly like
            // the start/goal-shared-branch requirement above -- q_actual_0 itself is not part of
            // what "shared branch" needs to mean, since it's a singular-case tie-break, not a
            // posture selector. The goal's q_actual_0 is what's recorded below (arbitrary choice
            // between the two -- see comment above for why it doesn't matter which).
            auto [goal_valid, goal_pose, goal_ambient, goal_q_actual_0] =
                find_valid_psi_pose(goal_eef_pos, branch);
            if (!goal_valid)
            {
                continue;
            }

            static_cast<void>(start_q_actual_0);
            return {true, start_pose, start_ambient, goal_pose, goal_ambient, {branch[0], branch[1], goal_q_actual_0}};
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

        // Cheap eef-collision prefilter (q7-independent, so any q7 value works here).
        if (!is_eef_pose_valid(make_pose_array(random_start_position, 0.0F)) ||
            !is_eef_pose_valid(make_pose_array(random_goal_position, 0.0F)))
        {
            continue;
        }

        // Search for a shared (case6_sel, case1_sel) branch on which a q7 resolves both
        // endpoints to a valid, in-limits, collision-free ambient configuration -- this also
        // gives us problem_start/problem_end, so it happens regardless of --check_ik (unlike
        // the plain is_eef_pose_valid check above).
        auto [ik_valid, start_pose_array, start_ambient, goal_pose_array, goal_ambient, branch] =
            find_valid_start_goal_on_shared_branch(random_start_position, random_goal_position);
        if (!ik_valid)
        {
            continue;
        }

        // find_valid_start_goal_on_shared_branch leaves `smm` set to whatever the goal pose's
        // last candidate used, which is exactly what is_straight_line_trivial's --check_ik path
        // needs (start and goal, and therefore every point interpolated between them, all
        // resolve on the same (case6_sel, case1_sel) branch; q_actual_0 only matters in the
        // singular case, which the interpolation sweep essentially never hits).
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
