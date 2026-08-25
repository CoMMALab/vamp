#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/robots/iiwa_marker.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::IiwaMarker;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Same maze environment iiwa_maze_solver.cc's main() loads, duplicated here so this standalone
// runner doesn't need to link against that file just to get a collision-checkable scene for
// do_cc. Falls back to an empty (obstacle-free) environment if the JSON can't be found, since
// this tool is about branch/psi resolvability more than any specific scene.
static auto build_environment() -> EnvironmentVector
{
    EnvironmentInput environment;

    std::ifstream ifs("resources/environments/real_maze.json");
    if (ifs.is_open())
    {
        nlohmann::json j;
        try
        {
            ifs >> j;
            if (j.is_array())
            {
                for (const auto &obj : j)
                {
                    if (!obj.is_object() || !obj.contains("x") || !obj.contains("y") ||
                        !obj.contains("z") || !obj.contains("dx") || !obj.contains("dy") ||
                        !obj.contains("dz"))
                    {
                        continue;
                    }

                    float x = obj.at("x").get<float>() + 0.05F;
                    float y = obj.at("y").get<float>();
                    float z = obj.at("z").get<float>();
                    float dx = obj.at("dx").get<float>();
                    float dy = obj.at("dy").get<float>();
                    float dz = obj.at("dz").get<float>();

                    float roll = obj.contains("roll") ? obj.at("roll").get<float>() : 0.0F;
                    float pitch = obj.contains("pitch") ? obj.at("pitch").get<float>() : 0.0F;
                    float yaw = obj.contains("yaw") ? obj.at("yaw").get<float>() : 0.0F;

                    std::array<float, 3> posf = {x, y, z};
                    std::array<float, 3> rotf = {roll, pitch, yaw};
                    std::array<float, 3> sizef = {dx / 2, dy / 2, dz / 2};
                    environment.cuboids.emplace_back(
                        vamp::collision::factory::cuboid::array(posf, rotf, sizef));
                }
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to parse maze JSON, continuing with empty environment: " << e.what()
                       << std::endl;
        }
    }
    else
    {
        std::cerr << "Maze JSON not found, continuing with empty environment." << std::endl;
    }

    environment.sort();
    return EnvironmentVector(environment);
}

// The 8 (elbow_sel, shoulder_sel, wrist_sel) self-motion-manifold branch combos IiwaMarker's
// single arm can be resolved on -- direct counterpart of rby1_gcp_branch_selector.cc's
// gcp_branch_combos, just for the one `smm` triple instead of left_gcp/right_gcp. Lane `i` of a
// resolve_block_mask call always corresponds to combos[i] below (fixed enumeration).
static constexpr std::array<std::array<float, 3>, 8> smm_branch_combos = {{
    {0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 1.0F},
    {0.0F, 1.0F, 0.0F},
    {0.0F, 1.0F, 1.0F},
    {1.0F, 0.0F, 0.0F},
    {1.0F, 0.0F, 1.0F},
    {1.0F, 1.0F, 0.0F},
    {1.0F, 1.0F, 1.0F},
}};

// A branch combo resolved at some sampled psi. Collision-checking is deliberately left out here
// (unlike the do_cc path in sweep_smm_branches_over_psi below, this just tracks IK resolvability
// (resolve_block_mask's within-joint-limits contract); no CoM/support-polygon check is needed --
// unlike rby1_bimanual, IiwaMarker is a fixed-base arm.
struct SmmBranchSolution
{
    std::array<float, 3> smm;
    float psi;
};

// How many iterations sweep_smm_branches_over_psi actually ran (<= max_iterations, fewer if it
// stopped early once every branch resolved) and how long each one's resolve_block_mask call
// took, for reporting -- not used to change the sweep's own behavior.
struct SmmSweepStats
{
    std::size_t iterations_run = 0;
    std::size_t max_iterations = 0;
    std::vector<std::chrono::nanoseconds> iteration_durations;
};

// Repeatedly samples a random psi and, at each sample, resolves all 8 combos in parallel in one
// resolve_block_mask call (rake == 8 lanes maps 1:1 onto the 8 branch combos, mirroring
// rby1_gcp_branch_selector.cc's sweep_gcp_branches_over_psi). `pose` carries (x, y, z, qx, qy,
// qz, qw); its own psi slot (index 7) is overwritten with the sampled value each iteration. Runs
// up to `max_iterations` iterations, discarding a lane's later results once it has already
// resolved (so a subsequent, possibly-different psi sample can never clobber an earlier
// success). Stops early once every branch has resolved at least once. Collision-checking
// (fkcc against `environment`) is gated behind `do_cc`; fkcc has no per-lane variant, so a
// surviving candidate lane's resolved ambient config is re-broadcast across the block and
// checked with the ordinary (block-aggregated) call -- one extra fkcc call per surviving
// candidate lane, not touching the rest of the batch's IK masking.
static auto sweep_smm_branches_over_psi(
    const ParameterizedSpace::State &pose,
    std::mt19937 &rng,
    bool do_cc,
    const EnvironmentVector &environment,
    std::size_t max_iterations = 50) -> std::pair<std::array<std::optional<SmmBranchSolution>, 8>, SmmSweepStats>
{
    static_assert(
        rake >= 8,
        "sweep_smm_branches_over_psi enumerates all 8 branch combos in a single "
        "resolve_block_mask call; needs rake >= 8");

    static const float two_pi = 2.0F * std::acos(-1.0F);
    std::uniform_real_distribution<float> psi_dist(0.0F, two_pi);

    auto base_pose_array = pose.to_array();
    std::array<std::optional<SmmBranchSolution>, 8> solved{};
    SmmSweepStats stats;
    stats.max_iterations = max_iterations;

    // `lanes` (and therefore the branch assignment) is the same on every iteration -- only the
    // sampled psi varies -- so set it once instead of re-setting it inside the loop.
    std::array<std::array<float, 3>, rake> lanes{};
    for (std::size_t lane = 0; lane < rake; ++lane)
    {
        lanes[lane] = smm_branch_combos[(lane < 8) ? lane : 0];
    }
    ParameterizedSpace::set_smm_lanes(lanes);

    for (std::size_t iteration = 0; iteration < max_iterations; ++iteration)
    {
        const bool all_solved =
            std::all_of(solved.begin(), solved.end(), [](const auto &s) { return s.has_value(); });
        if (all_solved)
        {
            break;
        }

        const auto iteration_start = std::chrono::steady_clock::now();

        const float psi = psi_dist(rng);
        auto iter_pose_array = base_pose_array;
        iter_pose_array[7] = psi;
        ParameterizedSpace::State iter_state(iter_pose_array.data());

        ParameterizedSpace::StateBlock<rake> block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            block[i] = iter_state.broadcast(i);
        }

        auto [valid_mask, ambient_block] = ParameterizedSpace::resolve_block_mask<rake>(block);

        for (std::size_t lane = 0; lane < 8; ++lane)
        {
            // Already solved on an earlier iteration -- this iteration's value for this lane is
            // computed (it's part of the same parallel call) but must not be used.
            if (solved[lane])
            {
                continue;
            }

            if (valid_mask[{0, lane}] == 0.0F)
            {
                continue;
            }

            if (do_cc)
            {
                Robot::ConfigurationArray lane_config{};
                for (std::size_t j = 0; j < Robot::dimension; ++j)
                {
                    lane_config[j] = ambient_block[{j, lane}];
                }
                Robot::Configuration lane_configuration(lane_config);

                Robot::ConfigurationBlock<rake> lane_block;
                for (std::size_t j = 0; j < Robot::dimension; ++j)
                {
                    lane_block[j] = lane_configuration.broadcast(j);
                }

                if (not Robot::template fkcc<rake>(environment, lane_block))
                {
                    continue;
                }
            }

            solved[lane] = SmmBranchSolution{smm_branch_combos[lane], psi};
        }

        stats.iteration_durations.push_back(std::chrono::steady_clock::now() - iteration_start);
        ++stats.iterations_run;
    }

    return {solved, stats};
}

// Sweeps all 8 branch combos at a single (broadcast) task-space state (psi included, taken as
// given rather than swept -- pair with sweep_smm_branches_over_psi above when psi itself should
// be searched too). One resolve_block_mask call, since IiwaMarker has only the one arm/one smm
// triple (unlike rby1_gcp_branch_selector.cc's select_gcp_branch, which needs two calls -- one
// per arm). Returns the first combo (in the fixed enumeration order above) whose lane resolves
// (ambient q within joint limits); nullopt if none did. Does not collision-check the resolved
// configuration -- IK validity only, same as resolve_block's own contract.
auto select_smm_branch(const ParameterizedSpace::State &state) -> std::optional<std::array<float, 3>>
{
    static_assert(
        rake >= 8,
        "select_smm_branch enumerates all 8 branch combos in a single resolve_block_mask call; "
        "needs rake >= 8");

    std::array<std::array<float, 3>, rake> lanes{};
    for (std::size_t lane = 0; lane < rake; ++lane)
    {
        lanes[lane] = smm_branch_combos[(lane < 8) ? lane : 0];
    }
    ParameterizedSpace::set_smm_lanes(lanes);

    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [valid_mask, ambient_block] = ParameterizedSpace::resolve_block_mask<rake>(block);
    std::cout << "smm sweep: valid_mask = " << valid_mask << std::endl;
    static_cast<void>(ambient_block);

    for (std::size_t lane = 0; lane < 8; ++lane)
    {
        if (valid_mask[{0, lane}] != 0.0F)
        {
            return smm_branch_combos[lane];
        }
    }

    return std::nullopt;
}

// Standalone branch-sweep runner: resolves one of the maze entry poses iiwa_maze_solver.cc uses
// (tool pointing straight down, on the z=0 plane), then reports which of the 8 possible
// (elbow_sel, shoulder_sel, wrist_sel) selectors select_smm_branch finds at psi=0, followed by a
// per-branch psi sweep (up to 50 iterations, all 8 combos resolved in parallel each iteration)
// run once without collision-checking and once with, so the two can be compared directly (same
// rng, drawn back to back).
auto main() -> int
{
    std::cout << std::boolalpha;

    ParameterizedSpace::StateArray pose_array = {
        {0.5383931994438171F, -0.2849438190460205F, 0.2260778248310089F, 0.0F, -1.0F, 0.0F, 0.0F, 0.0F}};
    ParameterizedSpace::State pose(pose_array.data());

    std::cout << "\n--- smm sweep on the smoke-test pose (psi = 0) ---" << std::endl;
    const auto swept = select_smm_branch(pose);

    if (swept)
    {
        std::cout << "smm: (" << (*swept)[0] << ", " << (*swept)[1] << ", " << (*swept)[2] << ")"
                   << std::endl;
    }
    else
    {
        std::cout << "smm: no branch resolved" << std::endl;
    }

    std::mt19937 rng(std::random_device{}());
    const auto environment = build_environment();

    auto report_sweep = [](
        const std::string &label,
        const std::array<std::optional<SmmBranchSolution>, 8> &solved,
        const SmmSweepStats &stats)
    {
        std::cout << "\n--- Per-branch psi sweep: " << label << " (up to 50 iterations) ---" << std::endl;

        std::chrono::nanoseconds total_duration{0};
        for (const auto &duration : stats.iteration_durations)
        {
            total_duration += duration;
        }

        for (std::size_t lane = 0; lane < solved.size(); ++lane)
        {
            const auto &combo = smm_branch_combos[lane];
            std::cout << "branch (" << combo[0] << ", " << combo[1] << ", " << combo[2] << "): ";
            if (solved[lane])
            {
                std::cout << "resolved at psi = " << solved[lane]->psi << std::endl;
            }
            else
            {
                std::cout << "unresolved" << std::endl;
            }
        }

        std::cout << "total iterations: " << stats.iterations_run << " / " << stats.max_iterations << std::endl;
        std::cout << "total time: " << total_duration.count() << " ns" << std::endl;
    };

    const auto [solved_no_cc, stats_no_cc] = sweep_smm_branches_over_psi(pose, rng, /*do_cc=*/false, environment);
    report_sweep("without collision-checking", solved_no_cc, stats_no_cc);

    const auto [solved_cc, stats_cc] = sweep_smm_branches_over_psi(pose, rng, /*do_cc=*/true, environment);
    report_sweep("with collision-checking", solved_cc, stats_cc);

    return 0;
}
