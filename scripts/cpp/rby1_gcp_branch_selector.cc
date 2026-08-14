#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/robots/rby1.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::RBY1;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

// Same table + attached-box environment rby1_task_space_planner.cc's main() builds, duplicated
// here so this standalone runner doesn't need to link against that file just to get a
// collision-checkable scene for do_cc.
static auto build_environment() -> EnvironmentVector
{
    EnvironmentInput environment;

    auto table_cuboid = vamp::collision::factory::cuboid::array(
        std::array<float, 3>{0.32F, -0.79F, 0.36F},
        std::array<float, 3>{0.0F, 0.0F, 0.0F},
        std::array<float, 3>{0.60F, 0.30F, 0.36F});
    environment.cuboids.emplace_back(table_cuboid);

    constexpr std::array<float, 3> box_size = {0.386F, 0.264F, 0.10F};
    constexpr std::array<float, 3> box_offset = {-0.193F, 0.0F, -0.13F};
    const float box_sphere_radius = box_size[2] / 2.0F;

    auto grid_centers = [](float extent, float radius) -> std::vector<float>
    {
        const float half = extent / 2.0F;
        if (half <= radius)
        {
            return {0.0F};
        }

        const auto n = static_cast<int>(std::ceil(extent / (2.0F * radius))) + 1;
        std::vector<float> centers(n);
        for (int i = 0; i < n; ++i)
        {
            centers[i] =
                -half + radius + (2.0F * (half - radius)) * static_cast<float>(i) / static_cast<float>(n - 1);
        }

        return centers;
    };

    Eigen::Isometry3f box_attachment_tf = Eigen::Isometry3f::Identity();
    box_attachment_tf.translation() = Eigen::Vector3f(box_offset[0], box_offset[1], box_offset[2]);

    vamp::collision::Attachment<float> box_attachment(box_attachment_tf);
    box_attachment.end_effector = 1;  // right hand
    box_attachment.excluded_end_effectors = {0};

    for (const auto cx : grid_centers(box_size[0], box_sphere_radius))
    {
        for (const auto cy : grid_centers(box_size[1], box_sphere_radius))
        {
            box_attachment.spheres.emplace_back(cx, cy, 0.0F, box_sphere_radius);
        }
    }

    environment.attachments.emplace_back(box_attachment);

    return EnvironmentVector(environment);
}

// The 8 (elbow_sel, shoulder_sel, wrist_sel) branch combos a single arm's redundant self-motion
// manifold can be resolved on; lane `i` of a resolve_block_mask call always corresponds to
// combos[i] below (fixed enumeration), which is what lets results be accumulated per-lane across
// separate calls -- lane identity IS branch identity, not just a slot.
static constexpr std::array<std::array<float, 3>, 8> gcp_branch_combos = {{
    {0.0F, 0.0F, 0.0F},
    {0.0F, 0.0F, 1.0F},
    {0.0F, 1.0F, 0.0F},
    {0.0F, 1.0F, 1.0F},
    {1.0F, 0.0F, 0.0F},
    {1.0F, 0.0F, 1.0F},
    {1.0F, 1.0F, 0.0F},
    {1.0F, 1.0F, 1.0F},
}};

// Static-stability support polygon, in the mobile base's local xy frame (matches
// ParameterizedSpace::compute_com's output frame). Vertices trace the ground-contact points in
// order (right wheel -> left wheel -> left caster -> right caster). Copied from
// ParameterizedLocalPlanner (private there) rather than shared, since this file needs a
// per-lane mask instead of that struct's aggregated `.all()`.
static constexpr std::array<std::array<float, 2>, 4> support_polygon_xy = {{
    {0.228000F, -0.265000F},   // right wheel
    {0.228000F, 0.265000F},    // left wheel
    {-0.248686F, 0.066310F},   // left caster
    {-0.248686F, -0.066310F},  // right caster
}};

// Per-lane counterpart of ParameterizedLocalPlanner's point_in_support_polygon: identical
// ray-casting test against support_polygon_xy, but returns the per-lane `inside` mask itself
// instead of collapsing it with `.all()`. That collapse is right for
// ParameterizedLocalPlanner's own use (many interpolated samples under one already-locked-in
// GCP branch, where every lane must be stable), but wrong here: this file's lanes are different
// GCP branch candidates for the same broadcast state, so one bad branch must not fail every
// lane's CoM check.
static inline auto point_in_support_polygon_mask(vamp::FloatVector<rake> x, vamp::FloatVector<rake> y) noexcept
    -> vamp::FloatVector<rake>
{
    using V = vamp::FloatVector<rake>;

    V inside = V::zero_vector();
    for (std::size_t i = 0, j = support_polygon_xy.size() - 1; i < support_polygon_xy.size(); j = i++)
    {
        const auto &pi = support_polygon_xy[i];
        const auto &pj = support_polygon_xy[j];

        const V pi_y(pi[1]);
        const V pj_y(pj[1]);
        const V crosses_y = (pi_y > y) ^ (pj_y > y);
        const V slope((pj[0] - pi[0]) / (pj[1] - pi[1]));
        const V x_at_y = V(pi[0]) + slope * (y - pi_y);

        inside = inside ^ (crosses_y & (x < x_at_y));
    }

    return inside;
}

// A branch combo (shared by both arms) resolved at some sampled (psi_left, psi_right).
// Collision-checking is deliberately left out here (unlike ParameterizedLocalPlanner::
// resolve_and_check_impl, which this is a "local", IK + CoM-only counterpart of -- that check
// needs a real Environment and is left for a later pass); this tracks IK resolvability
// (resolve_block_mask's zero-reach-violation, within-joint-limits contract) AND CoM stability
// (point_in_support_polygon_mask above).
struct GcpBranchSolution
{
    std::array<float, 3> gcp;
    float psi_left;
    float psi_right;
};

// How many iterations sweep_gcp_branches_over_psi actually ran (<= max_iterations, fewer if it
// stopped early once every branch resolved) and how long each one's resolve_block_mask call
// took, for reporting -- not used to change the sweep's own behavior.
struct GcpSweepStats
{
    std::size_t iterations_run = 0;
    std::size_t max_iterations = 0;
    std::vector<std::chrono::nanoseconds> iteration_durations;
};

// Local, per-lane counterpart of ParameterizedLocalPlanner::resolve_and_check_impl -- "local" in
// that it lives here rather than in that shared header. IK resolvability and CoM/support-polygon
// stability are always checked; collision-checking (fkcc/fkcc_attach against `environment`) is
// gated behind `do_cc` since, unlike the mask-based IK/CoM checks above, fkcc has no per-lane
// variant -- see the do_cc branch below for how that's worked around. Both arms are locked to
// the SAME GCP branch combo (left_gcp == right_gcp), so the 8
// candidate combos map 1:1 onto the `rake` == 8 lanes of a single resolve_block_mask call --
// unlike select_gcp_branch above (which sweeps each arm's 8 combos independently, needing two
// calls), this sweeps both arms' shared branch in exactly one.
//
// Repeatedly samples a random (psi_left, psi_right) pair and, at each sample, resolves all 8
// combos in parallel in that one resolve_block_mask call. Runs up to `max_iterations`
// iterations. A lane/branch that has already resolved on an earlier iteration is left alone on
// every later one: it is still computed (resolve_block_mask has no way to skip a lane), but the
// result is discarded rather than overwriting the stored solution, so later, possibly-different
// psi samples can never clobber an earlier success. Stops early once every branch has resolved
// at least once.
static auto sweep_gcp_branches_over_psi(
    const ParameterizedSpace::State &state,
    std::mt19937 &rng,
    bool do_cc,
    const EnvironmentVector &environment,
    std::size_t max_iterations = 50)
    -> std::pair<std::array<std::optional<GcpBranchSolution>, 8>, GcpSweepStats>
{
    static_assert(
        rake >= 8,
        "sweep_gcp_branches_over_psi enumerates all 8 shared-branch combos in a single "
        "resolve_block_mask call; needs rake >= 8");

    static const float two_pi = 2.0F * std::acos(-1.0F);
    std::uniform_real_distribution<float> psi_dist(0.0F, two_pi);

    auto base_state_array = state.to_array();
    std::array<std::optional<GcpBranchSolution>, 8> solved{};
    GcpSweepStats stats;
    stats.max_iterations = max_iterations;

    // `lanes` (and therefore the GCP assignment) is the same on every iteration -- only the
    // sampled psi varies -- so set it once instead of re-setting it inside the loop.
    std::array<std::array<float, 3>, rake> lanes{};
    for (std::size_t lane = 0; lane < rake; ++lane)
    {
        lanes[lane] = gcp_branch_combos[(lane < 8) ? lane : 0];
    }
    // Same GCP combo on both arms -- pass `lanes` as both left and right.
    ParameterizedSpace::set_gcp_lanes(lanes, lanes);

    for (std::size_t iteration = 0; iteration < max_iterations; ++iteration)
    {
        const bool all_solved =
            std::all_of(solved.begin(), solved.end(), [](const auto &s) { return s.has_value(); });
        if (all_solved)
        {
            break;
        }

        const auto iteration_start = std::chrono::steady_clock::now();

        const float psi_left = psi_dist(rng);
        const float psi_right = psi_dist(rng);
        auto iter_state_array = base_state_array;
        iter_state_array[10] = psi_left;
        iter_state_array[11] = psi_right;
        ParameterizedSpace::State iter_state(iter_state_array.data());

        ParameterizedSpace::StateBlock<rake> block;
        for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
        {
            block[i] = iter_state.broadcast(i);
        }

        auto [valid_mask, ambient_block] = ParameterizedSpace::resolve_block_mask<rake>(block);

        const auto com = ParameterizedSpace::compute_com<rake>(ambient_block);
        const auto com_mask = point_in_support_polygon_mask(com[0], com[1]);

        for (std::size_t lane = 0; lane < 8; ++lane)
        {
            // Already solved on an earlier iteration -- this iteration's value for this lane is
            // computed (it's part of the same parallel call) but must not be used.
            if (solved[lane])
            {
                continue;
            }

            // IK must have resolved on this lane AND its CoM must stay within the support
            // polygon -- a lane failing either one makes that branch/psi sample invalid.
            if (valid_mask[{0, lane}] == 0.0F or com_mask[{0, lane}] == 0.0F)
            {
                continue;
            }

            if (do_cc)
            {
                // fkcc/fkcc_attach only report a single aggregated bool across the whole rake
                // block (no per-lane variant), so to get a real per-lane collision verdict this
                // re-broadcasts just this lane's resolved ambient config across the block and
                // runs the ordinary (block-aggregated) check on that -- one extra fkcc call per
                // surviving candidate lane, not touching the rest of the batch's IK/CoM masking.
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

                const bool collision_free = (environment.attachments.empty()) ?
                    Robot::template fkcc<rake>(environment, lane_block) :
                    Robot::template fkcc_attach<rake>(environment, lane_block);

                if (not collision_free)
                {
                    continue;
                }
            }

            solved[lane] = GcpBranchSolution{gcp_branch_combos[lane], psi_left, psi_right};
        }

        stats.iteration_durations.push_back(std::chrono::steady_clock::now() - iteration_start);
        ++stats.iterations_run;
    }

    return {solved, stats};
}

// Sweeps GCP branch selectors for a single (broadcast) task-space state: the "one config,
// many GCP's" counterpart to the "many configs, one shared GCP" mode every resolve_block<rake>
// call in rby1_task_space_planner.cc uses (interpolated edge samples, all under whatever GCP is
// currently locked in). Tests each arm's 8 (elbow_sel, shoulder_sel, wrist_sel) combos
// independently, holding the other arm at `initial_left`/`initial_right` while it sweeps, via
// ParameterizedSpace::set_gcp_lanes + resolve_block_mask -- one call per arm instead of one call
// per candidate. Returns the first combo (in the fixed enumeration order below) whose lane
// resolves (zero reach_violation, ambient q within joint limits); nullopt if none did. Does not
// collision-check the resolved configuration -- IK validity only, same as resolve_block's own
// contract.
auto select_gcp_branch(
    const ParameterizedSpace::State &state,
    const std::array<float, 3> &initial_left,
    const std::array<float, 3> &initial_right) -> std::pair<std::optional<std::array<float, 3>>, std::optional<std::array<float, 3>>>
{
    static_assert(
        rake >= 8,
        "select_gcp_branch enumerates all 8 per-arm branch combos in a single resolve_block_mask "
        "call; needs rake >= 8");

    const auto &combos = gcp_branch_combos;

    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    // Sweeps `combos` on one arm (left == true -> left) while holding the other arm fixed at
    // `other`; lanes past the 8 candidates just repeat combo 0 (rake is typically 8/16, so
    // there's nothing meaningful left to test in them).
    auto sweep_one_arm = [&](bool left, const std::array<float, 3> &other) -> std::optional<std::array<float, 3>>
    {
        std::array<std::array<float, 3>, rake> left_lanes{};
        std::array<std::array<float, 3>, rake> right_lanes{};
        for (std::size_t lane = 0; lane < rake; ++lane)
        {
            const auto &combo = combos[(lane < 8) ? lane : 0];
            left_lanes[lane] = left ? combo : other;
            right_lanes[lane] = left ? other : combo;
        }

        ParameterizedSpace::set_gcp_lanes(left_lanes, right_lanes);
        auto [valid_mask, ambient_block] = ParameterizedSpace::resolve_block_mask<rake>(block);
        std::cout << (left ? "Left" : "Right") << " arm GCP sweep: valid_mask = " << valid_mask << std::endl;
        static_cast<void>(ambient_block);

        for (std::size_t lane = 0; lane < 8; ++lane)
        {
            if (valid_mask[{0, lane}] != 0.0F)
            {
                return combos[lane];
            }
        }

        return std::nullopt;
    };

    const auto chosen_left = sweep_one_arm(true, initial_right);
    const auto chosen_right = sweep_one_arm(false, chosen_left.value_or(initial_left));

    return {chosen_left, chosen_right};
}

// Standalone GCP-branch-sweep runner: resolves the same hand-picked smoke-test state used by
// rby1_task_space_planner.cc's smoke test, then reports which per-arm branch combo (of the 8
// possible (elbow_sel, shoulder_sel, wrist_sel) selectors) select_gcp_branch finds for each arm.
auto main() -> int
{
    std::cout << std::boolalpha;

    Robot::ConfigurationArray home_ambient = {
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
        -2.16360474e-03, 1.45860954e+00, -1.97468998e+00,  1.55147668e+00,  2.66302773e-01, -5.04128611e-01,
        1.17522024e-01,  9.91603153e-02,  4.46071004e-01, -1.67540527e+00, 3.29611651e-01,  5.74187322e-01,  1.65740047e+00,
        -1.22668093e+00,  3.92945961e-02,  1.20823988e+00, -6.24967729e-01, -1.61156583e-01,  5.79106863e-01, -2.52512556e+00,
    };

    ParameterizedSpace::compute_mid_pose(home_ambient);

    ParameterizedSpace::StateArray state_array = {{
        0.0, 0.0, 1.0, 0.0, 0.0207, -0.3034, 0.882, 1.1434, 0.0344, 0.0677, 0.1354, -0.0944, 0.553, 0.0073, 0.22, 0.0, 0.0, 0.0, 1.0
    }};
    ParameterizedSpace::State state(state_array.data());

    std::cout << "\n--- GCP sweep on the smoke-test state ---" << std::endl;
    const auto [swept_left, swept_right] = select_gcp_branch(state, {0.0F, 1.0F, 1.0F}, {0.0F, 1.0F, 1.0F});

    auto print_gcp = [](const std::string &label, const std::optional<std::array<float, 3>> &gcp)
    {
        if (not gcp)
        {
            std::cout << label << ": no branch resolved" << std::endl;
            return;
        }

        std::cout << label << ": (" << (*gcp)[0] << ", " << (*gcp)[1] << ", " << (*gcp)[2] << ")" << std::endl;
    };

    print_gcp("left_gcp", swept_left);
    print_gcp("right_gcp", swept_right);

    // --- Per-branch psi sweep: up to 50 random-(psi_left, psi_right) iterations, all 8
    // shared-branch combos resolved in parallel each iteration (one call, rake == 8 lanes),
    // first success per branch kept. Run once without collision-checking and once with, so the
    // two can be compared directly (same rng, drawn back to back).
    std::mt19937 rng(std::random_device{}());
    const auto environment = build_environment();

    auto report_sweep = [](
        const std::string &label,
        const std::array<std::optional<GcpBranchSolution>, 8> &solved,
        const GcpSweepStats &stats)
    {
        std::cout << "\n--- Per-branch psi sweep: " << label << " (up to 50 iterations) ---" << std::endl;

        std::chrono::nanoseconds total_duration{0};
        for (const auto &duration : stats.iteration_durations)
        {
            total_duration += duration;
        }

        for (std::size_t lane = 0; lane < solved.size(); ++lane)
        {
            const auto &combo = gcp_branch_combos[lane];
            std::cout << "branch (" << combo[0] << ", " << combo[1] << ", " << combo[2] << "): ";
            if (solved[lane])
            {
                std::cout << "resolved at psi_left = " << solved[lane]->psi_left
                           << ", psi_right = " << solved[lane]->psi_right << std::endl;
            }
            else
            {
                std::cout << "unresolved" << std::endl;
            }
        }

        std::cout << "total iterations: " << stats.iterations_run << " / " << stats.max_iterations << std::endl;
        std::cout << "total time: " << total_duration.count() << " ns" << std::endl;
    };

    const auto [solved_no_cc, stats_no_cc] = sweep_gcp_branches_over_psi(state, rng, /*do_cc=*/false, environment);
    report_sweep("without collision-checking", solved_no_cc, stats_no_cc);

    const auto [solved_cc, stats_cc] = sweep_gcp_branches_over_psi(state, rng, /*do_cc=*/true, environment);
    report_sweep("with collision-checking", solved_cc, stats_cc);

    return 0;
}
