// Projection-based (mcvamp) baseline for RBY1, analogous to bimanual_iiwa_projection_shelf.cc:
// where rby1_task_space_planner.cc plans through Robot::ParameterizedSpace via closed-form IK
// (ParameterizedSpace::resolve_block), this file plans directly over RBY1's ambient 24-dof
// joint space and enforces the same two invariants that IK bakes in implicitly -- the rigid
// left-hand/right-hand offset and center-of-mass stability -- as generic CBiRRT2-style
// task-space constraints (vamp::planning::constraint::BimanualTaskSpaceConstraint and
// CoMConstraint), iteratively projected by ConstrainedLocalPlanner. Unlike BimanualIiwa (a
// fixed-base 14-dof arm pair) or Digit (a floating-base biped that also needs its feet pinned),
// RBY1 has a fixed wheeled base and no legs to hold in place, so nothing beyond these two
// constraints is needed for a valid ambient configuration.
//
// Problems are read from resources/ruby/rrtc_calls.jsonl (the live rrtc-call log written by
// rby1's ruby side, one JSON object per line) using the same input conventions as
// rby1_task_space_planner.cc: "start"/"goal" hold each entry's endpoint, "file"/"task_index"/
// "kind" are carried through as bookkeeping, and a "leg" vs. "probe" `kind` plus a "solved"
// flag decide what gets planned. The only difference from that planner is what "start"/"goal"
// represent: there, ParameterizedSpace::dimension task-space floats to be IK-resolved; here,
// already-ambient Robot::dimension floats to be projected directly (this planner never
// touches ParameterizedSpace). rrtc_calls.jsonl interleaves both kinds of logged call under
// the same "start"/"goal" keys (tagged by a "planner" field, "task_space" vs. "mcvamp"), so
// each planner's own dimension check is what actually separates its entries out of the log --
// a "start"/"goal" of the wrong length for this planner is skipped, not an error.
//
// Usage:
//   vamp_rby1_mcvamp_planner <input_problems.jsonl|input_problems.json> <output_results.json>
//       [--run_unsolvable] [--fix_single_smm]
//
// --fix_single_smm restricts each problem's search to whatever GCP branch
// (elbow_sel/wrist_sel per arm) its own start configuration is already on, rejecting any
// candidate that would cross onto a different one -- see ConstraintSettings::fix_single_smm
// and RBY1::ParameterizedSpace::smm_mask_block. Off by default. shoulder_sel isn't
// classified/enforced yet (see classify_smm_block's comment for why).
//
// Same two required positional arguments (input, then output) and --run_unsolvable flag as
// rby1_task_space_planner.cc, and the same single-JSON output: no separate results CSV or
// trajectory-file directory -- every solved problem's shortcut path, interpolated to
// Robot::resolution, is instead embedded as a "trajectory" array (one
// {"ambient_configuration": [Robot::dimension floats]} object per waypoint) directly inside
// that problem's result object, exactly like rby1_task_space_planner.cc's output.
//
// A ".jsonl" path is read as one JSON object per line, each shaped like resources/ruby/
// rrtc_calls.jsonl's entries: { "file": <string>, "start": [<Robot::dimension floats>],
// "goal": [<... floats>], ... } or, for jsonl input specifically, "task_index"/"kind"/
// "solved" in place of "file" (see rby1_task_space_planner.cc's matching comment). Any other
// path is read as a single JSON document: a bare array, or the {"problems": [...]} wrapper,
// of entries in the same "start"/"goal" shape. Entries whose `kind` isn't "leg", that are
// logged as unsolved, or whose start/goal aren't Robot::dimension floats are skipped.

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/manifold/com_constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/validate.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/rng.hh>
#include <vamp/robots/rby1.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::RBY1;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using ProjMethod = vamp::planning::constraint::ProjMethod;
using ConstraintSetT = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstrainedLP = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
using BimanualTSC = vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>;
using CoMConstraintT = vamp::planning::constraint::CoMConstraint<Robot, rake>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// Wraps an inner ambient-space Halton<Robot> sampler and, on every draw, overwrites the base
// dimensions (0: base_x, 1: base_y, 2: cos(base_rz), 3: sin(base_rz) -- see
// Robot::joint_names in rby1.hh) with a fixed pose instead of whatever the inner sampler
// drew. RBY1's mobile base must not move for this problem (the "leg" is a fixed-base
// reach-to-grasp, not a navigation motion): every RRTC sample -- and therefore every steer
// candidate built from it -- needs to already have the base pinned, since a sampler that
// occasionally proposes a different base pose would waste the vast majority of samples on
// candidates guaranteed to fail the projection below regardless of how the arms/torso move.
//
// This only fixes what gets *proposed*; it doesn't stop constraint projection itself from
// nudging the base afterward (the CoM constraint's Jacobian has nonzero columns for
// base_x/base_y, since translating the whole base is a valid way to shift the CoM). Pin
// those same columns via ConstraintSet::set_pinned() on the constraint set this sampler is
// paired with if the base must also stay exactly fixed through projection, not just sampling.
struct RBY1FixedAmbientBaseSampler final : public vamp::rng::RNG<Robot, Robot>
{
    RBY1FixedAmbientBaseSampler(
        vamp::rng::RNG<Robot, Robot>::Ptr inner_in, const std::array<float, 4> &fixed_base_in) noexcept
      : inner(std::move(inner_in)), fixed_base(fixed_base_in)
    {
    }

    inline void reset() noexcept override
    {
        inner->reset();
        inner->dist.reset();
    }

    inline auto next() noexcept -> Robot::Configuration override
    {
        const auto full = inner->next().to_array();
        Robot::ConfigurationArray array{};
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            array[i] = full[i];
        }

        array[0] = fixed_base[0];
        array[1] = fixed_base[1];
        array[2] = fixed_base[2];
        array[3] = fixed_base[3];

        return Robot::Configuration(array);
    }

    vamp::rng::RNG<Robot, Robot>::Ptr inner;
    std::array<float, 4> fixed_base;
};

// One (start, goal) problem in RBY1's ambient joint space, read from input_json_path --
// `problem_index` is this entry's own raw position in the input (unique and gapless whether
// or not the entry gets skipped), matching rby1_task_space_planner.cc's "index"; `file_field`,
// `task_index`, and `kind` mirror that planner's bookkeeping fields, present here only when
// the source entry had them.
struct Problem
{
    std::size_t problem_index;
    std::string file_field;
    nlohmann::json task_index;
    nlohmann::json kind;
    Robot::ConfigurationArray start;
    Robot::ConfigurationArray goal;
};

// Same table + attached-box environment rby1_task_space_planner.cc's main() builds, duplicated
// here so this standalone binary doesn't need to link against that file just for a
// collision-checkable scene.
auto build_environment() -> EnvironmentInput
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
    return environment;
}

// Sanity-check an already-resolved joint configuration directly against the environment (no
// IK, no task-space resolution -- these are real joint angles).
auto is_config_valid(const Robot::Configuration &q, const EnvironmentVector &environment_v) -> bool
{
    Robot::ConfigurationBlock<rake> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return (environment_v.attachments.empty()) ? Robot::template fkcc<rake>(environment_v, block)
                                                : Robot::template fkcc_attach<rake>(environment_v, block);
}

// Raw (pre-projection) squared constraint violation of a single configuration: broadcasts it
// across all rake lanes (as ConstraintSet::project()/satisfied() do internally, but those
// don't expose the actual number) and reads back lane 0's summed squared error over every
// constraint in the set. Near zero means q already sits on (or very near) the manifold this
// set defines; a large value means it doesn't -- useful to know before spending any time
// projecting it.
auto constraint_squared_error(const ConstraintSetT &constraint_set, const Robot::Configuration &q) -> float
{
    Robot::ConfigurationBlock<rake> block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return constraint_set.squared_error(block)[{0, 0}];
}

// Right-hand (eef 1) pose expressed in the left hand's (eef 0) frame, as the (qw, qx, qy, qz,
// x, y, z) layout BimanualTaskSpaceConstraint/tsr_bimanual_error expect.
auto relative_pose(const Robot::ConfigurationArray &q) -> BimanualTSC::Transform
{
    const auto left = Robot::eefk(q, 0);
    const auto right = Robot::eefk(q, 1);
    const Eigen::Isometry3f rel = left.inverse() * right;
    const Eigen::Quaternionf quat(rel.rotation());
    const auto &t = rel.translation();
    return {quat.w(), quat.x(), quat.y(), quat.z(), t.x(), t.y(), t.z()};
}

template <typename PathT>
auto to_ambient_waypoints(const PathT &path) -> std::vector<Robot::ConfigurationArray>
{
    std::vector<Robot::ConfigurationArray> waypoints;
    waypoints.reserve(path.size());
    for (const auto &config : path)
    {
        const auto full = config.to_array();
        Robot::ConfigurationArray array{};
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            array[i] = full[i];
        }

        waypoints.push_back(array);
    }

    return waypoints;
}

auto compute_config_distance(const std::vector<Robot::ConfigurationArray> &waypoints) -> float
{
    float distance = 0.0F;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
    {
        distance += Robot::Configuration(waypoints[i]).distance(Robot::Configuration(waypoints[i + 1]));
    }

    return distance;
}

// Pulls start/goal ambient configurations out of one problem entry, from "start"/"goal" --
// the same keys rby1_task_space_planner.cc reads (that planner's entries hold
// ParameterizedSpace::dimension task-space floats there instead; a length mismatch is this
// planner's signal that an entry belongs to that planner, not this one, not a malformed
// entry). Returns false (leaving `problem` untouched) if the entry isn't a "leg" call (see
// rby1_task_space_planner.cc's matching filter -- "probe" calls are throwaway feasibility
// checks, not real reach-to-grasp problems), if it's logged as unsolved, if "start"/"goal"
// are missing, or if either has the wrong length.
auto extract_problem(
    const nlohmann::json &entry, std::size_t problem_index, bool run_unsolvable, Problem &problem) -> bool
{
    if (entry.value("kind", std::string("leg")) != "leg")
    {
        return false;
    }

    if (not run_unsolvable and not entry.value("solved", true))
    {
        return false;
    }

    if (not entry.contains("start") or not entry.contains("goal"))
    {
        return false;
    }

    const auto start_vec = entry.at("start").get<std::vector<float>>();
    const auto goal_vec = entry.at("goal").get<std::vector<float>>();
    if (start_vec.size() != Robot::dimension or goal_vec.size() != Robot::dimension)
    {
        return false;
    }

    problem.problem_index = problem_index;
    if (entry.contains("task_index"))
    {
        problem.task_index = entry.at("task_index");
    }
    if (entry.contains("kind"))
    {
        problem.kind = entry.at("kind");
    }

    // Same "file" fallback as rby1_task_space_planner.cc: jsonl entries have no "file" field
    // of their own, so build an equivalent trace-back string from "kind"/"task_index" instead.
    problem.file_field = entry.value("file", std::string());
    if (problem.file_field.empty())
    {
        const auto kind_field = entry.value("kind", std::string());
        problem.file_field = entry.contains("task_index") ?
            kind_field + " task_index=" + std::to_string(entry.at("task_index").get<long long>()) :
            kind_field;
    }

    std::copy(start_vec.begin(), start_vec.end(), problem.start.begin());
    std::copy(goal_vec.begin(), goal_vec.end(), problem.goal.begin());
    return true;
}

// Reads problems from the same three input shapes as rby1_task_space_planner.cc:
//  - a bare JSON array, each entry optionally carrying a "file" field for traceability
//    (e.g. resources/ruby/problem_set_skipped_intermediate.json);
//  - the earlier { "problems": [...] } wrapper, same per-entry shape minus "file";
//  - a ".jsonl" file (one JSON object per line, as logged live by ruby's RRTC call sites),
//    where each line has its own "start"/"goal" plus "task_index"/"kind"/"solved"/"dimension"
//    bookkeeping in place of "file" -- the live rrtc-call log format written to
//    resources/ruby/rrtc_calls.jsonl.
// Entries that aren't a "leg" call, are logged as unsolved (unless `run_unsolvable` is set),
// or whose start/goal aren't Robot::dimension floats (see extract_problem) are skipped.
auto load_problems(const std::string &path, bool run_unsolvable) -> std::vector<Problem>
{
    std::ifstream input_file(path);
    if (not input_file)
    {
        throw std::runtime_error("Failed to open input JSON file: " + path);
    }

    const bool is_jsonl = path.size() >= 6 and path.compare(path.size() - 6, 6, ".jsonl") == 0;

    nlohmann::json problems_json_storage = nlohmann::json::array();

    if (is_jsonl)
    {
        std::string line;
        while (std::getline(input_file, line))
        {
            if (line.find_first_not_of(" \t\r\n") == std::string::npos)
            {
                continue;
            }

            problems_json_storage.push_back(nlohmann::json::parse(line));
        }
    }
    else
    {
        nlohmann::json input_json;
        input_file >> input_json;
        problems_json_storage = input_json.is_array() ? input_json : input_json.at("problems");
    }

    std::vector<Problem> problems;
    for (std::size_t problem_index = 0; problem_index < problems_json_storage.size(); ++problem_index)
    {
        Problem problem;
        if (extract_problem(problems_json_storage[problem_index], problem_index, run_unsolvable, problem))
        {
            problems.push_back(problem);
        }
    }

    return problems;
}

auto main(int argc, char **argv) -> int
{
    // --run_unsolvable is a flag, not a positional argument -- pull it out first (it may
    // appear anywhere on the command line), same as rby1_task_space_planner.cc, so the
    // remaining positional-argument count check below is unaffected by where the caller
    // places it.
    bool run_unsolvable = false;
    // Optional: restrict every problem's RRTC search to the same GCP branch
    // (elbow_sel/wrist_sel per arm) its own start configuration is already on, rejecting any
    // candidate that would cross onto a different one -- see
    // ConstraintSettings::fix_single_smm and RBY1::ParameterizedSpace::smm_mask_block. Off by
    // default (matches today's behavior exactly).
    bool fix_single_smm = false;
    std::vector<std::string> positional_args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--run_unsolvable")
        {
            run_unsolvable = true;
        }
        else if (arg == "--fix_single_smm")
        {
            fix_single_smm = true;
        }
        else
        {
            positional_args.push_back(arg);
        }
    }

    if (positional_args.size() != 2)
    {
        std::cerr << "usage: " << argv[0]
                   << " <input_problems.json> <output_results.json> [--run_unsolvable] "
                      "[--fix_single_smm]"
                   << std::endl;
        return 1;
    }

    const std::string problems_path = positional_args[0];
    const std::filesystem::path results_json_path = positional_args[1];
    // Same as rby1_task_space_planner.cc's hardcoded RRTCSettings.range -- not exposed on the
    // command line, so this planner's CLI matches that one's exactly.
    const float range = 0.25F;
    if (not results_json_path.parent_path().empty())
    {
        std::filesystem::create_directories(results_json_path.parent_path());
    }

    std::cout << std::boolalpha;
    std::cout << "Robot::dimension (ambient/joint space): " << Robot::dimension << std::endl;

    std::vector<Problem> problems;
    try
    {
        problems = load_problems(problems_path, run_unsolvable);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    std::cout << "Loaded " << problems.size() << " ambient (start, goal) problem(s) from " << problems_path
               << std::endl;
    if (problems.empty())
    {
        std::cout << "Nothing to plan." << std::endl;
        return 0;
    }

    const EnvironmentInput environment = build_environment();
    const EnvironmentVector environment_v(environment);
    std::cout << "Environment has " << environment_v.cuboids.size() << " cuboids, "
               << environment_v.attachments.size() << " attachments." << std::endl;

    // Closure target: the hand-to-hand relative pose read off the first problem's start_q.
    // RBY1's task-space parameterization (ParameterizedSpace) derives every hand pose from a
    // shared mid-frame with fixed mid-to-hand offsets (t_mid_left/t_mid_right, computed once
    // in compute_mid_pose), so every resolve_block()-produced ambient configuration -- and
    // therefore every start_q/goal_q here -- shares the same rigid left-hand/right-hand
    // offset, exactly like BimanualIiwa's closed kinematic loop.
    const auto lTr = relative_pose(problems.front().start);
    std::cout << "Closure target (right-in-left, qw qx qy qz x y z): ";
    for (const auto v : lTr)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    constexpr float kTol = 1e-3F;
    const BimanualTSC::Bound lower = {-kTol, -kTol, -kTol, -kTol, -kTol, -kTol};
    const BimanualTSC::Bound upper = {kTol, kTol, kTol, kTol, kTol, kTol};

    // Static-stability support polygon: RBY1's four ground-contact points (right wheel ->
    // left wheel -> left caster -> right caster), the same vertices
    // ParameterizedLocalPlanner::support_polygon_xy defaults to (see
    // src/impl/vamp/planning/constraints/parameterized_local_planner.hh).
    const std::vector<CoMConstraintT::Vertex> support_polygon = {
        {0.228000F, -0.265000F},   // right wheel
        {0.228000F, 0.265000F},    // left wheel
        {-0.248686F, 0.066310F},   // left caster
        {-0.248686F, -0.066310F},  // right caster
    };

    // Same InnerLM projection settings bimanual_iiwa_projection_shelf.cc found best for a
    // 6-row rigid TSR; the CoM constraint's 2 rows are never chart-active (its interior has
    // positive measure), so it only ever contributes a corrective step when violated and
    // otherwise reports zero error.
    vamp::planning::constraint::ConstraintSettings constraint_settings;
    constraint_settings.method = ProjMethod::InnerLM;
    constraint_settings.descend_rate = 1.0F;
    constraint_settings.max_iterations = 25;
    constraint_settings.emit_all_waypoints = false;
    constraint_settings.perturbation_scale = 0.2F;
    constraint_settings.tolerance = 1e-6F;
    constraint_settings.fix_single_smm = fix_single_smm;

    // lTr/support_polygon/constraint_settings above are loop-invariant plain data; the
    // constraint objects and ConstrainedLocalPlanner built from them are not reused across
    // problems below -- BimanualTSC/CoMConstraintT cache mutable per-evaluation Jacobian/
    // error buffers, and ConstraintSet/ConstrainedLocalPlanner are documented as "not
    // thread-safe... use one instance (with unshared constraints) per thread", i.e. they're
    // meant to be scoped to a single solve, not accumulated across hundreds of sequential
    // ones. A fresh instance per problem costs nothing (a handful of small allocations) and
    // rules out stale evaluation state as a cause of a solve mysteriously stalling.

    vamp::planning::RRTCSettings settings;
    settings.range = range;
    settings.max_iterations = 100000;
    settings.max_samples = 1000000;
    settings.dynamic_domain = false;

    // Not constructed here: `rng` is built fresh per problem below, alongside the
    // constraint set/local planner. Halton's next() carries a `d`/`n` state that only gets
    // reset back down every 1,000,000 draws (see src/impl/vamp/random/halton.hh); a shared
    // instance run across many problems, each burning up to settings.max_iterations draws,
    // can accumulate enough growth in that state to hit its inf/NaN edge case before that
    // boundary -- at which point next()'s internal convergence loop never terminates. A
    // fresh Halton<Robot> per problem means that state never has the chance to accumulate.

    nlohmann::json output_json = nlohmann::json::array();

    std::size_t solved_count = 0;
    std::size_t attempted_count = 0;

    // `problem.problem_index` is this entry's own raw position in the input (see the Problem
    // struct comment) -- unique and gapless across every loaded problem, same as
    // rby1_task_space_planner.cc's "index"-named output field.
    for (const auto &problem : problems)
    {
        std::cout << "\n=== Problem " << problem.problem_index
                   << (problem.file_field.empty() ? "" : " (" + problem.file_field + ")") << " ===" << std::endl;

        // Fresh constraint objects, local planner, and Halton sampler for this problem alone
        // -- see the notes above the loop on why none of these are reused across problems.
        auto bimanual_constraint = std::make_shared<BimanualTSC>(lTr, lower, upper);
        auto com_constraint = std::make_shared<CoMConstraintT>(support_polygon);
        ConstraintSetT constraint_set(
            std::vector<ConstraintSetT::Ptr>{bimanual_constraint, com_constraint}, constraint_settings);
        ConstrainedLP local_planner(constraint_set);

        // The base must stay exactly where it starts (see RBY1FixedAmbientBaseSampler above)
        // -- fix it to this problem's own q_start base pose, not a shared constant, since
        // different problems can be posed at different places in the workspace.
        const std::array<float, 4> fixed_base = {
            problem.start[0], problem.start[1], problem.start[2], problem.start[3]};
        auto inner_rng = std::make_shared<vamp::rng::Halton<Robot>>();
        auto rng = std::make_shared<RBY1FixedAmbientBaseSampler>(inner_rng, fixed_base);

        // Project start/goal onto the constraint manifold: they should already sit on (or
        // extremely close to) it since they came out of resolve_block, but projecting removes
        // any residual numerical drift and pulls in the CoM constraint (which resolve_block's
        // task-space parameterization never explicitly projects onto -- it's only *checked*
        // there via resolve_and_check's support-polygon gate).
        Robot::Configuration start_config(problem.start);
        Robot::Configuration goal_config(problem.goal);

        // Fix the target GCP branch to whatever this problem's own start configuration is
        // already on -- "stay on the branch you started on" -- before any of this problem's
        // solving/projection runs, since ConstrainedLocalPlanner (via constraint_set below)
        // reads Robot::ParameterizedSpace::target_smm_left/right the moment fix_single_smm is
        // on. classify_smm_block now classifies all three (elbow_sel, shoulder_sel,
        // wrist_sel) per arm.
        //
        // Diagnostic: also classify the goal and print both, since --fix_single_smm can only
        // ever solve a problem whose start and goal already sit on the SAME branch -- every
        // candidate on the goal's own (different) branch fails the start-fixed
        // smm_mask_block gate by construction, so a start/goal branch mismatch looks
        // identical to "every problem fails" from the RRTC result alone.
        if (fix_single_smm)
        {
            Robot::ConfigurationBlock<rake> start_block;
            Robot::ConfigurationBlock<rake> goal_block;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                start_block[i] = start_config.broadcast(i);
                goal_block[i] = goal_config.broadcast(i);
            }

            const auto start_smm = Robot::ParameterizedSpace::classify_smm_block<rake>(start_block);
            const auto goal_smm = Robot::ParameterizedSpace::classify_smm_block<rake>(goal_block);

            const std::array<float, 3> start_left = {
                start_smm[0][0][{0, 0}], start_smm[0][1][{0, 0}], start_smm[0][2][{0, 0}]};
            const std::array<float, 3> start_right = {
                start_smm[1][0][{0, 0}], start_smm[1][1][{0, 0}], start_smm[1][2][{0, 0}]};
            const std::array<float, 3> goal_left = {
                goal_smm[0][0][{0, 0}], goal_smm[0][1][{0, 0}], goal_smm[0][2][{0, 0}]};
            const std::array<float, 3> goal_right = {
                goal_smm[1][0][{0, 0}], goal_smm[1][1][{0, 0}], goal_smm[1][2][{0, 0}]};

            auto print_smm = [](const char *label, const std::array<float, 3> &left,
                                 const std::array<float, 3> &right)
            {
                std::cout << "  " << label << " GCP (elbow_sel, shoulder_sel, wrist_sel): left=(" << left[0]
                           << ", " << left[1] << ", " << left[2] << ") right=(" << right[0] << ", " << right[1]
                           << ", " << right[2] << ")" << std::endl;
            };
            print_smm("start", start_left, start_right);
            print_smm("goal ", goal_left, goal_right);

            const bool same_smm = start_left == goal_left and start_right == goal_right;
            if (not same_smm)
            {
                std::cout << "  WARNING: start and goal are on DIFFERENT GCP branches -- "
                             "--fix_single_smm will reject this problem (every candidate on the "
                             "goal's own branch fails the start-fixed target_smm mask)."
                          << std::endl;
            }

            Robot::ParameterizedSpace::set_target_smm(start_left, start_right);
        }

        // Diagnostic: how far the raw, un-projected q_start/q_goal already sit from the
        // constraint manifold before we touch them at all. If these are consistently large
        // (not just numerical-drift small), start_q/start_goal aren't actually satisfying
        // the bimanual/CoM constraints this planner is enforcing, and projection or RRTC
        // failing downstream is a symptom of that, not a bug in this file.
        const float start_pre_error = constraint_squared_error(constraint_set, start_config);
        const float goal_pre_error = constraint_squared_error(constraint_set, goal_config);
        std::cout << "  pre-projection squared error (start, goal): " << start_pre_error << ", "
                   << goal_pre_error << " (tolerance " << constraint_settings.tolerance << ")" << std::endl;

        nlohmann::json problem_result;
        problem_result["index"] = problem.problem_index;
        if (not problem.file_field.empty())
        {
            problem_result["file"] = problem.file_field;
        }
        if (not problem.task_index.is_null())
        {
            problem_result["task_index"] = problem.task_index;
        }
        if (not problem.kind.is_null())
        {
            problem_result["kind"] = problem.kind;
        }

        const bool start_projected = local_planner.project(start_config);
        const bool goal_projected = local_planner.project(goal_config);
        if (not start_projected or not goal_projected)
        {
            std::cout << "Failed to project start/goal onto the constraint manifold; skipping." << std::endl;
            problem_result["solved"] = false;
            problem_result["error"] = "start or goal failed to project onto the constraint manifold";
            output_json.push_back(std::move(problem_result));
            continue;
        }

        if (not is_config_valid(start_config, environment_v) or not is_config_valid(goal_config, environment_v))
        {
            std::cout << "Projected start/goal is in collision; skipping." << std::endl;
            problem_result["solved"] = false;
            problem_result["error"] = "projected start or goal is in collision";
            output_json.push_back(std::move(problem_result));
            continue;
        }

        ++attempted_count;

        const auto t0 = std::chrono::steady_clock::now();
        auto result = RRTC::solve(start_config, goal_config, environment_v, settings, rng, local_planner);
        const auto elapsed_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        // std::cout << "solved: " << result.solved << ", iterations: " << result.iterations << ", " << elapsed_ms
        //            << " ms" << std::endl;
        std::cout << "solved: " << result.solved << std::endl;
        // std::cout << "cost: " << result.cost << std::endl;
        std::cout << "iterations: " << result.iterations << std::endl;
        std::cout << "milliseconds: " << (result.nanoseconds / 1000000.0F) << std::endl;

        problem_result["solved"] = result.solved;
        problem_result["planning_time_ms"] = result.nanoseconds / 1000000.0F;
        problem_result["iterations"] = result.iterations;

        if (result.solved)
        {
            ++solved_count;

            const auto shortcut_t0 = std::chrono::steady_clock::now();
            vamp::planning::SimplifySettings simplify_settings;
            simplify_settings.operations = {vamp::planning::SHORTCUT};
            auto shortcut_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
                result.path, environment_v, simplify_settings, rng, local_planner);
            const auto shortcut_elapsed_ms =
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - shortcut_t0).count();

            // Densify the shortcutted path to Robot::resolution before writing it out, same as
            // rby1_task_space_planner.cc, so the embedded trajectory is at a fixed step
            // resolution rather than just the shortcutted waypoints.
            shortcut_result.path.interpolate_to_resolution(Robot::resolution);
            const auto ambient_waypoints = to_ambient_waypoints(shortcut_result.path);
            const float config_distance = compute_config_distance(ambient_waypoints);

            std::cout << "  shortcut path size " << shortcut_result.path.size() << ", " << shortcut_elapsed_ms
                       << " ms, config distance " << config_distance << std::endl;

            problem_result["shortcut_time_ms"] = shortcut_elapsed_ms;
            problem_result["shortcut_path_size"] = shortcut_result.path.size();
            problem_result["config_distance"] = config_distance;

            // Embedded per-waypoint ambient configuration, same shape as
            // rby1_task_space_planner.cc's "trajectory" field (that planner's waypoints also
            // carry a "resolved" bool, since it IK-resolves each one from a task-space state;
            // this planner's waypoints are already ambient, so there's nothing to resolve).
            nlohmann::json trajectory_json = nlohmann::json::array();
            for (const auto &array : ambient_waypoints)
            {
                nlohmann::json ambient_configuration_json = nlohmann::json::array();
                for (std::size_t i = 0; i < Robot::dimension; ++i)
                {
                    ambient_configuration_json.push_back(array[i]);
                }

                nlohmann::json waypoint_json;
                waypoint_json["ambient_configuration"] = std::move(ambient_configuration_json);
                trajectory_json.push_back(std::move(waypoint_json));
            }

            problem_result["trajectory"] = std::move(trajectory_json);
        }

        output_json.push_back(std::move(problem_result));
    }

    std::ofstream output_file(results_json_path);
    if (not output_file)
    {
        std::cerr << "Failed to open output JSON file: " << results_json_path << std::endl;
        return 1;
    }
    output_file << output_json.dump(2) << std::endl;
    std::cout << "\nWrote " << output_json.size() << " planning result(s) to " << results_json_path << std::endl;

    std::cout << "\nSolved " << solved_count << " / " << attempted_count << " attempted problem(s) ("
               << problems.size() << " loaded)." << std::endl;

    return 0;
}
