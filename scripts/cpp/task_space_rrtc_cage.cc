// Single hard-coded Panda cage problem (MBM "cage", problem 0) for debugging TaskSpaceRRTC.
//
// A box cage (8 cuboids) around a small cube; the goal is inside the cage, behind the bars.
// The planner is given only the *pose* of the end effector at `goal` (an exact TSR), never
// the goal configuration; `goal` is kept so the pose can be derived and for reference. This
// is one of the problems the task-space planner currently fails at its default budget, while
// RRTC to the goal configuration solves it.
//
// Usage: task_space_rrtc_cage [output_prefix [max_iterations]]
// Writes <output_prefix>.json (problem, settings, result) and <output_prefix>.bin (every
// sample, extension and goal-directed step the planner took); scripts/task_space_trace_viz.py
// replays them in a viser visualizer. Default prefix: task_space_trace.
//
// Build (from the repository root; adjust the Eigen include path if needed):
//   g++ -std=c++17 -O2 -mavx2 -march=native -ffp-contract=fast -Isrc/impl \
//       -I<simdxorshift include> -isystem <pdqsort include> -isystem /usr/include/eigen3 \
//       scripts/cpp/task_space_rrtc_cage.cc -o task_space_rrtc_cage
// or configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_task_space_rrtc_cage target.

#include <array>
#include <cstdlib>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/planning/planners/task_space_trace.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using TaskSpaceConstraint = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;

// ---- problem -----------------------------------------------------------------------------

static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {
    -0.5545218656333819, 0.4202507223196937, 0.3286814744796756, -1.977673518937082,
    2.8973, 2.341192360593145, -2.31787312121598};

struct Box
{
    std::array<float, 3> center;
    std::array<float, 3> half_extents;
};

// All boxes share this orientation (Euler XYZ), about each box's own center.
static constexpr std::array<float, 3> box_euler_xyz = {0.F, 0.F, -0.07994997885362973F};

static const std::vector<Box> problem = {
    {{0.7444402652852363F, -0.1411031622851692F, 0.2599367587044526F}, {0.035F, 0.035F, 0.035F}},    // Cube1
    {{0.7444402652852363F, -0.1411031622851692F, 0.1799367587044526F}, {0.35F, 0.35F, 0.02F}},       // base
    {{1.093322261153294F, -0.1690558537319761F, 0.5399367587044527F}, {0.02F, 0.35F, 0.35F}},        // side_back
    {{0.7942805504092445F, -0.1450964039204273F, 0.8899367587044525F}, {0.325F, 0.325F, 0.02F}},     // side_cap
    {{0.3955582694171783F, -0.1131504708383623F, 0.3399367587044526F}, {0.02F, 0.35F, 0.02F}},       // side_frontA
    {{0.3955582694171783F, -0.1131504708383623F, 0.6399367587044525F}, {0.02F, 0.35F, 0.02F}},       // side_frontB
    {{0.7164875738384294F, -0.4899851581532272F, 0.5399367587044527F}, {0.35F, 0.02F, 0.35F}},       // side_left
    {{0.7723929567320432F, 0.2077788335828888F, 0.5399367587044527F}, {0.35F, 0.02F, 0.35F}},        // side_right
};

// ---- knobs to play with ------------------------------------------------------------------

static constexpr bool run_reference_rrtc = true;  // also solve to the goal *configuration* with RRTC
static constexpr bool print_path = true;

static auto task_space_settings() -> vamp::planning::TaskSpaceRRTCSettings
{
    vamp::planning::TaskSpaceRRTCSettings settings;
    // settings.range = 0.5;
    settings.max_greedy_steps = 20;
    settings.max_iterations = 1000000;
    settings.max_samples = 1000000;
    return settings;
}

static auto constraint_settings() -> vamp::planning::constraint::ConstraintSettings
{
    vamp::planning::constraint::ConstraintSettings settings;  // LM projection, tolerance 1e-6
    // settings.method = vamp::planning::constraint::ProjMethod::BoundedLS;
    return settings;
}

// ---- helpers -----------------------------------------------------------------------------

static auto print_configuration(const Robot::Configuration &c) -> void
{
    const auto array = c.to_array();
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        std::cout << array[i] << ", ";
    }

    std::cout << std::endl;
}

// End-effector pose of `q` as (qw, qx, qy, qz, x, y, z).
static auto pose_of(const Robot::ConfigurationArray &q) -> TaskSpaceConstraint::Transform
{
    const Eigen::Matrix4f pose = Robot::eefk(q).matrix();
    const Eigen::Quaternionf quaternion(Eigen::Matrix3f(pose.block<3, 3>(0, 0)));
    return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z(), pose(0, 3), pose(1, 3), pose(2, 3)};
}

// Exact TSR at the end-effector pose of `q`: identity offset, zero-width bounds.
static auto make_goal_region(const Robot::ConfigurationArray &q)
    -> vamp::planning::constraint::ConstraintSet<Robot, rake>
{
    const TaskSpaceConstraint::Transform eef_to_offset{1, 0, 0, 0, 0, 0, 0};
    const TaskSpaceConstraint::Transform world_to_reference = pose_of(q);
    const TaskSpaceConstraint::Bound zero{};

    std::vector<std::shared_ptr<const vamp::planning::constraint::Constraint<Robot, rake>>> constraints{
        std::make_shared<TaskSpaceConstraint>(
            std::array<TaskSpaceConstraint::Transform, 1>{eef_to_offset},
            std::array<TaskSpaceConstraint::Transform, 1>{world_to_reference},
            std::array<TaskSpaceConstraint::Bound, 1>{zero},
            std::array<TaskSpaceConstraint::Bound, 1>{zero})};

    return vamp::planning::constraint::ConstraintSet<Robot, rake>(std::move(constraints), constraint_settings());
}

template <typename Array>
static auto json_array(std::ostream &out, const Array &values) -> void
{
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        out << (i ? ", " : "") << values[i];
    }

    out << "]";
}

static auto write_problem_json(
    const std::string &path,
    const vamp::planning::TaskSpaceRRTCSettings &settings,
    const vamp::planning::PlanningResult<Robot> &result,
    std::size_t trace_entries) -> void
{
    std::ofstream out(path);
    out << std::setprecision(9);
    out << "{\n  \"robot\": \"panda\",\n  \"dimension\": " << Robot::dimension << ",\n  \"start\": ";
    json_array(out, start);
    out << ",\n  \"goal_configuration\": ";
    json_array(out, goal);
    out << ",\n  \"goal_pose_wxyz_xyz\": ";
    json_array(out, pose_of(goal));
    out << ",\n  \"boxes\": [\n";
    for (std::size_t i = 0; i < problem.size(); ++i)
    {
        out << "    {\"center\": ";
        json_array(out, problem[i].center);
        out << ", \"half_extents\": ";
        json_array(out, problem[i].half_extents);
        out << ", \"euler_xyz\": ";
        json_array(out, box_euler_xyz);
        out << "}" << (i + 1 < problem.size() ? "," : "") << "\n";
    }

    out << "  ],\n  \"settings\": {\"range\": " << settings.range
        << ", \"max_greedy_steps\": " << settings.max_greedy_steps
        << ", \"max_iterations\": " << settings.max_iterations
        << ", \"max_samples\": " << settings.max_samples << "},\n  \"result\": {\"solved\": "
        << (result.solved ? "true" : "false") << ", \"iterations\": " << result.iterations
        << ", \"nodes\": " << (result.size.empty() ? 0 : result.size[0])
        << ", \"milliseconds\": " << result.nanoseconds / 1e6 << ", \"trace_entries\": " << trace_entries
        << "}\n}\n";
}

auto main(int argc, char **argv) -> int
{
    const std::string prefix = (argc > 1) ? argv[1] : "task_space_trace";

    EnvironmentInput environment;
    for (const auto &box : problem)
    {
        environment.add_cuboid(
            vamp::collision::factory::cuboid::array(box.center, box_euler_xyz, box.half_extents));
    }

    const auto env_v = EnvironmentVector(environment);

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    const auto region = make_goal_region(goal);
    std::cout << "goal configuration satisfies its own TSR: " << std::boolalpha
              << region.satisfied(Robot::Configuration(goal)) << std::endl;

    auto settings = task_space_settings();
    if (argc > 2)
    {
        settings.max_iterations = std::strtoull(argv[2], nullptr, 10);
    }

    vamp::planning::TaskSpaceTrace<Robot> trace;
    const auto result = TaskSpaceRRTC::solve(
        Robot::Configuration(start),
        region,
        env_v,
        settings,
        rng,
        vamp::planning::UnconstrainedLocalPlanner<Robot, rake, Robot::resolution>(),
        &trace);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "TaskSpaceRRTC: solved=" << result.solved << " iterations=" << result.iterations
              << " nodes=" << (result.size.empty() ? 0 : result.size[0]) << " time=" << result.nanoseconds / 1e6
              << " ms cost=" << result.cost << std::endl;

    if (result.solved)
    {
        std::cout << "final node satisfies the TSR: " << region.satisfied(result.path[result.path.size() - 1])
                  << std::endl;
        if (print_path)
        {
            for (const auto &config : result.path)
            {
                print_configuration(config);
            }
        }
    }

    if (trace.save(prefix + ".bin"))
    {
        write_problem_json(prefix + ".json", settings, result, trace.entries.size());
        std::cout << "wrote " << trace.entries.size() << " trace entries to " << prefix << ".bin / .json"
                  << std::endl;
    }
    else
    {
        std::cerr << "could not write " << prefix << ".bin" << std::endl;
    }

    if (run_reference_rrtc)
    {
        vamp::planning::RRTCSettings rrtc_settings;
        rrtc_settings.range = settings.range;
        rng->reset();
        const auto reference =
            RRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, rng);
        std::cout << "reference RRTC (knows goal configuration): solved=" << reference.solved
                  << " iterations=" << reference.iterations << " time=" << reference.nanoseconds / 1e6 << " ms"
                  << std::endl;
    }

    return 0;
}
