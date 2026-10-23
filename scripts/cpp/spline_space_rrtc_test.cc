// Plans along a spline in (q, t) with TaskSpaceRRTC over Robot::SplineSpace, the space cricket generates with
// "spline_constraint": true: a state is the joints and the curve parameter t,
// t is sampled with the joints, and a projection onto the curve takes t as given and solves for the joints alone (the
// SplineConstraint's fixed-parameter mode), where spline_task_space_rrtc_test plans in q and finds the closest point of
// the curve on every projection. The nearest-neighbor tree ignores t.
//
// Every state of the tree is on the curve at its own t; a random sample is projected onto the curve at its t before
// it is used (settings.project_samples); the goal is the end of the curve, a position-only task-space region at P(1),
// reached by goal-directed steps that advance t by settings.parameter_step while the joints follow the region.
//
// The spline is fitted as in spline_task_space_rrtc_test (the first 20 end-effector positions of the first trajectory
// of a file written by evaluate_mbm.py --save_eef, default /tmp/eef_bookshelf_small.json, else an arc near the Panda's
// start end-effector); the environment is empty. The start is that trajectory's configuration 0 at its closest t.
//
// What is checked: the planner solves; every state of the path is on the curve at its own t; the path ends at P(1) and
// satisfies the goal region. The path is written to <prefix>_path.json in the format of cricket's
// tests/view_spline.py.
//
// Usage: vamp_spline_space_rrtc_test [output_prefix [trajectory.json [max_iterations]]]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_spline_space_rrtc_test target. Exits 1 if a check
// fails.

#include "spline_test_utils.hh"

#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/spline.hh>

using Space = Robot::SplineSpace;
using State = Space::State;
using TaskSpaceConstraint = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution, Space>;
using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution, Space>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

int main(int argc, char **argv)
{
    const std::string prefix = argc > 1 ? argv[1] : "spline_space_rrtc";
    const std::string trajectory_path = argc > 2 ? argv[2] : "/tmp/eef_bookshelf_small.json";

    const SplineProblem problem = make_spline_problem(trajectory_path, 0);
    const HostSpline &spline = problem.spline;
    const SplineData &data = problem.data;
    const auto constraint =
        std::make_shared<SplineConstraint>(data.control_points, data.knots, data.waypoints, data.parameters);

    // the manifold: the curve
    ConstraintSettings manifold_settings;
    manifold_settings.method = ProjMethod::InnerLM;
    manifold_settings.tolerance = 1e-8F;
    manifold_settings.max_iterations = 50;
    const ConstraintSet manifold(std::vector<ConstraintSet::Ptr>{constraint}, manifold_settings);

    // the goal: the end-effector at the end of the curve, any orientation
    const Eigen::Vector3d goal_position = spline.value(1.0);
    const TaskSpaceConstraint::Transform eef_to_offset{1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};
    const TaskSpaceConstraint::Transform world_to_reference{
        1.F,
        0.F,
        0.F,
        0.F,
        static_cast<float>(goal_position[0]),
        static_cast<float>(goal_position[1]),
        static_cast<float>(goal_position[2])};
    const TaskSpaceConstraint::Bound lower{0.F, 0.F, 0.F, -3.2F, -3.2F, -3.2F};
    const TaskSpaceConstraint::Bound upper{0.F, 0.F, 0.F, 3.2F, 3.2F, 3.2F};

    const auto goal_constraint = std::make_shared<TaskSpaceConstraint>(
        std::array<TaskSpaceConstraint::Transform, 1>{eef_to_offset},
        std::array<TaskSpaceConstraint::Transform, 1>{world_to_reference},
        std::array<TaskSpaceConstraint::Bound, 1>{lower},
        std::array<TaskSpaceConstraint::Bound, 1>{upper});
    ConstraintSettings region_settings;  // LM projection, tolerance 1e-6
    const ConstraintSet region(std::vector<ConstraintSet::Ptr>{goal_constraint}, region_settings);

    // the start has to be on the manifold; its t is where the curve is closest to it
    Configuration start_q(problem.start);
    CHECK(manifold.project(start_q));
    CHECK(manifold.satisfied(start_q));
    constraint->closest_squared_error(broadcast_block(start_q));
    const float start_t = constraint->current_t()[{0, 0}];
    const State start = Space::make_state(start_q, start_t);
    std::printf(
        "start: on the curve (distance %.2e m) at t %.4f; goal: P(1) = (%.3f, %.3f, %.3f)\n",
        std::sqrt(host_closest(spline, end_effector(start_q)).second),
        start_t,
        goal_position[0],
        goal_position[1],
        goal_position[2]);

    EnvironmentInput environment;
    environment.sort();
    const EnvironmentVector env(environment);

    vamp::planning::TaskSpaceRRTCSettings settings;
    settings.range = 0.3F;
    settings.max_greedy_steps = 20;
    settings.max_iterations = argc > 3 ? std::strtoull(argv[3], nullptr, 10) : 20000;
    settings.max_samples = 20000;
    settings.project_samples = true;
    settings.parameter_step = 0.05F;

    auto rng = std::make_shared<vamp::rng::SplineRNG<Robot>>(std::make_shared<vamp::rng::Halton<Robot>>());
    const auto result = TaskSpaceRRTC::solve<LocalPlanner>(start, region, env, settings, rng, LocalPlanner(manifold));

    // the planner held t fixed on the shared constraint: back to solving for it, for the one-off queries below
    manifold.clear_parameter();

    std::printf(
        "\nTaskSpaceRRTC over SplineSpace: solved=%d, iterations %zu, %zu nodes, %.1f ms, cost %.3f\n",
        result.solved,
        static_cast<std::size_t>(result.iterations),
        result.size.empty() ? std::size_t{0} : static_cast<std::size_t>(result.size[0]),
        result.nanoseconds / 1e6,
        result.cost);
    CHECK(result.solved);

    if (result.solved)
    {
        CHECK(result.path.size() >= 2);

        // the path: every state on the curve at its own t, ending at the goal
        std::vector<TrailEntry> trail;
        double worst_distance = 0.0;
        double worst_t_distance = 0.0;
        for (const auto &state : result.path)
        {
            const Configuration q = Space::to_ambient(state);
            const double t = Space::parameter(state);

            worst_distance = std::max(worst_distance, std::sqrt(host_closest(spline, end_effector(q)).second));
            worst_t_distance = std::max(worst_t_distance, (end_effector(q) - spline.value(t)).norm());

            TrailEntry entry;
            entry.q = values_of(q);
            entry.t = t;
            entry.end_effector = end_effector(q);
            entry.curve_point = spline.value(t);
            entry.tangent = (spline.value(t + 1e-5) - spline.value(t - 1e-5)) / 2e-5;
            entry.error_norm = (entry.end_effector - entry.curve_point).norm();
            trail.push_back(entry);
        }

        const Configuration last = Space::to_ambient(result.path[result.path.size() - 1]);
        const double final_distance = (end_effector(last) - goal_position).norm();
        std::printf(
            "path: %zu states, t %.4f -> %.4f, worst distance to the curve %.2e m (to P(t) of its own t: %.2e m), "
            "final distance to P(1) %.2e m\n",
            result.path.size(),
            trail.front().t,
            trail.back().t,
            worst_distance,
            worst_t_distance,
            final_distance);

        CHECK(worst_distance < 1e-3);    // every state on the curve, to a millimetre
        CHECK(worst_t_distance < 1e-3);  // and at the point its t names
        CHECK(final_distance < 2e-3);
        CHECK(region.satisfied(last));
        CHECK((Space::to_ambient(result.path[0]) - start_q).l2_norm() < 1e-3F);

        const std::string path = prefix + "_path.json";
        write_dump(path, "TaskSpaceRRTC over SplineSpace path", spline, data.waypoints, data.parameters, trail);
        std::printf("written to %s\n", path.c_str());
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\nspline_space_rrtc_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\nspline_space_rrtc_test: all checks passed\n");
    return 0;
}
