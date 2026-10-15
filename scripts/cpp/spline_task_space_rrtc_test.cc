// Plans along a spline with TaskSpaceRRTC: every state of the tree stays on the curve (ConstrainedLocalPlanner over a
// ConstraintSet holding the SplineConstraint), and the goal is the end of the curve, written as a position-only
// task-space region at P(1). No goal configuration is given: exploration steps toward random samples are projected
// onto the curve, and after each the goal-directed walk takes LM steps toward P(1), each handed to the local planner
// as a steer target so it is projected onto the curve too.
//
// The spline is fitted through the first 20 end-effector positions of the first trajectory in a file written by
// evaluate_mbm.py --save_eef (default /tmp/eef_bookshelf_small.json), starting from that trajectory's configuration 0
// (on the curve); without the file, an arc near the Panda's start end-effector. The environment is empty, so only
// the robot's self-collision is checked.
//
// What is checked: the planner solves; every state of the path is on the curve; the path ends at P(1) and satisfies
// the goal region. The path is written to <prefix>_path.json in the format of cricket's tests/view_spline.py:
//   python3 cricket/tests/view_spline.py spline_task_space_rrtc_path.json
//
// Usage: vamp_spline_task_space_rrtc_test [output_prefix [trajectory.json [max_iterations]]]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_spline_task_space_rrtc_test target. Exits 1 if a
// check fails.

#include "spline_test_utils.hh"

#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/task_space_constraint.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/random/halton.hh>

using TaskSpaceConstraint = vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution>;
using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

int main(int argc, char **argv)
{
    const std::string prefix = argc > 1 ? argv[1] : "spline_task_space_rrtc";
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

    // the goal: the end-effector at the end of the curve, any orientation (the rotation bounds are wider than the
    // widest rotation error, so only the position is constrained)
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

    // the start has to be on the manifold
    Configuration start(problem.start);
    CHECK(manifold.project(start));
    CHECK(manifold.satisfied(start));
    std::printf(
        "start: on the curve (distance %.2e m), closest t %.4f; goal: P(1) = (%.3f, %.3f, %.3f)\n",
        std::sqrt(host_closest(spline, end_effector(start)).second),
        host_closest(spline, end_effector(start)).first,
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

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
    const auto result = TaskSpaceRRTC::solve<LocalPlanner>(start, region, env, settings, rng, LocalPlanner(manifold));

    std::printf(
        "\nTaskSpaceRRTC: solved=%d, iterations %zu, %zu nodes, %.1f ms, cost %.3f\n",
        result.solved,
        static_cast<std::size_t>(result.iterations),
        result.size.empty() ? std::size_t{0} : static_cast<std::size_t>(result.size[0]),
        result.nanoseconds / 1e6,
        result.cost);
    CHECK(result.solved);

    if (result.solved)
    {
        CHECK(result.path.size() >= 2);

        // the path: every state on the curve, ending at the goal
        std::vector<TrailEntry> trail;
        double worst_distance = 0.0;
        for (const auto &configuration : result.path)
        {
            worst_distance = std::max(worst_distance, std::sqrt(host_closest(spline, end_effector(configuration)).second));

            const Block block = broadcast_block(configuration);
            const double error = std::sqrt(constraint->closest_squared_error(block)[{0, 0}]);
            const double t = constraint->current_t()[{0, 0}];

            TrailEntry entry;
            entry.q = values_of(configuration);
            entry.t = t;
            entry.end_effector = end_effector(configuration);
            entry.curve_point = spline.value(t);
            entry.tangent = (spline.value(t + 1e-5) - spline.value(t - 1e-5)) / 2e-5;
            entry.error_norm = error;
            trail.push_back(entry);
        }

        const Configuration &last = result.path[result.path.size() - 1];
        const double final_distance = (end_effector(last) - goal_position).norm();
        std::printf(
            "path: %zu states, t %.4f -> %.4f, worst distance to the curve %.2e m, final distance to P(1) %.2e m\n",
            result.path.size(),
            trail.front().t,
            trail.back().t,
            worst_distance,
            final_distance);

        CHECK(worst_distance < 1e-3);  // every state on the curve, to a millimetre
        CHECK(final_distance < 2e-3);
        CHECK(region.satisfied(last));
        CHECK((result.path[0] - start).l2_norm() < 1e-3F);

        const std::string path = prefix + "_path.json";
        write_dump(path, "TaskSpaceRRTC path", spline, data.waypoints, data.parameters, trail);
        std::printf("written to %s\n", path.c_str());
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\nspline_task_space_rrtc_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\nspline_task_space_rrtc_test: all checks passed\n");
    return 0;
}
