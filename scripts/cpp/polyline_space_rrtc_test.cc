// Plans along a polyline in (q, t): spline_space_rrtc_test with the PolylineConstraint, the straight-line
// interpolation of the points, as the manifold in place of the B-spline. The polyline joins the same 20 waypoints (the
// first 20 end-effector positions of the first trajectory of a file written by evaluate_mbm.py --save_eef, default
// /tmp/eef_bookshelf_small.json, else an arc near the Panda's start end-effector); t is the chord-length parameter.
//
// What is checked: the generated position-target kernel's error against the host evaluation of the curve, at random
// configurations and at parameters that include the corners and the ends; the start is on the curve at its closest t;
// the planner solves; every state of the path is on the curve at its own t; the path ends at P(1) and satisfies the goal
// region.
//
// With the argument "slide" the constraint slides: the t a state carries only says which segment it is on, the
// end-effector slides along the segment during every projection, and the state takes the t the projection leaves it at
// (PolylineConstraint's slide mode); the goal is still the end of the curve, at t = 1.
//
// Usage: vamp_polyline_space_rrtc_test [trajectory.json [max_iterations [slide]]]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_polyline_space_rrtc_test target. Needs a Panda
// header generated with "spline_constraint": true and "position_target_constraint": true. Exits 1 if a check fails.

#include "spline_test_utils.hh"

#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/curve_constraint.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/spline.hh>

using Space = Robot::SplineSpace;
using State = Space::State;
using PolylineConstraint = vamp::planning::constraint::PolylineConstraint<Robot, rake>;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution, Space>;
using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution, Space>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

int main(int argc, char **argv)
{
    const std::string trajectory_path = argc > 1 ? argv[1] : "/tmp/eef_bookshelf_small.json";

    const SplineProblem problem = make_spline_problem(trajectory_path, 0);
    const SplineData &data = problem.data;

    const bool slide = argc > 3 and std::string(argv[3]) == "slide";
    const std::vector<PolylineConstraint::Point> waypoints(data.waypoints.begin(), data.waypoints.end());
    const auto curve = std::make_shared<PolylineConstraint>(waypoints, slide);
    std::printf("projection: %s\n", slide ? "sliding along the segment of t" : "at the t of the state");

    // 1. the generated kernel against the host curve (at a given t: the fixed constraint)
    {
        const auto fixed_curve = std::make_shared<PolylineConstraint>(waypoints);
        vamp::rng::Halton<Robot> sampler;
        double worst = 0.0;
        for (int sample = 0; sample < 64; ++sample)
        {
            const Configuration q = sampler.next();
            const double t = (sample < 4) ? 0.25 * sample : static_cast<double>(sample % 17) / 16.0;

            fixed_curve->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t)));
            const double kernel = fixed_curve->squared_error(broadcast_block(q))[{0, 0}];
            const double host = (end_effector(q) - fixed_curve->value(t)).squaredNorm();
            worst = std::max(worst, std::abs(kernel - host) / std::max(1.0, host));
        }

        std::printf("kernel vs host curve: worst relative difference of the squared error %.2e\n", worst);
        CHECK(worst < 1e-3);
    }

    // the manifold: the curve, at the parameter of each state
    ConstraintSettings manifold_settings;
    manifold_settings.method = ProjMethod::InnerLM;
    manifold_settings.tolerance = 1e-8F;
    manifold_settings.max_iterations = 50;
    const ConstraintSet manifold(std::vector<ConstraintSet::Ptr>{curve}, manifold_settings);

    // the goal: the end of the curve
    const Eigen::Vector3d goal_position = curve->value(1.0);
    // the goal region: the curve's own constraint held at t = 1 (a copy, so the planner's parameter on the manifold's
    // constraint does not move it): a goal-directed step is a fixed-parameter step toward P(1)
    const auto goal_constraint = std::make_shared<PolylineConstraint>(*curve);
    goal_constraint->set_slide(false);  // the end of the curve, not of its last segment
    goal_constraint->set_parameter(vamp::FloatVector<rake, 1>::fill(1.F));
    ConstraintSettings region_settings;  // LM projection, tolerance 1e-6
    const ConstraintSet region(std::vector<ConstraintSet::Ptr>{goal_constraint}, region_settings);

    // 2. the start: on the curve at its closest t
    const Configuration start_q(problem.start);
    const auto [start_t, start_distance2] = curve->closest_parameter(start_q);
    State start = Space::make_state(start_q, start_t);

    LocalPlanner local_planner(manifold);
    CHECK(local_planner.project(start));
    CHECK(local_planner.satisfied(start));
    std::printf(
        "start: closest t %.4f (squared distance %.2e m^2); goal: P(1) = (%.3f, %.3f, %.3f)\n",
        start_t,
        start_distance2,
        goal_position[0],
        goal_position[1],
        goal_position[2]);

    EnvironmentInput environment;
    environment.sort();
    const EnvironmentVector env(environment);

    vamp::planning::TaskSpaceRRTCSettings settings;
    settings.range = 0.3F;
    settings.max_greedy_steps = 20;
    settings.max_iterations = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 20000;
    settings.max_samples = 20000;
    settings.project_samples = true;
    settings.parameter_step = 0.05F;

    auto rng = std::make_shared<vamp::rng::SplineRNG<Robot>>(std::make_shared<vamp::rng::Halton<Robot>>());
    const auto result = TaskSpaceRRTC::solve<LocalPlanner>(start, region, env, settings, rng, local_planner);

    std::printf(
        "\nTaskSpaceRRTC over SplineSpace, polyline: solved=%d, iterations %zu, %zu nodes, %.1f ms, cost %.3f\n",
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
        double worst_distance = 0.0;
        for (const auto &state : result.path)
        {
            const Configuration q = Space::to_ambient(state);
            worst_distance =
                std::max(worst_distance, (end_effector(q) - curve->value(Space::parameter(state))).norm());
        }

        const Configuration last = Space::to_ambient(result.path[result.path.size() - 1]);
        const double final_distance = (end_effector(last) - goal_position).norm();
        std::printf(
            "path: %zu states, t %.4f -> %.4f, worst distance to P(t) of its own t %.2e m, final distance to P(1) "
            "%.2e m\n",
            result.path.size(),
            Space::parameter(result.path[0]),
            Space::parameter(result.path[result.path.size() - 1]),
            worst_distance,
            final_distance);

        CHECK(worst_distance < 1e-3);
        CHECK(final_distance < 2e-3);
        CHECK(region.satisfied(last));
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\npolyline_space_rrtc_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\npolyline_space_rrtc_test: all checks passed\n");
    return 0;
}
