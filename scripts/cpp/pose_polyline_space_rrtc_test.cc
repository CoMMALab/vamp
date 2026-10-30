// Plans along a pose polyline in (q, t): the straight-line interpolation in R^3 x SO(3), the end-effector's position
// along the polyline of the keyframe positions and its rotation along the slerp between the keyframe rotations. The
// keyframe positions are the first 20 end-effector positions of the first trajectory of a file written by
// evaluate_mbm.py --save_eef (default /tmp/eef_bookshelf_small.json, else an arc near the Panda's start end-effector).
// The rotations start at the start configuration's end-effector rotation and turn about the world z axis, in equal
// steps per keyframe, by a total of 60 degrees (argument 3, in degrees). t is the chord-length parameter in the
// metric sqrt(|dp|^2 + w^2 theta^2), w = 0.1 m/rad. The goal is the last keyframe's pose, the curve held at t = 1.
//
// What is checked: the constraint's error (the generated TSR kernel with the rotation rows weighted) against the host
// evaluation of the curve, at random configurations and at parameters that include the keyframes and the ends; the
// start is on the curve at its closest t; the planner solves; every state of the path is on the curve, in position and
// in rotation, at its own t; the path ends at the last keyframe's pose and satisfies the goal region.
//
// With the argument "slide" the constraint slides: the t a state carries only says which segment it is on, the
// end-effector's pose slides along the segment during every projection, and the state takes the t the projection
// leaves it at (PosePolylineConstraint's slide mode); the goal is still the last keyframe's pose, at t = 1.
//
// Usage: vamp_pose_polyline_space_rrtc_test [trajectory.json [max_iterations [turn_degrees [slide]]]]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_pose_polyline_space_rrtc_test target. Needs a
// Panda header generated with "constraints": true (the TSR kernel), "spline_constraint": true (SplineSpace) and
// "pose_slide_constraint": true (the solvers of the sliding projection). Exits 1 if a check fails.

#include "spline_test_utils.hh"

#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/pose_curve_constraint.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/spline.hh>

using Space = Robot::SplineSpace;
using State = Space::State;
using PosePolylineConstraint = vamp::planning::constraint::PosePolylineConstraint<Robot, rake>;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution, Space>;
using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution, Space>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

static constexpr double rotation_weight = 0.1;

// The end-effector's rotation at q
static auto end_effector_rotation(const Configuration &q) -> Eigen::Quaterniond
{
    std::array<float, Robot::dimension> values;
    const auto array = q.to_array();
    for (auto j = 0U; j < Robot::dimension; ++j)
    {
        values[j] = array[j];
    }

    return Eigen::Quaterniond(Robot::eefk(values).rotation().cast<double>());
}

// The angle of the rotation between two rotations
static auto rotation_angle(const Eigen::Quaterniond &a, const Eigen::Quaterniond &b) -> double
{
    return 2.0 * std::acos(std::min(1.0, std::abs(a.dot(b))));
}

int main(int argc, char **argv)
{
    const std::string trajectory_path = argc > 1 ? argv[1] : "/tmp/eef_bookshelf_small.json";
    const double turn = (argc > 3 ? std::strtod(argv[3], nullptr) : 60.0) * M_PI / 180.0;

    const SplineProblem problem = make_spline_problem(trajectory_path, 0);
    const SplineData &data = problem.data;

    // keyframe rotations: the start's, turned about the world z axis a little more at every keyframe
    const Eigen::Quaterniond start_rotation = end_effector_rotation(Configuration(problem.start));
    std::vector<PosePolylineConstraint::Quaternion> rotations;
    for (std::size_t i = 0; i < data.waypoints.size(); ++i)
    {
        const double angle = turn * static_cast<double>(i) / static_cast<double>(data.waypoints.size() - 1);
        const Eigen::Quaterniond rotation =
            Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())) * start_rotation;
        rotations.push_back(
            {static_cast<float>(rotation.w()),
             static_cast<float>(rotation.x()),
             static_cast<float>(rotation.y()),
             static_cast<float>(rotation.z())});
    }

    const bool slide = argc > 4 and std::string(argv[4]) == "slide";
    const std::vector<PosePolylineConstraint::Point> waypoints(data.waypoints.begin(), data.waypoints.end());
    const auto curve = std::make_shared<PosePolylineConstraint>(
        waypoints, rotations, static_cast<float>(rotation_weight), slide);
    std::printf("projection: %s\n", slide ? "sliding along the segment of t" : "at the t of the state");

    // 1. the constraint's error against the host curve (at a given t: the fixed constraint): |dp|^2 + w^2 theta^2
    {
        const auto fixed_curve = std::make_shared<PosePolylineConstraint>(
            waypoints, rotations, static_cast<float>(rotation_weight));
        vamp::rng::Halton<Robot> sampler;
        double worst = 0.0;
        int checked = 0;
        for (int sample = 0; sample < 128; ++sample)
        {
            const Configuration q = sampler.next();
            const double t = (sample < 4) ? 0.25 * sample : static_cast<double>(sample % 17) / 16.0;
            const auto pose = curve->value(t);

            // the log map of the kernel is only the rotation angle below pi
            const double theta = rotation_angle(end_effector_rotation(q), pose.rotation);
            if (theta > 2.5)
            {
                continue;
            }

            fixed_curve->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t)));
            const double kernel = fixed_curve->squared_error(broadcast_block(q))[{0, 0}];
            const double host = (end_effector(q) - pose.position).squaredNorm() +
                                rotation_weight * rotation_weight * theta * theta;
            worst = std::max(worst, std::abs(kernel - host) / std::max(1.0, host));
            ++checked;
        }

        std::printf(
            "constraint error vs host curve: %d configurations, worst relative difference of the squared error "
            "%.2e\n",
            checked,
            worst);
        CHECK(checked > 16);
        CHECK(worst < 1e-3);
    }

    // the manifold: the curve, at the parameter of each state
    ConstraintSettings manifold_settings;
    manifold_settings.method = ProjMethod::InnerLM;
    manifold_settings.tolerance = 1e-8F;
    manifold_settings.max_iterations = 50;
    const ConstraintSet manifold(std::vector<ConstraintSet::Ptr>{curve}, manifold_settings);

    // the goal: the end of the curve, the last keyframe's pose
    const auto goal_pose = curve->value(1.0);
    // the goal region: the curve's own constraint held at t = 1 (a copy, so the planner's parameter on the manifold's
    // constraint does not move it): a goal-directed step is a fixed-parameter step toward P(1)
    const auto goal_constraint = std::make_shared<PosePolylineConstraint>(*curve);
    goal_constraint->set_slide(false);  // the end of the curve, not of its last segment
    goal_constraint->set_parameter(vamp::FloatVector<rake, 1>::fill(1.F));
    ConstraintSettings region_settings;  // LM projection, tolerance 1e-6
    const ConstraintSet region(std::vector<ConstraintSet::Ptr>{goal_constraint}, region_settings);

    // 2. the start: on the curve at its closest t
    const Configuration start_q(problem.start);
    const auto [start_t, start_distance2] = curve->closest_parameter(start_q);
    State start = Space::make_state(start_q, start_t);

    // 2b. the sliding constraint's column of t against the finite difference of the fixed constraint's error with t.
    // At the start (on the curve) all six rows are exact; away from the curve the translation rows still are, the
    // rotation rows being a first-order approximation there
    {
        const auto sliding = std::make_shared<PosePolylineConstraint>(
            waypoints, rotations, static_cast<float>(rotation_weight), true);
        const auto fixed = std::make_shared<PosePolylineConstraint>(
            waypoints, rotations, static_cast<float>(rotation_weight));

        // the column of the sliding constraint at t, against the difference of the fixed constraint's error between
        // t_a and t_b (two parameters in one segment), in the first `rows` rows
        const auto column_and_difference =
            [&](const Configuration &q, double t, double t_a, double t_b, std::size_t rows)
        {
            const auto block = broadcast_block(q);

            float column[6];
            sliding->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t)));
            sliding->squared_error(block);
            sliding->parameter_jacobian(0, column);

            float error_a[6];
            float error_b[6];
            float jacobian[6 * Robot::dimension];
            fixed->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t_a)));
            fixed->evaluate_error_jacobian(block);
            fixed->extract_error_jacobian(0, error_a, jacobian);
            fixed->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t_b)));
            fixed->evaluate_error_jacobian(block);
            fixed->extract_error_jacobian(0, error_b, jacobian);

            double worst = 0.0;
            for (std::size_t i = 0; i < rows; ++i)
            {
                worst = std::max(worst, std::abs(column[i] - (error_b[i] - error_a[i]) / (t_b - t_a)));
            }

            return worst;
        };

        // off the curve: central differences about the middle of a segment, translation rows
        double worst_off_curve = 0.0;
        vamp::rng::Halton<Robot> sampler;
        const double t_middle = 0.5 * (curve->curve().segment_begin(9) + curve->curve().segment_end(9));
        for (int sample = 0; sample < 32; ++sample)
        {
            const Configuration q = sampler.next();
            if (rotation_angle(end_effector_rotation(q), curve->value(t_middle).rotation) > 2.0)
            {
                continue;
            }

            worst_off_curve =
                std::max(worst_off_curve, column_and_difference(q, t_middle, t_middle - 1e-3, t_middle + 1e-3, 3));
        }

        // at the start, a difference from its t, inside its segment
        const bool forward = curve->curve().segment_of(start_t + 1e-3) == curve->curve().segment_of(start_t);
        const double worst_on_curve = column_and_difference(
            start_q, start_t, forward ? start_t : start_t - 1e-3, forward ? start_t + 1e-3 : start_t, 6);

        std::printf(
            "sliding column of t vs finite difference: translation rows off the curve %.2e, all rows at the start "
            "%.2e\n",
            worst_off_curve,
            worst_on_curve);
        CHECK(worst_off_curve < 5e-2);
        CHECK(worst_on_curve < 5e-2);
    }

    LocalPlanner local_planner(manifold);
    CHECK(local_planner.project(start));
    CHECK(local_planner.satisfied(start));
    std::printf(
        "start: closest t %.4f (squared distance %.2e); goal: P(1) = (%.3f, %.3f, %.3f), turned %.0f degrees\n",
        start_t,
        start_distance2,
        goal_pose.position[0],
        goal_pose.position[1],
        goal_pose.position[2],
        turn * 180.0 / M_PI);

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
        "\nTaskSpaceRRTC over SplineSpace, pose polyline: solved=%d, iterations %zu, %zu nodes, %.1f ms, cost %.3f\n",
        result.solved,
        static_cast<std::size_t>(result.iterations),
        result.size.empty() ? std::size_t{0} : static_cast<std::size_t>(result.size[0]),
        result.nanoseconds / 1e6,
        result.cost);
    CHECK(result.solved);

    if (result.solved)
    {
        CHECK(result.path.size() >= 2);

        // the path: every state on the curve at its own t, in position and in rotation, ending at the goal pose
        double worst_position = 0.0;
        double worst_rotation = 0.0;
        for (const auto &state : result.path)
        {
            const Configuration q = Space::to_ambient(state);
            const auto pose = curve->value(Space::parameter(state));
            worst_position = std::max(worst_position, (end_effector(q) - pose.position).norm());
            worst_rotation = std::max(worst_rotation, rotation_angle(end_effector_rotation(q), pose.rotation));
        }

        const Configuration last = Space::to_ambient(result.path[result.path.size() - 1]);
        const double final_position = (end_effector(last) - goal_pose.position).norm();
        const double final_rotation = rotation_angle(end_effector_rotation(last), goal_pose.rotation);
        std::printf(
            "path: %zu states, t %.4f -> %.4f, worst distance to the curve's pose at its own t %.2e m, %.2e rad; "
            "final distance to P(1) %.2e m, %.2e rad\n",
            result.path.size(),
            Space::parameter(result.path[0]),
            Space::parameter(result.path[result.path.size() - 1]),
            worst_position,
            worst_rotation,
            final_position,
            final_rotation);

        CHECK(worst_position < 1e-3);
        CHECK(worst_rotation < 1e-2);
        CHECK(final_position < 2e-3);
        CHECK(final_rotation < 2e-2);
        CHECK(region.satisfied(last));
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\npose_polyline_space_rrtc_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\npose_polyline_space_rrtc_test: all checks passed\n");
    return 0;
}
