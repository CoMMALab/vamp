// Drives ConstrainedLocalPlanner over a ConstraintSet holding the SplineConstraint, on its own, before any planner
// uses it: the extension step a tree planner takes (steer), and the validity of a local path (validate).
//
// The Panda and the host reference are those of spline_constraint_test (spline_test_utils.hh); the spline is
// fitted through the end-effector points of a planned trajectory (see below). The environment is empty, so only
// the robot's self-collision is checked.
//
// What is checked:
//   1. steer from a node on the curve toward several sample targets some distance away in joint space: most
//      extensions make progress (not Trapped); every waypoint of a returned chain is on the curve; the chain is
//      continuous; the endpoint does not stray further than a step from the node;
//   2. validate(node, endpoint) accepts the local path to a point the planner itself produced.
// Then it walks: repeated steer toward one far target, each step starting where the last ended, and writes the walk
// to <prefix>_walk.json in the format of cricket's tests/view_spline.py:
//   python3 cricket/tests/view_spline.py spline_local_planner_walk.json
//
// Usage: vamp_spline_local_planner_test [output_prefix [trajectory.json]]  (default trajectory:
//        /tmp/eef_bookshelf_small.json, written by evaluate_mbm.py --save_eef)
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_spline_local_planner_test target. Exits 1 if a
// check fails.

#include "spline_test_utils.hh"

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/local_planner.hh>

using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;
using SteerStatus = vamp::planning::SteerStatus;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

static const char *status_name(SteerStatus status)
{
    switch (status)
    {
        case SteerStatus::Rejected:
            return "Rejected";
        case SteerStatus::Trapped:
            return "Trapped";
        case SteerStatus::Advanced:
            return "Advanced";
        case SteerStatus::Reached:
            return "Reached";
    }

    return "?";
}

// a deterministic value in [-0.5, 0.5)
static auto next_offset(unsigned &state) -> float
{
    state = state * 1664525U + 1013904223U;
    return static_cast<float>(state >> 8) / 16777216.F - 0.5F;
}

// The distance from a configuration's end-effector to the host spline
static auto distance_to_host_curve(const HostSpline &spline, const Configuration &q) -> double
{
    return std::sqrt(host_closest(spline, end_effector(q)).second);
}

int main(int argc, char **argv)
{
    const std::string prefix = argc > 1 ? argv[1] : "spline_local_planner";
    const std::string trajectory_path = argc > 2 ? argv[2] : "/tmp/eef_bookshelf_small.json";

    // The spline is fitted through the first N end-effector positions of the first trajectory in the file written by
    // evaluate_mbm.py --save_eef, and the start is that trajectory's configuration 10, which is on the curve. Without
    // the file: an arc near the Panda's start end-effector.
    std::array<float, Robot::dimension> start_values = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
    HostSpline spline;
    SplineData data;
    if (const auto trajectory = load_trajectory(trajectory_path, 10))
    {
        start_values = trajectory->configuration;
        const auto [fitted, parameters] = fit_spline(trajectory->positions);
        spline = fitted;
        data = make_spline_data_for_fit(fitted, trajectory->positions, parameters);
        std::printf(
            "spline: fitted through the first %zu end-effector positions of %s, start: its configuration 10\n",
            N,
            trajectory_path.c_str());
    }
    else
    {
        spline = make_spline(end_effector(Configuration(start_values)));
        data = make_spline_data(spline);
        std::printf("spline: an arc near the start end-effector (no usable trajectory file at %s)\n", trajectory_path.c_str());
    }

    const Configuration start(start_values);
    const auto constraint =
        std::make_shared<SplineConstraint>(data.control_points, data.knots, data.waypoints, data.parameters);

    ConstraintSettings settings;
    settings.method = ProjMethod::InnerLM;
    settings.tolerance = 1e-8F;
    settings.max_iterations = 50;

    const ConstraintSet set(std::vector<ConstraintSet::Ptr>{constraint}, settings);
    const LocalPlanner local_planner(set);

    EnvironmentInput environment;
    environment.sort();
    const EnvironmentVector env(environment);

    // a node on the curve
    Configuration node = start;
    CHECK(set.project(node));
    CHECK(set.satisfied(node));
    std::printf(
        "node on the curve: distance to the host curve %.2e m, closest t %.4f\n\n",
        distance_to_host_curve(spline, node),
        host_closest(spline, end_effector(node)).first);

    constexpr float range = 0.3F;  // the length of one extension in joint space (rad)
    const auto &lower = Robot::lower_bound;
    const auto &upper = Robot::upper_bound;

    // 1. steer toward sample targets
    unsigned random_state = 12345U;
    std::size_t progressed = 0;
    constexpr std::size_t samples = 8;
    for (std::size_t sample = 0; sample < samples; ++sample)
    {
        std::array<float, Robot::dimension> target_values = values_of(node);
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            target_values[j] = std::clamp(target_values[j] + 1.2F * next_offset(random_state), lower[j], upper[j]);
        }

        const Configuration target(target_values);
        const float distance = (target - node).l2_norm();

        const auto extension = local_planner.steer(node, target, distance, range, true, env);
        std::printf("sample %zu: target %.2f rad away, %s", sample, distance, status_name(extension.status));

        if (extension.status != SteerStatus::Advanced and extension.status != SteerStatus::Reached)
        {
            std::printf("\n");
            continue;
        }

        ++progressed;
        const std::vector<Configuration> chain(extension.waypoints.begin(), extension.waypoints.end());
        CHECK(not chain.empty());

        double worst_distance = 0.0;
        float worst_gap = 0.F;
        Configuration previous = node;
        for (const auto &waypoint : chain)
        {
            worst_distance = std::max(worst_distance, distance_to_host_curve(spline, waypoint));
            worst_gap = std::max(worst_gap, (waypoint - previous).l2_norm());
            previous = waypoint;
        }

        const Configuration &endpoint = chain.back();
        const float stray = (endpoint - node).l2_norm();
        std::printf(
            ", %zu waypoints, worst distance to the curve %.2e m, largest gap %.3f rad, endpoint %.3f rad from the "
            "node, t %.4f -> %.4f\n",
            chain.size(),
            worst_distance,
            worst_gap,
            stray,
            host_closest(spline, end_effector(node)).first,
            host_closest(spline, end_effector(endpoint)).first);

        CHECK(worst_distance < 1e-3);  // on the curve, to a millimetre
        CHECK(worst_gap <= range);
        CHECK(stray <= 2.F * range);

        // 2. the local path to a point the planner produced is valid
        CHECK(local_planner.validate(node, endpoint, env));
    }

    std::printf("\n%zu of %zu extensions made progress\n\n", progressed, samples);
    CHECK(progressed >= samples / 2);

    // The walk: repeated steer toward one far target, each from where the last ended, recorded for the viewer
    std::vector<TrailEntry> walk;
    const auto record = [&](const Configuration &configuration)
    {
        std::array<float, Robot::dimension> values = values_of(configuration);
        const Block block = broadcast_block(configuration);
        const double error = std::sqrt(constraint->closest_squared_error(block)[{0, 0}]);
        const double t = constraint->current_t()[{0, 0}];

        TrailEntry entry;
        entry.q = values;
        entry.t = t;
        entry.end_effector = end_effector(configuration);
        entry.curve_point = spline.value(t);
        entry.tangent = (spline.value(t + 1e-5) - spline.value(t - 1e-5)) / 2e-5;
        entry.error_norm = error;
        walk.push_back(entry);
    };

    std::array<float, Robot::dimension> far_values = values_of(node);
    for (std::size_t j = 0; j < Robot::dimension; ++j)
    {
        far_values[j] = std::clamp(far_values[j] + 1.5F * next_offset(random_state), lower[j], upper[j]);
    }

    const Configuration far_target(far_values);
    Configuration walker = node;
    record(walker);
    std::size_t steps = 0;
    for (; steps < 30; ++steps)
    {
        const float distance = (far_target - walker).l2_norm();
        const auto extension = local_planner.steer(walker, far_target, distance, range, true, env);
        if (extension.status != SteerStatus::Advanced and extension.status != SteerStatus::Reached)
        {
            break;
        }

        const std::vector<Configuration> chain(extension.waypoints.begin(), extension.waypoints.end());
        for (const auto &waypoint : chain)
        {
            record(waypoint);
        }

        walker = chain.back();
        if (extension.status == SteerStatus::Reached)
        {
            break;
        }
    }

    const std::string path = prefix + "_walk.json";
    write_dump(path, "walk toward a far target", spline, data.waypoints, data.parameters, walk);
    std::printf(
        "walk: %zu extensions, %zu waypoints, t %.4f -> %.4f, written to %s\n",
        steps,
        walk.size(),
        walk.front().t,
        walk.back().t,
        path.c_str());

    for (const auto &entry : walk)
    {
        CHECK(entry.error_norm < 1e-3);
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\nspline_local_planner_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\nspline_local_planner_test: all checks passed\n");
    return 0;
}
