// Checks SplineConstraint on its own, with the Panda, before it is wired into a planner.
//
// The spline is built here so that no fit or data file is needed: control points on a gentle arc a few
// centimetres from the end-effector of the Panda's start configuration, the clamped knot vector, and the "waypoints"
// the closest-t kernel seeds from taken as the curve's points at the knot averages (so the curve passes through
// them at those parameters, as a fitted spline passes through its waypoints). A host evaluation of the same
// spline, in double, is the reference.
//
// What is checked:
//   1. the distance function: distance_to_spline(q) and the closest t it finds against a dense scan on the host;
//   2. satisfied(): false at the start configuration (off the curve), true after projecting it onto the curve;
//   3. projection with each method: it lands on the curve (the host's distance from the projected end-effector
//      to the curve is small), t stays in [0, 1], and the t it ends with is the closest t of the projected
//      configuration.
//
// Each method's projection is also recorded step by step and written to <prefix>_<method>.json (default prefix
// "spline_constraint"), in the format of cricket's tests/view_spline.py:
//   python3 cricket/tests/view_spline.py spline_constraint_inner_lm.json
// shows the curve, the waypoints, the Panda and the end-effector moving onto the curve as the slider steps through
// the projection (the t shown is the constraint's own t).
//
// Usage: vamp_spline_constraint_test [output_prefix]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_spline_constraint_test target. Needs a Panda
// header generated with "spline_constraint": true (cricket). Exits 1 if a check fails.

#include "spline_test_utils.hh"

int main(int argc, char **argv)
{
    const std::string prefix = argc > 1 ? argv[1] : "spline_constraint";

    const std::array<float, Robot::dimension> start_values = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
    const Configuration start(start_values);
    const Eigen::Vector3d start_position = end_effector(start);

    const HostSpline spline = make_spline(start_position);

    // the data the constraint takes: control points, knots, and the waypoints with the parameters they sit at
    std::array<std::array<float, 3>, N> control_points;
    std::array<float, K> knots;
    std::array<std::array<float, 3>, N> waypoints;
    std::array<float, N> parameters;
    for (std::size_t i = 0; i < K; ++i)
    {
        knots[i] = static_cast<float>(spline.knots[i]);
    }

    for (std::size_t i = 0; i < N; ++i)
    {
        // Greville abscissa: the average of the three knots that support the i-th control point's basis function
        const double parameter = (spline.knots[i + 1] + spline.knots[i + 2] + spline.knots[i + 3]) / 3.0;
        const Eigen::Vector3d waypoint = spline.value(parameter);
        parameters[i] = static_cast<float>(parameter);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            control_points[i][axis] = static_cast<float>(spline.control_points[i][axis]);
            waypoints[i][axis] = static_cast<float>(waypoint[axis]);
        }
    }

    const auto constraint = std::make_shared<SplineConstraint>(control_points, knots, waypoints, parameters);

    // 1. the distance function against a dense scan
    {
        const auto [host_t, host_d2] = host_closest(spline, start_position);
        const auto block = broadcast_block(start);
        const double kernel_d2 = constraint->distance_to_spline(block)[{0, 0}];
        const double kernel_t = constraint->current_t()[{0, 0}];

        std::printf(
            "distance to the spline at the start configuration:\n"
            "  kernel: squared distance %.6e, closest t %.6f\n"
            "  host:   squared distance %.6e, closest t %.6f\n\n",
            kernel_d2,
            kernel_t,
            host_d2,
            host_t);

        CHECK(host_d2 > 1e-5);  // the start configuration is off the curve, by construction
        CHECK(std::abs(kernel_d2 - host_d2) < 1e-6);
        CHECK(std::abs(kernel_t - host_t) < 1e-3);
    }

    // 2.-3. projection with each method, then satisfied() and the distance of the result
    struct MethodRun
    {
        const char *name;
        const char *slug;  // for the file name
        ProjMethod method;
        bool must_converge;  // plain gradient descent may need more than the iteration cap
    };

    const std::array<MethodRun, 4> runs = {{
        {"inner LM", "inner_lm", ProjMethod::InnerLM, true},
        {"outer LM", "outer_lm", ProjMethod::OuterLM, true},
        {"bounded LS", "bounded_ls", ProjMethod::BoundedLS, true},
        {"gradient descent", "gradient_descent", ProjMethod::GradDesc, false},
    }};

    for (const auto &run : runs)
    {
        ConstraintSettings settings;
        settings.method = run.method;
        settings.tolerance = 1e-8F;
        settings.max_iterations = 50;

        const ConstraintSet set(std::vector<ConstraintSet::Ptr>{constraint}, settings);

        // off the curve before, on it after
        CHECK(not set.satisfied(start));

        Configuration q = start;
        const bool converged = set.project(q);

        const Eigen::Vector3d position = end_effector(q);
        const auto [host_t, host_d2] = host_closest(spline, position);

        const bool satisfied = set.satisfied(q);
        const double kernel_t = constraint->current_t()[{0, 0}];  // left by satisfied(): the closest t of q

        std::printf(
            "%-17s converged %d, satisfied %d, host squared distance to the curve %.3e, t %.6f (host closest t %.6f)\n",
            run.name,
            converged,
            satisfied,
            host_d2,
            kernel_t,
            host_t);

        CHECK(kernel_t >= 0.0 and kernel_t <= 1.0);
        if (run.must_converge)
        {
            CHECK(converged);
            CHECK(satisfied);
            CHECK(host_d2 < 1e-7);
            CHECK(std::abs(kernel_t - host_t) < 1e-3);
        }

        // The same projection step by step, as ConstraintSet::project runs it (begin_projection once, then
        // squared_error and step until the error is below tolerance), recorded for the viewer
        {
            std::vector<TrailEntry> trail;
            Block block = broadcast_block(start);
            constraint->begin_projection(block);
            for (std::size_t iteration = 0; iteration <= settings.max_iterations; ++iteration)
            {
                const double error = constraint->squared_error(block)[{0, 0}];
                const double t = constraint->current_t()[{0, 0}];

                TrailEntry entry;
                std::array<float, Robot::dimension> values;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    entry.q[j] = values[j] = block[{j, 0}];
                }

                entry.t = t;
                entry.end_effector = end_effector(Configuration(values));
                entry.curve_point = spline.value(t);
                entry.tangent = (spline.value(t + 1e-5) - spline.value(t - 1e-5)) / 2e-5;
                entry.error_norm = std::sqrt(error);
                trail.push_back(entry);

                if (error <= settings.tolerance or iteration == settings.max_iterations)
                {
                    break;
                }

                constraint->step(block, run.method, settings.descend_rate);
            }

            const std::string path = prefix + "_" + run.slug + ".json";
            write_dump(path, run.name, spline, waypoints, parameters, trail);
            std::printf("  %zu steps, |err| %.4f -> %.2e, written to %s\n", trail.size() - 1, trail.front().error_norm, trail.back().error_norm, path.c_str());
        }
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\nspline_constraint_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\nspline_constraint_test: all checks passed\n");
    return 0;
}
