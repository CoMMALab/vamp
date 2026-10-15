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

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/spline_constraint.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using SplineConstraint = vamp::planning::constraint::SplineConstraint<Robot, rake>;
using ConstraintSet = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;
using ProjMethod = vamp::planning::constraint::ProjMethod;
using Configuration = Robot::Configuration;
using Block = Robot::ConfigurationBlock<rake>;

static constexpr std::size_t N = Robot::spline_control_points;
static constexpr std::size_t K = Robot::spline_knots;

static int failures = 0;

#define CHECK(condition)                                                                                     \
    do                                                                                                       \
    {                                                                                                        \
        if (not(condition))                                                                                  \
        {                                                                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                        \
            ++failures;                                                                                      \
        }                                                                                                    \
    } while (0)

// The curve on the host, in double: clamped cubic B-spline, evaluated with the Cox-de Boor recursion
struct HostSpline
{
    std::array<Eigen::Vector3d, N> control_points;
    std::array<double, K> knots;

    // knot span i with knots[i] <= t < knots[i + 1], in [3, N - 1]
    auto span(double t) const -> std::size_t
    {
        if (t >= knots[N])
        {
            return N - 1;
        }

        std::size_t i = 3;
        while (i + 1 < N and knots[i + 1] <= t)
        {
            ++i;
        }

        return i;
    }

    auto value(double t) const -> Eigen::Vector3d
    {
        t = std::clamp(t, 0.0, 1.0);
        const std::size_t i = span(t);

        double left[4];
        double right[4];
        double basis[4];
        basis[0] = 1.0;
        for (int j = 1; j <= 3; ++j)
        {
            left[j] = t - knots[i + 1 - j];
            right[j] = knots[i + j] - t;

            double saved = 0.0;
            for (int r = 0; r < j; ++r)
            {
                const double term = basis[r] / (right[r + 1] + left[j - r]);
                basis[r] = saved + right[r + 1] * term;
                saved = left[j - r] * term;
            }

            basis[j] = saved;
        }

        Eigen::Vector3d point = Eigen::Vector3d::Zero();
        for (std::size_t j = 0; j <= 3; ++j)
        {
            point += basis[j] * control_points[i - 3 + j];
        }

        return point;
    }
};

static auto make_spline(const Eigen::Vector3d &center) -> HostSpline
{
    HostSpline spline;

    for (std::size_t i = 0; i < K; ++i)
    {
        spline.knots[i] = (i < 4) ? 0.0 : (i >= N) ? 1.0 : static_cast<double>(i - 3) / static_cast<double>(N - 3);
    }

    // a gentle arc: 30 cm long, bowed 5 cm upward in the middle, a few centimetres from `center`
    for (std::size_t i = 0; i < N; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(N - 1);
        spline.control_points[i] =
            center + Eigen::Vector3d(0.05 * (s - 0.5), 0.30 * (s - 0.5), 0.05 * std::sin(3.14159265358979 * s));
    }

    return spline;
}

// The closest point of the host spline to x: a dense scan. Returns {t, squared distance}.
static auto host_closest(const HostSpline &spline, const Eigen::Vector3d &x) -> std::pair<double, double>
{
    constexpr int samples = 20001;

    double best_t = 0.0;
    double best_d2 = 1e30;
    for (int i = 0; i < samples; ++i)
    {
        const double t = i / static_cast<double>(samples - 1);
        const double d2 = (x - spline.value(t)).squaredNorm();
        if (d2 < best_d2)
        {
            best_d2 = d2;
            best_t = t;
        }
    }

    return {best_t, best_d2};
}

static auto end_effector(const Configuration &q) -> Eigen::Vector3d
{
    std::array<float, Robot::dimension> values;
    const auto array = q.to_array();
    for (auto j = 0U; j < Robot::dimension; ++j)
    {
        values[j] = array[j];
    }

    return Robot::eefk(values).translation().cast<double>();
}

static auto broadcast_block(const Configuration &q) -> Block
{
    Block block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return block;
}

// One recorded projection step: the state of lane 0, with the end-effector and the curve point it is being matched to
struct TrailEntry
{
    std::array<float, Robot::dimension> q;
    double t;
    Eigen::Vector3d end_effector;
    Eigen::Vector3d curve_point;
    Eigen::Vector3d tangent;
    double error_norm;
};

static void write_vector(std::ofstream &out, const Eigen::Vector3d &v)
{
    out << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
}

// The dump in the format of cricket's tests/view_spline.py: the spline (control points, knots, waypoints, a sampled
// curve and the points where the cubic pieces meet) and the projection steps. The viewer draws the robot from q,
// so "arm" only needs to hold the end-effector (its last entry).
static void write_dump(
    const std::string &path,
    const std::string &label,
    const HostSpline &spline,
    const std::array<std::array<float, 3>, N> &waypoints,
    const std::array<float, N> &parameters,
    const std::vector<TrailEntry> &trail)
{
    std::ofstream out(path);
    out << std::setprecision(9);
    out << "{\n  \"label\": \"" << label << "\",\n  \"nq\": " << Robot::dimension << ",\n  \"waypoints\": [";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (i == 0 ? "" : ", ");
        write_vector(out, Eigen::Vector3d(waypoints[i][0], waypoints[i][1], waypoints[i][2]));
    }

    out << "],\n  \"control_points\": [";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (i == 0 ? "" : ", ");
        write_vector(out, spline.control_points[i]);
    }

    out << "],\n  \"knots\": [";
    for (std::size_t i = 0; i < K; ++i)
    {
        out << (i == 0 ? "" : ", ") << spline.knots[i];
    }

    out << "],\n  \"parameters\": [";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (i == 0 ? "" : ", ") << parameters[i];
    }

    constexpr int samples = 400;
    out << "],\n  \"curve_t\": [";
    for (int i = 0; i <= samples; ++i)
    {
        out << (i == 0 ? "" : ", ") << i / static_cast<double>(samples);
    }

    out << "],\n  \"curve\": [";
    for (int i = 0; i <= samples; ++i)
    {
        out << (i == 0 ? "" : ", ");
        write_vector(out, spline.value(i / static_cast<double>(samples)));
    }

    out << "],\n  \"knot_points\": [";
    for (std::size_t i = 3; i <= N; ++i)
    {
        out << (i == 3 ? "" : ", ");
        write_vector(out, spline.value(spline.knots[i]));
    }

    out << "],\n  \"iterations\": [";
    for (std::size_t i = 0; i < trail.size(); ++i)
    {
        const auto &entry = trail[i];
        out << (i == 0 ? "\n    " : ",\n    ") << "{\"t\": " << entry.t << ", \"q\": [";
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            out << (j == 0 ? "" : ", ") << entry.q[j];
        }

        out << "], \"arm\": [";
        write_vector(out, entry.end_effector);
        out << "], \"curve_point\": ";
        write_vector(out, entry.curve_point);
        out << ", \"tangent\": ";
        write_vector(out, entry.tangent);
        out << ", \"error_norm\": " << entry.error_norm << "}";
    }

    out << "\n  ]\n}\n";
}

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
