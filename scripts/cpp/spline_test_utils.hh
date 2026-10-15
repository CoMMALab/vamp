#pragma once

// Shared by the spline constraint tests: a Panda, a spline built on the host in double (with a closest-point
// scan as the reference), the viewer's JSON writer, and the CHECK macro.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <memory>
#include <optional>
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

    // the four basis values that are non-zero on span i (Cox-de Boor)
    void basis_values(std::size_t i, double t, double (&basis)[4]) const
    {
        double left[4];
        double right[4];
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
    }

    auto value(double t) const -> Eigen::Vector3d
    {
        t = std::clamp(t, 0.0, 1.0);
        const std::size_t i = span(t);

        double basis[4];
        basis_values(i, t, basis);

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

// The values of a configuration as a plain array
static auto values_of(const Configuration &q) -> std::array<float, Robot::dimension>
{
    std::array<float, Robot::dimension> values;
    const auto array = q.to_array();
    for (auto j = 0U; j < Robot::dimension; ++j)
    {
        values[j] = array[j];
    }

    return values;
}

// The spline as the constraint takes it: control points, knots, and the waypoints with the parameters they sit at
struct SplineData
{
    std::array<std::array<float, 3>, N> control_points;
    std::array<float, K> knots;
    std::array<std::array<float, 3>, N> waypoints;
    std::array<float, N> parameters;
};

static auto make_spline_data(const HostSpline &spline) -> SplineData
{
    SplineData data;
    for (std::size_t i = 0; i < K; ++i)
    {
        data.knots[i] = static_cast<float>(spline.knots[i]);
    }

    for (std::size_t i = 0; i < N; ++i)
    {
        // Greville abscissa: the average of the three knots that support the i-th control point's basis function
        const double parameter = (spline.knots[i + 1] + spline.knots[i + 2] + spline.knots[i + 3]) / 3.0;
        const Eigen::Vector3d waypoint = spline.value(parameter);
        data.parameters[i] = static_cast<float>(parameter);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            data.control_points[i][axis] = static_cast<float>(spline.control_points[i][axis]);
            data.waypoints[i][axis] = static_cast<float>(waypoint[axis]);
        }
    }

    return data;
}


// Global interpolation through N points, as cricket's tests/spline.hh fits them: chord-length parameters, clamped
// knots that average neighbouring parameters, and the control points from the N x N system. Returns the spline
// and the parameters at which it passes through the points.
static auto fit_spline(const std::vector<Eigen::Vector3d> &points) -> std::pair<HostSpline, std::array<double, N>>
{
    HostSpline spline;
    std::array<double, N> parameters;

    double total = 0.0;
    for (std::size_t k = 1; k < N; ++k)
    {
        total += (points[k] - points[k - 1]).norm();
    }

    double travelled = 0.0;
    parameters[0] = 0.0;
    for (std::size_t k = 1; k < N; ++k)
    {
        travelled += (points[k] - points[k - 1]).norm();
        parameters[k] = travelled / total;
    }

    parameters[N - 1] = 1.0;

    for (std::size_t i = 0; i < K; ++i)
    {
        spline.knots[i] = (i >= N) ? 1.0 : 0.0;
    }

    for (std::size_t j = 1; j + 3 < N; ++j)
    {
        spline.knots[j + 3] = (parameters[j] + parameters[j + 1] + parameters[j + 2]) / 3.0;
    }

    Eigen::MatrixXd system = Eigen::MatrixXd::Zero(N, N);
    Eigen::MatrixXd rhs(N, 3);
    for (std::size_t k = 0; k < N; ++k)
    {
        const std::size_t span = spline.span(parameters[k]);
        double basis[4];
        spline.basis_values(span, parameters[k], basis);
        for (std::size_t j = 0; j <= 3; ++j)
        {
            system(k, span - 3 + j) = basis[j];
        }

        rhs.row(k) = points[k].transpose();
    }

    const Eigen::MatrixXd solution = system.partialPivLu().solve(rhs);
    for (std::size_t k = 0; k < N; ++k)
    {
        spline.control_points[k] = solution.row(k).transpose();
    }

    return {spline, parameters};
}

// The constraint's data for a spline fitted through `points`: the waypoints are the points themselves, at the
// parameters the fit passes through them
static auto make_spline_data_for_fit(
    const HostSpline &spline,
    const std::vector<Eigen::Vector3d> &points,
    const std::array<double, N> &parameters) -> SplineData
{
    SplineData data;
    for (std::size_t i = 0; i < K; ++i)
    {
        data.knots[i] = static_cast<float>(spline.knots[i]);
    }

    for (std::size_t i = 0; i < N; ++i)
    {
        data.parameters[i] = static_cast<float>(parameters[i]);
        for (std::size_t axis = 0; axis < 3; ++axis)
        {
            data.control_points[i][axis] = static_cast<float>(spline.control_points[i][axis]);
            data.waypoints[i][axis] = static_cast<float>(points[i][axis]);
        }
    }

    return data;
}

// ---- the trajectory file written by evaluate_mbm.py --save_eef ----------------------------------------------

// The rows of the number matrix that follows `"key":` in `text`, from its first occurrence: "key": [[1, 2], [3, 4]].
// A minimal reader for that one shape (nested arrays of numbers), enough to read the first trajectory of the file
// without a JSON dependency.
static auto read_matrix(const std::string &text, const std::string &key) -> std::vector<std::vector<double>>
{
    std::vector<std::vector<double>> rows;

    std::size_t position = text.find("\"" + key + "\"");
    if (position == std::string::npos)
    {
        return rows;
    }

    position = text.find('[', position);
    if (position == std::string::npos)
    {
        return rows;
    }

    const char *cursor = text.c_str() + position + 1;  // just inside the outer bracket
    while (*cursor != '\0')
    {
        while (*cursor == ' ' or *cursor == ',')
        {
            ++cursor;
        }

        if (*cursor == ']')
        {
            break;  // the outer array ends
        }

        if (*cursor != '[')
        {
            break;  // not a matrix
        }

        ++cursor;
        std::vector<double> row;
        while (*cursor != ']' and *cursor != '\0')
        {
            char *end = nullptr;
            const double value = std::strtod(cursor, &end);
            if (end == cursor)
            {
                ++cursor;  // a separator
                continue;
            }

            row.push_back(value);
            cursor = end;
        }

        rows.push_back(row);
        if (*cursor == ']')
        {
            ++cursor;
        }
    }

    return rows;
}

// The first N end-effector positions and the configuration with the given index, from the first trajectory of a
// file written by evaluate_mbm.py --save_eef. Empty if the file is missing or too short.
struct TrajectoryPoints
{
    std::vector<Eigen::Vector3d> positions;  // the first N end-effector positions
    std::array<float, Robot::dimension> configuration;  // the configuration at `configuration_index`
};

static auto load_trajectory(const std::string &path, std::size_t configuration_index)
    -> std::optional<TrajectoryPoints>
{
    std::ifstream in(path);
    if (not in)
    {
        return std::nullopt;
    }

    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto positions = read_matrix(text, "eef_positions");
    const auto configurations = read_matrix(text, "configurations");
    if (positions.size() < N or configurations.size() <= configuration_index or
        configurations[configuration_index].size() != Robot::dimension)
    {
        return std::nullopt;
    }

    TrajectoryPoints result;
    for (std::size_t k = 0; k < N; ++k)
    {
        result.positions.emplace_back(positions[k][0], positions[k][1], positions[k][2]);
    }

    for (std::size_t j = 0; j < Robot::dimension; ++j)
    {
        result.configuration[j] = static_cast<float>(configurations[configuration_index][j]);
    }

    return result;
}

// The spline problem the planner tests share: the spline fitted through the first N end-effector positions of the
// first trajectory in a file written by evaluate_mbm.py --save_eef, with that trajectory's configuration
// `configuration_index` as the start. Without a usable file: an arc near the Panda's start end-effector, and the
// Panda's start configuration.
struct SplineProblem
{
    std::array<float, Robot::dimension> start;
    HostSpline spline;
    SplineData data;
    bool from_file;
};

static auto make_spline_problem(const std::string &trajectory_path, std::size_t configuration_index) -> SplineProblem
{
    SplineProblem problem;
    if (const auto trajectory = load_trajectory(trajectory_path, configuration_index))
    {
        problem.start = trajectory->configuration;
        const auto [fitted, parameters] = fit_spline(trajectory->positions);
        problem.spline = fitted;
        problem.data = make_spline_data_for_fit(fitted, trajectory->positions, parameters);
        problem.from_file = true;
        std::printf(
            "spline: fitted through the first %zu end-effector positions of %s, start: its configuration %zu\n",
            N,
            trajectory_path.c_str(),
            configuration_index);
    }
    else
    {
        problem.start = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
        problem.spline = make_spline(end_effector(Configuration(problem.start)));
        problem.data = make_spline_data(problem.spline);
        problem.from_file = false;
        std::printf("spline: an arc near the start end-effector (no usable trajectory file at %s)\n", trajectory_path.c_str());
    }

    return problem;
}
