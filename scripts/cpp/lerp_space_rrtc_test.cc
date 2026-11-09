// Plans along a straight-line (lerp) curve in (q, t): TaskSpaceRRTC over CurveSpace, with the end-effector held on the
// curve by LerpR3Constraint (positions in R^3) or LerpSO3R3Constraint (poses in SO(3) x R^3, rotations slerped, with an
// adjustable rotation weight). The curve has Robot::lerp_points keyframes and starts at the end-effector's pose at the
// Panda's ready configuration, wandering from there in a gentle S (and, for poses, turning about the world z axis);
// t is the normalized chord length. The goal is the end of the curve.
//
// What is checked: the generated kernel's error against a host evaluation of the curve, at random configurations and at
// parameters that include the keyframes' and the ends; the start is on the curve at its closest t; the planner solves;
// every state of the path is on the curve at its own t; the path ends at the curve's last pose and satisfies the goal
// region.
//
// The path is printed in full (every state: t, the segment, the end-effector, the errors, q) and written to a JSON dump
// that cricket's tests/view_lerp.py shows in viser: the curve, the arm along the path, the end-effector and the curve's
// pose at each state's own t.
//
// With the sixth argument qs the manifold's constraint is in its segment form: t only picks the segment of a lane, a
// projection solves for t as well (bounded to the segment), and every state takes the t its projection ended at. The
// goal region is still the curve at exactly t = 1. The checks are the same: every state is on the curve at its own t.
//
// The curve is 0.54 m long at scale 1 (the seventh argument multiplies the S's extents: 20 x 12 x 8 cm); a larger scale
// makes its segments longer. The ready pose's end-effector is about 0.3 m in front of the base, so the curve leaves the
// arm's reach somewhere beyond a scale of 2 (poses reach less than positions).
//
// Usage: vamp_lerp_space_rrtc_test [r3|so3r3 [max_iterations [rotation_weight [dump.json [qt|qs [scale]]]]]]
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_lerp_space_rrtc_test target. Needs a Panda header
// generated with "lerp_r3_constraint": true and "lerp_so3r3_constraint": true. Exits 1 if a check fails.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/planning/curve_space.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/lerp_constraint.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/planning/planners/task_space_rrtc.hh>
#include <vamp/random/curve.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
static constexpr std::size_t N = Robot::lerp_points;

using ConstraintSet = vamp::planning::constraint::ConstraintSet<Robot, rake>;
using ConstraintSettings = vamp::planning::constraint::ConstraintSettings;
using ProjMethod = vamp::planning::constraint::ProjMethod;
using LerpR3Constraint = vamp::planning::constraint::LerpR3Constraint<Robot, rake>;
using LerpSO3R3Constraint = vamp::planning::constraint::LerpSO3R3Constraint<Robot, rake>;
using Configuration = Robot::Configuration;
using Block = Robot::ConfigurationBlock<rake>;
using Space = vamp::planning::CurveSpace<Robot>;
using State = Space::State;
using TaskSpaceRRTC = vamp::planning::TaskSpaceRRTC<Robot, rake, Robot::resolution, Space>;
using LocalPlanner = vamp::planning::constraint::ConstrainedLocalPlanner<Robot, rake, Robot::resolution, Space>;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

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

static auto end_effector(const Configuration &q) -> Eigen::Isometry3d
{
    return Robot::eefk(values_of(q)).cast<double>();
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

// What the test needs to know of a curve besides its constraint: its pose at t, and the end-effector's error at q
// against it, written out on the host: {position error, rotation error (weighted)}
struct CurveAccess
{
    bool has_rotation;
    std::function<std::pair<Eigen::Vector3d, Eigen::Quaterniond>(double)> pose;
    std::function<std::array<double, 2>(const Configuration &, double)> errors;
};

// One state of the path with what the printout and the dump show of it
struct PathEntry
{
    double t;
    std::size_t segment;
    double s;
    std::array<float, Robot::dimension> q;
    Eigen::Vector3d end_effector;
    Eigen::Quaterniond end_effector_rotation;
    Eigen::Vector3d curve_position;
    Eigen::Quaterniond curve_rotation;
    double position_error;
    double rotation_error;
};

static void write_vector(std::ofstream &out, const Eigen::Vector3d &v)
{
    out << "[" << v[0] << ", " << v[1] << ", " << v[2] << "]";
}

static void write_quaternion(std::ofstream &out, const Eigen::Quaterniond &q)
{
    out << "[" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";  // w x y z
}

// The dump in the format of cricket's tests/view_lerp.py: the curve (keyframes, their parameters, a sampled curve) and
// the path as the iterations, each with the arm's configuration, the end-effector and the curve's pose at its own t
static void write_dump(
    const std::string &path,
    const std::string &label,
    const CurveAccess &access,
    const std::array<double, N> &taus,
    double rotation_weight,
    const std::vector<PathEntry> &entries)
{
    std::ofstream out(path);
    if (not out)
    {
        std::fprintf(stderr, "cannot write %s\n", path.c_str());
        return;
    }

    out << std::setprecision(9);
    out << "{\n  \"kind\": \"" << (access.has_rotation ? "so3r3" : "r3") << "\",\n  \"mode\": \"path\",\n  \"label\": \""
        << label << "\",\n  \"nq\": " << Robot::dimension << ",\n  \"weight\": " << rotation_weight << ",\n  \"taus\": [";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (i == 0 ? "" : ", ") << taus[i];
    }

    out << "],\n  \"keyframes\": [";
    for (std::size_t i = 0; i < N; ++i)
    {
        out << (i == 0 ? "" : ", ");
        write_vector(out, access.pose(taus[i]).first);
    }

    out << "]";
    if (access.has_rotation)
    {
        out << ",\n  \"keyframe_quaternions\": [";
        for (std::size_t i = 0; i < N; ++i)
        {
            out << (i == 0 ? "" : ", ");
            write_quaternion(out, access.pose(taus[i]).second);
        }

        out << "]";
    }

    constexpr int samples = 400;
    out << ",\n  \"curve_t\": [";
    for (int i = 0; i <= samples; ++i)
    {
        out << (i == 0 ? "" : ", ") << i / static_cast<double>(samples);
    }

    out << "],\n  \"curve\": [";
    for (int i = 0; i <= samples; ++i)
    {
        out << (i == 0 ? "" : ", ");
        write_vector(out, access.pose(i / static_cast<double>(samples)).first);
    }

    out << "],\n  \"iterations\": [";
    for (std::size_t i = 0; i < entries.size(); ++i)
    {
        const auto &e = entries[i];
        out << (i == 0 ? "\n    " : ",\n    ") << "{\"t\": " << e.t << ", \"q\": [";
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            out << (j == 0 ? "" : ", ") << e.q[j];
        }

        out << "], \"arm\": [";
        write_vector(out, e.end_effector);
        out << "], \"curve_point\": ";
        write_vector(out, e.curve_position);
        out << ", \"segment\": " << e.segment << ", \"s\": " << e.s << ", \"error_norm\": "
            << std::sqrt(e.position_error * e.position_error + e.rotation_error * e.rotation_error)
            << ", \"position_error\": " << e.position_error;
        if (access.has_rotation)
        {
            out << ", \"rotation_error\": " << e.rotation_error << ", \"eef_quaternion\": ";
            write_quaternion(out, e.end_effector_rotation);
            out << ", \"curve_quaternion\": ";
            write_quaternion(out, e.curve_rotation);
        }

        out << "}";
    }

    out << "\n  ]\n}\n";
}

// The checks and the planner run, for a curve constraint and what the test knows of the curve on the host
template <typename Curve>
static void run(
    const std::shared_ptr<Curve> &curve,
    const CurveAccess &access,
    const std::array<float, Robot::dimension> &start_values,
    std::size_t max_iterations,
    const std::string &dump_path,
    double rotation_weight,
    const char *form)
{
    // the squared error at (q, t) on the host
    const auto host_error = [&](const Configuration &q, double t)
    {
        const auto e = access.errors(q, t);
        return e[0] * e[0] + e[1] * e[1];
    };

    using Clock = std::chrono::steady_clock;
    const auto milliseconds = [](Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); };
    const auto run_start = Clock::now();

    // 1. the generated kernel against the host curve
    {
        vamp::rng::Halton<Robot> sampler;
        double worst = 0.0;
        for (int sample = 0; sample < 64; ++sample)
        {
            const Configuration q = sampler.next();
            const double t = (sample < 4)    ? 0.25 * sample
                             : (sample < 24) ? curve->parameters()[static_cast<std::size_t>(sample - 4)]
                                             : static_cast<double>(sample % 17) / 16.0;

            curve->set_parameter(vamp::FloatVector<rake, 1>::fill(static_cast<float>(t)));
            const double kernel = curve->squared_error(broadcast_block(q))[{0, 0}];
            const double host = host_error(q, t);
            worst = std::max(worst, std::abs(kernel - host) / std::max(1.0, host));
        }

        std::printf("kernel vs host curve: worst relative difference of the squared error %.2e\n", worst);
        CHECK(worst < 1e-3);
    }

    // the manifold: the curve, at the parameter of each state
    ConstraintSettings manifold_settings;
    manifold_settings.method = ProjMethod::BoundedLS;
    manifold_settings.tolerance = 1e-8F;
    manifold_settings.max_iterations = 50;
    const ConstraintSet manifold(std::vector<ConstraintSet::Ptr>{curve}, manifold_settings);

    // the goal region: a copy of the curve's constraint (the planner holds the region at t = 1; a separate instance, so
    // the parameter the manifold's constraint is given does not move it)
    const auto goal_constraint = std::make_shared<Curve>(curve->fixed_copy());
    ConstraintSettings region_settings;
    const ConstraintSet region(std::vector<ConstraintSet::Ptr>{goal_constraint}, region_settings);

    // 2. the start: on the curve at its closest t
    const Configuration start_q(start_values);
    const auto [start_t, start_distance2] = curve->closest_parameter(start_q);
    State start = Space::make_state(start_q, start_t);

    LocalPlanner local_planner(manifold);
    CHECK(local_planner.project(start));
    CHECK(local_planner.satisfied(start));
    std::printf("start: closest t %.4f (squared distance %.2e)\n", start_t, start_distance2);

    EnvironmentInput environment;
    environment.sort();
    const EnvironmentVector env(environment);

    vamp::planning::TaskSpaceRRTCSettings settings;
    settings.range = 0.3F;
    settings.max_greedy_steps = 20;
    settings.max_iterations = max_iterations;
    settings.max_samples = 20000;
    settings.project_samples = false;

    auto rng = std::make_shared<vamp::rng::CurveRNG<Robot>>(std::make_shared<vamp::rng::Halton<Robot>>());
    const auto solve_start = Clock::now();
    const auto result = TaskSpaceRRTC::solve<LocalPlanner>(start, region, env, settings, rng, local_planner);
    const double solve_ms = milliseconds(Clock::now() - solve_start);

    std::printf(
        "\nTaskSpaceRRTC over CurveSpace: solved=%d, iterations %zu, %zu nodes, %.1f ms, cost %.3f\n",
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
        double worst_error = 0.0;
        for (const auto &state : result.path)
        {
            worst_error = std::max(
                worst_error, std::sqrt(host_error(Space::to_ambient(state), Space::parameter(state))));
        }

        const Configuration last = Space::to_ambient(result.path[result.path.size() - 1]);
        const double final_error = std::sqrt(host_error(last, 1.0));
        std::printf(
            "path: %zu states, t %.4f -> %.4f, worst error to the curve at its own t %.2e, final error to the last pose "
            "%.2e\n",
            result.path.size(),
            Space::parameter(result.path[0]),
            Space::parameter(result.path[result.path.size() - 1]),
            worst_error,
            final_error);

        CHECK(worst_error < 1e-3);
        CHECK(final_error < 2e-3);
        CHECK(region.satisfied(last));

        // the path in full
        const auto &taus = curve->parameters();
        std::vector<PathEntry> entries;
        std::printf(
            "\n  state        t  segment       s     end-effector (m)            position err  rotation err (weighted)\n");
        for (std::size_t i = 0; i < result.path.size(); ++i)
        {
            const auto &state = result.path[i];
            const Configuration q = Space::to_ambient(state);
            const double t = Space::parameter(state);

            PathEntry entry;
            entry.t = t;
            entry.segment = 0;
            while (entry.segment + 2 < N and t >= taus[entry.segment + 1])
            {
                ++entry.segment;
            }

            entry.s = (t - taus[entry.segment]) / (taus[entry.segment + 1] - taus[entry.segment]);
            entry.q = values_of(q);
            const auto pose = end_effector(q);
            entry.end_effector = pose.translation();
            entry.end_effector_rotation = Eigen::Quaterniond(pose.rotation());
            const auto target = access.pose(t);
            entry.curve_position = target.first;
            entry.curve_rotation = target.second;
            const auto errors = access.errors(q, t);
            entry.position_error = errors[0];
            entry.rotation_error = errors[1];
            entries.push_back(entry);

            std::printf(
                "  %5zu  %7.4f  %7zu  %6.3f   %7.3f %7.3f %7.3f   %12.2e  %12.2e\n",
                i,
                entry.t,
                entry.segment,
                entry.s,
                entry.end_effector[0],
                entry.end_effector[1],
                entry.end_effector[2],
                entry.position_error,
                entry.rotation_error);
        }

        if (not dump_path.empty())
        {
            write_dump(
                dump_path,
                std::string("lerp ") + (access.has_rotation ? "so3r3" : "r3") + ", " + form + ", path (" + std::to_string(entries.size()) +
                    " states)",
                access,
                taus,
                rotation_weight,
                entries);
            std::printf("\npath written to %s: python3 cricket/tests/view_lerp.py %s\n", dump_path.c_str(), dump_path.c_str());
        }
    }

    // the time: the solve call (the planner's own clock, which starts after its setup, is the figure printed above), and
    // the whole run: the kernel check, the constraint sets, the start's projection, the solve and the path checks
    std::printf(
        "\ntime: solve call %.1f ms, whole run %.1f ms (kernel check, setup, solve and path checks)\n",
        solve_ms,
        milliseconds(Clock::now() - run_start));
}

int main(int argc, char **argv)
{
    const std::string kind = argc > 1 ? argv[1] : "r3";
    const std::size_t max_iterations = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 20000;
    const float rotation_weight = argc > 3 ? std::strtof(argv[3], nullptr) : 0.3F;
    const std::string mode = argc > 5 ? argv[5] : "qt";
    const std::string dump_path = argc > 4 ? argv[4] : "lerp_" + kind + "_" + mode + "_path.json";
    const bool segment = mode == "qs";
    const double scale = argc > 6 ? std::strtod(argv[6], nullptr) : 1.0;  // the size of the curve

    if ((kind != "r3" and kind != "so3r3") or (mode != "qt" and mode != "qs"))
    {
        std::fprintf(stderr, "usage: %s [r3|so3r3 [max_iterations [rotation_weight [dump.json [qt|qs [scale]]]]]]\n", argv[0]);
        return 2;
    }

    // the Panda's ready configuration, and the pose of its end-effector, where the curve starts
    const std::array<float, Robot::dimension> start_values = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
    const Eigen::Isometry3d start_pose = end_effector(Configuration(start_values));
    constexpr double pi = 3.14159265358979323846;

    std::array<std::array<float, 3>, N> positions;
    std::array<std::array<float, 4>, N> quaternions;
    for (std::size_t i = 0; i < N; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(N - 1);
        const Eigen::Vector3d position =
            start_pose.translation() + scale * Eigen::Vector3d(0.20 * s, 0.12 * std::sin(2.0 * pi * s), 0.08 * s);
        const Eigen::Quaterniond rotation(
            Eigen::AngleAxisd(60.0 * pi / 180.0 * s, Eigen::Vector3d::UnitZ()) * Eigen::Quaterniond(start_pose.rotation()));
        for (auto axis = 0U; axis < 3; ++axis)
        {
            positions[i][axis] = static_cast<float>(position[axis]);
        }

        quaternions[i] = {
            static_cast<float>(rotation.w()),
            static_cast<float>(rotation.x()),
            static_cast<float>(rotation.y()),
            static_cast<float>(rotation.z())};
    }

    if (kind == "r3")
    {
        std::printf("lerp curve in R^3 through %zu points, %s\n", N, mode.c_str());
        const auto curve = std::make_shared<LerpR3Constraint>(positions, segment);

        CurveAccess access;
        access.has_rotation = false;
        access.pose = [&](double t) { return std::make_pair(curve->value(t), Eigen::Quaterniond::Identity()); };
        access.errors = [&](const Configuration &q, double t) -> std::array<double, 2>
        { return {(end_effector(q).translation() - curve->value(t)).norm(), 0.0}; };
        run(curve, access, start_values, max_iterations, dump_path, 0.0, mode.c_str());
    }
    else
    {
        std::printf("lerp curve in SO(3) x R^3 through %zu poses, rotation weight %.3g, %s\n", N, rotation_weight, mode.c_str());
        const auto curve = std::make_shared<LerpSO3R3Constraint>(positions, quaternions, rotation_weight, segment);

        CurveAccess access;
        access.has_rotation = true;
        access.pose = [&](double t) { return curve->value(t); };
        access.errors = [&](const Configuration &q, double t) -> std::array<double, 2>
        {
            const auto pose = end_effector(q);
            const auto e = curve->error(pose.translation(), Eigen::Quaterniond(pose.rotation()), t);
            return {e.template head<3>().norm(), e.template tail<3>().norm()};
        };
        run(curve, access, start_values, max_iterations, dump_path, rotation_weight, mode.c_str());
    }

    if (failures != 0)
    {
        std::fprintf(stderr, "\nlerp_space_rrtc_test: %d check(s) failed\n", failures);
        return 1;
    }

    std::printf("\nlerp_space_rrtc_test: all checks passed\n");
    return 0;
}
