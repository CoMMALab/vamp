// Compares the two projections onto a straight-line (lerp) curve from the same starts: qt, where t is held at the value
// the state has and the end-effector must reach the curve's pose at that t (the joints alone are solved for), and qs,
// where t only picks the segment it is on and the end-effector may be anywhere on that segment (t is solved for next to
// the joints, bounded to the segment, and the state takes the t where it ends).
//
// The starts are the Panda's ready configuration plus Gaussian noise (--perturb rad per joint), with t at the closest
// point of the curve to the end-effector plus an offset (a planner's t is near where its configuration is). Per mode it
// prints how many projections converged, the mean number of iterations of the converged ones, the mean time of a
// projection (the whole project_all call, all lanes holding the same start) and, for qs, where on its segment the
// projection ended: at either end (s = 0 or 1) or inside.
//
// The curve is the one of lerp_space_rrtc_test: 20 keyframes starting at the ready pose's end-effector pose, a gentle S
// (and, for poses, a 60 degree turn about the world z axis).
//
// Usage: vamp_lerp_projection_benchmark [r3|so3r3 [trials [method [t_offset [perturb [rotation_weight]]]]]]
//   method: bounded (default), inner, outer or gradient
// Build: configure with -DVAMP_BUILD_CPP_DEMO=ON and build the vamp_lerp_projection_benchmark target. Needs a Panda header
// generated with the lerp constraints and their segment forms ("lerp_r3_constraint", "lerp_so3r3_constraint",
// "lerp_r3_segment_constraint", "lerp_so3r3_segment_constraint").

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/lerp_constraint.hh>
#include <vamp/planning/constraints/settings.hh>
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
using Row = vamp::FloatVector<rake, 1>;

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

static auto broadcast_block(const Configuration &q) -> Block
{
    Block block;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = q.broadcast(i);
    }

    return block;
}

struct Outcome
{
    bool converged = false;
    std::size_t iterations = 0;
    double microseconds = 0.0;
    double s = 0.0;  // where on the segment it ended (qs)
};

struct Totals
{
    std::size_t converged = 0;
    double iterations = 0.0;
    double microseconds = 0.0;
    std::size_t at_start = 0;
    std::size_t at_end = 0;
    std::size_t inside = 0;
};

// Both projections of every start, with a constraint made in the fixed form and one in the segment form
template <typename Curve>
static void benchmark(
    const char *name,
    const Curve &fixed,
    const Curve &segment,
    const std::vector<std::pair<std::array<float, Robot::dimension>, float>> &starts,
    const ConstraintSettings &settings)
{
    const auto &taus = fixed.parameters();

    const auto run = [&](const Curve &curve, bool segment_form) -> Totals
    {
        const auto shared = std::make_shared<Curve>(curve);
        const ConstraintSet set(std::vector<ConstraintSet::Ptr>{shared}, settings);

        Totals totals;
        for (const auto &[values, t] : starts)
        {
            const Configuration q(values);

            // the iterations: the projection loop driven here, to count them
            Block block = broadcast_block(q);
            set.set_parameter(Row::fill(t));
            float error = set.squared_error(block)[{0, 0}];
            std::size_t iterations = 0;
            while (iterations < settings.max_iterations and error > settings.tolerance)
            {
                set.step_in_place(block);
                error = set.squared_error(block)[{0, 0}];
                ++iterations;
            }

            const bool converged = error <= settings.tolerance;
            if (converged)
            {
                totals.converged++;
                totals.iterations += static_cast<double>(iterations);
            }

            if (segment_form)
            {
                const auto start_segment = vamp::planning::constraint::lerp_detail::segment_of(taus, t);
                const double t_end = set.projected_parameter(Row::fill(t))[{0, 0}];
                const double s = (t_end - taus[start_segment]) / (taus[start_segment + 1] - taus[start_segment]);
                (s < 1e-3 ? totals.at_start : s > 1.0 - 1e-3 ? totals.at_end : totals.inside)++;
            }

            // the time: the whole projection, as the planner calls it
            Block timed = broadcast_block(q);
            set.set_parameter(Row::fill(t));
            const auto started = std::chrono::steady_clock::now();
            const bool ok = set.project_all(timed);
            const auto elapsed = std::chrono::steady_clock::now() - started;
            (void)ok;
            totals.microseconds += std::chrono::duration<double, std::micro>(elapsed).count();
        }

        return totals;
    };

    const auto report = [&](const char *mode, const Totals &totals, bool segment_form)
    {
        const double n = static_cast<double>(starts.size());
        std::printf(
            "  %-3s  %4zu/%zu converged   iterations %5.2f (converged)   time %7.2f us per projection",
            mode,
            totals.converged,
            starts.size(),
            totals.converged > 0 ? totals.iterations / static_cast<double>(totals.converged) : 0.0,
            totals.microseconds / n);
        if (segment_form)
        {
            std::printf(
                "   ended at s = 0: %4.1f%%, s = 1: %4.1f%%, inside: %4.1f%%",
                100.0 * totals.at_start / n,
                100.0 * totals.at_end / n,
                100.0 * totals.inside / n);
        }

        std::printf("\n");
    };

    std::printf("%s\n", name);
    report("qt", run(fixed, false), false);
    report("qs", run(segment, true), true);
}

int main(int argc, char **argv)
{
    const std::string kind = argc > 1 ? argv[1] : "r3";
    const std::size_t trials = argc > 2 ? std::strtoull(argv[2], nullptr, 10) : 1000;
    const std::string method = argc > 3 ? argv[3] : "bounded";
    const float t_offset = argc > 4 ? std::strtof(argv[4], nullptr) : 0.25F;
    const float perturb = argc > 5 ? std::strtof(argv[5], nullptr) : 0.55F;
    const float rotation_weight = argc > 6 ? std::strtof(argv[6], nullptr) : 0.3F;

    if (kind != "r3" and kind != "so3r3")
    {
        std::fprintf(stderr, "usage: %s [r3|so3r3 [trials [method [t_offset [perturb [rotation_weight]]]]]]\n", argv[0]);
        return 2;
    }

    ConstraintSettings settings;
    settings.tolerance = 1e-8F;
    settings.max_iterations = 50;
    if (method == "bounded")
    {
        settings.method = ProjMethod::BoundedLS;
    }
    else if (method == "inner")
    {
        settings.method = ProjMethod::InnerLM;
    }
    else if (method == "outer")
    {
        settings.method = ProjMethod::OuterLM;
    }
    else if (method == "gradient")
    {
        settings.method = ProjMethod::GradDesc;
    }
    else
    {
        std::fprintf(stderr, "method is bounded, inner, outer or gradient\n");
        return 2;
    }

    // the Panda's ready configuration, and the pose of its end-effector, where the curve starts
    const std::array<float, Robot::dimension> ready = {0.F, -0.785F, 0.F, -2.356F, 0.F, 1.571F, 0.785F};
    const auto start_pose = Robot::eefk(ready).cast<double>();
    constexpr double pi = 3.14159265358979323846;

    std::array<std::array<float, 3>, N> positions;
    std::array<std::array<float, 4>, N> quaternions;
    for (std::size_t i = 0; i < N; ++i)
    {
        const double s = static_cast<double>(i) / static_cast<double>(N - 1);
        const Eigen::Vector3d position =
            start_pose.translation() + Eigen::Vector3d(0.20 * s, 0.12 * std::sin(2.0 * pi * s), 0.08 * s);
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

    const LerpR3Constraint r3_fixed(positions, false);
    const LerpR3Constraint r3_segment(positions, true);
    const LerpSO3R3Constraint pose_fixed(positions, quaternions, rotation_weight, false);
    const LerpSO3R3Constraint pose_segment(positions, quaternions, rotation_weight, true);

    // the starts: the ready configuration plus noise (inside the joint limits), with t at the closest point plus an offset
    std::mt19937 random(0);
    std::normal_distribution<float> noise(0.F, perturb);
    std::vector<std::pair<std::array<float, Robot::dimension>, float>> starts;
    for (std::size_t trial = 0; trial < trials; ++trial)
    {
        std::array<float, Robot::dimension> values = ready;
        for (auto j = 0U; j < Robot::dimension; ++j)
        {
            values[j] = std::clamp(values[j] + noise(random), Robot::lower_bound[j], Robot::upper_bound[j]);
        }

        const Configuration q(values);
        const float closest = (kind == "r3") ? r3_fixed.closest_parameter(q).first : pose_fixed.closest_parameter(q).first;
        starts.emplace_back(values, std::clamp(closest + t_offset, 0.F, 1.F));
    }

    std::printf(
        "projections onto a lerp curve: %zu starts (ready + %.2f rad noise), t = closest + %.2f, method %s, tolerance %g\n\n",
        trials,
        perturb,
        t_offset,
        method.c_str(),
        static_cast<double>(settings.tolerance));

    if (kind == "r3")
    {
        benchmark("lerp curve in R^3", r3_fixed, r3_segment, starts, settings);
    }
    else
    {
        std::printf("rotation weight %.3g\n", rotation_weight);
        benchmark("lerp curve in SO(3) x R^3", pose_fixed, pose_segment, starts, settings);
    }

    return 0;
}
