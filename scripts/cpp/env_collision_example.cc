// Runs one planning problem through EnvironmentCollisionConstraint (see
// src/impl/vamp/planning/constraints/manifold/env_collision_constraint.hh): the same
// sphere-cage problem as scripts/cpp/rrtc_example.cc, but planned with a
// ConstrainedLocalPlanner that pushes every steered/connected configuration `margin` away
// from the cage using ONE representative sphere per link (see cricket's `env_collision`
// recipe key / RobotInfo::env_collision_middle_spheres) on top of the planner's ordinary
// hard collision checking (fkcc) -- the same "hand off to a constraint-aware local
// planner" pattern scripts/digit_example.py uses from Python, done here directly in C++.
//
// This only builds once vamp/src/impl/vamp/robots/panda.hh has been regenerated from
// cricket with `"env_collision": true` in resources/panda.json (see cricket's README for
// the fkcc_gen -> clang-format -> cp workflow) -- it exists so that regeneration can be
// smoke-tested with a lightweight `cmake --build` instead of a full `pip install` of the
// Python extension.

#include <array>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/local_planner.hh>
#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/manifold/env_collision_constraint.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/panda.hh>

using Robot = vamp::robots::Panda;
static constexpr std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

namespace vc = vamp::planning::constraint;
using EnvCollisionConstraint = vc::EnvironmentCollisionConstraint<Robot, rake>;
using ConstraintSet = vc::ConstraintSet<Robot, rake>;
using ConstrainedLocalPlanner = vc::ConstrainedLocalPlanner<Robot, rake, Robot::resolution>;

// Same problem as scripts/cpp/rrtc_example.cc: start and goal on opposite sides of a cage
// of obstacle spheres, so the planner must actually route around the cage rather than
// merely hold still (unlike the point-obstacle-at-a-single-config demo this replaces).
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

static const std::vector<std::array<float, 3>> cage = {
    {0.55, 0, 0.25},      {0.35, 0.35, 0.25},   {0, 0.55, 0.25},     {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25}, {0, -0.55, 0.25},     {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},       {-0.35, 0.35, 0.8},   {-0.55, 0, 0.8},     {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},      {0.35, -0.35, 0.8},
};
static constexpr float cage_sphere_radius = 0.2F;

// EnvironmentCollisionConstraint distance margin: every representative sphere is pushed
// to stay at least this far from the cage during planning, on top of ordinary hard
// collision checking.
static constexpr float margin = 0.03F;

auto main(int, char **) -> int
{
    std::cout << std::fixed << std::setprecision(4);

    std::cout << Robot::n_env_collision_spheres
              << " env-collision spheres (one per environment-checked link)\n\n";

    EnvironmentInput environment;
    for (const auto &sphere : cage)
    {
        environment.spheres.emplace_back(
            vamp::collision::factory::sphere::array(sphere, cage_sphere_radius));
    }

    environment.sort();
    const auto environment_v = EnvironmentVector(environment);

    auto constraint = std::make_shared<const EnvCollisionConstraint>(environment, margin);
    const std::vector<std::shared_ptr<const vc::Constraint<Robot, rake>>> constraints = {constraint};

    // Seed projection gets a generous iteration budget, independent of the planning
    // settings: start/goal must land on the manifold before the planner will accept them
    // (a ConstrainedLocalPlanner's steer/connect assume they start on-manifold).
    vc::ConstraintSettings seed_settings;
    seed_settings.method = vc::ProjMethod::InnerLM;
    seed_settings.max_iterations = 100;
    seed_settings.descend_rate = 0.75F;

    ConstraintSet seed_set(constraints, seed_settings);
    auto start_c = Robot::Configuration(start);
    auto goal_c = Robot::Configuration(goal);
    const bool start_ok = seed_set.project(start_c);
    const bool goal_ok = seed_set.project(goal_c);

    if (not start_ok or not goal_ok)
    {
        std::cerr << "Seed projection failed to converge (start: " << start_ok
                  << ", goal: " << goal_ok << ") -- try a larger margin or more seed iterations.\n";
        return 1;
    }

    vc::ConstraintSettings plan_settings;
    plan_settings.method = vc::ProjMethod::InnerLM;
    plan_settings.max_iterations = 10;
    plan_settings.descend_rate = 1.0F;

    ConstrainedLocalPlanner lp(ConstraintSet(constraints, plan_settings));

    if (not lp.satisfied(start_c) or not lp.satisfied(goal_c))
    {
        std::cerr << "Projected start/goal do not satisfy the constraint within tolerance.\n";
        return 1;
    }

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 1.0F;

    auto result = RRTC::solve(start_c, goal_c, environment_v, rrtc_settings, rng, lp);

    if (not result.solved)
    {
        std::cerr << "Planning failed to find a path.\n";
        return 1;
    }

    std::cout << "Solved in " << result.iterations << " iterations, " << result.path.size()
              << " states, cost " << result.path.cost() << " time : " << result.nanoseconds / 1e3 << "\n";

    vamp::planning::SimplifySettings simplify_settings;
    const auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
        result.path, environment_v, simplify_settings, rng, lp);

    std::cout << "Simplified to " << simplify_result.path.size() << " states, cost "
              << simplify_result.path.cost() << "\n\n";

    bool all_on_manifold = true;
    for (const auto &config : simplify_result.path)
    {
        all_on_manifold &= lp.satisfied(config);
    }
    std::cout << "All simplified waypoints satisfy the margin constraint: " << std::boolalpha
              << all_on_manifold << "\n\n";

    for (const auto &config : simplify_result.path)
    {
        const auto &array = config.to_array();
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            std::cout << array[i] << ", ";
        }

        std::cout << '\n';
    }

    return all_on_manifold ? 0 : 1;
}
