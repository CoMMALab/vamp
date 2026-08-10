#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/random/halton.hh>
#include <vamp/robots/rby1.hh>
#include <vamp/vector.hh>

using Robot = vamp::robots::RBY1;
using ParameterizedSpace = Robot::ParameterizedSpace;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

using TaskRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution, ParameterizedSpace>;
using TaskLocalPlanner = vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution>;
using CSpaceRRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

// Resolves a task-space State through IK and prints whether it's valid; used to sanity-check
// start/goal before handing them to RRTC (RRTC itself will silently fail to find a path if
// start/goal don't resolve, so it's worth knowing which case you're in).
auto resolve_and_report(const ParameterizedSpace::State &state, const std::string &label)
    -> std::pair<bool, Robot::ConfigurationBlock<rake>>
{
    ParameterizedSpace::StateBlock<rake> block;
    for (std::size_t i = 0; i < ParameterizedSpace::dimension; ++i)
    {
        block[i] = state.broadcast(i);
    }

    auto [valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
    std::cout << label << " resolve_block valid: " << std::boolalpha << valid << std::endl;
    return {valid, ambient_block};
}

// Collision-checks an ambient (joint-space) configuration block, printing the block and, on
// collision, the env/self-collision breakdown. Shared by both the c-space and task-space
// planning problems below since both ultimately validate through Robot::fkcc.
auto check_collision_free(
    const Robot::ConfigurationBlock<rake> &ambient_block,
    const std::string &label,
    const EnvironmentVector &environment_v) -> bool
{
    const bool collision_free = Robot::fkcc<rake>(environment_v, ambient_block);
    std::cout << label << " resolved configuration is collision free: " << std::boolalpha << collision_free
               << std::endl;

    std::cout << label << " resolved ambient configuration (lane 0): ";
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        std::cout << ambient_block[{i, 0}] << (i + 1 < Robot::dimension ? ", " : "\n");
    }

    if (not collision_free)
    {
        auto [env_collisions, self_collisions] = Robot::fkcc_debug<rake>(environment_v, ambient_block);
        std::cout << env_collisions.size() << " environment collisions, " << self_collisions.size()
                  << " self-collisions" << std::endl;

        for (const auto &pair : self_collisions)
        {
            std::cout << "  self-collision between links " << pair.first << " and " << pair.second
                       << std::endl;
        }
    }

    return collision_free;
}

// Problem-specific sampler for this planning problem: wraps an inner ParameterizedSpace
// sampler and, on every draw, (1) fixes the mobile base at the origin (this problem never
// moves it) and (2) remaps the t_mid_pose position from ParameterizedSpace::sample()'s
// default +/-2 box down to a +/-position_half_range box around a given start position.
// Torso, psi_left/psi_right, and the t_mid_pose orientation are left exactly as the inner
// sampler draws them ("sampled normally") -- this only overrides base and position.
struct RBY1FixedBaseSampler : public vamp::rng::RNG<Robot, ParameterizedSpace>
{
    // ParameterizedSpace::sample()'s default box for t_mid_pose position (see rby1.hh's
    // generated sample(): y[12..14] = -2 + 4*u). Used to invert back to the underlying
    // uniform draw before remapping into [start - half_range, start + half_range].
    static constexpr float default_position_lower = -2.0F;
    static constexpr float default_position_range = 4.0F;

    RBY1FixedBaseSampler(
        vamp::rng::RNG<Robot, ParameterizedSpace>::Ptr inner_in,
        std::array<float, 3> start_position_in,
        float position_half_range_in = 0.4F) noexcept
      : inner(std::move(inner_in))
      , start_position(start_position_in)
      , position_half_range(position_half_range_in)
    {
    }

    virtual ~RBY1FixedBaseSampler() = default;

    inline void reset() noexcept override
    {
        inner->reset();
        inner->dist.reset();
    }

    inline auto next() noexcept -> ParameterizedSpace::State override
    {
        auto sample = inner->next();
        auto y = sample.to_array();

        // base: fixed at the origin (x, y, cos(rz), sin(rz)).
        y[0] = 0.0F;
        y[1] = 0.0F;
        y[2] = 1.0F;
        y[3] = 0.0F;

        // t_mid_pose position: recover the inner sampler's underlying uniform draw from its
        // default box, then remap into the requested range around start_position.
        for (std::size_t i = 0; i < 3; ++i)
        {
            const float u = (y[12 + i] - default_position_lower) / default_position_range;
            y[12 + i] = start_position[i] - position_half_range + 2.0F * position_half_range * u;
        }

        return ParameterizedSpace::State(y.data());
    }

    vamp::rng::RNG<Robot, ParameterizedSpace>::Ptr inner;
    std::array<float, 3> start_position;
    float position_half_range;
};

// Smoke test + first end-to-end task-space RRTC plan over ParameterizedSpace: resolve_block(),
// compute_mid_pose(), and a single hand-picked point, then an actual RRTC<..., ParameterizedSpace>
// solve using ParameterizedLocalPlanner as the IK-resolving local planner. No sampler wrapper is
// needed -- vamp::rng::Halton<Robot, ParameterizedSpace> already draws directly from
// ParameterizedSpace::sample().
auto main(int, char **) -> int
{
    std::cout << std::boolalpha;
    std::cout << "Robot::dimension (ambient/joint space): " << Robot::dimension << std::endl;
    std::cout << "ParameterizedSpace::dimension (task space): " << ParameterizedSpace::dimension << std::endl;

    // --- compute_mid_pose(): derive the fixed T_mid -> hand offsets from a reference ambient
    // configuration.
    Robot::ConfigurationArray home_ambient = {
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
        -2.16360474e-03, 1.45860954e+00, -1.97468998e+00,  1.55147668e+00,  2.66302773e-01, -5.04128611e-01,
        1.17522024e-01,  9.91603153e-02,  4.46071004e-01, -1.67540527e+00, 3.29611651e-01,  5.74187322e-01,  1.65740047e+00,
        -1.22668093e+00,  3.92945961e-02,  1.20823988e+00, -6.24967729e-01, -1.61156583e-01,  5.79106863e-01, -2.52512556e+00,
    };

    ParameterizedSpace::compute_mid_pose(home_ambient);

    std::cout << "\nt_mid_left  (x, y, z, qx, qy, qz, qw): ";
    for (const auto v : ParameterizedSpace::t_mid_left)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    std::cout << "t_mid_right (x, y, z, qx, qy, qz, qw): ";
    for (const auto v : ParameterizedSpace::t_mid_right)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    EnvironmentInput environment;
    // TABLE_SIZE = [1.20, 0.60, 0.72]
    // TABLE_XYZ  = [0.32, -0.79, 0.36]
    // TABLE = SceneBox(size=TABLE_SIZE, xyz=TABLE_XYZ,
    //                 color=[0.82, 0.71, 0.55, 1.0], name="table")

    // create a cuboid obstacle representing the table in the environment
    // cuboids are defined by position, orientation (rpy), and half-extents (size/2)
    auto table_cuboid = vamp::collision::factory::cuboid::array(
        std::array<float, 3>{0.32F, -0.79F, 0.36F}, std::array<float, 3>{0.0F, 0.0F, 0.0F}, std::array<float, 3>{0.60F, 0.30F, 0.36F});
    environment.cuboids.emplace_back(table_cuboid);


    // add a box as attachment to the right hand
    // SIZE   = (0.386, 0.264, 0.11)   # (lx, ly, lz) full box dimensions [m]
    // OFFSET = (-0.193, 0, -0.13)    # box centre in the ee frame; -z points past the fingers
    // RPY    = (0.0, 0.0, 0.0)      # box orientation in the ee frame [rad]
    constexpr std::array<float, 3> box_size = {0.386F, 0.264F, 0.11F};
    constexpr std::array<float, 3> box_offset = {-0.193F, 0.0F, -0.13F};

    // RPY is zero, so the box is axis-aligned in the ee frame; approximate it with a grid of
    // spheres sized to its smallest (z) extent.
    const float box_sphere_radius = box_size[2] / 2.0F;

    // Evenly spaced sphere centers covering [-extent/2, extent/2] along one axis.
    auto grid_centers = [](float extent, float radius) -> std::vector<float>
    {
        const float half = extent / 2.0F;
        if (half <= radius)
        {
            return {0.0F};
        }

        const auto n = static_cast<int>(std::ceil(extent / (2.0F * radius))) + 1;
        std::vector<float> centers(n);
        for (int i = 0; i < n; ++i)
        {
            centers[i] =
                -half + radius + (2.0F * (half - radius)) * static_cast<float>(i) / static_cast<float>(n - 1);
        }

        return centers;
    };

    Eigen::Isometry3f box_attachment_tf = Eigen::Isometry3f::Identity();
    box_attachment_tf.translation() = Eigen::Vector3f(box_offset[0], box_offset[1], box_offset[2]);

    vamp::collision::Attachment<float> box_attachment(box_attachment_tf);
    box_attachment.end_effector = 1;  // right hand

    for (const auto cx : grid_centers(box_size[0], box_sphere_radius))
    {
        for (const auto cy : grid_centers(box_size[1], box_sphere_radius))
        {
            box_attachment.spheres.emplace_back(cx, cy, 0.0F, box_sphere_radius);
        }
    }

    // environment.attachments.emplace_back(box_attachment);

    EnvironmentVector environment_v(environment);

    // --- resolve_block() + fkcc() smoke test on a single hand-picked point.
    std::array<float, ParameterizedSpace::dimension> state_array = {
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
        -2.16360474e-03, 1.45860954e+00, -1.97468998e+00,  1.55147668e+00,  2.66302773e-01, -5.04128611e-01,
        -0.3154F, 0.9617F,
        0.5F, 0.0F, 0.92F, 0.0F, 0.0F, 0.0F, 1.0F  // t_mid_pose: x, y, z, qx, qy, qz, qw
    };

    ParameterizedSpace::State state(state_array.data());
    auto [valid, ambient_block] = resolve_and_report(state, "Smoke test");

    if (valid)
    {
        check_collision_free(ambient_block, "Smoke test", environment_v);
    }

    // ----------------- CSPACE PLANNING START -----------------
    Robot::ConfigurationArray start_ambient_array = {
        0, 0, 1, 0, 0.1682, 0.7809, -1.3941, 0.8259, -0.1562, -0.0383, -0.652034, 0.846282, -0.3154, -1.71205, 0.955445, 1.80577, 2.12459, -1.16081, -0.694043, 0.9617, -1.63612, -1.14521, 1.62616, -2.67506
    };

    Robot::ConfigurationArray goal_ambient_array = {
        0, 0, 1, 0, -0.0021636, 1.45861, -1.97469, 1.55148, 0.266303, -0.504129, 0.117516, 0.0991325, 0.4461, -1.67542, 0.32963, 0.574213, 1.65735, -1.22666, 0.039279, 1.2082, -0.625015, -0.16113, 0.579141, -2.5251,
    };

    Robot::Configuration start_cspace(start_ambient_array);
    Robot::Configuration goal_cspace(goal_ambient_array);

    // Broadcast into rake-wide blocks purely to reuse check_collision_free's fkcc-based check;
    // RRTC::solve itself takes the plain (non-blocked) Configuration below.
    Robot::ConfigurationBlock<rake> start_cspace_block;
    Robot::ConfigurationBlock<rake> goal_cspace_block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        start_cspace_block[i] = start_cspace.broadcast(i);
        goal_cspace_block[i] = goal_cspace.broadcast(i);
    }

    std::cout << "\n--- C-space planning problem ---" << std::endl;
    if (not check_collision_free(start_cspace_block, "C-space start", environment_v) or
        not check_collision_free(goal_cspace_block, "C-space goal", environment_v))
    {
        std::cout << "C-space start or goal is in collision; skipping c-space RRTC." << std::endl;
    }
    else
    {
        auto cspace_rng = std::make_shared<vamp::rng::Halton<Robot>>();
        vamp::planning::RRTCSettings cspace_settings;

        auto cspace_result = CSpaceRRTC::solve(start_cspace, goal_cspace, environment_v, cspace_settings, cspace_rng);

        std::cout << "\n--- C-space RRTC result ---" << std::endl;
        std::cout << "solved: " << cspace_result.solved << std::endl;
        std::cout << "cost: " << cspace_result.cost << std::endl;
        std::cout << "iterations: " << cspace_result.iterations << std::endl;
        std::cout << "nanoseconds: " << cspace_result.nanoseconds << std::endl;
        std::cout << "tree sizes (start, goal): " << cspace_result.size[0] << ", " << cspace_result.size[1]
                   << std::endl;
        std::cout << "path size: " << cspace_result.path.size() << std::endl;
    }
    // ----------------- CSPACE PLANNING END -----------------


    // --- Planning problem: start/goal in ParameterizedSpace's task space.
    std::array<float, ParameterizedSpace::dimension> start_array = {
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
        -2.16360474e-03, 1.45860954e+00, -1.97468998e+00,  1.55147668e+00,  2.66302773e-01, -5.04128611e-01,
        0.4461F, 1.2082F,
        0.5F, 0.0F, 0.22F, 0.0F, 0.0F, 0.0F, 1.0F
    };
    std::array<float, ParameterizedSpace::dimension> goal_array = {
        0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
        0.1682F, 0.7809F, -1.3941F, 0.8259F, -0.1562F, -0.0383F,
        -0.3154F, 0.9617F,
        0.5F, 0.0F, 0.92F, 0.0F, 0.0F, 0.0F, 1.0F
    };

    // std::array<float, ParameterizedSpace::dimension> goal_array = {
    //     0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0.00000000e+00,
    //     -0.0028, 1.4554, -1.9235, 1.5299, 0.2669, -0.4952,
    //     0.4662, 1.1943,
    //     0.5006, 0.0026, 0.2290, 0.0043, -0.0072, 0.0138, 0.9999
    // };

    ParameterizedSpace::State start_state(start_array.data());
    ParameterizedSpace::State goal_state(goal_array.data());

    std::cout << "\n--- Planning problem ---" << std::endl;
    const auto [start_valid, start_ambient] = resolve_and_report(start_state, "Start");
    const auto [goal_valid, goal_ambient] = resolve_and_report(goal_state, "Goal");

    if (not start_valid or not goal_valid)
    {
        std::cout << "Start or goal did not resolve through IK; skipping RRTC." << std::endl;
        return 0;
    }

    if (not check_collision_free(goal_ambient, "Goal", environment_v) or
        not check_collision_free(start_ambient, "Start", environment_v))
    {
        std::cout << "Start or goal resolves to a colliding ambient configuration; skipping RRTC." << std::endl;
        return 0;
    }

    // --- RRTC over ParameterizedSpace, with ParameterizedLocalPlanner as the IK-resolving
    // local planner. NN<ParameterizedSpace> (the KD-tree) is built implicitly inside RRTC::solve
    // from ParameterizedSpace::dimension/so3_offsets -- nothing to construct by hand here.
    //
    // Sampler: RBY1FixedBaseSampler wraps a plain Halton<Robot, ParameterizedSpace> and
    // restricts it to this problem -- base fixed at the origin, t_mid_pose position confined
    // to +/-0.4 around the start pose, everything else (torso, psi, orientation) unrestricted.
    auto inner_rng = std::make_shared<vamp::rng::Halton<Robot, ParameterizedSpace>>();
    const std::array<float, 3> start_position = {start_array[12], start_array[13], start_array[14]};
    auto rng = std::make_shared<RBY1FixedBaseSampler>(inner_rng, start_position, 0.4F);
    vamp::planning::RRTCSettings settings;

    // plan only for 20 steps for debugging
    settings.max_iterations = 2000;
    settings.range = 0.1F;

    auto result = TaskRRTC::solve<TaskLocalPlanner>(
        start_state, goal_state, environment_v, settings, rng, TaskLocalPlanner());

    // Print the resolved ambient configurations along the path, if any.
    if (result.solved)
    {
        std::cout << "\n--- Resolved ambient configurations along the path ---" << std::endl;
        for (std::size_t i = 0; i < result.path.size(); ++i)
        {
            const auto &state = result.path[i];
            // annoyingly resolve is only implemented for a full block, so we have to broadcast the state into a block first
            ParameterizedSpace::StateBlock<rake> block;
            for (std::size_t j = 0; j < ParameterizedSpace::dimension; ++j)
            {
                block[j] = state.broadcast(j);
            }
            auto [valid, ambient_block] = ParameterizedSpace::resolve_block<rake>(block);
            // std::cout << "Path index " << i << ": resolve valid = " << std::boolalpha << valid << ", ambient configuration = ";
            for (std::size_t j = 0; j < Robot::dimension; ++j)
            {
                // for j == 2,3 which is a continuous representation of a rotation, print the angle instead of the raw value
                if (j == 2)
                {
                    float angle = std::atan2(ambient_block[{3, 0}], ambient_block[{2, 0}]);
                    std::cout << angle << (j + 1 < Robot::dimension ? ", " : "\n");
                }
                else if (j == 3)
                {
                    // skip printing j == 3 since it's redundant with j == 2 for the angle
                    continue;
                }
                else
                {
                    std::cout << ambient_block[{j, 0}] << (j + 1 < Robot::dimension ? ", " : "\n");
                }
            }
        }
    }
    std::cout << "\n--- RRTC result ---" << std::endl;
    std::cout << "solved: " << result.solved << std::endl;
    std::cout << "cost: " << result.cost << std::endl;
    std::cout << "iterations: " << result.iterations << std::endl;
    std::cout << "nanoseconds: " << result.nanoseconds << std::endl;
    std::cout << "tree sizes (start, goal): " << result.size[0] << ", " << result.size[1] << std::endl;
    std::cout << "path size: " << result.path.size() << std::endl;

    return 0;
}
