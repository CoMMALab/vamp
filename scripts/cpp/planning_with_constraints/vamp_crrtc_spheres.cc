#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/crrtc_settings.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <vamp/planning/constraints/simplify_constraints.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;

static constexpr Robot::ConfigurationArray start =
    {1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100};
static constexpr Robot::ConfigurationArray goal =
    {-1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000};

// static constexpr Robot::ConfigurationArray goal = {-0.839708,  0.496555, -0.630832, -0.573204,
// 0.232247,  1.8259,   -0.467584};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    {0.56, 0, 0.450},
    {0.1, 0, 0.7},
    // {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    // {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    {0, 0.55, 0.8},
    {-0.35, 0.35, 0.8},
    {-0.55, 0, 0.8},
    {-0.35, -0.35, 0.8},
    {0, -0.55, 0.8},
    {0.35, -0.35, 0.8},
};
// Radius for obstacle spheres
static constexpr float radius = 0.15;

struct Attempt
{
    float range;
    bool dynamic_domain;
    vamp::planning::constraint::ProjMethod proj_method;
    float descend_rate;
    int num_projection_iterations;
    float std_dev_scaling_factor;
    bool insert_all_to_tree;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;
    std::size_t simplify_time;
    std::size_t simplify_path_length;
    std::size_t total_time;
    float simplified_path_cost;

    bool operator<(const Attempt &other) const
    {
        return planning_time < other.planning_time;
    }
};

auto main(int, char **) -> int
{
    // Setup RRTC and plan
    vamp::planning::CRRTCSettings crrtc_settings;

    float ranges[] = {0.5, 1.0, 1.5, 2.0};
    bool dd[] = {false, true};
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM,
        vamp::planning::constraint::ProjMethod::
            OuterLM};  //, vamp::planning::constraint::ProjMethod::GradDesc};

    // float descend_rates[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
    // float descend_rates[] = {1.0};
    //
    int num_projection_iterations[] = {5, 10, 25, 50, 100};
    bool insert_all_to_tree[] = {false, true};
    float std_dev_scaling_factors[] = {0.01F, 0.1F, 0.2F};

    std::vector<Attempt> succ_attempts;

    for (const auto range : ranges)
    {
        for (const auto dyndom : dd)
        {
            for (const auto &pm : projection_method)
            {
                for (const auto descent_rate : descend_rates)
                {
                    for (const auto num_projection_iterations : num_projection_iterations)
                    {
                        for (const auto insert_all_to_tree : insert_all_to_tree)
                        {
                            for (const auto std_dev_scaling_factor : std_dev_scaling_factors)
                            {
                                // if(pm < 2) continue;

                                // Build sphere cage environment
                                EnvironmentInput environment;
                                std::ofstream outfile_sph("/src/spheres.txt");
                                for (const auto &sphere : problem)
                                {
                                    outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
                                    environment.spheres.emplace_back(
                                        vamp::collision::factory::sphere::array(sphere, radius));
                                }
                                outfile_sph.close();

                                environment.sort();
                                auto env_v = EnvironmentVector(environment);
                                // Create RNG for planning
                                auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                                std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                                    -0.001, -10.01, -0.001, -0.01, -0.01, -0.01};

                                std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                                    0.001, 10.01, 0.001, 0.01, 0.01, 0.01};

                                std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
                                    {0, 1, 0, 0, 0.3486, 0.647752, 0.2399}};
                                std::array<std::array<float, 7>, Robot::n_eef>
                                    eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

                                vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> tsr_constraint(
                                    eef_transforms_ref_frame_w_world,
                                    eef_transforms,
                                    tsr_lower_bound,
                                    tsr_upper_bound);

                                vamp::planning::constraint::ComposableConstraints<
                                    Robot,
                                    rake,
                                    vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>>
                                    task_constraint(tsr_constraint);

                                crrtc_settings.rrtc_settings.range = range;
                                crrtc_settings.rrtc_settings.dynamic_domain = dyndom;
                                crrtc_settings.constraint_settings.projection_method = pm;
                                crrtc_settings.constraint_settings.descend_rate = descent_rate;
                                crrtc_settings.rrtc_settings.radius = 1.0;
                                crrtc_settings.constraint_settings.num_projection_iterations =
                                    num_projection_iterations;
                                crrtc_settings.constraint_settings.insert_all_to_tree = insert_all_to_tree;
                                crrtc_settings.constraint_settings.std_dev_scaling_factor =
                                    std_dev_scaling_factor;
                                // std::cout << "\n\n-----------------Starting to cbirrt------------ " <<
                                // std::endl; std::cout << range << ", " << dyndom << " " << pm << " " <<
                                // descent_rate << " ";

                                auto result = vamp::planning::
                                    CRRTC<Robot, rake, Robot::resolution, decltype(tsr_constraint)>::solve(
                                        Robot::Configuration(start),
                                        Robot::Configuration(goal),
                                        env_v,
                                        crrtc_settings,
                                        task_constraint,
                                        rng);

                                if (result.path.size() > 0)
                                {
                                    vamp::planning::SimplifySettings simplify_settings;
                                    auto simplify_result =
                                        vamp::planning::constraint::simplify_with_constraints<
                                            Robot,
                                            rake,
                                            Robot::resolution,
                                            decltype(tsr_constraint)>(
                                            result.path,
                                            env_v,
                                            task_constraint,
                                            simplify_settings,
                                            rng,
                                            pm,
                                            descent_rate,
                                            num_projection_iterations,
                                            insert_all_to_tree);

                                    Attempt a{
                                        range,
                                        dyndom,
                                        pm,
                                        descent_rate,
                                        num_projection_iterations,
                                        std_dev_scaling_factor,
                                        insert_all_to_tree,
                                        true,
                                        result.nanoseconds,
                                        result.iterations,
                                        result.path.size(),
                                        simplify_result.nanoseconds,
                                        simplify_result.path.size(),
                                        result.nanoseconds + simplify_result.nanoseconds,
                                        simplify_result.path.cost()};
                                    succ_attempts.push_back(a);
                                    std::sort(succ_attempts.begin(), succ_attempts.end());
                                }
                                else
                                    std::cout << "No path found with settings: " << range << ", " << dyndom << " " << pm << " "
                                              << descent_rate << " " << num_projection_iterations << " "
                                              << std_dev_scaling_factor << " " << insert_all_to_tree
                                              << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }
    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    for (const auto &a : succ_attempts)
    {
        std::cout << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate
                  << ", " << a.num_projection_iterations << ", " << a.std_dev_scaling_factor << ", "
                  << a.insert_all_to_tree << ", " << a.planning_time / 1e6 << ", " << a.planning_iterations
                  << ", " << a.path_length << ", " << a.simplify_time / 1e6 << ", " << a.simplify_path_length
                  << ", " << a.total_time / 1e6 << ", " << a.simplified_path_cost << std::endl;
    }
    std::cout << "---------------------------" << std::endl;

    // auto fka = Robot::eefk(goal);
    std::cout << std::endl << Robot::eefk(goal)[0].matrix() << std::endl;
    std::cout << std::endl << Robot::eefk(start)[0].matrix() << std::endl;

    return 0;
}
