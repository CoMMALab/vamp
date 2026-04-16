#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/feet_task_space_constraint.hh>
#include <vamp/planning/constraints/CoM_task_space_constraint.hh>
#include <vamp/planning/constraints/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/closed_link_constraint.hh>

#include <vamp/planning/constraints/crrtc_settings.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <vamp/planning/constraints/simplify_constraints.hh>

// #include "vamp/parsers/mjcf_parser.hh"

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/digit.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::Digit;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using AttachmentInput = vamp::collision::Attachment<float>;

static constexpr Robot::ConfigurationArray standing_pose = {
    0.0287480000000000,  -0.0189052000000000,   -0.0145299000000000, 0.00967345000000000,
    0.00825856000000000, -0.00215209000000000,  0.397209000000000,   -0.00820680000000000,
    0.284002000000000,   0.290042000000000,     -0.0213535000000000, -0.235481000000000,
    -0.0319210000000000, 0.00127043000000000,   -0.0782953000000000, 1.04159000000000,
    0.0284093000000000,  -0.0123198000000000,   -0.358284000000000,  -0.0133610000000000,
    -0.278441000000000,  -0.268447000000000,    0.0177607000000000,  0.212721000000000,
    -0.0692397000000000, -0.000343739000000000, 0.0689620000000000,  -1.23829000000000,
    0.0369306000000000,  -0.00862385000000000};

static constexpr Robot::ConfigurationArray box_top_shelf_pickup = {
    0.00779,  -0.02074, 0.00461, -0.00428, -0.00896, -0.00528, 0.38120,  0.00154, 0.28644,  0.32323,
    -0.00734, -0.29696, 0.07485, 0.01125,  0.08865,  -0.26348, -0.00645, 0.11539, -0.37495, -0.00330,
    -0.29074, -0.31914, 0.01103, 0.28118,  -0.13812, 0.00576,  0.16191,  0.27674, 0.06755,  -0.09470};

static constexpr Robot::ConfigurationArray rack_2 = {
    0.00927,  -0.01219, -0.47283, -0.01411, -0.02955, -0.00937, 0.40005,  0.01408, -0.23717, -0.84704,
    -0.02429, 0.90190,  -0.48186, 0.04543,  -0.17059, -0.36363, 0.06204,  1.09005, -0.36164, -0.00267,
    0.22962,  0.84538,  0.02448,  -0.90900, 0.40052,  0.03680,  -0.29646, 0.37895, -0.15478, -1.16821};

static constexpr Robot::ConfigurationArray rack_3 = {
    0.00927,  -0.01219, -0.47283, -0.01411, -0.02955, -0.00937, 0.40005,  0.01408, -0.23717, -0.84704,
    -0.02429, 0.90190,  -0.48186, 0.04543,  -0.17059, -0.36363, 0.06204,  1.09005, -0.36164, -0.00267,
    0.22962,  0.84538,  0.02448,  -0.90900, 0.40052,  0.03680,  -0.29646, 0.37895, -0.15478, -1.16821};

struct Attempt
{
    float range;
    bool dynamic_domain;
    vamp::planning::constraint::ProjMethod proj_method;
    float descend_rate;
    int num_projection_iterations;
    bool insert_all_to_tree;
    bool success;
    std::size_t planning_time;
    std::size_t planning_iterations;
    std::size_t path_length;

    bool operator<(const Attempt &other) const
    {
        return planning_time < other.planning_time;
    }
};

auto main(int, char **) -> int
{
    vamp::planning::CRRTCSettings crrtc_settings;

    float ranges[] = {0.75, 0.5};
    bool dd[] = {false, true};
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM};

    float descend_rates[] = {0.75, 1.0};
    int num_projection_iterations[] = {10, 25};
    bool insert_all_to_tree[] = {false, true};

    // float descend_rates[] = {1.0};
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
                            // if(pm < 2) continue;

                            EnvironmentInput environment;
                            // auto geoms = vamp::utils::parser::parseMJCF(
                            //     "resources/environments/cuboids/wooden_shelf.xml");
                            // for (const auto &g : geoms)
                            // {
                            //     if (g.type == vamp::utils::parser::GeomType::BOX)
                            //     {
                            //         // std::cout << "Adding cuboid with pos " << g.world_pose.pos.x << ","
                            //         <<
                            //         // g.world_pose.pos.y << "," << g.world_pose.pos.z << " and size " <<
                            //         // g.size.x << "," << g.size.y << "," << g.size.z << std::endl;
                            //         environment.cuboids.emplace_back(
                            //             vamp::collision::factory::cuboid::array(
                            //                 {g.world_pose.pos.x, g.world_pose.pos.y, g.world_pose.pos.z},
                            //                 {0.0, 0.0, 0.0},
                            //                 {g.size.x, g.size.y, g.size.z}));
                            //     }
                            // }

                            environment.sort();
                            auto env_v = EnvironmentVector(environment);

                            std::array<float, 8> polygon_points = {
                                0.01, -0.01, 0.01, 0.03, -0.05, 0.03, -0.05, -0.01};

                            std::array<float, 6> lower_bound = {-0.001, -0.001, -0.001, -0.1, -10.1, -10.1};
                            std::array<float, 6> upper_bound = {0.001, 0.001, 0.001, 0.1, 10.1, 10.1};

                            std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                                -10.0, -10.0, -10.0,  -10.0,  -10.0,  -10.0,  -10.0,  -10.0,
                                -10.0, -10.0, -10.0,  -10.0,  -0.001, -0.001, -0.001, -0.1,
                                -0.1,  -0.1,  -0.001, -0.001, -0.001, -0.1,   -0.1,   -0.1};

                            std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                                10.0,  10.0,  10.0,  10.0, 10.0, 10.0, 10.0,  10.0,  10.0,  10.0, 10.0, 10.0,
                                0.001, 0.001, 0.001, 0.1,  0.1,  0.1,  0.001, 0.001, 0.001, 0.1,  0.1,  0.1};

                            std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
                                {{1, 0, 0, 0, 0, 0, 0},
                                 {1, 0, 0, 0, 0.0, 0.0, 0.0},
                                 {0.59, 0.38, 0.4, 0.59, -0.02070, 0.06015, -0.95335},
                                 {0.61, -0.36, 0.35, -0.61, -0.02228, -0.11609, -0.94832}}};
                            std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world =
                                {{{1, 0, 0, 0, 0, 0, 0},
                                  {1, 0, 0, 0, 0, 0, 0},
                                  {1, 0, 0, 0, 0, 0, 0},
                                  {1, 0, 0, 0, 0, 0, 0}}};

                            vamp::planning::constraint::FeetTaskSpaceConstraint<Robot, rake>
                                feet_tsr_constraint(
                                    eef_transforms_ref_frame_w_world,
                                    eef_transforms,
                                    tsr_lower_bound,
                                    tsr_upper_bound);

                            std::array<float, 7> transform = {
                                0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356};
                            vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>
                                bimanual_task_constraint(transform, lower_bound, upper_bound);
                            vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
                                polygon_points);

                            vamp::planning::constraint::ClosedLinkConstraint<Robot, rake>
                                closed_link_constraint;

                            vamp::planning::constraint::ComposableConstraints<
                                Robot,
                                rake,
                                vamp::planning::constraint::FeetTaskSpaceConstraint<Robot, rake>,
                                vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4>,
                                vamp::planning::constraint::ClosedLinkConstraint<Robot, rake>,
                                vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>>
                                transport_task_constraint(
                                    feet_tsr_constraint,
                                    com_constraint,
                                    closed_link_constraint,
                                    bimanual_task_constraint);

                            vamp::planning::constraint::ComposableConstraints<
                                Robot,
                                rake,
                                vamp::planning::constraint::FeetTaskSpaceConstraint<Robot, rake>,
                                vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4>,
                                vamp::planning::constraint::ClosedLinkConstraint<Robot, rake>>
                                non_transport_task_constraint(
                                    feet_tsr_constraint, com_constraint, closed_link_constraint);

                            crrtc_settings.rrtc_settings.range = range;
                            crrtc_settings.rrtc_settings.max_iterations = 100000;
                            crrtc_settings.rrtc_settings.dynamic_domain = dyndom;
                            crrtc_settings.constraint_settings.projection_method = pm;
                            crrtc_settings.constraint_settings.descend_rate = descent_rate;

                            crrtc_settings.rrtc_settings.radius = 10.0;
                            crrtc_settings.constraint_settings.num_projection_iterations =
                                num_projection_iterations;
                            crrtc_settings.constraint_settings.insert_all_to_tree = insert_all_to_tree;

                            // std::cout << "\n\n-----------------Starting to cbirrt------------ " <<
                            // std::endl;
                            std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " "
                                      << insert_all_to_tree << " " << std::endl;

                            auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                            std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
                            auto result = vamp::planning::CRRTC<
                                Robot,
                                rake,
                                Robot::resolution,
                                decltype(feet_tsr_constraint),
                                decltype(com_constraint),
                                decltype(closed_link_constraint),
                                decltype(bimanual_task_constraint)>::
                                solve(
                                    Robot::Configuration(box_top_shelf_pickup),
                                    Robot::Configuration(rack_2),
                                    env_v,
                                    crrtc_settings,
                                    transport_task_constraint,
                                    rng);

                            if (result.path.size() > 0)
                            {
                                Attempt a{
                                    range,
                                    dyndom,
                                    pm,
                                    descent_rate,
                                    num_projection_iterations,
                                    insert_all_to_tree,
                                    true,
                                    result.nanoseconds,
                                    result.iterations,
                                    result.path.size()};

                                if ((succ_attempts.size() == 0) ||
                                    (succ_attempts.size() > 0 && a < succ_attempts[0]))
                                {
                                    vamp::planning::SimplifySettings simplify_settings;
                                    auto simplify_result =
                                        vamp::planning::constraint::simplify_with_constraints<
                                            Robot,
                                            rake,
                                            Robot::resolution,
                                            decltype(feet_tsr_constraint),
                                            decltype(com_constraint),
                                            decltype(closed_link_constraint),
                                            decltype(bimanual_task_constraint)>(
                                            result.path, env_v, transport_task_constraint, simplify_settings);
                                    std::cout << "Simplify took " << result.nanoseconds / 1e6 << " ms"
                                              << std::endl;

                                    std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
                                    simplify_result.path.interpolate_to_resolution(Robot::resolution);
                                    // Output configurations of simplified path
                                    std::cout << std::fixed << std::setprecision(3);
                                    std::ofstream outfile("/src/trajectory.txt");
                                    for (const auto &config : simplify_result.path)
                                    {
                                        const auto &array = config.to_array();
                                        Robot::ConfigurationArray soln;
                                        bool first = true;
                                        for (auto i = 0U; i < Robot::dimension; ++i)
                                        {
                                            // std::cout << array[i] << ", ";
                                            soln[i] = array[i];

                                            if (!first)
                                            {
                                                outfile << ",";
                                            }
                                            outfile << array[i];
                                            first = false;
                                        }

                                        // auto fka = Robot::eefk(soln);
                                        // std::cout <<std::endl << fka.matrix() <<std::endl;
                                        // std::cout << std::endl;
                                        outfile << "\n";
                                    }
                                    outfile.close();
                                }
                                // std::cin.ignore();
                                succ_attempts.push_back(a);
                                std::sort(succ_attempts.begin(), succ_attempts.end());
                            }
                        }  // insert to tree
                    }  // num projection iterations
                }  // descend rate
            }  // projection method
        }  // dynamic domain
    }  // range

    std::cout << "------Final Result --------" << std::endl;
    std::sort(succ_attempts.rbegin(), succ_attempts.rend());
    for (const auto &a : succ_attempts)
    {
        std::cout << a.range << ", " << a.dynamic_domain << ", " << a.proj_method << ", " << a.descend_rate
                  << ", " << a.num_projection_iterations << ", " << a.insert_all_to_tree << ", "
                  << a.planning_time / 1e6 << ", " << a.planning_iterations << ", " << a.path_length
                  << std::endl;
    }
    std::cout << "---------------------------" << std::endl;

    return 0;
}
