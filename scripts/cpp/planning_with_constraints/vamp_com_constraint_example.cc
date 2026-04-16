#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <vamp/planning/constraints/CoM_task_space_constraint.hh>
#include <vamp/planning/constraints/bimanual_task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/feet_task_space_constraint.hh>
#include <vamp/planning/constraints/self_collision_constraint.hh>
#include <vamp/planning/constraints/crrtc_settings.hh>
// #include <vamp/planning/simplify.hh>
#include <vamp/robots/g1_unitree.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution, 4>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {
    -0.01691, -0.00008, 0.52435, -0.00002, 0.00087,  0.00058,  -0.89966, -0.00864, 0.01527,
    1.75038,  -0.87267, 0.01757, -0.89936, 0.00875,  -0.01522, 1.75040,  -0.87267, -0.01753,
    1.59564,  0.14700,  0.37000, 0.00002,  0.00014,  0.00009,  -0.00003, -0.00006, 0.00000,
    -0.00001, -0.00001, 0.00017, 0.00007,  -0.00000, 0.00003,  0.00000,  0.00002};
static constexpr Robot::ConfigurationArray goal = {
    0.03977,  -0.00000, 0.72049,  0.00000,  0.03753,  -0.00002, -0.31120, 0.00002,  0.00004,
    0.64394,  -0.41457, -0.00000, -0.31121, -0.00004, 0.00000,  0.64394,  -0.41456, -0.00000,
    -0.00002, -0.00000, -0.24500, -0.00031, -0.00532, -0.00196, -0.00210, 0.00362,  -0.00156,
    0.00867,  -0.00646, 0.00363,  -0.00701, 0.00148,  -0.00521, 0.00134,  0.01348};

// static constexpr Robot::ConfigurationArray start =
// {-0.148774,1.59886,1.36434,-2.75007,0.544898,2.51704,-1.4485,2.07773,1.0024,-0.823622,-1.69743,-0.625681,2.59153,0.7243};
// static constexpr Robot::ConfigurationArray goal =
// {-1.62706,-0.100903,2.59477,-2.09287,1.2912,1.8256,-0.7243,1.75215,1.5907,-2.0261,-1.8123,-0.119768,2.50718,-0.7243};

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

    float ranges[] = {0.5, 0.75, 1.0};
    // float ranges[] = {1.0};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    // bool dd[] = {false};
    bool dd[] = {false, true};
    // vamp::planning::constraint::ProjMethod projection_method[] =
    // {vamp::planning::constraint::ProjMethod::InnerLM, vamp::planning::constraint::ProjMethod::OuterLM,
    // vamp::planning::constraint::ProjMethod::GradDesc};
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM, vamp::planning::constraint::ProjMethod::OuterLM};
    // vamp::planning::constraint::ProjMethod projection_method[] =
    // {vamp::planning::constraint::ProjMethod::InnerLM};

    // float descend_rates[] = {0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
    // float descend_rates[] = {1.0};
    int num_projection_iterations[] = {5, 10, 25, 50, 100};
    bool insert_all_to_tree[] = {false, true};

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

                            // Build sphere cage environment
                            EnvironmentInput environment;
                            // std::ofstream outfile_sph("spheres.txt");
                            // for (const auto &sphere : problem)
                            // {
                            //     outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," <<
                            //     radius << "\n";
                            //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere,
                            //     radius));
                            // }
                            // outfile_sph.close();
                            std::ifstream infile(
                                "resources/environments/cuboids/"
                                "humanoid_shelf.txt");
                            if (!infile.is_open())
                            {
                                std::cerr << "Failed to open file!" << std::endl;
                                return 1;
                            }

                            std::string line;
                            while (std::getline(infile, line))
                            {
                                std::istringstream iss(line);
                                char delim;
                                float x, y, z, dx, dy, dz;

                                if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >>
                                      delim >> dz))
                                {
                                    std::cerr << "Error reading line: " << line << std::endl;
                                    continue;
                                }
                                // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ",
                                // " << dz << std::endl;
                                environment.cuboids.emplace_back(
                                    vamp::collision::factory::cuboid::array(
                                        {x, y, z}, {0.0, 0.0, 0.0}, {dx / 2, dy / 2, dz / 2}));
                            }
                            infile.close();

                            environment.sort();

                            auto env_v = EnvironmentVector(environment);
                            // Create RNG for planning
                            auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                            std::array<float, 8> polygon_points = {
                                0.08, -0.045, 0.08, 0.045, 0.06, 0.045, 0.06, -0.045};

                            std::array<float, 6> lower_bound = {-0.001, -0.001, -0.001, -0.5, -0.001, -0.001};
                            std::array<float, 6> upper_bound = {0.001, 0.001, 0.001, 0.5, 0.001, 0.001};

                            std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                                -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0,
                                -10.0, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -0.05,
                                -0.05, -0.05, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05};

                            std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                                10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                                0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05};

                            // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
                            // Eigen::Matrix<float, 4, 4> T;
                            // T << 0.9999990, -0.0009995,  0.0010005, 0.0, 0.0010005,  0.9999990, -0.0009995,
                            // -0.303, -0.0009995,  0.0010005,  0.9999990, 0.0, 0.0, 0.0, 0.0, 1.0; const
                            // Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);

                            // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef>
                            // eef_transforms;

                            // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990,
                            // -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0,
                            // 0.0, 1.0; eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
                            // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990,
                            // -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0,
                            // 0.0, 1.0; eef_transforms[1] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
                            // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990,
                            // -0.0009995, 0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0,
                            // 0.0, 1.0; eef_transforms[2] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
                            // T << 0.9999990, -0.0009995,  0.0010005, 0.13, 0.0010005,  0.9999990,
                            // -0.0009995, -0.11, -0.0009995,  0.0010005,  0.9999990, -0.725, 0.0, 0.0,
                            // 0.0, 1.0; eef_transforms[3] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

                            // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef>
                            // eef_transforms_ref_frame_w_world; T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0,
                            // 0, 0, 1; eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3,
                            // Eigen::Isometry>(T); eef_transforms_ref_frame_w_world[1] =
                            // Eigen::Transform<float, 3, Eigen::Isometry>(T);
                            // eef_transforms_ref_frame_w_world[2] = Eigen::Transform<float, 3,
                            // Eigen::Isometry>(T); eef_transforms_ref_frame_w_world[3] =
                            // Eigen::Transform<float, 3, Eigen::Isometry>(T);

                            std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
                                {{1, 0, 0, 0, 0, 0, 0},
                                 {1, 0, 0, 0, 0.0, 0.0, 0.0},
                                 {1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0},
                                 {1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0}}};
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

                            // vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>
                            // feet_tsr_constraint(
                            //     eef_transforms_ref_frame_w_world,
                            //     eef_transforms,
                            //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
                            // );

                            std::array<float, 7> transform = {1., 0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0};
                            vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>
                                bimanual_task_constraint(transform, lower_bound, upper_bound);

                            // vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>
                            // bimanual_constraint(
                            //     target_pose,
                            //     std::make_pair(lower_bound, upper_bound)
                            // );
                            vamp::planning::constraint::CoMTaskSpaceConstraint<Robot, rake, 4> com_constraint(
                                polygon_points);

                            // vamp::planning::constraint::SelfCollisionConstraint<Robot, rake>
                            // self_collision_constraint;

                            vamp::planning::constraint::ComposableConstraints<
                                Robot,
                                rake,
                                decltype(feet_tsr_constraint),
                                decltype(com_constraint),
                                decltype(bimanual_task_constraint)>
                                task_constraint(
                                    feet_tsr_constraint, com_constraint, bimanual_task_constraint);
                            // vamp::planning::constraint::ComposableConstraints<Robot, rake,
                            // decltype(bimanual_constraint), decltype(com_constraint)> task_constraint(
                            //     bimanual_constraint,
                            //     com_constraint
                            // );

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
                            std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";

                            // std::cout << "\n\n-----------------Starting to cbirrt------------ " <<
                            // std::endl;
                            std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";
                            auto result = vamp::planning::CRRTC<
                                Robot,
                                rake,
                                Robot::resolution,
                                decltype(feet_tsr_constraint),
                                decltype(com_constraint),
                                decltype(bimanual_task_constraint)>::
                                solve(
                                    Robot::Configuration(start),
                                    Robot::Configuration(goal),
                                    env_v,
                                    crrtc_settings,
                                    task_constraint,
                                    rng);
                            // auto result =
                            //     vamp::planning::CRRTC<Robot, rake, Robot::resolution,
                            //     decltype(bimanual_constraint),
                            //     decltype(com_constraint)>::solve(Robot::Configuration(start),
                            //     Robot::Configuration(goal), env_v, rrtc_settings, task_constraint, rng);

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
                                    std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
                                    // Output configurations of simplified path
                                    std::cout << std::fixed << std::setprecision(3);
                                    std::ofstream outfile("trajectory.txt");
                                    for (const auto &config : result.path)
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
                  << ", " << a.num_projection_iterations << ", " << a.insert_all_to_tree << ", "
                  << a.planning_time / 1e6 << ", " << a.planning_iterations << ", " << a.path_length
                  << std::endl;
    }
    std::cout << "---------------------------" << std::endl;

    return 0;
}
