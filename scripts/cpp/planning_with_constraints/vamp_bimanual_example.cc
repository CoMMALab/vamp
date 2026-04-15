#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/bimanual_task_space_constraint.hh>
#include <vamp/planning/crrtc_settings.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <vamp/planning/constraints/simplify_constraints.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/g1_unitree.hh>
#include <vamp/random/halton.hh>
#include <fstream>

using Robot = vamp::robots::G1Unitree;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {
    0.0,    0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    -1.767, -0.16, 0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static constexpr Robot::ConfigurationArray goal = {
    0.0,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.702, -0.16, 0.52, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

    bool operator<(const Attempt &other) const
    {
        return planning_time < other.planning_time;
    }
};

auto main(int, char **) -> int
{
    vamp::planning::CRRTCSettings crrtc_settings;

    float ranges[] = {0.5, 0.75, 1.0, 1.5, 2.0};
    // float ranges[] = {0.5, 0.75};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    // bool dd[] = {false};
    bool dd[] = {false, true};
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM,
        vamp::planning::constraint::ProjMethod::
            OuterLM};  //, vamp::planning::constraint::ProjMethod::GradDesc};

    // float descend_rates[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
    // float descend_rates[] = {1.0};
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
                                // std::ofstream outfile_sph("spheres.txt");
                                // for (const auto &sphere : problem)
                                // {
                                //     outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << ","
                                //     << radius << "\n";
                                //     environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere,
                                //     radius));
                                // }
                                // outfile_sph.close();
                                std::ifstream infile("resources/environments/cuboids/"
                                                     "shelf_panda.txt");
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

                                    if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >>
                                          dy >> delim >> dz))
                                    {
                                        std::cerr << "Error reading line: " << line << std::endl;
                                        continue;
                                    };
                                    // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy <<
                                    // ", " << dz << std::endl;
                                    environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(
                                        {x, y, z}, {0.0, 0.0, 0.0}, {dx / 2, dy / 2, dz / 2}));
                                }
                                infile.close();

                                environment.sort();

                                auto env_v = EnvironmentVector(environment);
                                // Create RNG for planning
                                auto rng = std::make_shared<vamp::rng::Halton<Robot>>();
                                // auto rng = std::make_shared<vamp::rng::XORShift<Robot>>(2, 3);

                                std::array<float, 6> lower_bound = {
                                    -0.001, -0.001, -0.001, -0.001, -0.001, -0.001};
                                std::array<float, 6> upper_bound = {0.001, 0.001, 0.001, 0.001, 0.001, 0.001};

                                // Eigen::Transform<float, 3, Eigen::Isometry> target_pose;
                                // Eigen::Matrix<float, 4, 4> T;
                                // T << 1, 0, 0, 0, 0, 1, 0, -0.3, 0, 0, 1, 0, 0, 0, 0, 1;
                                // const Eigen::Transform<float, 3, Eigen::Isometry> target_pose(T);
                                // vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>
                                // bimanual_task_constraint(target_pose, std::make_pair(lower_bound,
                                // upper_bound));

                                std::array<float, 7> transform = {1.00, 0.0, 0.00, 0.00, 0.0, -0.3, 0.0};
                                vamp::planning::constraint::BimanualTaskSpaceConstraint<Robot, rake>
                                    bimanual_task_constraint(transform, lower_bound, upper_bound);

                                vamp::planning::constraint::
                                    ComposableConstraints<Robot, rake, decltype(bimanual_task_constraint)>
                                        task_constraint(bimanual_task_constraint);

                                crrtc_settings.rrtc_settings.range = range;
                                crrtc_settings.rrtc_settings.max_iterations = 100000;
                                crrtc_settings.rrtc_settings.dynamic_domain = dyndom;
                                crrtc_settings.constraint_settings.projection_method = pm;
                                crrtc_settings.constraint_settings.descend_rate = descent_rate;
                                crrtc_settings.constraint_settings.num_projection_iterations =
                                    num_projection_iterations;
                                crrtc_settings.constraint_settings.insert_all_to_tree = insert_all_to_tree;
                                crrtc_settings.constraint_settings.std_dev_scaling_factor =
                                    std_dev_scaling_factor;
                                // std::cout << "\n\n-----------------Starting to cbirrt------------ " <<
                                // std::endl;


                                std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate
                                          << " ";
                                auto result = vamp::planning::CRRTC<
                                    Robot,
                                    rake,
                                    Robot::resolution,
                                    decltype(bimanual_task_constraint)>::
                                    solve(
                                        Robot::Configuration(start),
                                        Robot::Configuration(goal),
                                        env_v,
                                        crrtc_settings,
                                        task_constraint,
                                        rng);

                                if (result.path.size() > 0)
                                {
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
                                        result.path.size()};

                                    if ((succ_attempts.size() == 0) ||
                                        (succ_attempts.size() > 0 && a < succ_attempts[0]))
                                    {
                                        std::cout << "\nPrinting Result!! " << result.path.size()
                                                  << std::endl;
                                        // Output configurations of simplified path
                                        std::cout << std::fixed << std::setprecision(3);
                                        std::ofstream outfile("trajectory.txt");


                                        // Simplify path with default settings
                                        vamp::planning::SimplifySettings simplify_settings;
                                        auto simplify_result =
                                            vamp::planning::constraint::simplify_with_constraints<
                                                Robot,
                                                rake,
                                                Robot::resolution,
                                                decltype(bimanual_task_constraint)>(
                                                result.path, env_v, task_constraint, simplify_settings, rng);
                                        std::cout << "Simplify took " << result.nanoseconds / 1e6 << " ms"
                                                  << std::endl;

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
                  << ", " << a.path_length << std::endl;
    }
    std::cout << "---------------------------" << std::endl;

    return 0;
}
