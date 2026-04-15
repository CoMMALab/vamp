#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/crrtc_settings.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

// Start and goal configurations
// static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
// static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
// static constexpr Robot::ConfigurationArray goal = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
// static constexpr Robot::ConfigurationArray start = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};

// static constexpr Robot::ConfigurationArray start = {0.99,1.43,-0.05,-0.28,0.33,1.98,1.42};
static constexpr Robot::ConfigurationArray start =
    {-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294};
static constexpr Robot::ConfigurationArray goal =
    {1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238};

// static constexpr Robot::ConfigurationArray start = {0.88,1.05,0.0,-0.66,0.0,1.73,0.0};
// static constexpr Robot::ConfigurationArray goal = {-0.92,1.05,0.0,-0.66,0.0,1.73,0.0};

// static constexpr Robot::ConfigurationArray goal = {-0.839708,  0.496555, -0.630832, -0.573204,
// 0.232247,  1.8259,   -0.467584};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    // {0.65, 0, 0.50},
    // {0.75, 0, 0.50},
    // {0.35, 0.35, 0.25},
    // {0, 0.55, 0.25},
    // {-0.55, 0, 0.25},
    // {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
    // {0.35, -0.35, 0.25},
    // {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
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

/// Parse the JSON file which is expected to be an array of objects. For each object,
/// extract x,y,z,dx,dy,dz and optional roll,pitch,yaw and append a cuboid to environment.
/// Uses nlohmann::json for robust parsing.
static bool load_cuboids_from_json(EnvironmentInput &environment, const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return false;
    }

    nlohmann::json j;
    try
    {
        ifs >> j;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return false;
    }

    if (!j.is_array())
    {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return false;
    }

    for (const auto &obj : j)
    {
        if (!obj.is_object())
        {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") || !obj.contains("dx") ||
            !obj.contains("dy") || !obj.contains("dz"))
        {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try
        {
            float x = obj.at("x").get<float>();
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (obj.contains("roll"))
            {
                roll = obj.at("roll").get<float>();
            }
            if (obj.contains("pitch"))
            {
                pitch = obj.at("pitch").get<float>();
            }
            if (obj.contains("yaw"))
            {
                yaw = obj.at("yaw").get<float>();
            }

            std::array<float, 3> posf = {x, y, z};
            std::array<float, 3> rotf = {roll, pitch, yaw};
            std::array<float, 3> sizef = {dx / 2, dy / 2, dz / 2};
            // std::cout << "Creating cuboid at position: " << posf[0] << ", " << posf[1] << ", " << posf[2]
            // << " with size: " << sizef[0] << ", " << sizef[1] << ", " << sizef[2] << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return true;
}

auto main(int, char **) -> int
{
    // Setup RRTC and plan
    vamp::planning::CRRTCSettings crrtc_settings;

    float ranges[] = {0.5, 0.75, 1.0, 0.1, 0.2};
    // float ranges[] = {0.5, 0.75, 1.0, 1.5, 2.0};
    // float ranges[] = {0.5, 0.75};
    // float ranges[] = {0.1, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0};
    bool dd[] = {false};
    // bool dd[] = {false, true};
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM,
        vamp::planning::constraint::ProjMethod::
            OuterLM};  //, vamp::planning::constraint::ProjMethod::GradDesc};

    // float descend_rates[] = {0.1, 0.25, 0.5, 0.75, 1.0};
    float descend_rates[] = {0.75, 1.0};
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

                            // Build environment from JSON cuboids
                            EnvironmentInput environment;

                            // Try a few likely paths for the JSON file
                            const std::vector<std::string> candidate_paths = {
                                "resources/environments/cuboids/real_maze.json",
                            };

                            bool loaded = false;
                            for (const auto &p : candidate_paths)
                            {
                                if (load_cuboids_from_json(environment, p))
                                {
                                    std::cout << "Loaded cuboids from: " << p << std::endl;
                                    loaded = true;
                                    break;
                                }
                            }
                            if (!loaded)
                            {
                                std::cerr << "Failed to load cuboids JSON from any candidate path. Exiting."
                                          << std::endl;
                                return 1;
                            }

                            environment.sort();

                            std::vector<vamp::collision::Sphere<float>> spheres;
                            for (auto i = 0U; i < 9; i++)
                            {
                                spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.01));
                            }
                            auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
                            attach_transform.translation().z() = 0.0;
                            AttachmentInput attachment(attach_transform);

                            attachment.spheres.insert(
                                attachment.spheres.end(), spheres.cbegin(), spheres.cend());
                            environment.attach(attachment, 0);

                            auto env_v = EnvironmentVector(environment);
                            // Create RNG for planning
                            auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

                            std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                                -10.01, -10.01, -0.01, -0.01, -0.01, -10.01};

                            std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                                10.01, 10.01, 0.01, 0.01, 0.01, 10.01};

                            std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
                                {0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783}};
                            std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world =
                                {{1, 0, 0, 0, 0, 0, 0}};

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
                            crrtc_settings.rrtc_settings.max_iterations = 100000;
                            crrtc_settings.rrtc_settings.dynamic_domain = dyndom;
                            crrtc_settings.constraint_settings.projection_method = pm;
                            crrtc_settings.constraint_settings.descend_rate = descent_rate;
                            crrtc_settings.rrtc_settings.radius = 1.0;
                            crrtc_settings.constraint_settings.num_projection_iterations =
                                num_projection_iterations;
                            crrtc_settings.constraint_settings.insert_all_to_tree = insert_all_to_tree;
                            // std::cout << "\n\n-----------------Starting to cbirrt------------ " <<
                            // std::endl;
                            std::cout << range << ", " << dyndom << " " << pm << " " << descent_rate << " ";

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
