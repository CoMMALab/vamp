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

static constexpr Robot::ConfigurationArray start = {-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761};
static constexpr Robot::ConfigurationArray goal = {-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034};

// static constexpr Robot::ConfigurationArray goal = {-0.839708,  0.496555, -0.630832, -0.573204,
// 0.232247,  1.8259,   -0.467584};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    // {0.55, 0, 0.25},
    // {0.55, 0, 0.50},
    // {0.55, 0, 0.60},
    {0.56, 0, 0.450},
    // {0.1, 0, 0.7},
    // {0.35, 0.35, 0.25},
    // {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    // {0, -0.55, 0.25},
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

auto main(int, char **) -> int
{
    vamp::planning::CRRTCSettings crrtc_settings;
    vamp::planning::constraint::ProjMethod projection_method[] = {
        vamp::planning::constraint::ProjMethod::InnerLM,
        vamp::planning::constraint::ProjMethod::OuterLM,
        vamp::planning::constraint::ProjMethod::GradDesc};

    EnvironmentInput environment;
    std::ifstream infile("resources/environments/cuboids/shelf_panda.txt");
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

        if (!(iss >> x >> delim >> y >> delim >> z >> delim >> dx >> delim >> dy >> delim >> dz))
        {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        };
        // std::cout << x << ", " << y << ", " << z << ", " << dx << ", " << dy << ", " << dz << std::endl;
        environment.cuboids.emplace_back(
            vamp::collision::factory::cuboid::array({x, y, z}, {0.0, 0.0, 0.0}, {dx / 2, dy / 2, dz / 2}));
    }
    infile.close();

    environment.sort();
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // Create an attachment and add it
    Eigen::Transform<float, 3, Eigen::Isometry> attachment_transform(
        Eigen::Translation<float, 3>(0.0f, 0.0f, 0.05f));
    vamp::collision::Attachment sphere_attachment(attachment_transform);
    sphere_attachment.spheres.emplace_back(vamp::collision::factory::sphere::array({0.0, 0.0, 0.0}, 0.03));
    sphere_attachment.spheres.emplace_back(vamp::collision::factory::sphere::array({0.0, 0.0, 0.03}, 0.03));

    // env_v.attach(sphere_attachment);

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {-10.01, -10.01, -0.01, -0.01, -0.01, -0.01};

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {10.01, 10.01, 0.01, 0.01, 0.01, 0.01};

    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {
        {0, 0.707107, 0, 0.707107, 0.354, 0.7, 0.243}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

    vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world, eef_transforms, tsr_lower_bound, tsr_upper_bound);

    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms;
    // Eigen::Matrix<float, 4, 4> T;
    // T << 0,0,1,   0.354,   0,-1,0,      0.700,   1,0,0,    0.243,          0,           0,           0, 1;
    // eef_transforms[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);
    // std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_transforms_ref_frame_w_world;
    // T << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // eef_transforms_ref_frame_w_world[0] = Eigen::Transform<float, 3, Eigen::Isometry>(T);

    // vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> tsr_constraint(
    //     eef_transforms_ref_frame_w_world,
    //     eef_transforms,
    //     std::make_pair(tsr_lower_bound, tsr_upper_bound)
    // );

    vamp::planning::constraint::ComposableConstraints<Robot, rake, decltype(tsr_constraint)> task_constraint(
        tsr_constraint);

    crrtc_settings.rrtc_settings.range = 1.0;
    crrtc_settings.rrtc_settings.dynamic_domain = false;
    crrtc_settings.constraint_settings.projection_method = vamp::planning::constraint::ProjMethod::InnerLM;
    crrtc_settings.constraint_settings.descend_rate = 1.0;

    auto result = vamp::planning::CRRTC<Robot, rake, Robot::resolution, decltype(tsr_constraint)>::solve(
        Robot::Configuration(start), Robot::Configuration(goal), env_v, crrtc_settings, task_constraint, rng);

    if (result.path.size() > 0)
    {
        // Simplify path with default settings
        vamp::planning::SimplifySettings simplify_settings;
        auto simplify_result = vamp::planning::constraint::
            simplify_with_constraints<Robot, rake, Robot::resolution, decltype(tsr_constraint)>(
                result.path, env_v, task_constraint, simplify_settings);

        std::cout << "\nPrinting Result!! " << result.path.size() << std::endl;
        // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        std::ofstream outfile("trajectory.txt");
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

            auto fka = Robot::eefk(soln);
            // std::cout <<std::endl << eef_transforms[0].inverse() * fka[0].matrix() <<std::endl;
            // std::cout << std::endl;
            outfile << "\n";
        }
        outfile.close();
    }
    else
    {
        std::cout << "Failed to plan " << std::endl;
    }

    return 0;
}
