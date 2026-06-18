#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/parameterized_ik_validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/robots/bimanual_iiwa.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::BimanualIiwa;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;


auto main(int, char **) -> int
{



    EnvironmentInput environment;
    std::ifstream infile(
        "resources/iiwa/cuboids/"
        "shelf_drake.txt");
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
        }
        environment.cuboids.emplace_back(
            vamp::collision::factory::cuboid::array(
                {x, y, z}, {0.0, 0.0, 0.0}, {dx / 2, dy / 2, dz / 2}));
    }
    infile.close();

    environment.sort();

    auto env_v = EnvironmentVector(environment);
    std::ifstream infile_start_goal(
        "resources/iiwa/"
        "example_points.txt");
    if (!infile_start_goal.is_open())
    {
        std::cerr << "Failed to open start/goal file: "
                        "resources/iiwa/example_points.txt"
                    << std::endl;
        return 1;
    }

    std::string line_start, line_goal;
    if (!std::getline(infile_start_goal, line_start) ||
        !std::getline(infile_start_goal, line_goal))
    {
        std::cerr << "Start/goal file must contain at least two lines."
                    << std::endl;
        return 1;
    }
    infile_start_goal.close();

    auto parse_configuration_line = [](const std::string &line,
                                        Robot::ConfigurationArray &cfg) -> bool
    {
        std::istringstream iss(line);
        char comma = '\0';
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            if (!(iss >> cfg[i]))
            {
                return false;
            }
            if (i + 1 < Robot::dimension)
            {
                if (!(iss >> comma) || comma != ',')
                {
                    return false;
                }
            }
        }
        return true;
    };

    Robot::ConfigurationArray start, goal;
    if (!parse_configuration_line(line_start, start))
    {
        std::cerr << "Error reading start configuration from line: " << line_start
                    << std::endl;
        return 1;
    }
    if (!parse_configuration_line(line_goal, goal))
    {
        std::cerr << "Error reading goal configuration from line: " << line_goal
                    << std::endl;
        return 1;
    }



    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // Setup RRTC and plan
    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.5;
    rrtc_settings.dynamic_domain = false;

    auto result =
        RRTC::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, rrtc_settings, rng);

    // If successful
    if (result.path.size() > 0)
    {
        std::cout << "Found solution in " << result.nanoseconds / 1e6 << " ms in " << result.iterations << " iterations." << std::endl;
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            // std::array<float, Robot::dimension+4> parameterized_construct;
            // for (auto i = 0U; i < Robot::dimension; ++i)
            // {
            //     parameterized_construct[i] = array[i];
            // }
            // parameterized_construct[Robot::dimension + 0] = 1.F;
            // parameterized_construct[Robot::dimension + 1] = 1.F;
            // parameterized_construct[Robot::dimension + 2] = -1.F;
            // parameterized_construct[Robot::dimension + 3] = 0.6F;

            // auto [param_valid, full_ambient_configuration] = Robot::template parameterized_ik(parameterized_construct);


            vamp::FloatVector<rake, Robot::dimension + 4> block;

            // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                block[i] = config.broadcast(i);
            }
            block[Robot::dimension + 0] = 1.F;
            block[Robot::dimension + 1] = 1.F;
            block[Robot::dimension + 2] = -1.F;
            block[Robot::dimension + 3] = 0.6F;

            auto [param_valid, full_ambient_configuration] = Robot::template parameterized_ik<vamp::FloatVector<rake, Robot::dimension + 4>, rake>(block);
            std::cout << param_valid << ": ";


            for (auto i = 0U; i < Robot::ambient_dimension; ++i)
            {
                std::cout << full_ambient_configuration[{i, 0}] << ", ";
            }
            // for (auto i = 0U; i < Robot::dimension; ++i)
            // {
            //     std::cout << array[i] << ", ";
            // }

            std::cout << std::endl;
        }
    }

    return 0;
}

