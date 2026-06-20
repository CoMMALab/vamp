#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/rrtc.hh>
#include <vamp/robots/bimanualiiwa.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::BimanualIiwa;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

float asin_function(float xx)
{
    float a, x, z;
    int sign, flag;

    x = xx;

    if( x > 0 )
        {
        sign = 1;
        a = x;
        }
    else
        {
        sign = -1;
        a = -x;
        }


    if( a < 1.0e-4 )
        {
        z = a;
        return z;
        }

    if( a > 0.5 )
        {
        z = 0.5 * (1.0 - a);
        x = sqrtf( z );
        flag = 1;
        }
    else
        {
        x = a;
        z = x * x;
        flag = 0;
        }

    z =
    (((( 4.2163199048E-2F * z
    + 2.4181311049E-2F) * z
    + 4.5470025998E-2F) * z
    + 7.4953002686E-2F) * z
    + 1.6666752422E-1F) * z * x
    + x;

    if( flag != 0 )
        {
        z = z + z;
        z = 1.5707963267948966F - z;
        }
    if( sign < 0 )
        z = -z;
    return( z );
}


auto main(int, char **) -> int
{

    // first check if atan implementation is correct
    std::cout << "Check atan2 implementation" << std::endl;
    for (auto i = -10; i <= 10; ++i)
    {
        for (auto j = -10; j <= 10; ++j)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }
            float angle = std::atan2(static_cast<float>(i), static_cast<float>(j));

            // now convert it to a FloatVector<rake> and call atan
            vamp::FloatVector<rake> i_vec;
            i_vec[0] = static_cast<float>(i);
            vamp::FloatVector<rake> j_vec;
            j_vec[0] = static_cast<float>(j);
            auto angle_vec = atan2(i_vec, j_vec);

            if (std::abs(angle - angle_vec[{0, 0}]) > 1e-6)
            {
                std::cout << "Mismatch for (" << i << ", " << j << "): "
                          << "std::atan2 = " << angle << ", "
                          << "FloatVector<rake>::atan2 = " << angle_vec[{0, 0}] << std::endl;
            }
        }
    }

    // check acos and asin   
    std::cout << "Check acos and asin implementation" << std::endl;
    for (auto i = -10; i <= 10; ++i)
    {
        float value = static_cast<float>(i) / 10.0f;  // values from -1.0 to 1.0
        if (value < -1.0f || value > 1.0f)
        {
            continue;  // skip out-of-range values
        }
        float angle = std::acos(value);
        float asin_angle = std::asin(value);

        vamp::FloatVector<rake> value_vec;
        value_vec[0] = value;
        auto angle_vec = acos(value_vec);
        auto asin_angle_vec = asin(value_vec);

        if (std::abs(angle - angle_vec[{0, 0}]) > 1e-6)
        {
            std::cout << "Mismatch for " << value << ": "
                      << "std::acos = " << angle << ", "
                      << "FloatVector<rake>::acos = " << angle_vec[{0, 0}] << std::endl;
        }
        if (std::abs(asin_angle - asin_angle_vec[{0, 0}]) > 1e-6)
        {
            std::cout << "Mismatch for " << value << ": "
                      << "std::asin = " << asin_angle << ", "
                      << "FloatVector<rake>::asin = " << asin_angle_vec[{0, 0}] << std::endl;
        }

    }






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

    // also use std::array and check ik parameterization
    std::array<float, Robot::dimension + 4> start_array;
    std::array<float, Robot::dimension + 4> goal_array;
    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        start_array[i] = start[i];
        goal_array[i] = goal[i];
    }
    start_array[Robot::dimension + 0] = 1.F;
    start_array[Robot::dimension + 1] = 1.F;
    start_array[Robot::dimension + 2] = -1.F;
    start_array[Robot::dimension + 3] = 0.6F;
    goal_array[Robot::dimension + 0] = 1.F;
    goal_array[Robot::dimension + 1] = 1.F;
    goal_array[Robot::dimension + 2] = -1.F;
    goal_array[Robot::dimension + 3] = 0.6F;

    auto [valid_start, output_start] = Robot::parameterized_ik(start_array);
    auto [valid_goal, output_goal] = Robot::parameterized_ik(goal_array);
    std::cout << "Start valid: " << valid_start << ", Goal valid: " << valid_goal << std::endl;
    for (auto i = 0U; i < Robot::ambient_dimension; ++i)
    {
        std::cout << output_start[i] << ", ";
    }
    std::cout << std::endl;
    for (auto i = 0U; i < Robot::ambient_dimension; ++i)
    {
        std::cout << output_goal[i] << ", ";
    }
    std::cout << std::endl;

    // first test if start and goal are valid
    if (!vamp::planning::validate_motion<Robot, rake, Robot::resolution>(Robot::Configuration(start), Robot::Configuration(start), env_v))
    {
        std::cerr << "Start configuration is invalid." << std::endl;
        return 1;
    }
    if (!vamp::planning::validate_motion<Robot, rake, Robot::resolution>(Robot::Configuration(goal), Robot::Configuration(goal), env_v))
    {

        std::cerr << "Goal configuration is invalid." << std::endl;
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
            // std::cout << param_valid << ": ";


            // for (auto i = 0U; i < Robot::ambient_dimension; ++i)
            // {
            //     std::cout << full_ambient_configuration[{i, 0}] << ", ";
            // }
            // for (auto i = 0U; i < Robot::dimension; ++i)
            // {
            //     std::cout << array[i] << ", ";
            // }

            // std::cout << std::endl;
        }
    }

    return 0;
}

