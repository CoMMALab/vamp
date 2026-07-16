#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/robots/iiwamarker.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::IiwaMarker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
// using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;

auto main(int, char **) -> int
{

    Robot::AmbientConfigurationArray ambient_config_array = {0.577635, 1.541946, 1.401291, 1.291197, 0.165969, -0.868276, 1.853091};
    // Robot::AmbientConfigurationArray ambient_config_array = {0.001470, 1.892451, 1.570978, 0.766473, 1.004831, -2.079666, 2.224666};

    auto eefk = Robot::eefk(ambient_config_array);
    // eefk is a Eigen::Isometry3f



    const Eigen::Vector3f translation = eefk.translation();
    const Eigen::Quaternionf rotation(eefk.rotation());
    std::cout << "End effector pose: " << std::fixed << std::setprecision(6) << translation.x() << ", "
              << translation.y() << ", " << translation.z() << ", " << rotation.x() << ", " << rotation.y()
              << ", " << rotation.z() << ", " << rotation.w() << std::endl;

    // first create a dummy start and goal configuration
    Robot::ConfigurationArray pose_array = {0.649573, -0.110000, 0.440114, 0.358849, -0.609368, 0.609519, 0.358307, 1.45};
    // Robot::ConfigurationArray goal_array = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};

    // // Create a Robot::Configuration start and then a Robot::ConfigurationBlock<rake> start_block

    Robot::Configuration pose(pose_array);
    // Robot::Configuration goal(goal_array);

    Robot::template ConfigurationBlock<rake> pose_block;
    for (std::size_t i = 0; i < Robot::dimension; ++i)
    {
        pose_block[i] = pose.broadcast(i);
    }

    // // first step through parameterized IK to get the ambient configurations
    auto [param_valid, start_ambient_block] = Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
    std::cout << "Start parameterized IK valid: " << std::boolalpha << param_valid << std::endl;

    for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
    {
        std::cout << start_ambient_block[{i, 0}]
                << (i < Robot::ambient_dimension - 1 ? ", " : "");
    }
    std::cout << std::endl;





    // return 0;
}

