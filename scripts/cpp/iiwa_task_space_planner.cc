#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/constraints/ik_parameterized_local_planner.hh>
#include <vamp/planning/constraints/task_space_informed_sampler.hh>
#include <vamp/robots/iiwamarker.hh>
#include <vamp/random/halton.hh>

using Robot = vamp::robots::IiwaMarker;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using RRTC = vamp::planning::RRTC<Robot, rake, Robot::resolution>;
using IKLocalPlanner = vamp::planning::IKParameterizedLocalPlanner<Robot, rake, Robot::resolution>;

auto main(int, char **) -> int
{

    // Robot::AmbientConfigurationArray ambient_config_array = {0.71370579,  1.96751046,  1.72862129,
    //     1.29729566,  0.16350904, -0.933994  ,  2.38605378};
    Robot::AmbientConfigurationArray ambient_config_array = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    auto eefk = Robot::eefk(ambient_config_array);
    // eefk is a Eigen::Isometry3f



    const Eigen::Vector3f translation = eefk.translation();
    const Eigen::Quaternionf rotation(eefk.rotation());
    std::cout << "End effector pose: " << std::fixed << std::setprecision(6) << translation.x() << ", "
              << translation.y() << ", " << translation.z() << ", " << rotation.x() << ", " << rotation.y()
              << ", " << rotation.z() << ", " << rotation.w() << std::endl;

    // first create a dummy start and goal configuration
    Robot::ConfigurationArray pose_array = {0.649573, -0.110000, 0.440114, 0.358849, -0.609368, 0.609519, 0.358307, 1.45};
    // Robot::ConfigurationArray pose_array = {0.571618, -0.207861, 0.448919, 0.368427, -0.607301, 0.580276, 0.398404, 0.513739};
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
    auto [param_valid, ambient_block] = Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
    std::cout << "Start parameterized IK valid: " << std::boolalpha << param_valid << std::endl;

    for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
    {
        std::cout << ambient_block[{i, 0}]
                << (i < Robot::ambient_dimension - 1 ? ", " : "");
    }
    std::cout << std::endl;

    // Take the ambient configuration produced by parameterized IK and run
    // forward kinematics on it again to check that it reproduces the IK input pose.
    Robot::AmbientConfigurationArray result_ambient_config_array;
    for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
    {
        result_ambient_config_array[i] = ambient_block[{i, 0}];
    }

    auto eefk_check = Robot::eefk(result_ambient_config_array);
    const Eigen::Vector3f check_translation = eefk_check.translation();
    const Eigen::Quaternionf check_rotation(eefk_check.rotation());
    std::cout << "Recomputed end effector pose from IK result: " << std::fixed << std::setprecision(6)
              << check_translation.x() << ", " << check_translation.y() << ", " << check_translation.z() << ", "
              << check_rotation.x() << ", " << check_rotation.y() << ", " << check_rotation.z() << ", "
              << check_rotation.w() << std::endl;



    Robot::ConfigurationArray start_pose_array = {0.649573, -0.110000, 0.440114, 0.358849, -0.609368, 0.609519, 0.358307, 1.45};
    Robot::ConfigurationArray goal_pose_array = {0.599573, -0.110000, 0.100114, 0.358849, -0.609368, 0.609519, 0.358307, 1.45};


    EnvironmentInput environment;
    std::ifstream infile(
        "/tmp/"
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

    // Task Space Region informed sampler: eef_to_offset/world_to_reference are identity here
    // (offset frame = eef frame, reference frame = world frame), so the bounds below are
    // directly a world-frame box around the start pose's position, with rotation left free
    // (bound width > 2*pi on all three log-map axes -> reuses Robot::sample()'s Shoemake
    // quaternion instead of drawing a bounded one).
    using TaskSampler = vamp::planning::TaskSpaceInformedSampler<Robot>;
    TaskSampler::Transform eef_to_offset = {0.F, 0.F, 0.F, 0.F, 0.F, 0.F, 1.F};
    // TaskSampler::Transform eef_to_offset = {-0.003750F, 0.000125F, 0.211500F, -0.707106F, 0.000000F, 0.000000F, 0.707107F};
    TaskSampler::Transform world_to_reference = {1.F, 0.F, 0.F, 0.358849, -0.609368, 0.609519, 0.358307};
    TaskSampler::Bound tsr_lower = {-1.F, -1.0F, -1.F, -0.015F, -0.015F, -0.015F};
    TaskSampler::Bound tsr_upper = {1.F, 1.0, 1.F, 0.015F, 0.015F, 0.015F};

    auto task_sampler = vamp::planning::make_task_space_informed_sampler<Robot>(
        eef_to_offset,
        world_to_reference,
        tsr_lower,
        tsr_upper,
        environment,
        std::make_shared<vamp::rng::Halton<Robot>>());

    std::cout << "\n--- TaskSpaceInformedSampler samples ---" << std::endl;
    for (int i = 0; i < 5; ++i)
    {
        Robot::Configuration sample_config(task_sampler->next());
        const auto sample_array = sample_config.to_array();

        std::cout << "Sample " << i << ": ";
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            std::cout << sample_array[j] << (j < Robot::dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        Robot::template ConfigurationBlock<rake> sample_block;
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            sample_block[j] = sample_config.broadcast(j);
        }

        auto [sample_valid, sample_ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(sample_block);
        std::cout << "  parameterized IK valid: " << std::boolalpha << sample_valid << std::endl;

        if (sample_valid)
        {
            Robot::AmbientConfigurationArray sample_ambient_array;
            for (std::size_t j = 0; j < Robot::ambient_dimension; ++j)
            {
                sample_ambient_array[j] = sample_ambient_block[{j, 0}];
            }

            auto sample_eefk = Robot::eefk(sample_ambient_array);
            const Eigen::Vector3f sample_translation = sample_eefk.translation();
            const Eigen::Quaternionf sample_rotation(sample_eefk.rotation());
            std::cout << "  recomputed eef pose: " << sample_translation.x() << ", " << sample_translation.y()
                      << ", " << sample_translation.z() << ", " << sample_rotation.x() << ", "
                      << sample_rotation.y() << ", " << sample_rotation.z() << ", " << sample_rotation.w()
                      << std::endl;
        }
    }
    std::cout << "--- end TaskSpaceInformedSampler samples ---\n" << std::endl;


    // Resolve a task-space pose through parameterized IK, then collision-check the
    // resulting ambient (joint-space) configuration; on collision, dump the fkcc_debug
    // breakdown. Shared by the start and goal checks below so they can't drift apart.
    auto resolve_and_check = [&](const Robot::ConfigurationArray &pose_array, const std::string &label)
    {
        Robot::Configuration pose(pose_array);
        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = pose.broadcast(i);
        }

        auto [param_valid, ambient_block] =
            Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
        std::cout << label << " parameterized IK valid: " << std::boolalpha << param_valid << std::endl;

        auto is_in_coll = Robot::template fkcc<rake>(env_v, ambient_block);
        std::cout << label << " configuration is collision free: " << std::boolalpha << is_in_coll << std::endl;

        if (not is_in_coll)
        {
            auto [env_cc, self_cc] = Robot::template fkcc_debug<rake>(env_v, ambient_block);
            std::cout << env_cc.size() << " environment collisions, " << self_cc.size() << " self-collisions"
                      << std::endl;

            // print self_cc, it is a pair of indices into the robot's links that are colliding
            for (const auto &pair : self_cc)
            {
                std::cout << "Self-collision between links " << pair.first << " and " << pair.second << std::endl;
            }
        }

        return ambient_block;
    };

    auto start_ambient_block = resolve_and_check(start_pose_array, "Start");
    auto goal_ambient_block = resolve_and_check(goal_pose_array, "Goal");

    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    vamp::planning::RRTCSettings rrtc_settings;
    rrtc_settings.range = 0.1;
    rrtc_settings.max_iterations = 1000000;
    rrtc_settings.max_samples = 1000000;

    IKLocalPlanner ik_local_planner;

    auto result = RRTC::solve(
        Robot::Configuration(start_pose_array),
        Robot::Configuration(goal_pose_array),
        env_v,
        rrtc_settings,
        task_sampler,
        ik_local_planner);

    std::cout << "RRTC path size: " << result.path.size() << ", iterations: " << result.iterations
              << ", nanoseconds: " << result.nanoseconds << ", with tree sizes : " <<result.size[0] << ", " << result.size[1] << std::endl;

    for (const auto &config : result.path)
    {
        const auto &array = config.to_array();
        // for (std::size_t i = 0; i < Robot::dimension; ++i)
        // {
        //     std::cout << array[i] << (i < Robot::dimension - 1 ? ", " : "");
        // }
        // std::cout << std::endl;


        Robot::template ConfigurationBlock<rake> pose_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            pose_block[i] = config.broadcast(i);
        }

        // // first step through parameterized IK to get the ambient configurations
        auto [param_valid, ambient_block] = Robot::template parameterized_ik<Robot::template ConfigurationBlock<rake>, rake>(pose_block);
        // std::cout << "Start parameterized IK valid: " << std::boolalpha << param_valid << std::endl;

        for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        {
            std::cout << ambient_block[{i, 0}]
                    << (i < Robot::ambient_dimension - 1 ? ", " : "");
        }
        std::cout << std::endl;

        // Take the ambient configuration produced by parameterized IK and run
        // forward kinematics on it again to check that it reproduces the IK input pose.
        Robot::AmbientConfigurationArray result_ambient_config_array;
        for (std::size_t i = 0; i < Robot::ambient_dimension; ++i)
        {
            result_ambient_config_array[i] = ambient_block[{i, 0}];
        }

        auto eefk = Robot::eefk(result_ambient_config_array);




        // auto eefk = Robot::eefk(ambient_config_array);
        const Eigen::Vector3f translation = eefk.translation();
        const Eigen::Quaternionf rotation(eefk.rotation());
        // std::cout << translation.x() << ", "
        //         << translation.y() << ", " << translation.z() << ", " << rotation.x() << ", " << rotation.y()
        //         << ", " << rotation.z() << ", " << rotation.w() << std::endl;

        // i want to compute the norm between the position and orientation of the eefk and the original pose_array
        // store them as separate norms and print them out
        Eigen::Vector3f original_translation(array[0], array[1], array[2]);
        Eigen::Quaternionf original_rotation(array[6], array[3], array[4], array[5]);
        float translation_norm = (translation - original_translation).norm();
        
        // rotation norm is a bit more complicated, we can compute the angle between the two quaternions and use that as a measure of difference
        // we cannot use dot product directly because quaternions can be negated and still represent the same rotation
        // so let us do a true angle difference using the formula: angle = 2 * acos(|q1 * q2|), where q1 and q2 are the two quaternions
        float rotation_angle = 2 * std::acos(std::abs(rotation.dot(original_rotation)));
        float rotation_norm = std::abs(rotation_angle);




        // now create two two original and new as se3, and compute the se3 norm which is a@b-1.log()

        Eigen::Isometry3f original_pose = Eigen::Isometry3f::Identity();
        original_pose.translation() = original_translation;
        original_pose.linear() = original_rotation.toRotationMatrix();
        Eigen::Isometry3f new_pose = Eigen::Isometry3f::Identity();
        new_pose.translation() = translation;
        new_pose.linear() = rotation.toRotationMatrix();
        Eigen::Isometry3f diff = original_pose.inverse() * new_pose;

        // extract the translation and rotation from diff
        Eigen::Vector3f diff_translation = diff.translation();
        Eigen::Matrix3f diff_rotation_matrix = diff.rotation();
        Eigen::Quaternionf diff_rotation(diff_rotation_matrix);


        // float se3_norm = diff.log().norm();

        std::cout << "Translation norm: " << translation_norm << ", Rotation norm: " << rotation_norm << ", actual translation difference: " << diff_translation.transpose() << ", actual rotation difference (as quaternion): " << diff_rotation.coeffs().transpose() << std::endl;

        // for sanity print the two quaternions as well
        // std::cout << "Original rotation: " << original_rotation.x() << ", " << original_rotation.y() << ", " << original_rotation.z() << ", " << original_rotation.w() << std::endl;
        // std::cout << "Computed rotation: " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ", " << rotation.w() << std::endl;



    }

    return 0;
}

