#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>
#include <future>
#include <atomic>
#include <thread>
#include <nlohmann/json.hpp>

#include <vamp/collision/factory.hh>
// #include <vamp/planning/validate.hh>
#include <vamp/planning/crrtc.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate_constraint.hh>

#include <Eigen/Geometry>
#include <Eigen/Dense>

// #include <vamp/planning/simplify.hh>
#include <vamp/robots/panda.hh>
#include <vamp/random/halton.hh>
#include <fstream>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

using Robot = vamp::robots::Panda;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using CRRTC = vamp::planning::CRRTC<Robot, rake, Robot::resolution>;
using AttachmentInput = vamp::collision::Attachment<float>;

using Configuration = typename Robot::Configuration;
using ConfigurationArray = typename Robot::ConfigurationArray;
static constexpr auto dimension = Robot::dimension;
using RNG = typename vamp::rng::RNG<Robot>;


// Define your problem structure
struct Problem {
	std::array<float, 7> problem_start;
	std::array<float, 7> problem_end;
    std::array<float, 3> start_eef_pos;
    std::array<float, 3> goal_eef_pos;
};

// This helper function allows nlohmann::json to "just work" with your struct
void to_json(json& j, const Problem& p) {
	j = json{
		{"problem_start", p.problem_start},
		{"problem_end", p.problem_end},
        {"start_eef_pos", p.start_eef_pos},
        {"goal_eef_pos", p.goal_eef_pos}
	};
}


inline auto config_to_array(const Configuration &config){
	auto array = config.to_array();
	Robot::ConfigurationArray soln;
	bool first = true;
	for (auto i = 0U; i < Robot::dimension; ++i)
	{
		soln[i] = array[i];
	}
	return soln;

}

static auto load_cuboids_from_json(EnvironmentInput &environment, const std::string &path) {

    std::array<float, 2> min_bound = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    std::array<float, 2> max_bound = {std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest()};

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open JSON file: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    nlohmann::json j;
    try {
        ifs >> j;
    } catch (const std::exception &e) {
        std::cerr << "Failed to parse JSON file: " << path << " error: " << e.what() << std::endl;
        return std::make_pair(min_bound, max_bound);
    }

    if (!j.is_array()) {
        std::cerr << "Expected top-level JSON array in: " << path << std::endl;
        return std::make_pair(min_bound, max_bound);
    }


    for (const auto &obj : j) {
        if (!obj.is_object()) {
            std::cerr << "Skipping non-object element in array" << std::endl;
            continue;
        }

        // required fields
        if (!obj.contains("x") || !obj.contains("y") || !obj.contains("z") ||
            !obj.contains("dx") || !obj.contains("dy") || !obj.contains("dz")) {
            std::cerr << "Skipping object missing required fields (x,y,z,dx,dy,dz)" << std::endl;
            continue;
        }

        try {
            float x = obj.at("x").get<float>();
            float y = obj.at("y").get<float>();
            float z = obj.at("z").get<float>();
            float dx = obj.at("dx").get<float>();
            float dy = obj.at("dy").get<float>();
            float dz = obj.at("dz").get<float>();

            float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
            if (obj.contains("roll")) roll = obj.at("roll").get<float>();
            if (obj.contains("pitch")) pitch = obj.at("pitch").get<float>();
            if (obj.contains("yaw")) yaw = obj.at("yaw").get<float>();

            std::array<float,3> posf = {x, y, z};
            std::array<float,3> rotf = {roll, pitch, yaw};
            std::array<float,3> sizef = {dx / 2, dy / 2, dz / 2};
            // std::cout << "Creating cuboid at position: " << posf[0] << ", " << posf[1] << ", " << posf[2] << " with size: " << sizef[0] << ", " << sizef[1] << ", " << sizef[2] << std::endl;
            environment.cuboids.emplace_back(vamp::collision::factory::cuboid::array(posf, rotf, sizef));
            // Update bounds
            for (int i = 0; i < 2; ++i) {
                min_bound[i] = std::min(min_bound[i], posf[i] - sizef[i]);
                max_bound[i] = std::max(max_bound[i], posf[i] + sizef[i]);
            }
        } catch (const std::exception &e) {
            std::cerr << "Error reading object fields: " << e.what() << " -- skipping object" << std::endl;
            continue;
        }
    }

    return std::make_pair(min_bound, max_bound);
}




int main() {

	std::vector<Problem> problems;
	const char *output_path = "resources/environments/cuboids/maze_problems.json";
	std::ofstream file(output_path);


    EnvironmentInput environment;

    auto bounds = load_cuboids_from_json(environment, "resources/environments/cuboids/real_maze.json");
    std::cout << "Loaded environment bounds: x[" << bounds.first[0] << ", " << bounds.second[0] << "], y[" << bounds.first[1] << ", " << bounds.second[1] << "]" << std::endl;

    environment.sort();

    std::vector<vamp::collision::Sphere<float>> spheres;
    for(auto i=0U; i < 9; i++){
        spheres.push_back(vamp::collision::Sphere<float>(0.0, 0.0, i * 0.02, 0.01));
    }
    auto attach_transform = Eigen::Transform<float, 3, Eigen::Isometry>::Identity();
    attach_transform.translation().z() = 0.0;
    AttachmentInput attachment(attach_transform);

    attachment.spheres.insert(attachment.spheres.end(), spheres.cbegin(), spheres.cend());
    environment.attach(attachment, 0);
    auto env_v = EnvironmentVector(environment);
    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    size_t total_problems_added = 0;

    while(problems.size() < 100) {

        // sample random position for the problem within the bounds of the environment
        std::array<float, 3> random_start_position = {
            bounds.first[0] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[0] - bounds.first[0]),
            bounds.first[1] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[1] - bounds.first[1]),
            0.20607783f // fixed height for the problem
        };
        std::array<float, 3> random_goal_position = {
            bounds.first[0] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[0] - bounds.first[0]),
            bounds.first[1] + static_cast<float>(rand()) / RAND_MAX * (bounds.second[1] - bounds.first[1]),
            0.20607783f // fixed height for the problem
        };

        // if the points are too close to the obstacles, resample
        if((random_start_position[0] - random_goal_position[0]) * (random_start_position[0] - random_goal_position[0]) + (random_start_position[1] - random_goal_position[1]) * (random_start_position[1] - random_goal_position[1]) < 0.1)
            continue;
        
        std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
            -0.0025, -0.0025, -0.0025, -0.0025, -0.0025, -10.0025
        };

        std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
            0.0025, 0.0025, 0.0025, 0.0025, 0.0025, 10.0025
        };

        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1.0, 0, 0.0, random_start_position[0], random_start_position[1], random_start_position[2]}};
        std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

        vamp::planning::TaskSpaceConstraint<Robot, rake> start_ik_constraint(
            eef_transforms_ref_frame_w_world,
            eef_transforms,
            tsr_lower_bound,
            tsr_upper_bound
        );


        vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> start_task_constraint(
            start_ik_constraint
        );

        bool goal_added_for_start = false;
        // try a bunch of random start and goal and try to project them to the constraint and collision free
        for(size_t config_attempt = 0U; config_attempt < 500; config_attempt++){
            if(goal_added_for_start)
                break;
            auto start = rng->next();
            std::vector<typename Robot::Configuration> start_projected_vector;
            if (vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
                    start,
                    start,
                    start_projected_vector,
                    start_task_constraint,
                    env_v,
                    vamp::planning::ProjMethod::OuterLM,
                    1.0,
                    100,
                    false,
                    true
                ))
                {

                    // create constraint for goal
                    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {
                        -0.0025, -0.0025, -0.0025, -0.0025, -0.0025, -10.0025
                    };

                    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {
                        0.0025, 0.0025, 0.0025, 0.0025, 0.0025, 10.0025
                    };

                    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1.0, 0, 0.0, random_goal_position[0], random_goal_position[1], random_goal_position[2]}};
                    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

                    vamp::planning::TaskSpaceConstraint<Robot, rake> goal_ik_constraint(
                        eef_transforms_ref_frame_w_world,
                        eef_transforms,
                        tsr_lower_bound,
                        tsr_upper_bound
                    );


                    vamp::planning::ComposableConstraints<Robot, rake, vamp::planning::TaskSpaceConstraint<Robot, rake>> goal_task_constraint(
                        goal_ik_constraint
                    );

                    // std::cout << "Able to get start " << start << start_projected_vector.back() << std::endl;
                    for(size_t goal_config_attempt = 0U; goal_config_attempt < 500; goal_config_attempt++){
                        auto goal = rng->next();
                        std::vector<typename Robot::Configuration> goal_projected_vector;

                        if (vamp::planning::project_constraint_motion<Robot, rake, Robot::resolution>(
                                goal,
                                goal,
                                goal_projected_vector,
                                goal_task_constraint,
                                env_v,
                                vamp::planning::ProjMethod::OuterLM,
                                1.0,
                                100,
                                false,
                                true
                            )) {
                                auto start_config = start_projected_vector.back();
                                auto goal_config = goal_projected_vector.back();
                                
                                // else add to problem set
                                Problem p;
                                p.problem_start = config_to_array(start_config);
                                p.problem_end = config_to_array(goal_config);
                                p.start_eef_pos = {random_start_position[0], random_start_position[1], random_start_position[2]};
                                p.goal_eef_pos = {random_goal_position[0], random_goal_position[1], random_goal_position[2]};
                                std::cout << "Adding start goal to problem set :) " << start_projected_vector.back() << goal_projected_vector.back() << std::endl;
                                problems.push_back(p);
                                goal_added_for_start = true;
                                break;

                            }
                    }
                }
            }

            if (file.is_open()) {
                file.seekp(0);
                file << json(problems).dump(4);
                file.flush();
            }
        }

	json j_all_problems = problems;
	if (file.is_open()) {
		file << j_all_problems.dump(4); // '4' is the indentation level for readability
		std::cout << "Successfully saved to problems.json" << std::endl;
	}

	return 1;

}
