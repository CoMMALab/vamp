#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <iomanip>

#include <filesystem>
#include <nlohmann/json.hpp>
#include <fstream>


#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/topple.hh>
// #include <vamp/planning/simplify.hh>
#include <vamp/planning/topple_settings.hh>
#include <vamp/robots/pandatopp.hh>
#include <vamp/random/halton.hh>
using json = nlohmann::json;


using Robot = vamp::robots::Pandatopp;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using TOPPLE = vamp::planning::TOPPLE<Robot, rake, Robot::resolution>;

// Start and goal configurations
static constexpr Robot::ConfigurationArray start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static constexpr Robot::ConfigurationArray goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};

// Spheres for the cage problem - (x, y, z) center coordinates with fixed, common radius defined below
static const std::vector<std::array<float, 3>> problem = {
    {0.55, 0, 0.25},
    {0.35, 0.35, 0.25},
    {0, 0.55, 0.25},
    {-0.55, 0, 0.25},
    {-0.35, -0.35, 0.25},
    {0, -0.55, 0.25},
    {0.35, -0.35, 0.25},
    {0.35, 0.35, 0.8},
    // {0, 0.55, 0.8},
    // // {-0.35, 0.35, 0.8},
    // {-0.55, 0, 0.8},
    // {-0.35, -0.35, 0.8},
    // {0, -0.55, 0.8},
    // {0.35, -0.35, 0.8},
};

// Radius for obstacle spheres
static constexpr float radius = 0.2;

static auto get_src_output_path(const std::string &filename) -> std::filesystem::path
{
    const auto source_dir = std::filesystem::path(__FILE__).parent_path();
    const auto repo_src_from_source = source_dir.parent_path().parent_path() / "src";
    if (std::filesystem::exists(repo_src_from_source))
    {
        return repo_src_from_source / filename;
    }

    const auto repo_src_from_cwd = std::filesystem::current_path() / "src";
    return repo_src_from_cwd / filename;
}


auto main(int, char **) -> int
{
    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }

    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create RNG for planning
    auto rng = std::make_shared<vamp::rng::Halton<Robot>>();

    // Setup TOPPLE and plan
    vamp::planning::TOPPLESettings topple_rrtc_settings;
    topple_rrtc_settings.optimize = false;

    topple_rrtc_settings.rrtc.max_iterations = 100000;
    topple_rrtc_settings.max_samples = 10000000;
    topple_rrtc_settings.simplify.bez = true;
    topple_rrtc_settings.use_phs = false;
    topple_rrtc_settings.simplify_intermediate = false;
    topple_rrtc_settings.max_runs = 1;
    topple_rrtc_settings.cost_bound_resample = false;
    topple_rrtc_settings.bez_range = 0.25;
    topple_rrtc_settings.k_nearest = 16;
    topple_rrtc_settings.alpha = 0.5;
    topple_rrtc_settings.dynamic_extension = false;

    vamp::planning::TreeGrowthData tree_growth;

   

    auto result =
        TOPPLE::solve(Robot::Configuration(start), Robot::Configuration(goal), env_v, topple_rrtc_settings, rng, &tree_growth);

    std::cout << "Planning completed in " << result.iterations << " iterations and "
              << result.nanoseconds << "nseconds." << " with path of length " << result.path.size() << std::endl;

    // // If successful
    if (result.path.size() > 0)
    {
    //     // // Simplify path with default settings
    //     // vamp::planning::SimplifySettings simplify_settings;
    //     // auto simplify_result = vamp::planning::simplify<Robot, rake, Robot::resolution>(
    //     //     result.path, env_v, simplify_settings, rng);

    //     // Output configurations of simplified path
        std::cout << std::fixed << std::setprecision(3);
        for (const auto &config : result.path)
        {
            const auto &array = config.to_array();
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                std::cout << array[i] << ", ";
            }

            std::cout << std::endl;
        }
    }

    json tree_json;
    tree_json["nodes"] = json::array();
    tree_json["failed_extensions"] = json::array();
    tree_json["bridge_edges"] = json::array();
    tree_json["obstacles"] = json::array();
    tree_json["cuboids"] = json::array();

    tree_json["start_config"] = start;
    tree_json["goal_config"] = goal;
    // tree_json["goal_xyz"] = {
    //     eef_transforms[0][4],
    //     eef_transforms[0][5],
    //     eef_transforms[0][6]
    // };
    
    for (const auto& node : tree_growth.nodes) {
        json node_json;
        node_json["planner_index"] = node.planner_index;
        node_json["config"] = node.config;
        node_json["parent_index"] = node.parent_index;
        node_json["iteration"] = node.iteration;
        node_json["loop_type"] = node.loop_type;
        node_json["tree_side"] = (node.tree_side == vamp::planning::TreeGrowthData::TreeSide::Start) ? "Start" : "Goal";
        tree_json["nodes"].push_back(node_json);
    }

    for (const auto& failed : tree_growth.failed_extensions) {
        json failed_json;
        failed_json["config"] = failed.config;
        failed_json["iteration"] = failed.iteration;
        failed_json["loop_type"] = failed.loop_type;
        failed_json["tree_side"] = (failed.tree_side == vamp::planning::TreeGrowthData::TreeSide::Start) ? "Start" : "Goal";
        tree_json["failed_extensions"].push_back(failed_json);
    }

    for (const auto &bridge : tree_growth.bridge_edges)
    {
        json bridge_json;
        bridge_json["from_index"] = bridge.from_index;
        bridge_json["to_index"] = bridge.to_index;
        bridge_json["iteration"] = bridge.iteration;
        bridge_json["loop_type"] = bridge.loop_type;
        bridge_json["tree_side"] = (bridge.tree_side == vamp::planning::TreeGrowthData::TreeSide::Start) ? "Start" : "Goal";
        tree_json["bridge_edges"].push_back(bridge_json);
    }
    
    for (const auto &sphere : problem)
    {
        json obstacle_json;
        obstacle_json["center"] = {sphere[0], sphere[1], sphere[2]};
        obstacle_json["radius"] = radius;
        tree_json["obstacles"].push_back(obstacle_json);
    }

    // for (const auto &cuboid : cuboid_problem)
    // {
    //     json cuboid_json;
    //     cuboid_json["center"] = {cuboid.center[0], cuboid.center[1], cuboid.center[2]};
    //     cuboid_json["euler_xyz"] = {cuboid.euler_xyz[0], cuboid.euler_xyz[1], cuboid.euler_xyz[2]};
    //     cuboid_json["half_extents"] = {
    //         cuboid.half_extents[0],
    //         cuboid.half_extents[1],
    //         cuboid.half_extents[2]};
    //     tree_json["cuboids"].push_back(cuboid_json);
    // }

    tree_json["iteration_starts"] = tree_growth.iteration_starts;
    
    // Write to file
    std::ofstream output_file("tree_growth_data.json");
    output_file << tree_json.dump(2) << std::endl;
    output_file.close();
    
    std::cout << "\nTree growth data exported to tree_growth_data.json" << std::endl;
    std::cout << "Total nodes: " << tree_growth.nodes.size() << std::endl;
    std::cout << "Failed extensions: " << tree_growth.failed_extensions.size() << std::endl;
    std::cout << "Bridge edges: " << tree_growth.bridge_edges.size() << std::endl;
    std::cout << "Iterations: " << tree_growth.iteration_starts.size() << std::endl;



    return 0;
}
