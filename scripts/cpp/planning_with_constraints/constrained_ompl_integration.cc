#include <vector>
#include <array>
#include <utility>
#include <csignal>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vamp/vector.hh>
#include <vamp/vector/interface.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>

#include <vamp/robots/panda.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Exception.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/constraints/validate_constraint_motion.hh>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <csignal>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
static constexpr std::size_t dimension = Robot::dimension;
using Configuration = Robot::Configuration;
static constexpr const std::size_t rake = vamp::FloatVectorWidth;
using EnvironmentInput = vamp::collision::Environment<float>;
using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;
using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;

// Maximum planning time
static constexpr float planning_time = 30.0;
static constexpr int maxIterations = 100000;

// Maximum simplification time
static constexpr float simplification_time = 1.0;

// Helper function to convert projected state into a double veector
std::vector<double> extractStateReals(const ob::State *state, const ob::ProjectedStateSpace *space)
{
    // Cast the top-level state space to ProjectedStateSpace

    std::size_t dim = space->getDimension();

    std::vector<double> values(dim);
    space->copyToReals(values, state);

    return values;
}

// outputs into state. takes a double vector and turns it into state for projectedstatespace
void fillStateFromReals(
    const std::vector<double> &values,
    ob::State *state,
    const ob::ProjectedStateSpace *space)
{
    std::size_t dim = space->getDimension();

    if (values.size() != dim)
    {
        throw std::runtime_error("Input size does not match ProjectedStateSpace dimension");
    }

    space->copyFromReals(state, values);
}

// Helper function to turn double vector into vamp vector
inline static auto double_vector_to_vamp(const std::vector<double> &values) -> Configuration
{
    if (values.size() != dimension)
    {
        throw std::runtime_error("Input vector size does not match robot dimension");
    }

    // Create an aligned buffer for the VAMP configuration
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy the vector values into the aligned buffer
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(values[i]);
    }

    // Construct a Configuration from the aligned buffer
    return Configuration(aligned_buffer.data());
}

inline static std::vector<double> vamp_to_double_vector(const Configuration &c)
{
    std::vector<double> values(dimension);

    std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> temp_array;
    c.to_array(temp_array.data());

    for (size_t i = 0; i < dimension; ++i)
    {
        values[i] = static_cast<double>(temp_array[i]);
    }

    return values;
}

// Convert an OMPL state into a VAMP vector
inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
{
    // Create an aligned memory buffer to load VAMP vector from
    alignas(Configuration::S::Alignment)
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer;

    // Copy OMPL data into aligned buffer
    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        aligned_buffer[i] = static_cast<float>(as->values[i]);
    }

    // Create configuration from aligned buffer data
    return Configuration(aligned_buffer.data());
}

// Convert a VAMP vector to an OMPL state
inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
{
    std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> temp_array;
    c.to_array(temp_array.data());

    auto *as = state->as<ob::RealVectorStateSpace::StateType>();
    for (auto i = 0U; i < dimension; ++i)
    {
        as->values[i] = static_cast<double>(temp_array[i]);
    }
}

class CustomConstraint : public ob::Constraint
{
public:
    mutable vamp::planning::constraint::
        ComposableConstraints<Robot, rake, vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>>
            constraints;
    mutable size_t num_failed_projections = 0;

private:
    std::weak_ptr<ob::ProjectedStateSpace> ps_space_;

public:
    CustomConstraint(
        vamp::planning::constraint::
            ComposableConstraints<Robot, rake, vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>>
                &x)
      : ob::Constraint(Robot::dimension, Robot::dimension - 6), constraints(x)
    {
    }

    // Member fields
    void setProjectedStateSpace(const std::shared_ptr<ob::ProjectedStateSpace> &space)
    {
        ps_space_ = space;
    }

    std::shared_ptr<ob::ProjectedStateSpace> getProjectedStateSpace() const
    {
        return ps_space_.lock();  // Returns nullptr if expired
    }

    ConfigurationBlock turn_configuration_into_configuration_block(const Robot::Configuration &c) const
    {
        typename Robot::template ConfigurationBlock<rake> block;

        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = c.broadcast(i) + 0.0;
        }
        return block;
    }

    // Here we define the actual constraint function, which takes in some state "x"
    // (from the ambient space) and sets the values of "out" to the result of the
    // constraint function. Note that we are implementing `function` which has this
    // function signature, not the one that takes in ompl::base::State.
    // for now, I left out the function, since it might not be necessary.
    bool project(ob::State *state) const override
    {
        std::shared_ptr<ob::ProjectedStateSpace> pss_ptr = getProjectedStateSpace();
        auto pss = pss_ptr.get();
        if (!pss)
        {
            throw std::runtime_error("ProjectedStateSpace member field weak pointer no longer exists!");
        }
        // std::cout << "At the very begnning of project, the state is: \n";
        // pss->printState(state, std::cout);
        Configuration c = double_vector_to_vamp(extractStateReals(state, pss));
        // raise(SIGTRAP);

        bool result = project(c);
        // TODO: after c has been modified, convert the Configuration c into a double vector.
        std::vector<double> newReals = vamp_to_double_vector(c);
        // std::cout << "After converting vamp to double vector, we get: ";
        // for (double d : newReals) {
        //    std::cout << d << ", ";
        // }
        // std::cout << "From " << c ;
        fillStateFromReals(newReals, state, pss);
        // std::cout << "At the end, we get: \n";
        //  pss->printState(state, std::cout);
        //  std::cout << result << "\n";
        //
        if (!result)
        {
            num_failed_projections++;
        }
        return result;
    }

    double distance(const ob::State *state) const override
    {
        auto configuration = ompl_to_vamp(state);
        return distance(configuration);
    }

    bool isSatisfied(const ob::State *state) const override
    {
        std::cout << "our custom implementation of isSatisfied is called!\n";
        return distance(state) < 0.0001;
    }

    bool project(Configuration &configuration) const
    {
        // std::cout << "Project: COnfiguration is " << configuration << "\n";
        ConfigurationBlock block = turn_configuration_into_configuration_block(configuration);
        ConfigurationBlock last_projected_block;
        // std::cout << "After converted to configuration block, it is " << block << "\n";
        bool result = constraints.projectConfiguration(
            block,
            last_projected_block,
            vamp::planning::constraint::ProjMethod::InnerLM,
            10.0,
            1.0,
            15,
            false);
        // std::cout << "After pojection, the configuration block is " << block << "\n";
        // std::cout << "Result of projection is " << result << "\n";
        typename Robot::ConfigurationArray last_projected;
        for (auto i = rake - 1; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = last_projected_block[{j, i}];
            }
        }
        configuration = Robot::Configuration(last_projected);
        // std::cout << "After Project: COnfiguration is " << configuration << result << "\n";
        return result;
    }

    double distance(const Configuration &configuration) const
    {
        ConfigurationBlock block = turn_configuration_into_configuration_block(configuration);
        // std::cout << "block is" << block << "\n";
        auto simd_float_vector = constraints.distanceToConstraint(block);
        // std::cout << "distanceToConstraint returned " << simd_float_vector << "\n";
        // std::cout << "is the start position on the manifold?? Answer: " <<
        // simd_float_vector.test_all_less_equal(0.0001F) << "\n";
        double dist = simd_float_vector[{0, 0}];
        // std::cout << "dist is" << dist << "\n";
        return dist;
    }

    bool isSatisfied(const Configuration &configuration) const
    {
        // std::cout << "our custom implementation of isSatisfied is called! CONFIGURATION version\n";
        return distance(configuration) < 0.0001;
    }

    // Fully dont know what is going on, need to ask shru
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        std::cout << "THIS IS REALLY BAD!!! FUNCTION GOT CALLED!";
        return;
    }
};

// State validator using VAMP
struct VAMPStateValidator : public ob::StateValidityChecker
{
    const EnvironmentVector &env_v;

    VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::StateValidityChecker(si), env_v(env_v)
    {
    }

    auto isValid(const ob::State *state) const -> bool override
    {
        std::cout << "Called state validator " << "\n";
        // Convert OMPL to VAMP vector and validate
        auto *pss = dynamic_cast<ob::ProjectedStateSpace *>(si_->getStateSpace().get());
        if (!pss)
        {
            throw ompl::Exception("Expected ProjectedStateSpace");
        }

        // Step 2. Get the OMPL Constraint object
        auto constraint_base = pss->getConstraint();

        // Step 3. Downcast it to CustomConstraint
        auto *custom_constraint = dynamic_cast<CustomConstraint *>(constraint_base.get());
        if (!custom_constraint)
        {
            throw ompl::Exception("Expected CustomConstraint");
        }

        Configuration c = double_vector_to_vamp(extractStateReals(state, pss));
        // auto *projState = state->as<ob::ProjectedStateSpace::StateType>();
        // const ob::State *ambient = projState->getAmbientState();
        // si_->getStateSpace()->printState(ambient, std::cout);
        auto result = custom_constraint->isSatisfied(c);
        // std::cout << "At the end of isValid for stateValdiator, is valid returns " << result << "\n";
        if (!result)
        {
            std::cout << "Constraint not satisfied\n";
            return result;
        }
        typename Robot::template ConfigurationBlock<rake> temp_block;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            temp_block[i] = c.broadcast(i);
        }

        auto fkcc_out = Robot::template fkcc<rake>(env_v, temp_block);
        std::cout << "fkcc_out: " << fkcc_out << "\n";
        return fkcc_out;
    }
};

struct VAMPMotionValidator : public ob::MotionValidator
{
    const EnvironmentVector &env_v;
    inline static size_t project_constrained_motion_failed_counter = 0;

    VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        // std::cout << "checkMotion called with states:\n";
        // Convert OMPL states to VAMP vectors and check motion between states
        auto *pss = dynamic_cast<ob::ProjectedStateSpace *>(si_->getStateSpace().get());
        if (!pss)
        {
            throw ompl::Exception("Expected ProjectedStateSpace");
        }

        // Step 2. Get the OMPL Constraint object
        auto constraint_base = pss->getConstraint();

        // Step 3. Downcast it to your CustomConstraint
        auto *custom_constraint = dynamic_cast<CustomConstraint *>(constraint_base.get());
        if (!custom_constraint)
        {
            throw ompl::Exception("Expected CustomConstraint");
        }

        // Step 4. Convert state into VAMP configuration
        // std::cout << "Check motion called with these following states:\n";
        // State(s1, std::cout);
        // pss->printState(s2, std::cout);
        Configuration configuration1 = double_vector_to_vamp(extractStateReals(s1, pss));
        ;
        Configuration configuration2 = double_vector_to_vamp(extractStateReals(s2, pss));
        ;
        // std::cout << "configuration 1 for checkMotion is " << configuration1 << "\n";
        // std::cout << "configuration 2 for checkMotion is " << configuration2 << "\n";
        std::vector<Configuration> projected_vector_dummy;

        // Step 5. Call VAMP’s constrained validator
        auto result = vamp::planning::constraint::project_constraint_motion<Robot, rake, 1>(
            configuration1,
            configuration2,
            projected_vector_dummy,
            custom_constraint->constraints,
            env_v,
            vamp::planning::constraint::ProjMethod::OuterLM,
            1.0,
            20);

        if (!result)
        {
            project_constrained_motion_failed_counter++;
        }

        // std::cout << "The result of project_constraint_motion is " << result << "\n";
        return result;
        // return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
        // ompl_to_vamp(s1), ompl_to_vamp(s2), env_v);
    }

    auto
    checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const -> bool override
    {
        throw ompl::Exception("Not implemented!");
    }
};

struct OMPLMotionValidator : public ob::MotionValidator
{
    const EnvironmentVector &env_v;
    inline static size_t project_constrained_motion_failed_counter = 0;

    OMPLMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    OMPLMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
      : ob::MotionValidator(si), env_v(env_v)
    {
    }

    auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
    {
        std::cout << "Calling geodesic checkMotion\n";
        auto *css = dynamic_cast<ob::ProjectedStateSpace *>(si_->getStateSpace().get());
        if (!css)
        {
            throw ompl::Exception("Expected ProjectedStateSpace");
        }

        auto constraint_base = css->getConstraint();

        auto *custom_constraint = dynamic_cast<CustomConstraint *>(constraint_base.get());
        if (!custom_constraint)
        {
            throw ompl::Exception("Expected CustomConstraint");
        }

        Configuration configuration2 = double_vector_to_vamp(extractStateReals(s2, css));
        if (!(custom_constraint->isSatisfied(configuration2)))
        {
            std::cout << "configuration2 is not satisfied within checkMotion!\n";
            return false;
        }

        auto discrete_geodesic = css->discreteGeodesic(s1, s2, false);
        if (!discrete_geodesic)
        {
            std::cout << "discrete_geodesic is not satisfied within checkMotion!\n";
            css->printState(s1, std::cout);
            css->printState(s2, std::cout);

            project_constrained_motion_failed_counter++;
        }
        else
        {
            std::cout << "discrete_geodesic is satisfied within checkMotion!\n";
            css->printState(s1, std::cout);
            css->printState(s2, std::cout);
        }
        // std::cout << project_constrained_motion_failed_counter << std::endl;
        return discrete_geodesic;
    }

    auto
    checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const -> bool override
    {
        // Intentionally not implemented
        throw ompl::Exception("Not implemented!");
    }
};

auto main(int argc, char **argv) -> int
{
    // if the argument state_only is provided, save it
    bool state_only = false;
    if (argc > 1 && std::string(argv[1]) == "state_only")
    {
        state_only = true;
    }

    std::array<float, 6 * Robot::n_eef> tsr_lower_bound = {-0.01, -10.01, -0.01, -10.01, -10.01, -10.01};

    std::array<float, 6 * Robot::n_eef> tsr_upper_bound = {0.01, 10.01, 0.01, 10.01, 10.01, 10.01};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms = {{0, 1, 0, 0, 0.3486, 0.647752, 0.2399}};
    std::array<std::array<float, 7>, Robot::n_eef> eef_transforms_ref_frame_w_world = {{1, 0, 0, 0, 0, 0, 0}};

    vamp::planning::constraint::TaskSpaceConstraint<Robot, rake> tsr_constraint(
        eef_transforms_ref_frame_w_world, eef_transforms, tsr_lower_bound, tsr_upper_bound);
    vamp::planning::constraint::
        ComposableConstraints<Robot, rake, vamp::planning::constraint::TaskSpaceConstraint<Robot, rake>>
            task_constraint(tsr_constraint);

    static constexpr std::array<float, dimension> start = {
        1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100};
    static constexpr std::array<float, dimension> goal = {
        -1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000};

    static const std::vector<std::array<float, 3>> problem = {
        // {0.55, 0, 0.25},
        // {0.55, 0, 0.50},
        // {0.55, 0, 0.60},
        {0.56, 0, 0.450},
        {0.1, 0, 0.7},
        // {0.35, 0.35, 0.25},
        {0, 0.55, 0.25},
        {-0.55, 0, 0.25},
        {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25},
        // {0.35, -0.35, 0.25},
        {0.35, 0.35, 0.8},
        {0, 0.55, 0.8},
        {-0.35, 0.35, 0.8},
        {-0.55, 0, 0.8},
        {-0.35, -0.35, 0.8},
        {0, -0.55, 0.8},
        {0.35, -0.35, 0.8},
    };
    static constexpr float radius = 0.15;

    // Build sphere cage environment
    EnvironmentInput environment;
    for (const auto &sphere : problem)
    {
        // outfile_sph << sphere[0] << "," << sphere[1] << "," << sphere[2] << "," << radius << "\n";
        environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
    }
    environment.sort();
    auto env_v = EnvironmentVector(environment);

    // Create OMPL state space
    auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);

    // Get bounds from VAMP Robot information, scale 0/1 config to min/max
    static constexpr std::array<float, dimension> zeros = {0., 0., 0., 0., 0., 0., 0.};
    static constexpr std::array<float, dimension> ones = {1., 1., 1., 1., 1., 1., 1.};

    auto zero_v = Configuration(zeros);
    auto one_v = Configuration(ones);
    Robot::scale_configuration(zero_v);
    Robot::scale_configuration(one_v);

    ob::RealVectorBounds bounds(dimension);
    for (auto i = 0U; i < dimension; ++i)
    {
        bounds.setLow(i, zero_v[{0, i}]);
        bounds.setHigh(i, one_v[{0, i}]);
    }

    space->setBounds(bounds);

    std::shared_ptr<ob::Constraint> constraint = std::make_shared<CustomConstraint>(task_constraint);


    // Code used for finding a feasible start and goal if the start and goal are not good
    /*
    typename Robot::template ConfigurationBlock<rake> block, projected_block;
    // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth

    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = Robot::Configuration(start).broadcast(i);
    }

    std::cout << "Start Block values: ";
    for (auto i = 0U; i < Robot::dimension; ++i){
        std::cout << block[{i, 0}] << ", ";
    }
    std::cout << std::endl;


    bool ableToProject = task_constraint.projectConfiguration(block, projected_block);

    std::cout << "New STart Block values: ";
    for (auto i = 0U; i < Robot::dimension; ++i){
        std::cout << projected_block[{i, 0}] << ", ";
    }
    std::cout << std::endl;

    for (auto i = 0U; i < Robot::dimension; ++i)
    {
        block[i] = Robot::Configuration(goal).broadcast(i);
    }

    std::cout << "Goal Block values: ";
    for (auto i = 0U; i < Robot::dimension; ++i){
        std::cout << block[{i, 0}] << ", ";
    }
    std::cout << std::endl;


    ableToProject =  task_constraint.projectConfiguration(block, projected_block);

    std::cout << "New Goal Block values: ";
    for (auto i = 0U; i < Robot::dimension; ++i){
        std::cout << projected_block[{i, 0}] << ", ";
    }
    std::cout << std::endl;
    */

    // Combine the ambient space and the constraint into a constrained state space.
    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
    auto cc = std::dynamic_pointer_cast<CustomConstraint>(constraint);
    if (!cc)
    {
        throw std::runtime_error("Constraint is not a CustomConstraint");
    }

    cc->setProjectedStateSpace(css);
    // Define the constrained space information for this constrained state space.
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    // Create space information and set state validator and custom VAMP motion validator
    // auto si = std::make_shared<ob::SpaceInformation>(space);

    auto mv = std::shared_ptr<ob::MotionValidator>();

    if (state_only)
    {
        mv = std::make_shared<OMPLMotionValidator>(csi, env_v);
        std::cout << "OMPL Motion Validator created" << std::endl;
    }
    else
    {
        mv = std::make_shared<VAMPMotionValidator>(csi, env_v);
        std::cout << "VAMP Motion Validator created" << std::endl;
    }

    csi->setStateValidityChecker(std::make_shared<VAMPStateValidator>(csi, env_v));
    csi->setMotionValidator(mv);
    csi->setup();

    // Set start and goal
    ob::ScopedState<> start_ompl(css), goal_ompl(css);
    for (auto i = 0U; i < dimension; ++i)
    {
        start_ompl[i] = start[i];
        goal_ompl[i] = goal[i];
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);
    pdef->setStartAndGoalStates(start_ompl, goal_ompl);
    std::cout << "Start valid? " << csi->isValid(start_ompl.get()) << "\n";
    std::cout << "Goal valid? " << csi->isValid(goal_ompl.get()) << "\n";

    // Set optimization objective
    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(csi);
    pdef->setOptimizationObjective(obj);

    // if (not param.optimize)
    // {
    //     // Set planner to terminate as soon as a solution is found.
    obj->setCostThreshold(obj->infiniteCost());
    // }

    // Create planner - BITstar by default, but you can change this to use other geometric planners instead
    auto planner = std::make_shared<og::RRTConnect>(csi);

    planner->setProblemDefinition(pdef);
    planner->setRange(1.0);
    planner->setup();

    // Solve the problem
    auto counter = std::make_shared<unsigned int>(0);

    // Create a PlannerTerminationCondition with a lambda
    ompl::base::PlannerTerminationCondition ptc(
        [counter, maxIterations]() mutable
        {
            (*counter)++;
            return *counter >= maxIterations;
        });
    auto start_time = std::chrono::steady_clock::now();

    ob::PlannerStatus solved = planner->solve(ptc);
    auto nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);

    if (solved)
    {
        og::PathGeometric *path = pdef->getSolutionPath()->as<og::PathGeometric>();
        std::ofstream outfile("trajectory.txt");
        outfile << std::fixed << std::setprecision(10);
        std::cout << "Raw path length: " << path->length() << std::endl;
        std::cout << "Raw path states: " << path->getStateCount() << std::endl;
        std::cout << "Number of iterations: " << *counter << "\n";

        path->print(std::cout);
        for (std::size_t i = 0; i < path->getStateCount(); ++i)
        {
            std::vector<double> result = extractStateReals(path->getState(i), css.get());

            for (std::size_t j = 0; j < dimension; ++j)
            {
                if (j > 0)
                {
                    outfile << ",";
                }
                outfile << result[j];
            }
            outfile << "\n";
        }
        outfile.close();
    }

    std::cout << "Number of iterations: " << *counter << "\n";

    auto customConstraint = std::dynamic_pointer_cast<CustomConstraint>(constraint);
    std::cout << "Number of failed projections: " << customConstraint->num_failed_projections << std::endl;

    // Only accept exact solutions
    if (solved == ob::PlannerStatus::EXACT_SOLUTION)
    {
        std::cout << "Found solution in " << nanoseconds / 1e6 << "ms! Simplfying..." << std::endl;

        // std::cout << "Project constrained motion failed counter: " <<
        // mv->project_constrained_motion_failed_counter << std::endl;

        // Simplify the path using OMPL's path simplification
        /*
        const ob::PathPtr &path = pdef->getSolutionPath();
        og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

        auto initial_cost = path_geometric.cost(obj);

        og::PathSimplifier simplifier(csi, pdef->getGoal(), obj);
        if (not simplifier.simplify(path_geometric, simplification_time))
        {
            std::cout << "Path not valid!" << std::endl;
        }

        auto simplified_cost = path_geometric.cost(obj);

        // Output statistics
        std::cout << "Found initial solution with cost " << initial_cost.value() << std::endl;
        std::cout << "Simplified solution to cost " << simplified_cost.value() << std::endl;
        std::cout << "Simplified solution:" << std::endl;

        path_geometric.print(std::cout);
        */
        return 0;
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return 1;
    }
}
