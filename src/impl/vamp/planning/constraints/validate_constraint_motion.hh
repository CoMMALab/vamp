#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/task_space_constraint.hh>
#include <vamp/planning/constraints/composable_constraint.hh>
#include <vamp/planning/validate.hh>

namespace vamp::planning::constraint
{

    // Generate stddev samples: 0, -0.25, +0.25, -0.5, +0.5, -0.75, +0.75, -1.0
    template <std::size_t I>
    inline constexpr auto stddev_sample() -> float
    {
        if constexpr (I == 0)
        {
            return 0.0F;
        }
        else
        {
            const auto step = (I + 1) / 2;  // 1, 1, 2, 2, 3, 3, 4 for I = 1, 2, 3, 4, 5, 6, 7
            const auto magnitude = static_cast<float>(step) * 0.25F;
            return (I % 2 == 1) ? -magnitude : magnitude;  // odd indices negative, even positive
        }
    }

    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_stddev_samples(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {stddev_sample<I>()...};
    }

    template <std::size_t n>
    struct StdDevSamples
    {
        inline static constexpr auto samples = generate_stddev_samples<n>(std::make_index_sequence<n>());
    };

    template <std::size_t rake, std::size_t dimension>
    inline constexpr auto inter_lane_distance_block(
        const vamp::FloatVector<rake, dimension> &block,
        const vamp::FloatVector<dimension> &start) -> vamp::FloatVector<rake, dimension>
    {
        auto out = block;
        for (std::size_t i = 0; i < dimension; ++i)
        {
            auto svec = start.broadcast(i);
            out[i] = out[i].inter_lane_distance(svec[{0, 0}]);
        }
        return out;
    }

    // template <std::size_t rake, std::size_t dimension>
    // inline constexpr auto inter_lane_distance_block(const vamp::FloatVector<rake, dimension> &block,
    //                                                 const vamp::FloatVector<dimension> &start) ->
    //                                                 vamp::FloatVector<rake, dimension>
    // {
    //     auto out = block;
    //     for (std::size_t i = 0; i < dimension; ++i)
    //     {
    //         // Avoid element(i) on flattened buffers by using broadcast, which is shape-safe
    //         const auto start_vec = start.broadcast(i);
    //         out[i] = out[i].S::template inter_lane_distance<0>(out[i].d()->data[0],
    //         start_vec.d()->data[0]);
    //     }
    //     return out;
    // }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline constexpr auto project_constraint_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        std::vector<typename Robot::Configuration> &projected_vector,
        ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const collision::Environment<FloatVector<rake>> &environment,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0F,
        int num_projection_iterations = 25,
        float std_dev_scaling_factor = 0.1F,
        bool insert_all_to_tree = false) -> bool
    {
        projected_vector.clear();

        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block, start_block;
        typename Robot::template ConfigurationBlock<rake> projected_block;
        typename Robot::template ConfigurationBlock<rake> initial_projected_block;

        const auto stddev_multipliers = FloatVector<rake>(StdDevSamples<rake>::samples);

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            // block[i] = (start + vector).broadcast(i);
            block[i] = (start).broadcast(i) +
                       (vector.broadcast(i) *
                        (1.0F + stddev_multipliers * std_dev_scaling_factor));  // 0.1F is a scaling factor
                                                                                // for std_dev
            start_block[i] = start.broadcast(i);
        }

        int end_config_projection_index = constraint.projectAnyConfiguration(
            block,
            initial_projected_block,
            projection_method,
            distance,
            projection_descent_rate,
            num_projection_iterations,
            false);
        if (end_config_projection_index == -1)
        {
            // std::cout << "Unable to project " << std::endl;
            return false;
        }

        // check if projected end config is too far from start config. If so, return false immediately without
        // checking inter-rake distances
        float projected_distance = 0.F;
        typename Robot::ConfigurationArray end_config_projected_array;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            float diff = initial_projected_block[{i, end_config_projection_index}] -
                         start_block[{i, end_config_projection_index}];
            projected_distance = projected_distance + diff * diff;
            if (projected_distance > 4 * (distance) * (distance))
            {
                return false;
            }
            end_config_projected_array[i] = initial_projected_block[{i, end_config_projection_index}];
            // rewrite initial_projected_block to be the projected end config
            initial_projected_block[i] = initial_projected_block[i].broadcast(end_config_projection_index);
        }
        typename Robot::Configuration end_config_projected(end_config_projected_array);

        // now check if the projected end config is valid. If not, return false immediately without checking
        // inter-rake distances
        bool end_projected_valid =
            (environment.eef_attachments.size()) ?
                Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                Robot::template fkcc<rake>(environment, initial_projected_block);
        if (not end_projected_valid)
        {
            return false;
        }
        if (distance > 100.F){
            // For infinite distance, just check the last projected config
            typename Robot::ConfigurationArray last_projected_initial_attempt;
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected_initial_attempt[j] = initial_projected_block[{j, 0}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected_initial_attempt));
            return end_projected_valid;

        }


        auto adjusted_vector = end_config_projected - start;
        // First project just the final config
        distance = std::sqrt(projected_distance);

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (adjusted_vector.broadcast(i) * percents);
            start_block[i] = start.broadcast(i);
        }

        std::size_t n = static_cast<std::size_t>(
            std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F));

        // std::cout << "Proj method " << projection_method << std::endl;

        bool ableToProject = constraint.projectConfiguration(
            block,
            initial_projected_block,
            projection_method,
            distance,
            projection_descent_rate,
            num_projection_iterations);
        if (not ableToProject)
        {
            // std::cout << "Unable to project " << std::endl;
            return ableToProject;
        }

        float max_inter_dist = 0.F;

        // auto shifted_block = inter_lane_distance_block(initial_projected_block, start);
        // auto inter_rake_distance = shifted_block[0] * shifted_block[0];
        // for (auto dim = 1U; dim < Robot::dimension; dim++)
        // {
        //     inter_rake_distance = inter_rake_distance + shifted_block[dim] * shifted_block[dim];
        //     if (inter_rake_distance.test_any_greater(4 * (distance / rake) * (distance / rake)))
        //     {
        //         // std::cout << "Invalid config due to distance constraint" << inter_rake_distance <<  "
        //         with max distance allowed is " << 4 * (distance / rake) * (distance / rake) << std::endl;
        //         return false;
        //     }

        // }
        // auto max_inter_dist = inter_rake_distance.sqrt();

        // Compute inter-rake differences
        std::array<vamp::FloatT, Robot::dimension * rake> diff_arr;
        for (auto i = 0U; i < rake; i++)
        {
            float inter_distance = 0.F;
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                if (i == 0)
                {
                    diff_arr[i + j * rake] = initial_projected_block[{j, i}] - start_block[{j, i}];
                }
                else
                {
                    diff_arr[i + j * rake] =
                        initial_projected_block[{j, i}] - initial_projected_block[{j, i - 1}];
                }
                inter_distance = inter_distance + diff_arr[i + j * rake] * diff_arr[i + j * rake];
            }
            if (inter_distance > 4 * (distance / rake) * (distance / rake))
            {
                // std::cout << "Invalid config due to distance constraint" << inter_distance << " at " << i
                // << " with max distance allowed is " << 4 * (distance / rake) * (distance / rake) <<
                // std::endl;
                return false;
            }
            max_inter_dist = std::max(max_inter_dist, inter_distance);
        }
        max_inter_dist = std::sqrt(max_inter_dist);
        typename Robot::template ConfigurationBlock<rake> shifted_block =
            typename Robot::template ConfigurationBlock<rake>(diff_arr);

        // std::cout << "Attachment: " << environment.eef_attachments.size() << std::endl;
        bool valid = (environment.eef_attachments.size()) ?
                         Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                         Robot::template fkcc<rake>(environment, initial_projected_block);

        typename Robot::ConfigurationArray last_projected;
        std::size_t start_add = insert_all_to_tree ? 0 : rake - 1;
        for (auto i = start_add; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = initial_projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }
        if (distance > 100.F)
        {
            return valid;
        }
        // auto max_inter_dist = std::sqrt(inter_rake_distance.hmax());

        // std::cout << "Max inter distance: " << max_inter_dist <<  " " << distance << std::endl;
        if (not valid or max_inter_dist <= (distance / rake) / n)
        {
            return valid;
        }

        std::size_t n_steps = static_cast<std::size_t>(std::max(std::ceil(max_inter_dist * resolution), 1.F));

        for (auto i = 1U; i < n_steps; ++i)
        {
            initial_projected_block = initial_projected_block - shifted_block / n;
            if (not constraint.projectConfiguration(
                    initial_projected_block,
                    projected_block,
                    projection_method,
                    max_inter_dist * rake,
                    projection_descent_rate,
                    num_projection_iterations))
            {
                return false;
            }
            auto q_dist = (projected_block[0] - initial_projected_block[0]) *
                          (projected_block[0] - initial_projected_block[0]);
            for (auto j = 1U; j < Robot::dimension; j++)
            {
                q_dist = q_dist + (projected_block[j] - initial_projected_block[j]) *
                                      (projected_block[j] - initial_projected_block[j]);
            }
            if (q_dist.test_any_greater(4 * max_inter_dist / n * max_inter_dist / n))
            {
                return false;
            }

            bool valid_inside = (environment.eef_attachments.size()) ?
                                    Robot::template fkcc_attach<rake>(environment, projected_block) :
                                    Robot::template fkcc<rake>(environment, projected_block);

            if (not valid_inside)
            {
                return false;
            }
        }
        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline constexpr auto project_constraint_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<typename Robot::Configuration> &projected_vector,
        ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const collision::Environment<FloatVector<rake>> &environment,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0F,
        int num_projection_iterations = 25,
        bool insert_all_to_tree = false,
        bool infinite_distance = false) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(
            start,
            vector,
            infinite_distance ? 10000.F : vector.l2_norm(),
            projected_vector,
            constraint,
            environment,
            projection_method,
            projection_descent_rate);
    }
}  // namespace vamp::planning::constraint
