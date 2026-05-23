#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>
#include <vamp/planning/validate.hh>
#include <iostream>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto discrete_time_trajopt_loop(
        std::vector<FloatVector<Robot::dimension>> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const float learning_rate=0.05,
        const size_t num_iterations = 10) -> bool
    {
        // for each path segment,
        // I call topple_nn_time_forward_backward_arr
        // that gets me the cost of the segment and the gradient of the cost with respect to the start and end points
        // I then take a step in the direction of the negative gradient for both the start and end points.
        // If any segment is invalid, I do not update the path for that segment. I repeat this process for num_iterations iterations.
        // There are two ways to do this: I can either update the path after checking all segments, or I can update the path after checking each segment. 
        // Since the former is more principled, I will do that. This means that the gradients I get from the NN will be slightly stale, but this is a common practice in deep learning and should not be a problem.

        std::vector<std::array<float, Robot::dimension>> path_arr(path.size());
        for (auto i = 0U; i < path.size(); ++i)
        {
            // const auto &array = path[i].to_array();
            // path_arr.push_back(array);
            std::array<float, Robot::dimension> x;
            for (auto k = 0U; k < Robot::dimension; k++) {
                x[k] = static_cast<float>(path[i].to_array()[k]);
            }
            path_arr[i] = x;

        }


        for (auto iter = 0U; iter < num_iterations; ++iter)
        {
            // datastructure to store the gradients for each point in the path
            // std::vector<FloatVector<Robot::dimension>> gradients(path.size(), FloatVector<Robot::dimension>(0));
            std::vector<std::array<float, Robot::dimension>> gradients(path.size());
            std::vector<float> costs(path.size() - 1);
            std::cout << "\nIteration " << iter << ":" << std::endl;

            for (auto i = 0U; i < path.size() - 1; ++i)
            {
                std::array<float, 2 * Robot::dimension> pair_arr;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    pair_arr[j] = path_arr[i][j];
                    pair_arr[j + Robot::dimension] = path_arr[i+1][j];
                }
                // for(auto j=0U; j < 2 * Robot::dimension; ++j) {
                //     std::cout << pair_arr[j] << ", ";
                // }
                // std::cout << std::endl;

                auto cost_and_grads =
                    Robot::template topple_nn_time_forward_backward_arr(pair_arr);

                // now store the gradients for the start and end points
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    gradients[i][j] += cost_and_grads[j + 1];
                    gradients[i + 1][j] += cost_and_grads[j + 1 + Robot::dimension];
                }
                costs[i] = cost_and_grads[0];
            }

            int total_valid_segments = 0;

            Bezier bez = compute_bez<Robot, rake>(path_arr[0], path_arr[1]);
            bool segment_valid = validate_bez<Robot, rake, resolution>(bez, environment);


            // take a step in the direction of the negative gradient
            for (auto i = 1U; i < path.size() - 1; ++i)
            {
                std::cout << costs[i-1] << ", ";

                // std::cout << "Cost for segment " << i << ": " << costs[i-1] << " : " << std::endl;
                Bezier bez = compute_bez<Robot, rake>(path_arr[i], path_arr[i+1]);
                bool current_segment_valid = validate_bez<Robot, rake, resolution>(bez, environment);
                if (segment_valid && current_segment_valid)
                {
                    for (auto j = 0U; j < Robot::dimension; ++j)
                    {
                        path_arr[i][j] = path_arr[i][j] - learning_rate * gradients[i][j];
                    }
                }
                else
                {
                    std::cout << "Segment " << i << " is invalid, not updating." << std::endl;
                }
                segment_valid = current_segment_valid;
                total_valid_segments += current_segment_valid;

            }

            if (total_valid_segments == 0) {
                std::cout << "No valid segments found in iteration " << iter << ", stopping optimization." << std::endl;
                break;
            }

         }
        
        return true;

    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto trajopt2(
        std::vector<FloatVector<Robot::dimension>> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const size_t num_iteration)
    {
        for (auto i = 0; i < num_iteration; i++) {
            std::vector<FloatVector<Robot::dimension>> gradients(path.size(), FloatVector<Robot::dimension>(0));

            // skip start and goal
            for (auto i = 1U; i < path.size() - 1; ++i) {
                const auto &start = path[i];
                const auto &end = path[i + 1];

                std::array<Robot::dimension> start_arr;
                std::array<Robot::dimension> end_arr;
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    start_arr[j] = start[j];
                    end_arr[j] = end[j];
                }

                auto nn_time_start = std::chrono::steady_clock::now();
                auto cost_and_grads =
                    Robot::template topple_nn_time_forward_backward_arr(start_arr, end_arr);
                auto nn_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - nn_time_start).count();
                vamp::profiling::get_profiler()["nn_inference"].push_back(nn_time);

                // now store the gradients for the start and end points
                for (auto j = 0U; j < Robot::dimension; ++j)
                {
                    gradients[i][j] += cost_and_grads[j + 1];
                    gradients[i + 1][j] += cost_and_grads[j + 1 + Robot::dimension];
                }
                // take a step in the direction of the negative gradient
            }
        }
    }
}