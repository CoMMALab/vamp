#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>
#include <vamp/profiler_utils.hh>
#include <chrono>
#include <iostream>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto discrete_time_trajopt_loop(
        std::vector<FloatVector<Robot::dimension>> &path,
        const collision::Environment<FloatVector<rake>> &environment,
        const size_t num_iterations) -> bool
    {
        // for each path segment,
        // I call topple_nn_time_forward_backward_arr
        // that gets me the cost of the segment and the gradient of the cost with respect to the start and end points
        // I then take a step in the direction of the negative gradient for both the start and end points.
        // If any segment is invalid, I do not update the path for that segment. I repeat this process for num_iterations iterations.
        // There are two ways to do this: I can either update the path after checking all segments, or I can update the path after checking each segment. 
        // Since the former is more principled, I will do that. This means that the gradients I get from the NN will be slightly stale, but this is a common practice in deep learning and should not be a problem.

        for (auto iter = 0U; iter < num_iterations; ++iter)
        {
            // datastructure to store the gradients for each point in the path
            std::vector<FloatVector<Robot::dimension>> gradients(path.size(), FloatVector<Robot::dimension>(0));

            for (auto i = 0U; i < path.size() - 1; ++i)
            {
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