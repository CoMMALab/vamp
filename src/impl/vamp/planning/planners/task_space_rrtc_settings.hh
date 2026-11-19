#pragma once

#include <cstddef>

namespace vamp::planning
{
    struct TaskSpaceRRTCSettings
    {
        // Longest extension: both an exploration step toward a random sample and one
        // goal-directed step toward the task-space region.
        float range = 2.;

        // Steps taken by the goal-directed walk that follows each exploration step.
        std::size_t max_greedy_steps = 20;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;

        // (1 + epsilon)-approximate nearest-neighbor queries; 0 is exact.
        float nn_epsilon = 0.;
    };
}  // namespace vamp::planning
