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

        // Project every random sample onto the manifold (lp.project) before the nearest-neighbor query and the
        // extension toward it, instead of leaving it for the extension's projection. A sample that does not project
        // costs its iteration. For planning in a Space whose states carry a curve parameter (SplineSpace), where a
        // sample has to be on the curve at its own parameter for "nearest in the joints" to mean "nearest on the
        // curve". Only constrained (projecting) local planners project.
        bool project_samples = false;

        // For a Space with a curve parameter t: how far each goal-directed step advances t toward the end of the curve
        // (Space::advance_parameter); the step's joints still follow the region's error gradient.
        float parameter_step = 0.05;
    };
}  // namespace vamp::planning
