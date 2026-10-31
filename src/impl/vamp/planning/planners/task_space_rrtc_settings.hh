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
        // costs its iteration. For planning in a Space whose states carry a curve parameter (CurveSpace), where a
        // sample has to be on the curve at its own parameter for "nearest" to mean "nearest on the curve". Only
        // constrained (projecting) local planners project.
        bool project_samples = false;

        // For a Space with a curve parameter t: the weight of t in the metric of the run, in rad per unit of t (the
        // joints' distance is in rad). The nearest-neighbor tree measures distances as sqrt(|dq|^2 + (w dt)^2), and
        // `range` bounds a step in that metric. With the goal-directed walk heading for t = 1, a larger weight makes t
        // advance more slowly per step (about range / w when far from the goal), a smaller one faster.
        float parameter_weight = 1.;
    };
}  // namespace vamp::planning
