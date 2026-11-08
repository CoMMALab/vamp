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

        // Optional, off by default: the longest step of the goal-directed walk, if it should differ from the
        // exploration `range`. 0 uses `range`. Near obstacles a long walk step can jump over the narrow way to the
        // region (the straight segment of a step is checked, not the way between steps), so a smaller walk step
        // follows it more closely.
        float walk_range = 0.;

        // Optional, off by default: interleave IK with planning. On this fraction of the iterations (0 turns it
        // off; 0.01 is every hundredth), a random sample is projected onto the region (a full projection, with the
        // region's own method and iteration limit: a bounded least-squares method and 100+ iterations work well),
        // and a collision-free result becomes the root of a goal tree, up to `ik_pool_size` roots. On the same
        // iterations the goal tree is extended toward a random sample, and new nodes of either tree are connected to
        // the nearest node of the other, as in RRT-Connect. Meeting solves the problem. This helps when the region is
        // hard to walk into, e.g. a pose in a tight space.
        float ik_rate = 0.F;
        std::size_t ik_pool_size = 100;
    };
}  // namespace vamp::planning
