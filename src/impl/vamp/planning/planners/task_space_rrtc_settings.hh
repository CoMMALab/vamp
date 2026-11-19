#pragma once

#include <cstddef>

#include <vamp/planning/planners/rrtc_settings.hh>

namespace vamp::planning
{
    // The settings of the task-space RRT-Connect: those of RRTC (the longest extension `range`, which bounds both an exploration
    // step toward a random sample and a goal-directed step toward the region; max_iterations; max_samples; nn_epsilon) and
    // what is specific to planning toward a region. RRTC's settings for the two trees it grows from a start and a goal
    // configuration (dynamic_domain, radius, alpha, min_radius, balance, tree_ratio, start_tree_first) are not used.
    struct TaskSpaceRRTCSettings : RRTCSettings
    {
        // Steps taken by the goal-directed walk that follows each exploration step.
        std::size_t max_greedy_steps = 20;

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

        // Project every random sample onto the manifold (lp.project) before the nearest-neighbor query and the
        // extension toward it, instead of leaving it for the extension's projection. A sample that does not project
        // costs its iteration. For planning in a Space whose states carry a curve parameter (CurveSpace), where a
        // sample has to be on the curve at its own parameter for "nearest" to mean "nearest on the curve". Only
        // constrained (projecting) local planners project.
        bool project_samples = false;

        // For a Space with a curve parameter t: the weight of t in the metric of the run, in rad per unit of t (the
        // joints' distance is in rad). The nearest-neighbor tree measures distances as sqrt(|dq|^2 + (w dt)^2), and
        // `range` bounds a step in that metric. With the goal-directed walk heading for t = 1, a larger weight makes t
        // advance more slowly per step (about range / w when far from the goal), a smaller one faster. 0 (the default)
        // is 1 / the number of joints: the whole curve costs as much as one joint's rad on average.
        float parameter_weight = 0.;
    };
}  // namespace vamp::planning
