#pragma once

#include <vector>

namespace vamp::planning
{
    enum SimplifyRoutine
    {
        BSPLINE,
        REDUCE,
        SHORTCUT,
        PERTURB,
        INTERP,
        VELOPT,
        POLISH,
    };

    struct BSplineSettings
    {
        std::size_t max_steps{1};
        float min_change{0.1};
        float midpoint_interpolation{0.5};
    };

    struct ReduceSettings
    {
        std::size_t max_steps{10};
        std::size_t max_empty_steps{5};
        float range_ratio{1. / 2.};
    };

    struct ShortcutSettings
    {
    };

    struct PerturbSettings
    {
        std::size_t max_steps{10};
        std::size_t max_empty_steps{5};
        std::size_t perturbation_attempts{5};
        float range{0.1};
    };

    struct VeloptSettings
    {
        std::size_t max_steps{3};             // gradient-descent passes over interior vertices
        std::size_t line_search_max{6};       // backtracking halvings per vertex per step
        float initial_step{1.0};              // starting alpha for backtracking
        float min_improvement{1e-4};          // stop early when a pass improves < this fraction
    };

    struct PolishSettings
    {
        std::size_t max_steps{5};             // simultaneous descent iterations
        std::size_t line_search_max{6};       // backtracking halvings per iteration
        float initial_step{1.0};              // starting alpha
        float min_improvement{1e-4};          // relative-cost stop threshold
    };

    struct SimplifySettings
    {
        std::size_t max_iterations{5};
        std::size_t interpolate{0};
        std::vector<SimplifyRoutine> operations{{SHORTCUT, BSPLINE}};

        // Optional final full-path polish after the ops loop. Uses PolishSettings below;
        // no-op for robots without Robot::cost_grad. Off by default because on the
        // primary op list it adds cost with little payoff (see sweep on panda cage).
        bool polish_at_end{false};

        ReduceSettings reduce;
        ShortcutSettings shortcut;
        BSplineSettings bspline;
        PerturbSettings perturb;
        VeloptSettings velopt;
        PolishSettings polish;
    };
}  // namespace vamp::planning
