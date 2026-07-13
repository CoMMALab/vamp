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

    struct SimplifySettings
    {
        std::size_t max_iterations{5};
        std::size_t interpolate{0};
        std::vector<SimplifyRoutine> operations{{SHORTCUT, BSPLINE}};

        ReduceSettings reduce;
        ShortcutSettings shortcut;
        BSplineSettings bspline;
        PerturbSettings perturb;
        VeloptSettings velopt;
    };
}  // namespace vamp::planning
