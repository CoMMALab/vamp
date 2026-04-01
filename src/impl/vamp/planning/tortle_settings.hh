#pragma once

#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

namespace vamp::planning
{
    struct TORTLESettings
    {
        RRTCSettings rrtc;
        SimplifySettings simplify;

        bool optimize = true;
        bool cost_bound_resample = true;
        bool simplify_intermediate = true;
        bool use_phs = false;
        bool anytime = false;
        float bez_range = 0.2;
        int k_nearest = 10;

        std::size_t max_iterations = 100000;
        std::size_t max_internal_iterations = 100000;
        std::size_t max_samples = 100000;
        std::size_t max_cost_bound_resamples = 1000;
        std::size_t max_runs = 10;
        float t_radius = 2.0;
        float min_t_radius = 1.5;
    };

}  // namespace vamp::planning
