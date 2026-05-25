#pragma once

#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

namespace vamp::planning
{
    struct AOTOPPLESettings
    {
        RRTCSettings rrtc;
        SimplifySettings simplify;

        bool optimize = true;
        bool cost_bound_resample = true;
        // bool simplify_intermediate = true;
        bool use_phs = true;
        // bool anytime = false;

        float bez_range = 0.2;
        bool rand_connect = true;
        float rand_ratio = 0.2;
        float alpha = 0.1;
        bool dynamic_extension = true;

        std::size_t max_iterations = 100000;
        std::size_t max_internal_iterations = 100000;
        std::size_t max_samples = 100000;
        std::size_t max_cost_bound_resamples = 1000;
    };

}  // namespace vamp::planning