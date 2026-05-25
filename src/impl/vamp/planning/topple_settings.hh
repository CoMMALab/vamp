#pragma once

#include <vamp/planning/rrtc_settings.hh>
#include <vamp/planning/simplify_settings.hh>

namespace vamp::planning
{
    struct TOPPLESettings
    {
        RRTCSettings rrtc;
        SimplifySettings simplify;

        float bez_range = 0.5;
        bool rand_connect = true;
        float rand_ratio = 0.2;
        float alpha = 0.00001;
        bool dynamic_extension = true;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
    };

}  // namespace vamp::planning
