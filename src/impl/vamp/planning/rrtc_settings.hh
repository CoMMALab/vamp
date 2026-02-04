#pragma once

namespace vamp::planning
{
    struct RRTCSettings
    {
        float range = 2.;

        bool dynamic_domain = true;
        float radius = 4.;
        float alpha = 0.0001;
        float min_radius = 1.;

        bool balance = true;
        float tree_ratio = 1.;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        bool start_tree_first = true;
        
        // Dev WIP
        bool random_connect = false; // Attempt to connect randomly or only nearest nodes
        // int random_connect_attempts = 5; // Number of attempts when random connecting, -1 for all in tree
        float random_connect_attempts = 0.1; // Ratio of nodes to attempt when random connecting shoud be in (0,1]
        int random_connect_interval = 1; // How often to attempt random connections (every N iterations)
    };
}  // namespace vamp::planning
