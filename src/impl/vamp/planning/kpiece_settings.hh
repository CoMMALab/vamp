#pragma once

#include <cstddef>
#include <vector>

namespace vamp::planning
{
    struct KPIECESettings
    {
        // Per-dimension cell sizes in projection space (e.g., radians or meters).
        // One value for each dimension of the projection space.
        std::vector<float> cell_size{0.1F};

        // Maximum range of a single extension step (same as OMPL's "range").
        float range{2.0F};

        // Fraction of iterations that sample the goal directly (KPIECE only;
        // ignored by BKPIECE).
        float goal_bias{0.05F};

        // Minimum fraction of a motion that must be valid to keep the new state.
        float min_valid_path_fraction{0.5F};

        // Score multiplier applied to a cell after a failed expansion.
        float failed_expansion_score_factor{0.5F};

        // Fraction of selection attempts that prefer boundary cells (OMPL default 0.9).
        float border_fraction{0.9F};

        // Hard limit on the number of tree nodes (prevents unbounded memory use).
        std::size_t max_samples{100000};

        // Hard limit on planning iterations.
        std::size_t max_iterations{100000};

        // Maximum attempts to sample a valid configuration from a single state.
        std::size_t max_valid_sample_attempts{100};
    };
}  // namespace vamp::planning