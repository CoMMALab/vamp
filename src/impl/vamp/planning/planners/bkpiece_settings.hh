#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace vamp::planning
{
    struct BKPIECESettings
    {
        // Projection onto the coverage grid, one cell size per component. Components index the
        // robot state (joints; for flask robots q then qdot) unless object_projection, in which
        // case they index [x, y, z, vx, vy, vz] of an object frame (object_in_eef, wxyz + xyz) on
        // the robot's end effector (velocity components need a flask robot). At most 6.
        std::vector<int> components{0, 1, 2};
        std::vector<float> cell_size{0.1F, 0.1F, 0.1F};
        bool object_projection = false;
        std::array<float, 7> object_in_eef{1.F, 0.F, 0.F, 0.F, 0.F, 0.F, 0.F};

        // Maximum extension step (the local planner's range).
        float range = 1.F;

        // Pull each sampler draw toward the selected state to within `range` (positions and
        // velocities alike) before steering -- OMPL KPIECE's local sampling; samplers with a local
        // model (RNG::next_near) draw near the selected state instead. Without it the
        // target's velocity is unrelated to the selected state's, and the clipped edge must change
        // velocity abruptly.
        bool local_targets = true;

        // Steers tried per expansion (each from a fresh draw); the valid one ending in the least
        // covered cell is kept.
        std::size_t extension_candidates = 1;

        // Probability of expanding from a border cell rather than an interior one.
        float border_fraction = 0.9F;

        // Score multiplier applied to a cell after a failed expansion from it.
        float failed_expansion_score_factor = 0.5F;

        // Connections tried per expansion into the other tree's cell at the new state.
        std::size_t connect_attempts = 2;

        bool start_tree_first = true;
        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
    };

    // Admissible velocity propagation (planners/avp.hh) over BKPIECE's search.
    struct AVPSettings : BKPIECESettings
    {
        // Collocation points per edge for speed propagation, and the torque bound as a fraction of
        // the effort limits.
        std::size_t edge_samples = 16;
        float effort_fraction = 1.F;
    };
}  // namespace vamp::planning
