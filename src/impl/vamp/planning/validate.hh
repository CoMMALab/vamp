#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    // ─── validate_vector ─────────────────────────────────────────────────────
    //
    // Unchanged from the original.  Checks the full segment [start, start+vector]
    // using a scattered-then-backstep pattern optimised for fast full-rejection.
    // Returns true iff the entire segment is collision-free.
    //
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    // ─── validate_vector_partial ─────────────────────────────────────────────
    //
    // Forward sequential variant that returns both a validity fraction in [0, 1]
    // and the last collision-free configuration along the segment.
    //
    // Design rationale vs. validate_vector
    // ──────────────────────────────────────
    // validate_vector scatters rake sample points across the *whole* segment on
    // the first pass so that an obstacle anywhere causes an early global exit.
    // That ordering is ideal when you only need a boolean answer.
    //
    // For KPIECE-style partial extensions we need the *longest valid prefix*.
    // The scattered order makes it impossible to recover that prefix, because a
    // failure in pass 1 could come from the far end of the segment while the
    // near end is still clear.
    //
    // This function instead divides the segment into n sequential steps and
    // checks each step as a block of rake evenly-spaced points.  On the first
    // failing block it refines within that block by broadcasting each candidate
    // configuration (largest fraction first) across all rake SIMD lanes so that
    // fkcc/fkcc_attach can evaluate it with zero scalar fallback.
    //
    // Stepping order
    // ──────────────
    // Step i covers the sub-segment
    //     [ start + vector*(i/n),  start + vector*((i+1)/n) ]
    // with rake uniformly-spaced sample points at fractions
    //     start + vector * ( i/n + k/(n*rake) )  for k = 1..rake
    //
    // So the check density is identical to validate_vector (resolution points
    // per unit distance), and when the whole segment is valid the two functions
    // visit the same set of configurations — just in a different order.
    //
    // Refinement within a failing step
    // ──────────────────────────────────
    // When step i fails we know the last fully-valid configuration is
    // somewhere in [i/n, (i+1)/n].  We scan the rake candidate points of that
    // step backwards (from the furthest to nearest).  Each candidate is
    // broadcast across all rake SIMD lanes so fkcc treats it as a uniform
    // block.  The first candidate that passes becomes last_valid.
    //
    // Worst-case overhead: 1 failed block check + rake single-config checks.
    // This is only incurred on the fallback path, not the happy path.
    //
    // Parameters
    // ──────────
    //   start       — start configuration
    //   vector      — goal - start (not normalised)
    //   distance    — l2_norm(vector); passed in to avoid a redundant sqrt
    //   environment — collision environment
    //   last_valid  — [out] last collision-free configuration found
    //
    // Returns
    // ───────
    //   1.0f  → entire segment valid; last_valid == start + vector (the goal)
    //   0.0f  → first step already invalid; last_valid == start
    //   (0,1) → partial; last_valid is the furthest reachable configuration
    //
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline auto validate_vector_partial(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment,
        typename Robot::Configuration &last_valid) -> float
    {
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        const std::size_t n = std::max(
            static_cast<std::size_t>(std::ceil(distance / static_cast<float>(rake) * resolution)),
            std::size_t(1));

        // Each step advances by (1/n) of the full vector.
        const auto step = vector / static_cast<float>(n);

        typename Robot::template ConfigurationBlock<rake> block;

        for (std::size_t i = 0; i < n; ++i)
        {
            // ── Build block for step i ────────────────────────────────────
            // step_start = start + step * i
            // block lane k = step_start + step * percents[k]
            //              = start + vector * (i/n + (k+1)/(n*rake))
            const auto step_start = start + step * static_cast<float>(i);

            for (auto j = 0U; j < Robot::dimension; ++j)
                block[j] = step_start.broadcast(j) + (step.broadcast(j) * percents);

            const bool step_valid =
                (environment.attachments) ?
                    Robot::template fkcc_attach<rake>(environment, block) :
                    Robot::template fkcc<rake>(environment, block);

            if (not step_valid)
            {
                // ── Refine: find the furthest individually-valid point ────
                // Scan backward through the rake points of this step.
                // The k-th point (0-indexed from the back) is at fraction:
                //     base_frac + (rake - k) / (n * rake)
                // where base_frac = i / n.
                const float base_frac  = static_cast<float>(i) / static_cast<float>(n);
                const float lane_width = 1.0f / static_cast<float>(n * rake);

                for (int k = static_cast<int>(rake) - 1; k >= 0; --k)
                {
                    const float frac      = base_frac + static_cast<float>(k + 1) * lane_width;
                    const auto candidate  = start + vector * frac;

                    // Broadcast this single config across all rake SIMD lanes.
                    // fkcc/fkcc_attach then checks the same config `rake` times
                    // in parallel — equivalent to a single collision test.
                    for (auto j = 0U; j < Robot::dimension; ++j)
                        block[j] = candidate.broadcast(j);

                    const bool single_valid =
                        (environment.attachments) ?
                            Robot::template fkcc_attach<rake>(environment, block) :
                            Robot::template fkcc<rake>(environment, block);

                    if (single_valid)
                    {
                        last_valid = candidate;
                        return frac;
                    }
                }

                // Every lane in this step was invalid; the last valid point is
                // the start of this step (= end of the previous fully-valid step).
                last_valid = step_start;
                return base_frac;
            }

            // ── Step was fully valid; record progress ─────────────────────
            // step_end = start + step * (i + 1)
            // We update last_valid lazily — only needed if a later step fails.
            // If the whole loop completes, we set last_valid to goal below.
        }

        // Full segment is valid.
        last_valid = start + vector;
        return 1.0f;
    }

    // ─── validate_motion ─────────────────────────────────────────────────────

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
    }

    // ─── validate_motion_partial ─────────────────────────────────────────────
    //
    // Convenience wrapper around validate_vector_partial that computes the
    // direction vector internally.  Mirrors the validate_motion / validate_vector
    // split in the original file.
    //
    // Returns the valid fraction in [0, 1] and writes the last valid
    // configuration to `last_valid`.
    //
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline auto validate_motion_partial(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        typename Robot::Configuration &last_valid) -> float
    {
        auto vector = goal - start;
        return validate_vector_partial<Robot, rake, resolution>(
            start, vector, vector.l2_norm(), environment, last_valid);
    }

}  // namespace vamp::planning
