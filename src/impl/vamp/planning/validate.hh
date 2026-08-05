#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>

namespace vamp::planning
{
    // Default accept predicate: accepts every candidate.
    struct AlwaysTrue
    {
        template <typename T>
        constexpr auto operator()(const T &) const noexcept -> bool
        {
            return true;
        }
    };

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

    // Collision-check a block, dispatching to the attachment-aware kernel when the
    // environment carries attachments. True when every lane is collision-free.
    template <typename Robot, std::size_t rake>
    inline auto fkcc_block(
        const collision::Environment<FloatVector<rake>> &environment,
        const typename Robot::template ConfigurationBlock<rake> &block) noexcept -> bool
    {
        if (not environment.attachments.empty())
            return Robot::template fkcc_attach<rake>(environment, block);
#ifdef VAMP_FKCC_SINCOS
        return Robot::template fkcc_sincos<rake>(environment, block);
#else
        return Robot::template fkcc<rake>(environment, block);
#endif
    }

    // Collision-check a single configuration by broadcasting it across every rake lane
    // and running the fused FK+CC kernel once.
    template <typename Robot, std::size_t rake>
    inline auto validate_configuration(
        const typename Robot::Configuration &configuration,
        const collision::Environment<FloatVector<rake>> &environment) noexcept -> bool
    {
        typename Robot::template ConfigurationBlock<rake> block;
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = configuration.broadcast(i);
        }

        return fkcc_block<Robot, rake>(environment, block);
    }

    // `block_accept` is invoked on every interpolated sample block before the collision
    // check; returning false invalidates the motion. All rake lanes of every block are
    // legitimate samples of the motion (trailing batches backstep over earlier ones), so
    // the predicate must judge all lanes. The default accepts everything at zero cost.
    template <typename Robot, std::size_t rake, std::size_t resolution, typename BlockAccept = AlwaysTrue>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment,
        const BlockAccept &block_accept = BlockAccept()) -> bool
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

        bool valid = block_accept(block) and fkcc_block<Robot, rake>(environment, block);
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

            bool valid = block_accept(block) and fkcc_block<Robot, rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename BlockAccept = AlwaysTrue>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        const BlockAccept &block_accept = BlockAccept()) -> bool
    {
        if constexpr (Robot::euclidean)
        {
            auto vector = goal - start;
            return validate_vector<Robot, rake, resolution>(
                start, vector, vector.l2_norm(), environment, block_accept);
        }
        else
        {
            // Joint-aware motion validation: interpolate via Robot::interpolate_block.
            // Total sub-states needed: ceil(distance * resolution); processed in batches of `rake`.
            const float distance = Robot::distance(start, goal);
            const std::size_t n =
                std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
            const auto percents = FloatVector<rake>(Percents<rake>::percents);
            const auto t_step = FloatVector<rake>::fill(1.F / static_cast<float>(rake * n));

            typename Robot::template ConfigurationBlock<rake> block;
            auto t_block = percents;
            Robot::template interpolate_block<rake>(start, goal, t_block, block);

            bool valid = block_accept(block) and fkcc_block<Robot, rake>(environment, block);
            if (not valid or n == 1)
            {
                return valid;
            }

            for (auto i = 1U; i < n; ++i)
            {
                t_block = t_block - t_step;
                Robot::template interpolate_block<rake>(start, goal, t_block, block);

                valid = block_accept(block) and fkcc_block<Robot, rake>(environment, block);
                if (not valid)
                {
                    return false;
                }
            }
            return true;
        }
    }
}  // namespace vamp::planning
