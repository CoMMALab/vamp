#pragma once

#include <cstdint>

#include <type_traits>

#include <vamp/planning/space.hh>
#include <vamp/utils.hh>
#include <vamp/robots/traits.hh>
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
        return (not environment.attachments.empty()) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                       Robot::template fkcc<rake>(environment, block);
    }

    // Collision-check a block of states of `Space`: the block is resolved into the ambient robot's
    // configurations first (a lane that cannot be resolved makes the block invalid), then checked there. For
    // the robot's own space the block is already its configurations.
    template <typename Robot, std::size_t rake, typename Space = Robot>
    inline auto fkcc_state_block(
        const collision::Environment<FloatVector<rake>> &environment,
        const space::StateBlock<Space, rake> &block) noexcept -> bool
    {
        if constexpr (std::is_same_v<Space, Robot>)
        {
            return fkcc_block<Robot, rake>(environment, block);
        }
        else
        {
            static_assert(
                std::is_same_v<space::Ambient<Space>, Robot>, "the space must sit over the planner's robot");
            const auto [resolved, ambient] = space::resolve<Space, rake>(block);
            return resolved and fkcc_block<Robot, rake>(environment, ambient);
        }
    }

    // Collision-check a single state by broadcasting it across every rake lane
    // and running the fused FK+CC kernel once.
    template <typename Robot, std::size_t rake, typename Space = Robot>
    inline auto validate_configuration(
        const space::State<Space> &configuration,
        const collision::Environment<FloatVector<rake>> &environment) noexcept -> bool
    {
        space::StateBlock<Space, rake> block;
        for (auto i = 0U; i < Space::dimension; ++i)
        {
            block[i] = configuration.broadcast(i);
        }

        return fkcc_state_block<Robot, rake, Space>(environment, block);
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

    // Validity of the motion start -> goal in `Space` (the robot's own space by default). In another space the
    // motion is sampled in the space (interpolate_block) and every sample block is resolved into the ambient
    // robot's configurations and checked there; `block_accept` then judges the resolved blocks.
    template <
        typename Robot,
        std::size_t rake,
        std::size_t resolution,
        typename BlockAccept = AlwaysTrue,
        typename Space = Robot>
    inline constexpr auto validate_motion(
        const space::State<Space> &start,
        const space::State<Space> &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        const BlockAccept &block_accept = BlockAccept()) -> bool
    {
        if constexpr (not std::is_same_v<Space, Robot>)
        {
            const float distance = space::distance<Space>(start, goal);
            const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
            const auto percents = FloatVector<rake>(Percents<rake>::percents);
            const auto t_step = FloatVector<rake>::fill(1.F / static_cast<float>(rake * n));

            space::StateBlock<Space, rake> block;
            auto t_block = percents;
            for (auto i = 0U; i < n; ++i)
            {
                space::interpolate_block<Space, rake>(start, goal, t_block, block);
                const auto [resolved, ambient] = space::resolve<Space, rake>(block);
                if (not(resolved and block_accept(ambient) and fkcc_block<Robot, rake>(environment, ambient)))
                {
                    return false;
                }

                t_block = t_block - t_step;
            }

            return true;
        }
        else if constexpr (robots::is_euclidean_v<Robot>)
        {
            auto vector = goal - start;
            return validate_vector<Robot, rake, resolution>(
                start, vector, vector.l2_norm(), environment, block_accept);
        }
        else
        {
            // Joint-aware motion validation: interpolate via Robot::interpolate_block.
            // Total sub-states needed: ceil(distance * resolution); processed in batches of `rake`.
            const float distance = robots::distance<Robot>(start, goal);
            const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
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
