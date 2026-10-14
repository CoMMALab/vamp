#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace vamp::planning
{
    enum class TraceEvent : std::uint8_t
    {
        Root,          // the start node (node 0)
        Sample,        // a random configuration drawn for exploration
        Extend,        // a node added by the exploration step (parent = the nearest node)
        ExtendFailed,  // exploration step blocked (parent = the nearest node, configuration = the sample)
        WalkStart,     // a goal-directed walk begins at `node`
        WalkLm,        // unclamped LM endpoint proposed from `parent`
        WalkProposal,  // clamped step endpoint tried from `parent`; see outcome
        Solved,        // `node` satisfies the task-space region
    };

    enum class TraceOutcome : std::uint8_t
    {
        None,
        Accepted,
        ZeroStep,
        NoImprovement,
        OutOfBounds,
        Collision,
        TreeFull,
    };

    // Optional record of everything TaskSpaceRRTC does, for debugging and visualization.
    // Entries are in execution order; pass a trace to TaskSpaceRRTC::solve to fill it.
    template <typename Robot>
    struct TaskSpaceTrace
    {
        static constexpr std::size_t dimension = Robot::dimension;

        struct Entry
        {
            TraceEvent event;
            TraceOutcome outcome;
            std::uint32_t iteration;
            std::int32_t node;    // node index created or acted on; -1 if none
            std::int32_t parent;  // node the entry extends from; -1 if none
            float error;          // squared task-space error at `configuration` (NaN if not evaluated)
            float reference_error;  // squared error at the node a walk step started from (NaN if n/a)
            std::array<float, dimension> configuration;
        };

        std::vector<Entry> entries;

        // Binary layout, little endian and packed: "TSRTRACE", u32 version (1), u32 dimension,
        // u64 count, then per entry: u8 event, u8 outcome, u32 iteration, i32 node, i32 parent,
        // f32 error, f32 reference_error, f32 configuration[dimension].
        inline auto save(const std::string &path) const -> bool
        {
            std::ofstream out(path, std::ios::binary);
            if (not out)
            {
                return false;
            }

            const auto put = [&out](const auto &value)
            { out.write(reinterpret_cast<const char *>(&value), sizeof(value)); };

            out.write("TSRTRACE", 8);
            put(std::uint32_t{1});
            put(static_cast<std::uint32_t>(dimension));
            put(static_cast<std::uint64_t>(entries.size()));
            for (const auto &entry : entries)
            {
                put(static_cast<std::uint8_t>(entry.event));
                put(static_cast<std::uint8_t>(entry.outcome));
                put(entry.iteration);
                put(entry.node);
                put(entry.parent);
                put(entry.error);
                put(entry.reference_error);
                for (const auto value : entry.configuration)
                {
                    put(value);
                }
            }

            return static_cast<bool>(out);
        }
    };
}  // namespace vamp::planning
