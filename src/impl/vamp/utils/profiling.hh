#pragma once

// Opt-in, near-zero-cost kernel profiling for the planning pipeline (RRTC steering,
// task-space IK resolution, collision checking, NN queries, sampling, ...).
//
// Disabled by default: every VAMP_PROFILE_SCOPE(...) call expands to nothing unless the
// translation unit is compiled with -DVAMP_PROFILING (wired up by the CMake option
// VAMP_ENABLE_PROFILING), so normal release builds carry no instrumentation at all -- not
// even a runtime branch.
//
// When enabled, each kernel gets a thread_local {count, total_ns} slot indexed directly by
// the Kernel enum (no map lookups, no allocation, no atomics) so the accounting itself stays
// cheap relative to what it's measuring.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <ostream>
#include <string_view>

namespace vamp::utils::profiling
{
    enum class Kernel : std::size_t
    {
        Sample,           // rng->next() draws in RRTC::solve
        NNNearest,        // NN<Space>::nearest() KD-tree queries
        NNInsert,         // NN<Space>::insert() KD-tree insertions
        Steer,            // LocalPlanner::steer() (includes Validate below)
        ConnectWithin,    // LocalPlanner::connect_within() (direct start->goal attempts)
        Validate,         // LocalPlanner::validate()/validate_resolved() (includes ResolveIK
                          // + SupportPolygon + CollisionCheck below)
        ResolveIK,        // Space::resolve_block() -- task-space -> ambient IK resolution
        SupportPolygon,   // com_within_support_polygon() static-stability check (RBY1 only)
        CollisionCheck,   // Ambient::fkcc()/fkcc_attach()
        ShortcutAttempt,  // shortcut_path()'s own connect_within() attempts
        EEFCollisionCheck,  // Space::eefs_collision_free() -- task-space eef collision prefilter
        Count,            // sentinel: number of kernels, not a real kernel
    };

    inline constexpr std::array<std::string_view, static_cast<std::size_t>(Kernel::Count)> kKernelNames = {
        "Sample",
        "NNNearest",
        "NNInsert",
        "Steer",
        "ConnectWithin",
        "Validate",
        "ResolveIK",
        "SupportPolygon",
        "CollisionCheck",
        "ShortcutAttempt",
        "EEFCollisionCheck"
    };

    struct KernelStats
    {
        std::uint64_t count = 0;
        std::uint64_t total_ns = 0;
    };

    using StatsTable = std::array<KernelStats, static_cast<std::size_t>(Kernel::Count)>;

    // thread_local: RRTC::solve() runs single-threaded per call today, but this keeps the
    // scheme safe (no atomics needed) if callers ever plan multiple problems concurrently
    // from different threads. Merge threads' tables yourself if that ever happens.
    inline thread_local StatsTable stats_table{};

    inline auto reset() noexcept -> void
    {
        stats_table = StatsTable{};
    }

    inline auto record(Kernel kernel, std::uint64_t elapsed_ns) noexcept -> void
    {
        auto &entry = stats_table[static_cast<std::size_t>(kernel)];
        entry.count += 1;
        entry.total_ns += elapsed_ns;
    }

    // Prints a simple aggregate table (this thread's counters only -- see thread_local note
    // above) to `os`. Call once at the end of a driver's main(), after all planning is done.
    inline auto report(std::ostream & os) -> void
    {
        os << std::left << std::setw(18) << "kernel" << std::right << std::setw(12) << "count"
           << std::setw(14) << "total_ms" << std::setw(14) << "mean_us" << '\n';

        for (std::size_t i = 0; i < stats_table.size(); ++i)
        {
            const auto &entry = stats_table[i];
            if (entry.count == 0)
            {
                continue;
            }

            const double total_ms = static_cast<double>(entry.total_ns) / 1.0e6;
            const double mean_us =
                static_cast<double>(entry.total_ns) / static_cast<double>(entry.count) / 1.0e3;

            os << std::left << std::setw(18) << kKernelNames[i] << std::right << std::setw(12)
               << entry.count << std::setw(14) << std::fixed << std::setprecision(3) << total_ms
               << std::setw(14) << std::setprecision(3) << mean_us << '\n';
        }
    }

    // RAII scope timer: counts one call and accumulates its duration into stats_table on
    // destruction. std::chrono::steady_clock is used for portability; this is only compiled
    // in profiling builds, so its overhead is never paid by default.
    class ScopeProfiler
    {
    public:
        explicit ScopeProfiler(Kernel kernel) noexcept : kernel_(kernel), start_(std::chrono::steady_clock::now())
        {
        }

        ScopeProfiler(const ScopeProfiler &) = delete;
        auto operator=(const ScopeProfiler &) -> ScopeProfiler & = delete;
        ScopeProfiler(ScopeProfiler &&) = delete;
        auto operator=(ScopeProfiler &&) -> ScopeProfiler & = delete;

        ~ScopeProfiler()
        {
            const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                      std::chrono::steady_clock::now() - start_)
                                      .count();
            record(kernel_, static_cast<std::uint64_t>(elapsed));
        }

    private:
        Kernel kernel_;
        std::chrono::time_point<std::chrono::steady_clock> start_;
    };
}  // namespace vamp::utils::profiling

#ifdef VAMP_PROFILING
#define VAMP_PROFILE_CONCAT_INNER(a, b) a##b
#define VAMP_PROFILE_CONCAT(a, b) VAMP_PROFILE_CONCAT_INNER(a, b)
#define VAMP_PROFILE_SCOPE(kernel)                                                                            \
    ::vamp::utils::profiling::ScopeProfiler VAMP_PROFILE_CONCAT(_vamp_prof_, __LINE__)(                       \
        ::vamp::utils::profiling::Kernel::kernel)
#else
#define VAMP_PROFILE_SCOPE(kernel) ((void)0)
#endif
