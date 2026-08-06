#pragma once

#include <array>
#include <cstdint>

#include <vamp/vector.hh>
#include <vamp/random/rng.hh>

namespace vamp::rng
{
    // Self-contained SIMD xorshift sampler -- no external simdxorshift dependency. Runs one
    // Marsaglia xorshift32 generator per 32-bit lane of the sample vector, i.e. one independent
    // stream per sampled dimension, advanced together each next() via the vamp vector ops
    // (xor / shift). Lanes are seeded with decorrelated, nonzero splitmix32 values so the
    // per-dimension streams do not correlate. Quality is ample for RRT sampling (period 2^32-1
    // per lane); the point of xorshift here is to avoid high-dimensional Halton correlation, not
    // to pass BigCrush.
    template <typename Robot>
    struct XORShiftNative : public RNG<Robot>
    {
        using Configuration = typename Robot::Configuration;
        using IntVector = Vector<SIMDVector<__m256i>, 1, Robot::sample_dimension>;
        using IntScalar = typename IntVector::S::ScalarT;
        static constexpr std::size_t n_seed = IntVector::num_scalars;

        explicit XORShiftNative(std::uint32_t seed = 0x9e3779b9u) noexcept : seed_init(seed)
        {
            reseed(seed);
        }

        virtual ~XORShiftNative() = default;

        std::uint32_t seed_init;
        IntVector state;

        // splitmix32: scrambles a counter into a well-mixed 32-bit value; used only to derive
        // decorrelated per-lane seeds.
        static inline auto splitmix32(std::uint32_t &x) noexcept -> std::uint32_t
        {
            x += 0x9e3779b9u;
            std::uint32_t z = x;
            z = (z ^ (z >> 16)) * 0x21f0aaadu;
            z = (z ^ (z >> 15)) * 0x735a2d97u;
            return z ^ (z >> 15);
        }

        inline void reseed(std::uint32_t seed) noexcept
        {
            std::array<IntScalar, n_seed> s{};
            std::uint32_t x = seed;
            for (auto &v : s)
            {
                const std::uint32_t r = splitmix32(x) | 1u;  // nonzero: xorshift fixes 0
                v = static_cast<IntScalar>(r);
            }

            state = IntVector(s);
        }

        inline void reset() noexcept override final
        {
            reseed(seed_init);
        }

        inline auto next() noexcept -> Configuration override final
        {
            // xorshift32 step across every lane at once.
            state = state.xor_(state << 13u);
            state = state.xor_(state >> 17u);
            state = state.xor_(state << 5u);

            return Robot::sample(
                FloatVector<Robot::sample_dimension>::map_to_range(state, 0.F, 1.F));
        }
    };
}  // namespace vamp::rng
