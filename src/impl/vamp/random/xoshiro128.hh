#pragma once

#include <array>
#include <cstdint>

#include <vamp/vector.hh>
#include <vamp/random/rng.hh>

namespace vamp::rng
{
    // Self-contained SIMD xoshiro128** sampler -- no external simdxorshift dependency, but the
    // full 128-bit-state generator (Blackman & Vigna), which passes BigCrush, rather than the
    // period-2^32 xorshift32 in xorshift_native.hh. One independent generator per 32-bit lane
    // (i.e. per sampled dimension): four state words s0..s3 held as four lane-vectors, advanced
    // together each next() with the vamp vector ops (mul / shift / rotate / xor). Lanes are seeded
    // with decorrelated nonzero splitmix32 values (all-zero state is the only forbidden state and
    // is guarded against). This is the intended dependency-free replacement for the external
    // xorshift128+.
    template <typename Robot>
    struct Xoshiro128 : public RNG<Robot>
    {
        using Configuration = typename Robot::Configuration;
        using IntVector = Vector<SIMDVector<__m256i>, 1, Robot::sample_dimension>;
        using IntScalar = typename IntVector::S::ScalarT;
        static constexpr std::size_t n_seed = IntVector::num_scalars;

        explicit Xoshiro128(std::uint32_t seed = 0x9e3779b9u) noexcept : seed_init(seed)
        {
            reseed(seed);
        }

        virtual ~Xoshiro128() = default;

        std::uint32_t seed_init;
        IntVector s0, s1, s2, s3;

        static inline auto splitmix32(std::uint32_t &x) noexcept -> std::uint32_t
        {
            x += 0x9e3779b9u;
            std::uint32_t z = x;
            z = (z ^ (z >> 16)) * 0x21f0aaadu;
            z = (z ^ (z >> 15)) * 0x735a2d97u;
            return z ^ (z >> 15);
        }

        // Lane-wise 32-bit rotate-left, built from the shift/or ops (0 < k < 32).
        static inline auto rotl(const IntVector &x, unsigned int k) noexcept -> IntVector
        {
            return (x << k).or_(x >> (32u - k));
        }

        inline void reseed(std::uint32_t seed) noexcept
        {
            std::array<IntScalar, n_seed> a0{}, a1{}, a2{}, a3{};
            std::uint32_t x = seed;
            for (std::size_t i = 0; i < n_seed; ++i)
            {
                a0[i] = static_cast<IntScalar>(splitmix32(x) | 1u);  // guard: state never all-zero
                a1[i] = static_cast<IntScalar>(splitmix32(x));
                a2[i] = static_cast<IntScalar>(splitmix32(x));
                a3[i] = static_cast<IntScalar>(splitmix32(x));
            }

            s0 = IntVector(a0);
            s1 = IntVector(a1);
            s2 = IntVector(a2);
            s3 = IntVector(a3);
        }

        inline void reset() noexcept override final
        {
            reseed(seed_init);
        }

        inline auto next() noexcept -> Configuration override final
        {
            // result = rotl(s1 * 5, 7) * 9   (computed from state before the update)
            const auto result = rotl(s1.mul(IntVector::fill(5)), 7).mul(IntVector::fill(9));

            const auto t = s1 << 9u;
            s2 = s2.xor_(s0);
            s3 = s3.xor_(s1);
            s1 = s1.xor_(s2);
            s0 = s0.xor_(s3);
            s2 = s2.xor_(t);
            s3 = rotl(s3, 11);

            return Robot::sample(
                FloatVector<Robot::sample_dimension>::map_to_range(result, 0.F, 1.F));
        }
    };
}  // namespace vamp::rng
