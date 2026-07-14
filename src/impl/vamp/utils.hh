#pragma once

#include <chrono>
#include <cstdlib>
#include <memory>
#include <type_traits>
#include <utility>
#include <cstdint>

#define VAMP_DEFINE_HAS_METHOD(method_name)                                                                  \
    template <typename T, typename = void>                                                                   \
    struct has_##method_name : std::false_type                                                               \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    struct has_##method_name<T, std::void_t<decltype(T::method_name)>> : std::true_type                      \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    constexpr bool has_##method_name##_v = has_##method_name<T>::value;

// Nested-type analogue of VAMP_DEFINE_HAS_METHOD: detects `typename T::type_name`.
#define VAMP_DEFINE_HAS_TYPE(type_name)                                                                      \
    template <typename T, typename = void>                                                                   \
    struct has_type_##type_name : std::false_type                                                            \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    struct has_type_##type_name<T, std::void_t<typename T::type_name>> : std::true_type                      \
    {                                                                                                        \
    };                                                                                                       \
                                                                                                             \
    template <typename T>                                                                                    \
    constexpr bool has_type_##type_name##_v = has_type_##type_name<T>::value;

namespace vamp::utils
{
    inline constexpr auto round_size(std::size_t size, std::size_t block) noexcept -> std::size_t
    {
        return ((size + block - 1) / block) * block;
    }

    inline bool is_aligned(const void *ptr, uintptr_t alignment) noexcept
    {
        auto iptr = reinterpret_cast<uintptr_t>(ptr);
        return not(iptr % alignment);
    }

    template <typename T>
    using buffer_ptr = std::unique_ptr<T[], decltype(&free)>;

    template <typename T, std::size_t alignment>
    inline auto buffer_alloc(std::size_t n) noexcept -> buffer_ptr<T>
    {
        static_assert(std::is_trivial_v<T>);
        return buffer_ptr<T>(
            static_cast<T *>(aligned_alloc(alignment, round_size(sizeof(T) * n, alignment))), &free);
    }

    // Because ceil isn't constexpr until C++23 for some reason.....
    inline constexpr auto c_ceil(double d) noexcept -> std::size_t
    {
        const auto s = static_cast<std::size_t>(d);
        return d > s ? s + 1 : s;
    }

    // Same deal with div()...
    inline constexpr auto c_div(std::size_t idx, std::size_t dim) noexcept
        -> std::pair<std::size_t, std::size_t>
    {
        return {idx / dim, idx % dim};
    }

    inline auto get_elapsed_nanoseconds(const std::chrono::time_point<std::chrono::steady_clock> &start)
        -> std::size_t
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
            .count();
    }
}  // namespace vamp::utils
