#pragma once

#include <array>

#include <vamp/collision/shapes.hh>
#include <vamp/collision/math.hh>

// Oriented box vs. oriented box collision checking via the separating axis theorem (SAT).
//
// Convention (shared with all kernels in this directory): the returned vector is NEGATIVE in lanes
// where the shapes overlap, so `not result.test_zero()` means "some lane is in collision".
//
// NOTE: Unlike the sphere kernels, the magnitude is NOT a distance. It is the largest signed gap over
// all 15 candidate axes, and the nine edge-edge axes are not normalized, so only the sign is
// meaningful.
namespace vamp::collision
{
    namespace detail
    {
        // Added to |R| so that nearly-parallel edge pairs (whose cross product axis degenerates to ~0)
        // are never reported as separating because of floating point noise. This makes the test
        // slightly conservative: boxes within ~eps * (sum of half-lengths) of touching are reported as
        // colliding.
        inline constexpr float sat_epsilon = 1e-5F;

        // Core of the SAT test, following Ericson, Real-Time Collision Detection, 4.4.1.
        //
        // - `ea`, `eb`: half-lengths of boxes A and B along their own axes.
        // - `R[i][j]`: dot(A axis i, B axis j), i.e., B's axes expressed in A's frame.
        // - `t[i]`: dot(B center - A center, A axis i), i.e., the center offset in A's frame.
        template <typename DataT>
        inline auto box_box_separation(
            const std::array<DataT, 3> &ea,
            const std::array<DataT, 3> &eb,
            const std::array<std::array<DataT, 3>, 3> &R,
            const std::array<DataT, 3> &t) noexcept -> DataT
        {
            std::array<std::array<DataT, 3>, 3> abs_r;
            for (auto i = 0U; i < 3; ++i)
            {
                for (auto j = 0U; j < 3; ++j)
                {
                    abs_r[i][j] = R[i][j].abs() + sat_epsilon;
                }
            }

            // Face axes of A
            DataT gap = t[0].abs() - (ea[0] + eb[0] * abs_r[0][0] + eb[1] * abs_r[0][1] + eb[2] * abs_r[0][2]);
            for (auto i = 1U; i < 3; ++i)
            {
                gap = gap.max(
                    t[i].abs() -
                    (ea[i] + eb[0] * abs_r[i][0] + eb[1] * abs_r[i][1] + eb[2] * abs_r[i][2]));
            }

            // Face axes of B
            for (auto j = 0U; j < 3; ++j)
            {
                const auto proj = t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j];
                const auto ra = ea[0] * abs_r[0][j] + ea[1] * abs_r[1][j] + ea[2] * abs_r[2][j];
                gap = gap.max(proj.abs() - (ra + eb[j]));
            }

            // Edge-edge axes: A axis i x B axis j
            for (auto i = 0U; i < 3; ++i)
            {
                const auto i1 = (i + 1) % 3;
                const auto i2 = (i + 2) % 3;

                for (auto j = 0U; j < 3; ++j)
                {
                    const auto j1 = (j + 1) % 3;
                    const auto j2 = (j + 2) % 3;

                    const auto ra = ea[i1] * abs_r[i2][j] + ea[i2] * abs_r[i1][j];
                    const auto rb = eb[j1] * abs_r[i][j2] + eb[j2] * abs_r[i][j1];
                    const auto proj = t[i2] * R[i1][j] - t[i1] * R[i2][j];
                    gap = gap.max(proj.abs() - (ra + rb));
                }
            }

            return gap;
        }
    }  // namespace detail

    // Collision between a (robot) box `a` and an (environment) cuboid `b`. Either may be an
    // OrientedBox or a Cuboid; they only need the shared field layout and the same DataT.
    template <typename BoxA, typename BoxB>
    inline auto cuboid_cuboid(const BoxA &a, const BoxB &b) noexcept -> decltype(a.x)
    {
        using DataT = decltype(a.x);

        const std::array<DataT, 3> ea = {a.axis_1_r, a.axis_2_r, a.axis_3_r};
        const std::array<DataT, 3> eb = {b.axis_1_r, b.axis_2_r, b.axis_3_r};

        const std::array<std::array<DataT, 3>, 3> R = {{
            {dot_3(a.axis_1_x, a.axis_1_y, a.axis_1_z, b.axis_1_x, b.axis_1_y, b.axis_1_z),
             dot_3(a.axis_1_x, a.axis_1_y, a.axis_1_z, b.axis_2_x, b.axis_2_y, b.axis_2_z),
             dot_3(a.axis_1_x, a.axis_1_y, a.axis_1_z, b.axis_3_x, b.axis_3_y, b.axis_3_z)},
            {dot_3(a.axis_2_x, a.axis_2_y, a.axis_2_z, b.axis_1_x, b.axis_1_y, b.axis_1_z),
             dot_3(a.axis_2_x, a.axis_2_y, a.axis_2_z, b.axis_2_x, b.axis_2_y, b.axis_2_z),
             dot_3(a.axis_2_x, a.axis_2_y, a.axis_2_z, b.axis_3_x, b.axis_3_y, b.axis_3_z)},
            {dot_3(a.axis_3_x, a.axis_3_y, a.axis_3_z, b.axis_1_x, b.axis_1_y, b.axis_1_z),
             dot_3(a.axis_3_x, a.axis_3_y, a.axis_3_z, b.axis_2_x, b.axis_2_y, b.axis_2_z),
             dot_3(a.axis_3_x, a.axis_3_y, a.axis_3_z, b.axis_3_x, b.axis_3_y, b.axis_3_z)},
        }};

        const auto dx = b.x - a.x;
        const auto dy = b.y - a.y;
        const auto dz = b.z - a.z;

        const std::array<DataT, 3> t = {
            dot_3(dx, dy, dz, a.axis_1_x, a.axis_1_y, a.axis_1_z),
            dot_3(dx, dy, dz, a.axis_2_x, a.axis_2_y, a.axis_2_z),
            dot_3(dx, dy, dz, a.axis_3_x, a.axis_3_y, a.axis_3_z)};

        return detail::box_box_separation<DataT>(ea, eb, R, t);
    }

    // Same as cuboid_cuboid, but `b` is a z-aligned cuboid (see Environment::z_aligned_cuboids): its
    // third axis is exactly +z and the other two lie in the xy-plane, so only their x/y components are
    // read, matching sphere_z_aligned_cuboid. This makes the rotation matrix cheaper to form.
    template <typename BoxA, typename BoxB>
    inline auto cuboid_z_aligned_cuboid(const BoxA &a, const BoxB &b) noexcept -> decltype(a.x)
    {
        using DataT = decltype(a.x);

        const std::array<DataT, 3> ea = {a.axis_1_r, a.axis_2_r, a.axis_3_r};
        const std::array<DataT, 3> eb = {b.axis_1_r, b.axis_2_r, b.axis_3_r};

        const std::array<std::array<DataT, 3>, 3> R = {{
            {dot_2(a.axis_1_x, a.axis_1_y, b.axis_1_x, b.axis_1_y),
             dot_2(a.axis_1_x, a.axis_1_y, b.axis_2_x, b.axis_2_y),
             a.axis_1_z},
            {dot_2(a.axis_2_x, a.axis_2_y, b.axis_1_x, b.axis_1_y),
             dot_2(a.axis_2_x, a.axis_2_y, b.axis_2_x, b.axis_2_y),
             a.axis_2_z},
            {dot_2(a.axis_3_x, a.axis_3_y, b.axis_1_x, b.axis_1_y),
             dot_2(a.axis_3_x, a.axis_3_y, b.axis_2_x, b.axis_2_y),
             a.axis_3_z},
        }};

        const auto dx = b.x - a.x;
        const auto dy = b.y - a.y;
        const auto dz = b.z - a.z;

        const std::array<DataT, 3> t = {
            dot_3(dx, dy, dz, a.axis_1_x, a.axis_1_y, a.axis_1_z),
            dot_3(dx, dy, dz, a.axis_2_x, a.axis_2_y, a.axis_2_z),
            dot_3(dx, dy, dz, a.axis_3_x, a.axis_3_y, a.axis_3_z)};

        return detail::box_box_separation<DataT>(ea, eb, R, t);
    }
}  // namespace vamp::collision
