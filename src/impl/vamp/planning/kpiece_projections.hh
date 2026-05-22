#pragma once

// kpiece_projections.hh — Built-in projections for KPIECE / BKPIECE.
//
// Each projection maps a robot Configuration to a proj_dim-dimensional
// floating-point coordinate.  The planners then bin these floats into integer
// grid coordinates using settings.cell_size.
//
// Projection concept (updated):
//
//  struct MyProjection {
//      static constexpr std::size_t proj_dim = 2;
//      static void project(const Configuration &q,
//                          std::array<float, proj_dim> &coord) noexcept;
//  };

#include <array>
#include <cmath>

#include <Eigen/Geometry>

#include <vamp/vector.hh>

namespace vamp::planning
{

// ── ShoulderProjection<Robot, NDim> ─────────────────────────────────────────
//
// Projects down to the first NDim dimensions of the configuration space.

template <typename Robot, std::size_t NDim>
struct ShoulderProjection
{
    static constexpr std::size_t proj_dim = NDim;
    static_assert(proj_dim <= Robot::dimension,
                  "ShoulderProjection dimensionality cannot exceed Robot::dimension");

    static void project(
        const typename Robot::Configuration &q,
        std::array<float, proj_dim>         &coord) noexcept
    {
        const auto arr = q.to_array();
        for (std::size_t i = 0; i < proj_dim; ++i)
        {
            coord[i] = arr[i];
        }
    }
};

// ── EEProjection<Robot> ─────────────────────────────────────────────────────
//
// Projects to 3D using the end-effector position computed by Robot::eefk().
//
// Requires:  Robot::eefk(const std::array<float, Robot::dimension>&) -> Eigen::Isometry3f

template <typename Robot>
struct EEProjection
{
    static constexpr std::size_t proj_dim = 3;

    static void project(
        const typename Robot::Configuration &q,
        std::array<float, 3>                &coord) noexcept
    {
        const Eigen::Isometry3f pose = Robot::eefk(q.to_array());
        const Eigen::Vector3f   pos  = pose.translation();
        coord[0] = pos.x();
        coord[1] = pos.y();
        coord[2] = pos.z();
    }
};

}  // namespace vamp::planning