#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <vamp/planning/constraints/manifold/constraint_set.hh>

namespace vamp::planning::constraint
{
    // Orthonormal basis of the constraint manifold's tangent space at q0: chart
    // coordinates u map to the ambient pre-image q0 + B u, and lift onto the manifold
    // by constraint projection.
    template <std::size_t d>
    struct Chart
    {
        std::array<float, d> q0{};
        // basis[c][j]: c-th orthonormal tangent vector, ambient component j
        std::array<std::array<float, d>, d> basis{};
        std::size_t nc = 0;
        bool valid = false;
    };

    // B^T x: chart coordinates of an ambient displacement or velocity.
    template <std::size_t d>
    inline auto to_chart(const Chart<d> &chart, const std::array<float, d> &x) noexcept
        -> std::array<float, d>
    {
        std::array<float, d> u{};
        for (auto c = 0U; c < chart.nc; ++c)
        {
            float dot = 0.F;
            for (auto j = 0U; j < d; ++j)
            {
                dot += chart.basis[c][j] * x[j];
            }

            u[c] = dot;
        }

        return u;
    }

    // B u: ambient displacement of chart coordinates.
    template <std::size_t d>
    inline auto from_chart(const Chart<d> &chart, const std::array<float, d> &u) noexcept
        -> std::array<float, d>
    {
        std::array<float, d> x{};
        for (auto c = 0U; c < chart.nc; ++c)
        {
            for (auto j = 0U; j < d; ++j)
            {
                x[j] += chart.basis[c][j] * u[c];
            }
        }

        return x;
    }

    // psi(u) = q0 + B u: the ambient pre-image of chart coordinates u. Summation
    // starts from q0 (not B u + q0): edge samples were validated with this exact
    // accumulation order, and reconstruction must reproduce their bits.
    template <std::size_t d>
    inline auto chart_point(const Chart<d> &chart, const std::array<float, d> &u) noexcept
        -> std::array<float, d>
    {
        auto q = chart.q0;
        for (auto c = 0U; c < chart.nc; ++c)
        {
            for (auto j = 0U; j < d; ++j)
            {
                q[j] += chart.basis[c][j] * u[c];
            }
        }

        return q;
    }

    // Project v into the tangent space at the chart center: v <- B B^T v.
    template <std::size_t d>
    inline auto tangent_project(const Chart<d> &chart, const std::array<float, d> &v) noexcept
        -> std::array<float, d>
    {
        return from_chart(chart, to_chart(chart, v));
    }

    // Builds tangent-space charts of the manifold cut out by a ConstraintSet's active
    // rows, via column-pivoted QR of the transposed stacked constraint Jacobian. The
    // constraint set is passed per call, never stored: callers own both and may move
    // freely. Holds mutable evaluation buffers: not thread-safe, use one instance per
    // thread.
    template <typename Ambient, std::size_t rake>
    struct ChartBuilder
    {
        static constexpr std::size_t d = Ambient::dimension;

        using AmbientConfiguration = typename Ambient::Configuration;
        using AmbientBlock = typename Ambient::template ConfigurationBlock<rake>;

        ChartBuilder(const ConstraintSet<Ambient, rake> &constraints, float rank_tolerance) noexcept
          : rank_tolerance_(rank_tolerance)
          , m_rows_(constraints.total_rows())
          , active_(std::make_unique<bool[]>(m_rows_))
          , pfaffian_(std::make_unique<bool[]>(m_rows_))
          , err_(m_rows_)
          , jac_(m_rows_ * d)
        {
            constraints.active_rows(active_.get());
            constraints.pfaffian_rows(pfaffian_.get());
            for (auto i = 0U; i < m_rows_; ++i)
            {
                n_active_ += active_[i];
                n_pfaffian_ += pfaffian_[i];
            }

            if (n_active_ > 0)
            {
                jat_.resize(d, static_cast<Eigen::Index>(n_active_));
                q_.resize(d, d);
            }
        }

        // Total stacked constraint rows (the error/Jacobian row count).
        auto rows() const noexcept -> std::size_t
        {
            return m_rows_;
        }

        // Number of manifold-defining (chart-active) rows.
        auto n_active() const noexcept -> std::size_t
        {
            return n_active_;
        }

        // Number of Pfaffian rows.
        auto n_pfaffian() const noexcept -> std::size_t
        {
            return n_pfaffian_;
        }

        // Accumulated displacement of `chord` along the Pfaffian rows' normals:
        // sum_r |row_r . chord| over the Pfaffian rows of a stacked Jacobian
        // (layout as ConstraintSet::error_jacobian). Non-finite rows contribute nothing,
        // matching the chart machinery's leniency toward NaN Jacobians on the manifold.
        auto slip(const float *jac, const float *chord) const noexcept -> float
        {
            float s = 0.F;
            for (auto r = 0U; r < m_rows_; ++r)
            {
                if (not pfaffian_[r])
                {
                    continue;
                }

                float dot = 0.F;
                for (auto c = 0U; c < d; ++c)
                {
                    dot += jac[r * d + c] * chord[c];
                }

                if (std::isfinite(dot))
                {
                    s += std::abs(dot);
                }
            }

            return s;
        }

        // Whether every chart-active row of a stacked Jacobian (layout as
        // ConstraintSet::error_jacobian) is finite.
        auto active_rows_finite(const float *jac) const noexcept -> bool
        {
            for (auto r = 0U; r < m_rows_; ++r)
            {
                if (not active_[r])
                {
                    continue;
                }

                for (auto c = 0U; c < d; ++c)
                {
                    if (not std::isfinite(jac[r * d + c]))
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        // Orthonormal tangent basis of ker J_active at q0 via pivoted QR of the transposed
        // stacked active-row constraint Jacobian. An empty or slab-only constraint set
        // yields the identity chart (nc = d), reducing every edge to an unconstrained
        // ambient LQMT.
        auto make_chart(
            const ConstraintSet<Ambient, rake> &constraints,
            const std::array<float, d> &q0) const noexcept -> Chart<d>
        {
            if (n_active_ == 0)
            {
                Chart<d> chart;
                chart.q0 = q0;
                chart.nc = d;
                for (auto c = 0U; c < d; ++c)
                {
                    chart.basis[c][c] = 1.F;
                }

                chart.valid = true;
                return chart;
            }

            // The traced rotation-error Jacobian is NaN exactly on the manifold (acos at
            // argument 1). J is continuous, so evaluate at a deterministically perturbed
            // point when needed; the O(1e-3) offset is far below the chart error budget.
            for (auto attempt = 0U; attempt < 4; ++attempt)
            {
                auto q_eval = q0;
                if (attempt > 0)
                {
                    const float delta = 1e-3F * static_cast<float>(attempt);
                    for (auto j = 0U; j < d; ++j)
                    {
                        q_eval[j] += (j % 2 == 0) ? delta : -delta;
                    }
                }

                constraints.error_jacobian(broadcast(q_eval), 0, err_.data(), jac_.data());
                if (active_rows_finite(jac_.data()))
                {
                    return chart_from_jacobian(q0, jac_.data());
                }
            }

            Chart<d> chart;
            chart.q0 = q0;
            return chart;
        }

        // Chart at q0 built from a precomputed, finite stacked Jacobian evaluated there
        // (assumes n_active() > 0; use make_chart when no Jacobian is at hand).
        auto chart_from_jacobian(const std::array<float, d> &q0, const float *jac)
            const noexcept -> Chart<d>
        {
            Chart<d> chart;
            chart.q0 = q0;

            std::size_t k = 0;
            for (auto r = 0U; r < m_rows_; ++r)
            {
                if (not active_[r])
                {
                    continue;
                }

                for (auto c = 0U; c < d; ++c)
                {
                    jat_(c, k) = jac[r * d + c];
                }

                ++k;
            }

            // Column-pivoted QR of J_active^T: the leading rank columns of Q span
            // range(J^T) and the trailing d - rank columns its orthogonal complement,
            // ker J -- the tangent basis. Pivoting keeps the R diagonal non-increasing in
            // magnitude, so it stands in for the singular values in the rank cutoff at a
            // fraction of the cost, and the preallocated decomposition avoids per-call
            // heap traffic.
            qr_.compute(jat_);
            const auto &QR = qr_.matrixQR();

            std::size_t rank = 0;
            const float tol = rank_tolerance_ * std::max(std::abs(QR(0, 0)), 1e-6F);
            const auto n_diag = std::min<std::size_t>(d, n_active_);
            for (auto i = 0U; i < n_diag; ++i)
            {
                rank += std::abs(QR(i, i)) > tol;
            }

            chart.nc = d - rank;
            q_ = qr_.householderQ();  // d x d
            for (auto c = 0U; c < chart.nc; ++c)
            {
                for (auto j = 0U; j < d; ++j)
                {
                    chart.basis[c][j] = q_(j, rank + c);
                }
            }

            chart.valid = chart.nc > 0;
            return chart;
        }

        // Stacked constraint-violation error norm at q, over all rows (non-finite entries
        // -- the traced rotation error is NaN exactly on the manifold -- are skipped).
        auto error_norm(
            const ConstraintSet<Ambient, rake> &constraints,
            const std::array<float, d> &q) const noexcept -> float
        {
            constraints.error_jacobian(broadcast(q), 0, err_.data(), jac_.data());
            float e2 = 0.F;
            for (auto r = 0U; r < m_rows_; ++r)
            {
                if (std::isfinite(err_[r]))
                {
                    e2 += err_[r] * err_[r];
                }
            }

            return std::sqrt(e2);
        }

    private:
        inline static auto broadcast(const std::array<float, d> &q) noexcept -> AmbientBlock
        {
            AmbientConfiguration cfg(q);
            AmbientBlock block;
            for (auto i = 0U; i < d; ++i)
            {
                block[i] = cfg.broadcast(i);
            }

            return block;
        }

        float rank_tolerance_;
        std::size_t m_rows_;
        std::size_t n_active_ = 0;
        std::size_t n_pfaffian_ = 0;
        std::unique_ptr<bool[]> active_;
        std::unique_ptr<bool[]> pfaffian_;
        mutable std::vector<float> err_;
        mutable std::vector<float> jac_;
        mutable Eigen::MatrixXf jat_;  // stacked active-row Jacobian, transposed (d x n_active)
        mutable Eigen::ColPivHouseholderQR<Eigen::MatrixXf> qr_;
        mutable Eigen::MatrixXf q_;  // materialized Q factor (d x d)
    };
}  // namespace vamp::planning::constraint
