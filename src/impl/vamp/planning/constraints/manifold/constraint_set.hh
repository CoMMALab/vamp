#pragma once

#include <array>
#include <memory>
#include <vector>

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/settings.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Runtime collection of constraints defining a manifold, with the iterative-projection
    // loops planners use. An empty set is valid and satisfied everywhere. Not thread-safe
    // (constraints cache per-evaluation state): use one set (with unshared constraints) per
    // thread.
    template <typename Robot, std::size_t rake>
    class ConstraintSet
    {
    public:
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        using Row = FloatVector<rake, 1>;
        using Ptr = std::shared_ptr<const Constraint<Robot, rake>>;

        ConstraintSet() = default;

        explicit ConstraintSet(std::vector<Ptr> constraints, ConstraintSettings settings = {}) noexcept
          : constraints_(std::move(constraints)), settings_(settings)
        {
        }

        auto empty() const noexcept -> bool
        {
            return constraints_.empty();
        }

        auto settings() const noexcept -> const ConstraintSettings &
        {
            return settings_;
        }

        // Pin dimensions during projection (pinned[j] true): every descent step's Jacobian
        // columns for the pinned dimensions are zeroed, so projection solves within the
        // active subspace and pinned joints hold their values exactly. Configurations fed
        // to the projection loops must already sit on the pinned slice.
        void set_pinned(const std::array<bool, Robot::dimension> &pinned) noexcept
        {
            pinned_ = pinned;
            has_pins_ = false;
            for (const auto p : pinned)
            {
                has_pins_ |= p;
            }
        }

        // Per-lane squared violation error summed over all constraints. Caches each
        // constraint's Jacobian and error for step_in_place(). An empty set has zero
        // error everywhere.
        auto squared_error(const Block &q) const noexcept -> Row
        {
            if (empty())
            {
                return Row::fill(0.F);
            }

            auto d = constraints_.front()->squared_error(q);
            for (auto it = std::next(constraints_.begin()); it != constraints_.end(); ++it)
            {
                d = d + (*it)->squared_error(q);
            }

            return d;
        }

        // Total scalar rows across all constraints in a stacked error/Jacobian.
        auto total_rows() const noexcept -> std::size_t
        {
            std::size_t n = 0;
            for (const auto &c : constraints_)
            {
                n += c->n_rows();
            }

            return n;
        }

        // Concatenated manifold-defining row flags (total_rows() entries), in constraint
        // order.
        void active_rows(bool *rows) const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->active_rows(rows);
                rows += c->n_rows();
            }
        }

        // Concatenated Pfaffian row flags (total_rows() entries), in constraint order.
        void pfaffian_rows(bool *rows) const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->pfaffian_rows(rows);
                rows += c->n_rows();
            }
        }

        // Stacked violation error (total_rows() entries) and raw-error Jacobian (total_rows()
        // x Robot::dimension, row-major) of one lane of q, in constraint order. The stacked
        // Jacobian's null space is the tangent space of the manifold the whole set defines.
        // Overwrites the constraints' squared_error() caches.
        void error_jacobian(const Block &q, std::size_t lane, float *err, float *jac)
            const noexcept
        {
            evaluate_error_jacobian(q);
            extract_error_jacobian(lane, err, jac);
        }

        // Run every constraint's error/Jacobian kernel on the whole block, caching all
        // lanes for extract_error_jacobian(): batch callers pay each kernel once instead
        // of per lane. Overwrites the constraints' squared_error() caches.
        void evaluate_error_jacobian(const Block &q) const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->evaluate_error_jacobian(q);
            }
        }

        // Stacked per-lane error and Jacobian (layout as error_jacobian()) from the caches
        // of the last evaluate_error_jacobian() call.
        void extract_error_jacobian(std::size_t lane, float *err, float *jac) const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->extract_error_jacobian(lane, err, jac);
                err += c->n_rows();
                jac += c->n_rows() * Robot::dimension;
            }
        }

        // One simultaneous (Jacobi-style) descent step across all constraints, consuming the
        // caches of the last squared_error() call on the same q.
        void step_in_place(Block &q) const noexcept
        {
            for (const auto &c : constraints_)
            {
                if (has_pins_)
                {
                    c->mask_jacobian(pinned_.data());
                }

                c->step(q, settings_.method, settings_.descend_rate);
            }
        }

        // Iteratively project every lane of q onto the manifold. Returns false if any lane
        // fails to converge within the iteration cap, stalls, or drifts more than
        // 2 * max_distance from where it started.
        auto project_all(Block &q, float max_distance = no_max_distance) const noexcept -> bool
        {
            if (empty())
            {
                return true;
            }

            return descend<true>(q, max_distance).test_all_less_equal(Row::fill(settings_.tolerance));
        }

        // Project lanes of q until any lane lands on the manifold. Returns the first
        // converged lane, or -1 if none converged. Lanes only abort collectively: the loop
        // bails when every lane stalls or every lane drifts beyond 2 * max_distance.
        auto project_any(Block &q, float max_distance = no_max_distance) const noexcept -> int
        {
            if (empty())
            {
                return 0;
            }

            const auto dist = descend<false>(q, max_distance);
            for (auto lane = 0U; lane < rake; ++lane)
            {
                if (dist[{0, lane}] <= settings_.tolerance)
                {
                    return static_cast<int>(lane);
                }
            }

            return -1;
        }

        // Project a single configuration onto the manifold (broadcast across lanes, so all
        // lanes are identical and the result is deterministic).
        auto project(Configuration &q) const noexcept -> bool
        {
            if (empty())
            {
                return true;
            }

            Block block;
            broadcast(q, block);
            if (not project_all(block))
            {
                return false;
            }

            std::array<float, Robot::dimension> array;
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                array[i] = block[{i, 0}];
            }

            q = Configuration(array);
            return true;
        }

        // Whether a configuration satisfies every constraint within tolerance.
        auto satisfied(const Configuration &q) const noexcept -> bool
        {
            if (empty())
            {
                return true;
            }

            Block block;
            broadcast(q, block);
            return squared_error(block)[{0, 0}] <= settings_.tolerance;
        }

        static auto squared_distance(const Block &a, const Block &b) noexcept -> Row
        {
            auto d = (a[0] - b[0]) * (a[0] - b[0]);
            for (auto i = 1U; i < Robot::dimension; ++i)
            {
                d = d + (a[i] - b[i]) * (a[i] - b[i]);
            }

            return d;
        }

    private:
        // Effectively unbounded drift: the deviation threshold check never trips.
        static constexpr float no_max_distance = 1e9F;

        // Squared step size below which a lane counts as stalled. This must sit well below
        // the error tolerance: near the tolerance boundary a converging descent takes steps
        // of roughly sqrt(tolerance) / |J|, so judging stall by the error tolerance itself
        // aborts runs that are still making progress.
        static constexpr float stall_epsilon2 = 1e-12F;

        static void broadcast(const Configuration &q, Block &block) noexcept
        {
            for (auto i = 0U; i < Robot::dimension; ++i)
            {
                block[i] = q.broadcast(i);
            }
        }

        // Shared descent loop of project_all()/project_any(), returning the final per-lane
        // squared errors. Iterates until the convergence test passes (`all` lanes vs. any
        // lane within tolerance), every lane stalls, or the drift guard trips -- any lane
        // for all-mode (one runaway lane already fails it), every lane for any-mode (one
        // live lane may still converge; its threshold is padded so the tiny step lengths
        // steer passes as max_distance don't trip the guard on the first iteration).
        template <bool all>
        auto descend(Block &q, float max_distance) const noexcept -> Row
        {
            const auto tolerance = Row::fill(settings_.tolerance);
            const auto stall = Row::fill(stall_epsilon2);

            float threshold2 = 4.F * max_distance * max_distance;
            if constexpr (not all)
            {
                threshold2 += 1e-6F;
            }
            const auto threshold = Row::fill(threshold2);

            const auto converged = [&tolerance](const Row &d)
            {
                if constexpr (all)
                {
                    return d.test_all_less_equal(tolerance);
                }
                else
                {
                    return d.test_any_less_equal(tolerance);
                }
            };

            const auto drifted = [&threshold](const Row &d2)
            {
                if constexpr (all)
                {
                    return d2.test_any_greater_equal(threshold);
                }
                else
                {
                    return d2.test_all_greater_equal(threshold);
                }
            };

            const auto start = q;
            Block prev;

            auto dist = squared_error(q);
            for (std::size_t i = 0; i < settings_.max_iterations and not converged(dist); ++i)
            {
                prev = q;
                step_in_place(q);
                dist = squared_error(q);

                const auto step_dist = squared_distance(q, prev);
                if (step_dist.test_all_less_equal(stall))  // every lane stalled
                {
                    break;
                }

                if (drifted(step_dist) or drifted(squared_distance(q, start)))
                {
                    break;
                }
            }

            return dist;
        }

        std::vector<Ptr> constraints_;
        ConstraintSettings settings_;
        std::array<bool, Robot::dimension> pinned_{};
        bool has_pins_ = false;
    };
}  // namespace vamp::planning::constraint
