#pragma once

#include <array>
#include <memory>
#include <vector>

#include <vamp/planning/constraints/constraint.hh>
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

        // Per-lane squared hinged error summed over all constraints. Caches each
        // constraint's Jacobian and error for step_in_place().
        auto squared_error(const Block &q) const noexcept -> Row
        {
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

        // Stacked hinged error (total_rows() entries) and raw-error Jacobian (total_rows()
        // x Robot::dimension, row-major) of one lane of q, in constraint order. The stacked
        // Jacobian's null space is the tangent space of the manifold the whole set defines.
        // Overwrites the constraints' squared_error() caches.
        void error_jacobian(const Block &q, std::size_t lane, float *err, float *jac)
            const noexcept
        {
            for (const auto &c : constraints_)
            {
                c->error_jacobian(q, lane, err, jac);
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

            const auto tolerance = Row::fill(settings_.tolerance);
            const auto stall = Row::fill(stall_epsilon2);
            const auto threshold = Row::fill(4.F * max_distance * max_distance);
            const auto start = q;
            Block prev;

            auto dist = squared_error(q);
            for (std::size_t i = 0;
                 i < settings_.max_iterations and not dist.test_all_less_equal(tolerance);
                 ++i)
            {
                prev = q;
                step_in_place(q);
                dist = squared_error(q);

                const auto step_dist = squared_distance(q, prev);
                if (step_dist.test_all_less_equal(stall))
                {
                    break;
                }

                if (step_dist.test_any_greater_equal(threshold) or
                    squared_distance(q, start).test_any_greater_equal(threshold))
                {
                    break;
                }
            }

            return dist.test_all_less_equal(tolerance);
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

            const auto tolerance = Row::fill(settings_.tolerance);
            const auto stall = Row::fill(stall_epsilon2);
            const auto threshold = Row::fill(4.F * max_distance * max_distance + 1e-6F);
            const auto start = q;
            Block prev;

            auto dist = squared_error(q);
            for (std::size_t i = 0;
                 i < settings_.max_iterations and not dist.test_any_less_equal(tolerance);
                 ++i)
            {
                prev = q;
                step_in_place(q);
                dist = squared_error(q);

                const auto step_dist = squared_distance(q, prev);
                if (step_dist.test_all_less_equal(stall))  // all lanes stalled
                {
                    break;
                }

                if (step_dist.test_all_greater_equal(threshold) or
                    squared_distance(q, start).test_all_greater_equal(threshold))
                {
                    break;
                }
            }

            if (dist.test_any_less_equal(tolerance))
            {
                for (auto lane = 0U; lane < rake; ++lane)
                {
                    if (dist[{0, lane}] <= settings_.tolerance)
                    {
                        return static_cast<int>(lane);
                    }
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

        std::vector<Ptr> constraints_;
        ConstraintSettings settings_;
    };
}  // namespace vamp::planning::constraint
