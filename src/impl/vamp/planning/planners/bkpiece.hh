#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <vamp/collision/environment.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/planners/bkpiece_settings.hh>
#include <vamp/planning/planners/kpiece_grid.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    namespace detail
    {
        template <typename R, typename = void>
        struct has_ambient : std::false_type
        {
        };

        template <typename R>
        struct has_ambient<R, std::void_t<typename R::Ambient>> : std::true_type
        {
        };

        template <typename R, typename = void>
        struct has_eefk : std::false_type
        {
        };

        template <typename R>
        struct has_eefk<
            R,
            std::void_t<decltype(R::eefk(std::declval<const std::array<float, R::dimension> &>()))>>
          : std::true_type
        {
        };

        // Coverage-grid projection described by BKPIECESettings: raw state components, or the
        // position/velocity of an object frame on the end effector (flask robots project through
        // their ambient robot's forward kinematics; velocity by a forward difference along qdot).
        template <typename Robot>
        struct KPIECEProjection
        {
            static constexpr std::size_t max_dim = 6;
            using Configuration = typename Robot::Configuration;
            using Coord = std::array<int, max_dim>;

            explicit KPIECEProjection(const BKPIECESettings &s) noexcept
              : components_(s.components), cell_size_(s.cell_size), object_(s.object_projection)
            {
                const std::size_t n = std::min({components_.size(), cell_size_.size(), max_dim});
                components_.resize(n);
                cell_size_.resize(n);
                const auto &o = s.object_in_eef;
                offset_.linear() = Eigen::Quaternionf(o[0], o[1], o[2], o[3]).normalized().toRotationMatrix();
                offset_.translation() = Eigen::Vector3f(o[4], o[5], o[6]);
            }

            auto dim() const noexcept -> std::size_t
            {
                return components_.size();
            }

            auto operator()(const Configuration &c) const noexcept -> Coord
            {
                const auto arr = c.to_array();
                std::array<float, max_dim> v{};
                if (object_)
                {
                    object_values(arr, v);
                }
                else
                {
                    for (auto i = 0U; i < dim(); ++i)
                    {
                        const auto k = static_cast<std::size_t>(std::max(components_[i], 0));
                        v[i] = (k < arr.size()) ? arr[k] : 0.F;
                    }
                }

                Coord coord{};
                for (auto i = 0U; i < dim(); ++i)
                {
                    coord[i] = static_cast<int>(std::floor(v[i] / cell_size_[i]));
                }

                return coord;
            }

        private:
            template <typename Array>
            void object_values(const Array &arr, std::array<float, max_dim> &v) const noexcept
            {
                if constexpr (has_ambient<Robot>::value)
                {
                    using Ambient = typename Robot::Ambient;
                    if constexpr (has_eefk<Ambient>::value)
                    {
                        constexpr std::size_t d = Ambient::dimension;
                        std::array<float, d> q, q2;
                        for (auto j = 0U; j < d; ++j)
                        {
                            q[j] = arr[j];
                            q2[j] = arr[j] + h * arr[d + j];
                        }

                        fill(Ambient::eefk(q), Ambient::eefk(q2), v);
                    }
                }
                else if constexpr (has_eefk<Robot>::value)
                {
                    std::array<float, Robot::dimension> q;
                    std::copy_n(arr.begin(), Robot::dimension, q.begin());
                    const auto pose = Robot::eefk(q);
                    fill(pose, pose, v);
                }
            }

            template <typename Pose>
            void fill(const Pose &pose, const Pose &ahead, std::array<float, max_dim> &v) const noexcept
            {
                const Eigen::Vector3f p = (pose * offset_).translation();
                const Eigen::Vector3f pv = ((ahead * offset_).translation() - p) / h;
                for (auto i = 0U; i < dim(); ++i)
                {
                    const int k = components_[i];
                    v[i] = (k >= 0 and k < 3) ? p[k] : (k >= 3 and k < 6) ? pv[k - 3] : 0.F;
                }
            }

            static constexpr float h = 1e-3F;
            std::vector<int> components_;
            std::vector<float> cell_size_;
            bool object_;
            Eigen::Isometry3f offset_ = Eigen::Isometry3f::Identity();
        };
    }  // namespace detail

    // Bidirectional KPIECE (Sucan & Kavraki, "A sampling-based tree planner for systems with
    // complex dynamics", T-RO 2012; OMPL's BKPIECE1) over the local-planner interface. Each tree
    // keeps a coverage grid over a low-dimensional projection; an expansion picks the most
    // important cell (border cells with probability border_fraction; importance favours new,
    // sparsely covered, rarely selected cells with few neighbours), a state in it (biased toward
    // its newest), and steers from that state toward a draw of the sampler by at most `range` --
    // node selection by coverage instead of nearest-neighbour Voronoi bias, so an informed sampler
    // supplies directions without being pulled back toward dense regions. A failed expansion
    // halves the cell's score. After each expansion the trees try to connect through the other
    // tree's cell at the new state. Edges are directed as in RRT-Connect: goal-tree edges run
    // child to parent.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct BKPIECE
    {
        using Configuration = typename Robot::Configuration;
        using RNG = vamp::rng::RNG<Robot>;
        using Projection = detail::KPIECEProjection<Robot>;
        using Grid = detail::Grid<Projection::max_dim>;
        using Coord = typename Grid::Coord;
        using Cell = typename Grid::Cell;

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const BKPIECESettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng, lp);
        }

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const BKPIECESettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;
            lp.reset_stats();
            const auto start_time = std::chrono::steady_clock::now();
            constexpr auto unbounded = [] { return std::numeric_limits<float>::infinity(); };
            constexpr std::size_t root = std::numeric_limits<std::size_t>::max();

            for (const auto &goal : goals)
            {
                const auto direct = lp.connect_within(
                    start, goal, environment, unbounded, std::numeric_limits<std::size_t>::max());
                if (direct.status == SteerStatus::Reached)
                {
                    result.path.emplace_back(start);
                    for (const auto &c : direct.waypoints)
                    {
                        result.path.emplace_back(c);
                    }

                    result.path.emplace_back(goal);
                    result.solved = true;
                    result.cost = result.path.cost();
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.size = {1, 1};
                    result.stats = lp.stats();
                    return result;
                }
            }

            const Projection project(settings);
            struct Tree
            {
                std::vector<Configuration> nodes;
                std::vector<std::size_t> parents;
                Grid grid;
                unsigned iteration;
                bool forward;
            };

            std::array<Tree, 2> trees{
                Tree{{}, {}, Grid(project.dim()), 1, true}, Tree{{}, {}, Grid(project.dim()), 1, false}};
            const auto total = [&]() { return trees[0].nodes.size() + trees[1].nodes.size(); };

            const auto add = [&](Tree &t, const Configuration &c, std::size_t parent) -> std::size_t
            {
                const std::size_t index = t.nodes.size();
                t.nodes.emplace_back(c);
                t.parents.emplace_back((parent == root) ? index : parent);
                const Coord coord = project(c);
                Cell *cell = t.grid.getCell(coord);
                if (cell)
                {
                    cell->coverage += 1.0;
                }
                else
                {
                    cell = t.grid.createCell(coord, t.iteration, 1.0 + std::log(static_cast<double>(t.iteration)));
                }

                cell->motion_indices.emplace_back(index);
                t.grid.updateImportance(cell, coord);
                return index;
            };

            add(trees[0], start, root);
            for (const auto &goal : goals)
            {
                add(trees[1], goal, root);
            }

            // OMPL's halfNormalInt(0, n - 1, focus 3): biased toward the newest state of a cell.
            std::normal_distribution<float> normal;
            const auto half_normal = [&](std::size_t n) -> std::size_t
            {
                const float v = static_cast<float>(n) * (1.F - std::abs(normal(rng->dist.rng)) / 3.F);
                return (v <= 0.F) ? 0 : std::min(static_cast<std::size_t>(v), n - 1);
            };

            std::size_t samples_drawn = 0;
            std::size_t iter = 0;
            std::size_t active = (settings.start_tree_first) ? 0 : 1;
            std::vector<Configuration> chain;
            while (iter++ < settings.max_iterations and total() < settings.max_samples)
            {
                Tree &a = trees[active];
                Tree &b = trees[1 - active];
                active = 1 - active;
                ++a.iteration;

                const auto selected = a.grid.selectCell(settings.border_fraction, rng->dist);
                if (not selected)
                {
                    break;
                }

                Cell *cell = a.grid.getCell(*selected);
                if (cell->score < std::numeric_limits<double>::epsilon())
                {
                    a.grid.rescoreAllCells(a.iteration);
                }

                ++cell->selections;
                const std::size_t from_index = cell->motion_indices[half_normal(cell->motion_indices.size())];
                const Configuration from = a.nodes[from_index];

                // Up to extension_candidates steers from the selected state; the valid one ending in
                // the least covered cell (of this tree's grid) is kept (its waypoints copied: the local
                // planner reuses its buffer).
                bool extended = false;
                double extension_coverage = std::numeric_limits<double>::infinity();
                for (std::size_t c = 0; c < std::max<std::size_t>(settings.extension_candidates, 1); ++c)
                {
                    std::optional<FloatVector<Robot::dimension>> near;
                    if (settings.local_targets)
                    {
                        near = rng->next_near(from, settings.range);
                    }

                    auto target = near ? Configuration(*near) : Configuration(rng->next());
                    ++samples_drawn;
                    if (settings.local_targets)
                    {
                        const float dist = Robot::distance(from, target);
                        if (dist > settings.range)
                        {
                            target = from + (target - from) * (settings.range / dist);
                        }
                    }

                    const auto steered =
                        lp.steer(from, target, Robot::distance(from, target), settings.range, a.forward, environment);
                    if (steered.status == SteerStatus::Trapped or steered.status == SteerStatus::Rejected or
                        steered.waypoints.empty())
                    {
                        continue;
                    }

                    const Cell *landing = a.grid.getCell(project(steered.waypoints.back()));
                    const double coverage = landing ? landing->coverage : 0.0;
                    if (coverage < extension_coverage)
                    {
                        extension_coverage = coverage;
                        chain.assign(steered.waypoints.begin(), steered.waypoints.end());
                        extended = true;
                    }
                }

                if (not extended)
                {
                    cell->score *= settings.failed_expansion_score_factor;
                    a.grid.updateImportance(cell, *selected);
                    continue;
                }

                a.grid.updateImportance(cell, *selected);
                std::size_t last = from_index;
                for (const auto &w : chain)
                {
                    if (total() >= settings.max_samples)
                    {
                        break;
                    }

                    last = add(a, w, last);
                }

                if (last == from_index)
                {
                    continue;
                }

                // Connect through the other tree's cell at the new state.
                Cell *other = b.grid.getCell(project(a.nodes[last]));
                if (not other)
                {
                    continue;
                }

                for (std::size_t k = 0; k < settings.connect_attempts; ++k)
                {
                    const std::size_t j = other->motion_indices[rng->dist.template uniform_integer<std::size_t>(
                        0, other->motion_indices.size() - 1)];
                    const Configuration here = a.nodes[last];
                    const Configuration there = b.nodes[j];
                    const auto step = lp.steer(
                        here,
                        there,
                        Robot::distance(here, there),
                        std::numeric_limits<float>::max(),
                        a.forward,
                        environment);
                    if (step.status != SteerStatus::Reached)
                    {
                        continue;
                    }

                    // The attained endpoint stands in for `there`, as in RRT-Connect.
                    chain.assign(step.waypoints.begin(), step.waypoints.end());
                    std::size_t tip = last;
                    for (const auto &w : chain)
                    {
                        tip = add(a, w, tip);
                    }

                    for (std::size_t cur = tip;; cur = a.parents[cur])
                    {
                        result.path.emplace_back(a.nodes[cur]);
                        if (a.parents[cur] == cur)
                        {
                            break;
                        }
                    }

                    std::reverse(result.path.begin(), result.path.end());
                    for (std::size_t cur = j; b.parents[cur] != cur;)
                    {
                        cur = b.parents[cur];
                        result.path.emplace_back(b.nodes[cur]);
                    }

                    if (not a.forward)
                    {
                        std::reverse(result.path.begin(), result.path.end());
                    }

                    result.cost = result.path.cost();
                    result.solved = true;
                    break;
                }

                if (result.solved)
                {
                    break;
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size = {trees[0].nodes.size(), trees[1].nodes.size()};
            result.stats = lp.stats();
            result.stats.samples_drawn = samples_drawn;
            return result;
        }
    };
}  // namespace vamp::planning
