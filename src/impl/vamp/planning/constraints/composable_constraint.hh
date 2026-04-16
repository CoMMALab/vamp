#pragma once

#include <memory>
#include <utility>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
#include <vamp/vector/math.hh>
#include <iomanip>

namespace vamp::planning::constraint
{

    template <typename Robot, std::size_t rake, typename... Constraints>
    class ComposableConstraints
    {
        std::tuple<Constraints...> constraints_;

    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        using Configuration = typename Robot::Configuration;
        ConfigurationBlock q_old;

    public:
        using ConstraintPack = std::tuple<Constraints...>;
        static constexpr std::size_t total_size = (Constraints::size + ...);
        static constexpr float projection_tolerance = 0.000001F;

    private:
        template <std::size_t... Is>
        static auto squaredDistanceImpl(
            const ConfigurationBlock &lhs,
            const ConfigurationBlock &rhs,
            std::index_sequence<Is...>)
        {
            auto distance = (lhs[0] - rhs[0]) * (lhs[0] - rhs[0]);
            ((distance = distance + (lhs[Is + 1] - rhs[Is + 1]) * (lhs[Is + 1] - rhs[Is + 1])), ...);
            return distance;
        }

        static auto squaredDistance(const ConfigurationBlock &lhs, const ConfigurationBlock &rhs)
        {
            if constexpr (Robot::dimension == 1)
            {
                return (lhs[0] - rhs[0]) * (lhs[0] - rhs[0]);
            }
            else
            {
                return squaredDistanceImpl(lhs, rhs, std::make_index_sequence<Robot::dimension - 1>{});
            }
        }

    public:
        explicit ComposableConstraints(Constraints... cs) : constraints_(std::move(cs)...)
        {
        }

        template <typename Dist>
        static bool isConverged(const Dist &dist)
        {
            return dist.test_all_less_equal(projection_tolerance);
        }

        template <typename Dist>
        static bool hasAnyConverged(const Dist &dist)
        {
            return dist.test_any_less_equal(projection_tolerance);
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            return std::apply(
                [&](const auto &...c) { return (c.distanceToConstraint(q) + ...); }, constraints_);
        }

        void print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            std::apply([&](const auto &...c) { (c.print_robot_tsr_error(q), ...); }, constraints_);
        }

        void projectStepInPlace(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0f)
        {
            q_new = q;

            if constexpr (sizeof...(Constraints) == 0)
            {
                return;
            }

            std::apply(
                [&](auto &...c)
                {
                    auto applyConstraint = [&](auto &constraint)
                    { constraint.projectStepInPlace(q_new, projection_method, alpha); };
                    (applyConstraint(c), ...);
                },
                constraints_);
        }

        auto projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0f)
        {
            ConfigurationBlock q_new;
            projectStepInPlace(q, q_new, projection_method, alpha);
            return q_new;
        }

        bool projectConfiguration(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float max_q_dist = 5.0F,
            float descend_rate = 1.0F,
            int num_projection_iterations = 25,
            bool verbose = false)
        {
            /**
             * project a configuration block in parallel onto the constraint manifold
             * @param q - original config
             * @param q_new - projected config
             * @param projection_method - something from ProjMethod
             * @param max_q_dist - break out early if projected config is farther than max_q_dist away from
             * start
             *
             * @return success of projection
             */

            bool success = false;
            auto dist = distanceToConstraint(q);

            int project_iter = 0;
            q_new = q;
            q_old = q;

            auto deviation_threshold = 4 * max_q_dist * max_q_dist;
            // for(size_t rdim = 0U; rdim < Robot::dimension; rdim++) {
            //     q_old[rdim] = q[rdim];
            //     q_new[rdim] = q[rdim];
            // }

            while ((project_iter < num_projection_iterations) and (not isConverged(dist)))
            {
                projectStepInPlace(q_old, q_new, projection_method, descend_rate);
                dist = distanceToConstraint(q_new);
                // std::cout << "Iteration " << project_iter << " Distance: " << dist << std::endl;
                // std::cout << q_old << q_new << std::endl;
                auto q_dist_from_prev = squaredDistance(q_new, q_old);
                auto q_dist_from_start = squaredDistance(q_new, q);
                // auto q_dist_from_start = (q_new[0] - q[0]) * (q_new[0] - q[0]);

                // std::cout << q_dist_from_prev << " " << dist << std::endl;
                if (isConverged(q_dist_from_prev))  // if i make no forward progress
                {
                    // std::cout << "Minimal progress " << dist << q_dist_from_prev << std::endl << q <<
                    // std::endl;
                    break;
                }

                if (q_dist_from_prev.test_any_greater(deviation_threshold) or
                    q_dist_from_start.test_any_greater(deviation_threshold))  // from triangle
                                                                              // inequality
                {
                    // std::cout << "Too large step " << q_dist_from_prev << std::endl;
                    // std::cout << q_old << std::endl;
                    break;
                }

                // for(size_t rdim = 0U; rdim < Robot::dimension; rdim++) {
                //     q_old[rdim] = q_new[rdim];
                // }
                q_old = q_new;
                project_iter += 1;
            }
            if (isConverged(dist))
            {
                success = true;
            }
            if (verbose)
            {
                std::cout << "Num projection steps : " << project_iter << " " << dist
                          << " and success : " << success << " " << std::endl;
                std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist "
                          << dist << " q " << q << " q_new " << q_new << std::endl;
            }

            return success;
        }

        int projectAnyConfiguration(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float max_q_dist = 5.0F,
            float descend_rate = 1.0F,
            int num_projection_iterations = 25,
            bool verbose = false)
        {
            /**
             * project a configuration block in parallel onto the constraint manifold
             * @param q - original config
             * @param q_new - projected config
             * @param projection_method - something from ProjMethod
             * @param max_q_dist - break out early if projected config is farther than max_q_dist away from
             * start
             *
             * @return the rake position of the successfully projected configuration, -1 if all failed
             *         priority is implicitly encoded in the rake position (lower is higher priority)
             */

            int success_position = -1;
            auto dist = distanceToConstraint(q);

            int project_iter = 0;
            q_new = q;
            q_old = q;
            auto deviation_threshold =
                4 * max_q_dist * max_q_dist + 1e-6F;  // add a small epsilon to avoid numerical issues
            // for(size_t rdim = 0U; rdim < Robot::dimension; rdim++) {
            //     q_old[rdim] = q[rdim];
            //     q_new[rdim] = q[rdim];
            // }

            // std::cout << q << std::endl;

            while ((project_iter < num_projection_iterations) and (not hasAnyConverged(dist)))
            {
                projectStepInPlace(q_old, q_new, projection_method, descend_rate);
                dist = distanceToConstraint(q_new);
                // std::cout << "Iteration " << project_iter << " Distance: " << dist << std::endl;
                // std::cout << q_old << q_new << std::endl;
                auto q_dist_from_prev = squaredDistance(q_new, q_old);
                auto q_dist_from_start = squaredDistance(q_new, q);
                // auto q_dist_from_start = (q_new[0] - q[0]) * (q_new[0] - q[0]);

                // std::cout << q_dist_from_prev << " " << dist << std::endl;
                if (isConverged(q_dist_from_prev))  // if i make no forward progress in any
                                                    // of them
                {
                    // std::cout << "Minimal progress " << dist << q_dist_from_prev << std::endl << q <<
                    // std::endl;
                    break;
                }

                if (q_dist_from_prev.test_all_greater_equal(deviation_threshold) or
                    q_dist_from_start.test_all_greater_equal(deviation_threshold))  // from triangle
                                                                                    // inequality
                {
                    // std::cout << "Too large step " << q_dist_from_prev << std::endl;
                    // std::cout << q_old << std::endl;
                    break;
                }
                // for(size_t rdim = 0U; rdim < Robot::dimension; rdim++) {
                //     q_old[rdim] = q_new[rdim];
                // }
                q_old = q_new;
                project_iter += 1;
            }
            if (hasAnyConverged(dist))
            {
                for (size_t i = 0; i < rake; i++)
                {
                    if (dist[{0, i}] <= projection_tolerance)
                    {
                        success_position = i;
                        break;
                    }
                }
            }
            if (verbose)
            {
                std::cout << "Num projection steps : " << project_iter << " " << dist
                          << " and success : " << success_position << " " << std::endl;
                std::cout << "Num steps : " << project_iter << " and success : " << success_position << " "
                          << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;
            }

            return success_position;
        }
    };
}  // namespace vamp::planning::constraint