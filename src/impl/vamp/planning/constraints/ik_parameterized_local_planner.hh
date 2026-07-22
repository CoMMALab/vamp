#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/validate.hh>
#include <vamp/vector.hh>
#include <vamp/profiler_utils.hh>

namespace vamp::planning
{
    // SteerStatus, Extension, and insert_chain are defined in vamp/planning/local_planner.hh
    // (included above) and reused here as-is; this file only adds the IK-parameterized local
    // planner that resolves each interpolated block through Robot::parameterized_ik before
    // collision checking.

    // Local planners generate and validate the local paths that connect configurations. All
    // operations are directed: a local path from a to b is traversed in execution order
    // (start to goal), which matters for robots with asymmetric local paths (e.g. flask) and
    // planners whose goal-tree edges run child to parent.
    //
    // IKParameterizedLocalPlanner steers by Robot::interpolate exactly like the
    // unconstrained planner, but validates each interpolated block by first resolving it
    // through Robot::parameterized_ik: the interpolated joint values plus the robot's
    // ik_parameters are packed into a config_block, which parameterized_ik turns into the
    // ambient block that is actually collision-checked. This mirrors the
    // Robot::use_parameterized_ik branch of validate_vector in ik_params_old.hh. Local
    // paths have no interior waypoints, so planners using it perform the same arithmetic as
    // calling validate directly.
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct IKParameterizedLocalPlanner
    {
        static_assert(
            Robot::use_parameterized_ik,
            "IKParameterizedLocalPlanner requires a robot with parameterized IK "
            "(Robot::use_parameterized_ik)");

        using Configuration = typename Robot::Configuration;
        using Environment = collision::Environment<FloatVector<rake>>;
        // using ConfigBlock = FloatVector<rake, Robot::dimension>;
        using ConfigurationBlock = typename Robot::template ConfigurationBlock<rake>;

        IKParameterizedLocalPlanner() = default;

        // True if configurations must be projected to remain valid (e.g. onto a constraint
        // manifold): planners must lp.project() any configuration they synthesize
        // (interpolated midpoints, perturbations) before use.
        static constexpr bool projecting = false;

        // Multiplier on the number of steps a connect loop may take: projection drift can
        // require more steps than ceil(distance / range).
        static constexpr float connect_slack = 1.F;

        // Validity of the local path a -> b in execution order; `forward` is false when the
        // caller's a precedes b in tree order but follows it in execution order (goal trees).
        inline auto validate(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            bool forward = true) const noexcept -> bool
        {
            const Configuration &start = (forward) ? a : b;
            const Configuration &goal = (forward) ? b : a;
            return validate_parameterized_ik(start, goal, e);
        }

        // Admit the local path a -> b only if it is valid, has fewer interior waypoints than
        // `max_states`, and costs no more than `budget()`. The budget is lazy so that this
        // planner, whose local paths have no interior waypoints and never re-cost, does not
        // evaluate it at all. Reached on success (with the interior waypoints, here none),
        // Trapped otherwise.
        template <typename Budget>
        inline auto connect_within(
            const Configuration &a,
            const Configuration &b,
            const Environment &e,
            Budget &&,
            std::size_t max_states) const noexcept -> Extension<Robot>
        {
            chain_.clear();
            const bool valid = 0 < max_states and validate(a, b, e);
            return {(valid) ? SteerStatus::Reached : SteerStatus::Trapped, chain_};
        }

        // Steer from `from` toward `target` by at most `range`. `forward` is true iff `from`
        // precedes `target` in execution order. `distance` is Robot::distance(from, target),
        // precomputed by callers (usually from a nearest-neighbor query). `accept` is invoked
        // on the candidate endpoint after interpolation (and projection) but before
        // validation; returning false aborts with Rejected, letting optimizing planners
        // prune an extension on cost before paying for the validity check.
        template <typename Accept = AlwaysTrue>
        inline auto steer(
            const Configuration &from,
            const Configuration &target,
            float distance,
            float range,
            bool forward,
            const Environment &e,
            Accept &&accept = Accept()) const noexcept -> Extension<Robot>
        {
            chain_.clear();

            const bool reach = distance < range;
            const float step = range / distance;

            // auto interpolate_start_time = std::chrono::steady_clock::now();
            const auto next = (reach)    ? target :
                              (forward)  ? Robot::interpolate(from, target, step) :
                                           Robot::interpolate(target, from, 1.F - step);
            // auto interpolate_end_time = std::chrono::steady_clock::now();
            // auto interpolate_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(interpolate_end_time - interpolate_start_time).count();
            // vamp::profiling::get_profiler()["interpolate"].push_back(interpolate_duration);

            if (not accept(next))
            {
                // std::cout << "Steer rejected by accept function." << std::endl;
                return {SteerStatus::Rejected, chain_};
            }
            // auto validate_start_time = std::chrono::steady_clock::now();
            const bool valid = validate(from, next, e, forward);
            // auto validate_end_time = std::chrono::steady_clock::now();
            // auto validate_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(validate_end_time - validate_start_time).count();
            // vamp::profiling::get_profiler()["validate"].push_back(validate_duration);
            if (not valid)
            {
                // std::cout << "Steer failed validation." << std::endl;
                return {SteerStatus::Trapped, chain_};
            }

            chain_.emplace_back(next);
            return {(reach) ? SteerStatus::Reached : SteerStatus::Advanced, chain_};
        }

        // Project a configuration onto the constraint manifold (identity here: the IK
        // parameters are resolved per-block during validation, not on the stored
        // configuration itself). Returns false if projection fails; the configuration is
        // then unusable.
        inline auto project(Configuration &) const noexcept -> bool
        {
            return true;
        }

        // Whether a configuration satisfies the constraint: no manifold on the stored
        // configuration itself, so trivially true.
        inline auto satisfied(const Configuration &) const noexcept -> bool
        {
            return true;
        }

    private:
        mutable std::vector<Configuration> chain_;

        // Port of the Robot::use_parameterized_ik branch of validate_vector
        // (ik_params_old.hh), updated to sample in SE3 rather than assuming a Euclidean
        // configuration space: samples along start -> goal are produced by
        // Robot::interpolate_block (as validate_motion's non-euclidean branch does), not
        // by broadcasting a linear start + vector * percents. Each sample's
        // Robot::dimension rows are copied into config_block alongside the robot's
        // ik_parameters, and the whole config_block is resolved through
        // Robot::parameterized_ik before collision checking.
        inline auto validate_parameterized_ik(
            const Configuration &start,
            const Configuration &goal,
            const Environment &environment) const noexcept -> bool
        {

            const float distance = Robot::distance(start, goal);
            const std::size_t n =
                std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
            const auto percents = FloatVector<rake>(Percents<rake>::percents);
            const auto t_step = FloatVector<rake>::fill(1.F / static_cast<float>(rake * n));

            typename Robot::template ConfigurationBlock<rake> interp_block;
            typename Robot::template AmbientConfigurationBlock<rake> block;

            auto t_block = percents;
            // auto interpolate_block_start_time = std::chrono::steady_clock::now();
            Robot::template interpolate_block<rake>(start, goal, t_block, interp_block);
            // auto interpolate_end_time = std::chrono::steady_clock::now();
            // auto interpolate_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(interpolate_end_time - interpolate_block_start_time).count();
            // vamp::profiling::get_profiler()["interpolate_block"].push_back(interpolate_duration);


            // before calling IK, to speed things up, we can check if the interpolated block is already valid, by
            // inserting a dummy sphere at the end-effector position and checking for collisions. If the interpolated block is invalid, then we can skip calling IK and return false immediately.

            // auto is_eef_in_collision_start_time = std::chrono::steady_clock::now();
            bool eef_in_collision = Robot::template check_if_ik_valid_block<rake>(environment, interp_block);
            // auto is_eef_in_collision_end_time = std::chrono::steady_clock::now();
            // auto is_eef_in_collision_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(is_eef_in_collision_end_time - is_eef_in_collision_start_time).count();
            // vamp::profiling::get_profiler()["check_if_ik_valid_block"].push_back(is_eef_in_collision_duration);
            if (not eef_in_collision)
            {
                // vamp::profiling::get_profiler()["check_if_ik_valid_block_failed"].push_back(is_eef_in_collision_duration);
                return false;
            }

            

            // auto parameterized_ik_start_time = std::chrono::steady_clock::now();
            bool param_valid;
            std::tie(param_valid, block) =
                Robot::template parameterized_ik<ConfigurationBlock, rake>(interp_block);
            // auto parameterized_ik_end_time = std::chrono::steady_clock::now();
            // auto parameterized_ik_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(parameterized_ik_end_time - parameterized_ik_start_time).count();
            // vamp::profiling::get_profiler()["parameterized_ik"].push_back(parameterized_ik_duration);

            // auto check_param_valid_start_time = std::chrono::steady_clock::now();
            if (not param_valid)
            {
                // auto check_param_valid_end_time = std::chrono::steady_clock::now();
                // auto check_param_valid_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(check_param_valid_end_time - check_param_valid_start_time).count();
                // vamp::profiling::get_profiler()["check_param_valid"].push_back(check_param_valid_duration);
                return false;
            }

            // auto fkcc_start_time = std::chrono::steady_clock::now();
            bool valid = (environment.attachments) ?
                Robot::template fkcc_attach<rake>(environment, block) :
                Robot::template fkcc<rake>(environment, block);
            // auto fkcc_end_time = std::chrono::steady_clock::now();
            // auto fkcc_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(fkcc_end_time - fkcc_start_time).count();
            // vamp::profiling::get_profiler()["fkcc"].push_back(fkcc_duration);
            if (not valid or n == 1)
            {
                return valid;
            }

            for (auto i = 1U; i < n; ++i)
            {
                t_block = t_block - t_step;
                // auto inner_interpolate_start_time = std::chrono::steady_clock::now();
                Robot::template interpolate_block<rake>(start, goal, t_block, interp_block);
                // auto inner_interpolate_end_time = std::chrono::steady_clock::now();
                // auto inner_interpolate_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(inner_interpolate_end_time - inner_interpolate_start_time).count();
                // vamp::profiling::get_profiler()["inner_interpolate_block"].push_back(inner_interpolate_duration);

                // auto inner_parameterized_ik_start_time = std::chrono::steady_clock::now();
                std::tie(param_valid, block) =
                    Robot::template parameterized_ik<ConfigurationBlock, rake>(interp_block);
                // auto inner_parameterized_ik_end_time = std::chrono::steady_clock::now();
                // auto inner_parameterized_ik_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(inner_parameterized_ik_end_time - inner_parameterized_ik_start_time).count();
                // vamp::profiling::get_profiler()["inner_parameterized_ik"].push_back(inner_parameterized_ik_duration);
                if (not param_valid)
                {
                    // vamp::profiling::get_profiler()["inner_parameterized_ik_failed"].push_back(inner_parameterized_ik_duration);
                    // std::cout << "Parameterization failed at sample " << i << ": " << interp_block << t_step << " : " << distance << std::endl;
                    return false;
                }

                // auto inner_fkcc_start_time = std::chrono::steady_clock::now();
                valid = (environment.attachments) ?
                    Robot::template fkcc_attach<rake>(environment, block) :
                    Robot::template fkcc<rake>(environment, block);
                // auto inner_fkcc_end_time = std::chrono::steady_clock::now();
                // auto inner_fkcc_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(inner_fkcc_end_time - inner_fkcc_start_time).count();
                // vamp::profiling::get_profiler()["inner_fkcc"].push_back(inner_fkcc_duration);
                if (not valid)
                {
                    return false;
                }
            }

            return true;
        }
    };
}  // namespace vamp::planning
