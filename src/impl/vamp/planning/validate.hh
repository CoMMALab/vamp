#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>
#include <vamp/profiler_utils.hh>
#include <chrono>
#include <iostream>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        auto coll_check_time_start = std::chrono::steady_clock::now();
        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        auto coll_check_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - coll_check_time_start).count();
        vamp::profiling::get_profiler()["collision_check_vector"].push_back(coll_check_time);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }
            auto coll_check_inside = std::chrono::steady_clock::now();
            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            auto coll_check_inside_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - coll_check_inside).count();
            vamp::profiling::get_profiler()["collision_check_inside_vector"].push_back(coll_check_inside_time);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
    }

    // template <std::size_t rake, std::size_t dim>
    // inline static auto assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
    // {
    //     for (size_t i = 0; i < dim; i++)
    //     {
    //         dest[i] = src[i];
    //     }
    // }

    // topple addons
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        const auto percents = FloatVector<rake>(Percents<rake>::percents);
        typename Robot::template ConfigurationBlock<rake> block;
        int robot_dim_q = Robot::dimension / 3;
        
        auto bezier_copy_time_start = std::chrono::steady_clock::now();
        vamp::FloatVector<rake, (Robot::topple_out_dim + 2) * (size_t)(Robot::dimension / 3)> bez_anchors_vec;
        for(size_t i = 0; i < (Robot::topple_out_dim + 2) * robot_dim_q; ++i)
        {
            bez_anchors_vec[i] = bez.anchors(i / robot_dim_q, i % robot_dim_q);
        }

        auto bezier_copy_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - bezier_copy_time_start).count();
        vamp::profiling::get_profiler()["bezier_copy_to_simd"].push_back(bezier_copy_time);

        auto bez_validate_outer_start = std::chrono::steady_clock::now();
        Robot::bezier(bez_anchors_vec, percents, block);
        // const std::size_t n = std::max(resolution * T / rake, 1.0F);
        
        auto bezier_call_time = std::chrono::steady_clock::now();
        // std::cout << n << std::endl;
        auto bez_validate_outer_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            bezier_call_time - bez_validate_outer_start).count();
        vamp::profiling::get_profiler()["bezier_call_function"].push_back(bez_validate_outer_time);


        // row_matrix states(rake, robot_dim_q);  
        // for (auto i = 0U; i < rake; i++)  
        // {  
        //     const auto &state_i = bez.evaluate(static_cast<float>(percents_arr[i]));  
        //     states.row(i) = state_i;
        // }
        // row_matrix block_matrix = states.transpose();
        // for (auto j = 0U; j < robot_dim_q; j++)
        // {            
        //     block[j] = FloatVector<rake>(block_matrix.row(j).data());
        // }

        float dist = 0;
        for (auto i = 0U; i < rake - 1; i++)
        {
            float sq_dist = 0;
            for(auto j = 0U; j < robot_dim_q; j++)
            {
                sq_dist = sq_dist + block[{j, i}] * block[{j, i}];
            }
            dist = dist + std::sqrt(sq_dist);
        }

        auto bez_distance_call_time = std::chrono::steady_clock::now();
        auto bez_distance_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            bez_distance_call_time - bezier_call_time).count();
        vamp::profiling::get_profiler()["bezier_distance_calculation"].push_back(bez_distance_time);

        const std::size_t n = std::max(resolution * dist / rake, 1.0F);

        auto coll_check_time_start = std::chrono::steady_clock::now();
        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        auto coll_check_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - coll_check_time_start).count();
        vamp::profiling::get_profiler()["collision_check_bezier"].push_back(coll_check_time);
                                                
        // std::cout << valid << std::endl;
        if (not valid or n == 1)
        {
            return valid;
        }

        auto bez_validate_inner_loop_start = std::chrono::steady_clock::now();

        // slide the rake back along bez (i.e. compute new timesteps to rake)
        const auto backstep = percents.broadcast(0) / n;
        for (auto i = 1U; i < n; i++)
        {
            auto bez_vec_call_start = std::chrono::steady_clock::now();
            Robot::bezier(bez_anchors_vec, percents - i * backstep, block);
            auto bez_vec_call_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - bez_vec_call_start).count();
            vamp::profiling::get_profiler()["bezier_call_function_inside"].push_back(bez_vec_call_time);

            auto coll_check_inside = std::chrono::steady_clock::now();
            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            auto coll_check_inside_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - coll_check_inside).count();
            vamp::profiling::get_profiler()["collision_check_inside_bezier"].push_back(coll_check_inside_time);
            if (not valid)
            {
                auto bez_validate_inner_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now() - bez_validate_inner_loop_start).count();
                vamp::profiling::get_profiler()["internal_rake_back_val_time"].push_back(bez_validate_inner_time);
                return false;
            }
        }
        auto bez_validate_inner_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - bez_validate_inner_loop_start).count();
        vamp::profiling::get_profiler()["internal_rake_back_val_time"].push_back(bez_validate_inner_time);
        return true;
    }

    // rake this at some point
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_dbez(
        Bezier bez,
        float T
    ) 
    {
        // obtain the position only as floatvector, do the same collision check as normal vamp
        const auto percents = FloatVector<rake>(Percents<rake>::percents);
        const auto robot_dim_q = Robot::dimension / 3;

        // typename Robot::template ConfigurationBlock<rake> block;
        std::array<float, robot_dim_q> vlim_arr;
        for (auto i = 0U; i < robot_dim_q; i++) {
            vlim_arr[i] = Robot::s_a[robot_dim_q + i] + Robot::s_m[robot_dim_q + i];
        }
        // FloatVector<Robot::dimension / 3> vlim(vlim_arr);
        auto dbez = bez.derivative();

        auto percents_arr = percents.to_array();
        for (auto i = 0U; i < percents_arr.size(); i++) {
            auto vel = dbez.evaluate(static_cast<float>(percents_arr[i])) / T;
            for (auto j = 0U; j < robot_dim_q; j++) {
                if (std::abs(vel(j)) > vlim_arr[j]) {
                    return false;
                }
            }
        }

        const std::size_t n = resolution * T * 2 / rake;
        const auto backstep = percents.broadcast(0) / n;
        for (auto i = 1U; i < n; ++i)
        {
            auto times = (percents - i * backstep).to_array();
            for (auto j = 0U; j < times.size(); j++) {
                auto vel = dbez.evaluate(static_cast<float>(times[j])) / T;
                for (auto k = 0U; k < robot_dim_q; k++) {
                    if (std::abs(vel(k)) > vlim_arr[k]) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_ddbez(
        Bezier bez,
        float T
    ) 
    {
        // obtain the position only as floatvector, do the same collision check as normal vamp
        const auto percents = FloatVector<rake>(Percents<rake>::percents);
        const auto robot_dim_q = Robot::dimension / 3;
        // typename Robot::template ConfigurationBlock<rake> block;

        std::array<float, robot_dim_q> alim_arr;
        for (auto i = 0U; i < robot_dim_q; i++) {
            alim_arr[i] = Robot::s_a[2 * robot_dim_q + i] + Robot::s_m[2 * robot_dim_q + i];
        }
        // FloatVector<Robot::dimension / 3> vlim(vlim_arr);
        auto ddbez = bez.derivative().derivative();

        auto percents_arr = percents.to_array();
        for (auto i = 0U; i < percents_arr.size(); i++) {
            auto acc = ddbez.evaluate(static_cast<float>(percents_arr[i])) / (T * T);
            for (auto j = 0U; j < robot_dim_q; j++) {
                if (std::abs(acc(j)) > alim_arr[j]) {
                    return false;
                }
            }
        }
        
        const std::size_t n = resolution * T * 2 / rake;
        const auto backstep = percents.broadcast(0) / n;
        for (auto i = 1U; i < n; ++i)
        {
            auto times = (percents - i * backstep).to_array();
            for (auto j = 0U; j < times.size(); j++) {
                auto acc = ddbez.evaluate(static_cast<float>(times[j])) / (T * T);
                for (auto k = 0U; k < robot_dim_q; k++) {
                    if (std::abs(acc(k)) > alim_arr[k]) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // std::cout << "inside validate bez motion" << std::endl;
        // build input to MLP
        std::array<float, Robot::dimension * 2> x;
        auto start_arr = start.to_array();
        int robot_dim_q = Robot::dimension / 3;

        for (auto i = 0U; i < Robot::dimension; i++) {
            x[i] = static_cast<float>(start_arr[i]);
        }
        auto goal_arr = goal.to_array();
        for (auto i = 0U; i < Robot::dimension; i++) {
            x[Robot::dimension + i] = static_cast<float>(goal_arr[i]);
        }

        // array to store inference output
        // Profile NN inference
        auto nn_start = std::chrono::steady_clock::now();
        auto out = Robot::template topple_nn_forward(x);
        auto nn_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - nn_start).count();
        vamp::profiling::get_profiler()["nn_inference"].push_back(nn_time);

        // build the anchors
        row_matrix anchors(Robot::topple_out_dim + 2, robot_dim_q);

        // initial point
        for (auto i = 0U; i < robot_dim_q; i++) {
            anchors(0, i) = static_cast<float>(start_arr[i]);
        }

        // intermediate points
        for (auto i = 1U; i <= Robot::topple_out_dim; i++) {
            for (auto j = 0U; j < robot_dim_q; j++) {
                anchors(i, j) = out[(i - 1) * robot_dim_q + j];
            }
        }

        // time in seconds
        float T = out[Robot::topple_out_dim * Robot::dimension / 3];

        // final point
        for (auto i = 0U; i < robot_dim_q; i++) {
            anchors(Robot::topple_out_dim + 1, i) = static_cast<float>(goal_arr[i]);
        }

        Bezier bez(anchors);
        bez.time = T;

        // collision check only on sub_bez
        bool bez_valid = validate_bez<Robot, rake, resolution>(bez, environment);

        // return both sub_bez and bez_valid
        return bez_valid;
    }

    template <typename Robot, std::size_t rake>
    inline constexpr auto compute_bez(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal
    ) -> Bezier
    {

        // std::cout << "???" << std::endl;
        std::array<float, Robot::dimension * 2> x;
        // std::cout << "CONVERT START TO ARRAY" << std::endl;
        auto start_arr = start.to_array();
        int robot_dim_q = Robot::dimension / 3;

        // std::cout << "CREATE INPUT TO NN" << std::endl;
        for (auto i = 0U; i < Robot::dimension; i++) {
            x[i] = static_cast<float>(start_arr[i]);
        }
        auto goal_arr = goal.to_array();
        for (auto i = 0U; i < Robot::dimension; i++) {
            x[Robot::dimension + i] = static_cast<float>(goal_arr[i]);
        }

        // array to store inference output
        // Profile NN inference
        auto nn_start = std::chrono::steady_clock::now();
        auto out = Robot::template topple_nn_forward(x);
        auto nn_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - nn_start).count();
        vamp::profiling::get_profiler()["nn_inference"].push_back(nn_time);

        // build the anchors
        row_matrix anchors(Robot::topple_out_dim + 2, robot_dim_q);

        // initial point
        for (auto i = 0U; i < robot_dim_q; i++) {
            anchors(0, i) = static_cast<float>(start_arr[i]);
        }

        // intermediate points
        for (auto i = 1U; i <= Robot::topple_out_dim; i++) {
            for (auto j = 0U; j < robot_dim_q; j++) {
                anchors(i, j) = out[(i - 1) * robot_dim_q + j];
            }
        }

        // time in seconds
        float T = out[Robot::topple_out_dim * Robot::dimension / 3];

        // final point
        for (auto i = 0U; i < robot_dim_q; i++) {
            anchors(Robot::topple_out_dim + 1, i) = static_cast<float>(goal_arr[i]);
        }

        Bezier bez(anchors);
        bez.time = T;
        return bez;
    }
    template <typename Robot, std::size_t rake>
    inline constexpr auto compute_sub_bez(
        Bezier &bez,
        const float range
    ) -> Bezier
    {
        Bezier sub_bez;
        if (range < 1) {
            sub_bez = bez.subdivide(range).first;
        }
        else {
            sub_bez = bez;
        }
        return sub_bez;
    }


    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_sub_bez_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        const float bez_range) -> std::pair<bool, Bezier>
    {
        auto bez_compute_start = std::chrono::steady_clock::now();
        Bezier bez = compute_bez<Robot, rake>(start, goal);
        auto bez_compute_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - bez_compute_start).count();
        vamp::profiling::get_profiler()["bezier_computation"].push_back(bez_compute_time);
        Bezier sub_bez = compute_sub_bez<Robot, rake>(bez, bez_range);

        auto bez_validate_start = std::chrono::steady_clock::now();
        bool bez_valid = validate_bez<Robot, rake, resolution>(sub_bez, environment);
        auto bez_validate_time = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - bez_validate_start).count();
        vamp::profiling::get_profiler()["bez_validation"].push_back(bez_validate_time);

        return {bez_valid, sub_bez};
    }
}  // namespace vamp::planning
