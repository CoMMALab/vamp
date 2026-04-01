#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>
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

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
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

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
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

    // topple addons
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        const typename Robot::Configuration &start,
        float T,
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        auto percents_arr = percents.to_array();
        int robot_dim_q = Robot::dimension / 3;

        for (auto j = 0U; j < robot_dim_q; j++)  
        {  
            // Collect the i-th dimension values for all rake configurations  
            std::array<float, rake> dim_values;  
            for (auto k = 0U; k < rake; k++)  
            {  
                const auto &state = bez.evaluate(static_cast<float>(percents_arr[k]));  
                dim_values[k] = state[j];
            }  
            // Broadcast these values across SIMD lanes  
            block[j] = FloatVector<rake>(dim_values);  
        }

        const std::size_t n = resolution * T * rake;
        // std::cout << n << std::endl;

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
                                                
        // std::cout << valid << std::endl;
        if (not valid or n == 1)
        {
            return valid;
        }

        // slide the rake back along bez (i.e. compute new timesteps to rake)
        const auto backstep = percents.broadcast(0) / n;
        for (auto i = 1U; i < n; i++)
        {
            // evaluate states in rake
            auto times = (percents - i * backstep).to_array();

            for (auto j = 0U; j < robot_dim_q; j++)  
            {  
                // Collect the i-th dimension values for all rake configurations  
                std::array<float, rake> dim_values;  
                for (auto k = 0U; k < rake; k++)  
                {  
                    const auto &state = bez.evaluate(static_cast<float>(times[k]));  
                    dim_values[k] = state[j];
                }  
                // Broadcast these values across SIMD lanes  
                block[j] = FloatVector<rake>(dim_values);  
            }
            // std::cout << block << std::endl;

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }
        // auto tf = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::milli> elapsed_ms = tf - ts;
        // std::cout << "CC time: " << elapsed_ms.count() << std::endl;
        return true;
    }

    // attempt to do more informed sampling for narrow problems
    // if the line works, try to find some combination of velocity and accel
    // so the bez works too
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez_linear(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment
    ) {
        // obtain the position only as floatvector, do the same collision check as normal vamp
        auto start_arr = start.to_array();
        auto goal_arr = goal.to_array();
        std::array<float, Robot::dimension / 3> q_start;
        std::array<float, Robot::dimension / 3> q_goal;
        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            q_start[i] = start_arr[i];
            q_goal[i] = goal_arr[i];
        }
        FloatVector<Robot::dimension / 3> start_vec(q_start);
        FloatVector<Robot::dimension / 3> goal_vec(q_goal);
        auto vector = goal_vec - start_vec;

        // collision check routine
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start_vec.broadcast(i) + (vector.broadcast(i) * percents);
        }

        auto distance = vector.l2_norm();
        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
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

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

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
        std::array<float, Robot::topple_out_dim * Robot::dimension / 3 + 1> out;

        // auto ts = std::chrono::steady_clock::now();
        Robot::template topple_nn_forward(x, out);
        // auto tf = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::milli> elapsed_ms = tf - ts;
        // std::cout << "NN inference: " << elapsed_ms.count() << std::endl;

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
        bool bez_valid = validate_bez<Robot, rake, resolution>(start, T, bez, environment);

        // return both sub_bez and bez_valid
        return bez_valid;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_sub_bez_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        const float bez_range) -> std::pair<bool, Bezier>
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
        std::array<float, Robot::topple_out_dim * Robot::dimension / 3 + 1> out;

        // auto ts = std::chrono::steady_clock::now();
        Robot::template topple_nn_forward(x, out);
        // auto tf = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::milli> elapsed_ms = tf - ts;
        // std::cout << "NN inference: " << elapsed_ms.count() << std::endl;

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
        Bezier sub_bez;

        if (bez_range < 1) {
            sub_bez = bez.subdivide(bez, bez_range);
        }
        else {
            sub_bez = bez;
        }
        // collision check only on sub_bez
        bool bez_valid = validate_bez<Robot, rake, resolution>(start, sub_bez.time, sub_bez, environment);
        // return both sub_bez and bez_valid
        return {bez_valid, sub_bez};
    }
}  // namespace vamp::planning
