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
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        const auto percents = FloatVector<rake>(Percents<rake>::percents);
        const auto percents_arr = percents.to_array();
        int robot_dim_q = Robot::dimension / 3;

        // vamp::FloatVector<rake, 7 * (Robot::topple_out_dim + 2) * (size_t)(Robot::dimension / 3)> bez_anchors_vec;
        // for(size_t i = 0; i < (Robot::topple_out_dim + 2) * robot_dim_q; ++i)
        // {
        //     bez_anchors_vec[i] = bez.anchors(i / robot_dim_q, i % robot_dim_q);
        // }

        typename Robot::template ConfigurationBlock<rake> block;
        // Robot::bezier(bez_anchors_vec, percents, block);
        std::vector<std::vector<float>> states;
        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            std::vector<float> dof_i;
            states.push_back(dof_i);
        }

        for (auto i = 0U; i < rake; i++) {
            state state_vec = bez.evaluate(percents_arr[i]);
            for (auto j = 0U; j < Robot::dimension; j++) {
                states[j].push_back(state_vec(j));
            }
        }

        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            std::array<float, rake> dim_rake;
            for (auto j = 0U; j < rake; j++) {
                dim_rake[j] = states[i][j];
            }
            block[i] = FloatVector<rake>(dim_rake);
        }

        float dist = 0;
        for (auto i = 0U; i < rake - 1; i++)
        {
            float sq_dist = 0;
            for(auto j = 0U; j < robot_dim_q; j++)
            {
                sq_dist = sq_dist + (block[{j, i}] - block[{j, i + 1}]) * (block[{j, i}] - block[{j, i + 1}]);
            }
            dist = dist + std::sqrt(sq_dist);
        }

        const std::size_t n = std::max(resolution * dist / rake, 1.0F);
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
            auto times = (percents - i * backstep).to_array();
            // Robot::bezier(bez_anchors_vec, percents - i * backstep, block);
            for (auto j = 0U; j < rake; j++) {
                state state_vec = bez.evaluate(times[j]);
                for (auto k = 0U; k < Robot::dimension; k++) {
                    states[k][j] = state_vec(k);
                }
            }

            for (auto j = 0U; j < Robot::dimension / 3; j++) {
                std::array<float, rake> dim_rake;
                for (auto k = 0U; k < rake; k++) {
                    dim_rake[k] = states[j][k];
                }
                block[j] = FloatVector<rake>(dim_rake);
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

        const std::size_t n = resolution * T / rake;
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
        
        const std::size_t n = resolution * T / rake;
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

    template <typename Robot, std::size_t rake>
    inline constexpr auto compute_bez(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> weights,
        std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> bias
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
        auto ts = std::chrono::high_resolution_clock::now();
        auto out = Robot::template topple_nn_forward(weights, bias, x);
        auto tf = std::chrono::high_resolution_clock::now();
        // std::cout << "NN inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(tf - ts).count() << " ms" << std::endl;

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

        state v0_target(Robot::dimension / 3);
        state a0_target(Robot::dimension / 3);
        state v1_target(Robot::dimension / 3);
        state a1_target(Robot::dimension / 3);
        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            v0_target(i) = start_arr[Robot::dimension / 3 + i];
            a0_target(i) = start_arr[2 * Robot::dimension / 3 + i];

            v1_target(i) = goal_arr[Robot::dimension / 3 + i];
            a1_target(i) = goal_arr[2 * Robot::dimension / 3 + i];
        }
        // bez.smoothen(v0_target, a0_target, v1_target, a1_target, true);
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
    inline constexpr auto validate_bez_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment,
        const float bez_range,
        std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> weights,
        std::vector<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> bias
    ) -> std::pair<bool, Bezier>
    {
        Bezier bez = compute_bez<Robot, rake>(start, goal, weights, bias);
        Bezier sub_bez = compute_sub_bez<Robot, rake>(bez, bez_range);

        bool bez_valid = validate_bez<Robot, rake, resolution>(sub_bez, environment);
        bool dbez_valid = validate_dbez<Robot, rake, resolution>(sub_bez, sub_bez.time);
        bool ddbez_valid = validate_ddbez<Robot, rake, resolution>(sub_bez, sub_bez.time);
        if (not bez_valid or not dbez_valid or not ddbez_valid) {
            return {false, sub_bez};
        }
        return {true, sub_bez};
    }
}  // namespace vamp::planning
