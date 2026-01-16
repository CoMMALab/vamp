#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>

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
    // template <typename Robot, std::size_t rake, std::size_t resolution>
    // inline constexpr auto validate_bez(
    //     const typename Robot::Configuration &start,
    //     float distance,
    //     Bezier bez,
    //     const collision::Environment<FloatVector<rake>> &environment) -> bool
    // {
    //     // std::cout << "inside validate bez" << std::endl;
    //     // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
    //     const auto percents = FloatVector<rake>(Percents<rake>::percents);

    //     typename Robot::template ConfigurationBlock<rake> block;

    //     // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
    //     // use percents as times to get configs
    //     // std::array<state, rake> states_arr;
    //     auto percents_arr = percents.to_array();

    //     for (auto j = 0U; j < Robot::dimension / 3; j++)  
    //     {  
    //         // Collect the i-th dimension values for all rake configurations  
    //         std::array<float, rake> dim_values;  
    //         for (auto k = 0U; k < rake; k++)  
    //         {  
    //             const auto &state = bez.evaluate(static_cast<float>(percents_arr[k]));  
    //             dim_values[k] = state[j];
    //         }  
    //         // Broadcast these values across SIMD lanes  
    //         block[j] = FloatVector<rake>(dim_values);  
    //     }

    //     const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

    //     bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
    //                                              Robot::template fkcc<rake>(environment, block);
                                                
    //     // std::cout << valid << std::endl;
    //     if (not valid or n == 1)
    //     {
    //         return valid;
    //     }

    //     // slide the rake back along bez (i.e. compute new timesteps to rake)
    //     const auto backstep = percents.broadcast(0) / n;
    //     for (auto i = 1U; i < n; i++)
    //     {
    //         // evaluate states in rake
    //         auto times = (percents - i * backstep).to_array();

    //         for (auto j = 0U; j < Robot::dimension / 3; j++)  
    //         {  
    //             // Collect the i-th dimension values for all rake configurations  
    //             std::array<float, rake> dim_values;  
    //             for (auto k = 0U; k < rake; k++)  
    //             {  
    //                 const auto &state = bez.evaluate(static_cast<float>(times[k]));  
    //                 dim_values[k] = state[j];
    //             }  
    //             // Broadcast these values across SIMD lanes  
    //             block[j] = FloatVector<rake>(dim_values);  
    //         }
    //         // std::cout << block << std::endl;

    //         bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
    //                                                  Robot::template fkcc<rake>(environment, block);
    //         if (not valid)
    //         {
    //             return false;
    //         }
    //     }

    //     return true;
    // }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        const typename Robot::Configuration &start,
        int T,
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // just check every configuration along bez
        for (auto t = 0U; t < T; t += rake) {
            typename Robot::template ConfigurationBlock<rake> block;
            for (auto i = 0U; i < Robot::dimension / 3; i++) {
                std::array<float, rake> dim_values;
                for (auto j = 0U; j < rake; j++) {
                    float time = static_cast<float>(t + j) / static_cast<float>(T);
                    if (time > 1.0) {
                        time = 1.0;
                    }
                    const auto state = bez.evaluate(time);
                    dim_values[j] = state(i);
                }
                block[i] = FloatVector<rake>(dim_values);
            }
            // print block for debug
            // for (auto i = 0U; i < Robot::dimension / 3; i++) {
            //     std::cout << "block row " << i << ": " << block[i] << std::endl;
            // }
            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid) {
                return false;
            }
        }
        return true;
    }
    // only call this
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

        for (auto i = 0U; i < Robot::dimension; i++) {
            x[i] = static_cast<float>(start_arr[i]);
        }
        auto goal_arr = goal.to_array();
        for (auto i = 0U; i < Robot::dimension; i++) {
            x[Robot::dimension + i] = static_cast<float>(goal_arr[i]);
        }

        // array to store inference output
        std::array<float, 29> out;
        Robot::template topple_nn_forward(x, out);

        // build the anchors
        row_matrix anchors(6, Robot::dimension / 3);

        // initial point
        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            anchors(0, i) = static_cast<float>(start_arr[i]);
        }

        // intermediate points
        for (auto i = 1U; i <= 4; i++) {
            for (auto j = 0U; j < Robot::dimension / 3; j++) {
                anchors(i, j) = out[(i - 1) * Robot::dimension / 3 + j];
            }
        }

        int T = out[28] * 1000;

        // final point
        for (auto i = 0U; i < Robot::dimension / 3; i++) {
            anchors(5, i) = static_cast<float>(goal_arr[i]);
        }

        Bezier bez(anchors);
        return validate_bez<Robot, rake, resolution>(start, T, bez, environment);
    }
}  // namespace vamp::planning
