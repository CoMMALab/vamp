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
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        const typename Robot::Configuration &start,
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // std::cout << "inside validate bez" << std::endl;
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);
        // for (int i = 0; i < rake; i++) {
        //     std::cout << percents << std::endl;
        // }

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // use percents as times to get configs
        // std::array<state, rake> states_arr;
        auto percents_arr = percents.to_array();

        std::array<std::array<float, rake>, Robot::dimension / 3> configs;
        for (int i = 0; i < rake; i++) {
            const auto &state = bez.evaluate(static_cast<double>(percents_arr[i]));

            for (auto j = 0U; j < Robot::dimension / 3; ++j)
            {
                configs[j][i] = state[j];
            }
        }

        // CHECK FOR CORRECTNES
        for (int i = 0; i < Robot::dimension / 3; i++)
        {
            block[i] = FloatVector<rake>(configs[i]);
            // std::cout << block[i] << std::endl;
        }

        // const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
        // no idea if this is correct (its not)
        // std::cout << "check collision" << std::endl;
        const std::size_t n = static_cast<float>(resolution) / 2;
        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
                                                
        // std::cout << valid << std::endl;
        if (not valid or n == 1)
        {
            return valid;
        }

        // slide the rake back along bez (i.e. compute new timesteps to rake)
        // this is stupid
        const auto backstep = percents.broadcast(0) / n;
        // for (int i = 0; i < backstep.to_array().size(); i++) {
        //     std::cout << backstep.to_array()[i] << std::endl;
        // }

        // std::cout << "sliding rake" << std::endl;
        for (int i = 1; i < n; i++)
        {
            // evaluate states in rake
            // this is wrong as fuk
            auto times = (percents - i * backstep).to_array();
            // for (int j = 0; j < rake; j++) {
            //     // std::cout << times[j] << std::endl;
            //     states_arr[j] = (bez.evaluate(static_cast<double>(times[j])));
            // }

            std::array<std::array<float, rake>, Robot::dimension / 3> configs;
            for (int i = 0; i < rake; i++) {
                const auto &state = bez.evaluate(static_cast<double>(times[i]));

                for (auto j = 0U; j < Robot::dimension / 3; ++j)
                {
                    configs[j][i] = state[j];
                }
            }

            // CHECK FOR CORRECTNES
            for (int i = 0; i < Robot::dimension / 3; i++)
            {
                block[i] = FloatVector<rake>(configs[i]);
                // std::cout << block[i] << std::endl;
            }

            // collision checkig is broken, ask someone wtf any of this shit is
            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
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
        std::array<double, 42> x;
        auto start_arr = start.to_array();

        for (int i = 0; i < Robot::dimension; ++i) {
            // this line for some reason changes the value of i
            x[i] = static_cast<double>(start_arr[i]);
        }
        auto goal_arr = goal.to_array();
        for (int i = 0U; i < Robot::dimension; ++i) {
            x[Robot::dimension + i] = static_cast<double>(goal_arr[i]);
        }

        // array to store inference output
        std::array<double, 29> out;
        Robot::template topple_nn_forward(x, out);

        // build the anchors
        row_matrix anchors(6, Robot::dimension / 3);

        // initial point
        for (int i = 0; i < Robot::dimension / 3; ++i) {
            anchors(0, i) = static_cast<double>(start_arr[i]);
        }

        // intermediate points
        for (int i = 1; i <= 4; i++) {
            for (int j = 0; j < Robot::dimension / 3; ++j) {
                anchors(i, j) = out[(i - 1) * Robot::dimension / 3 + j];
            }
        }

        // final point
        for (int i = 0; i < Robot::dimension / 3; i++) {
            anchors(5, i) = static_cast<double>(goal_arr[i]);
        }

        Bezier bez(anchors);
        return validate_bez<Robot, rake, resolution>(start, bez, environment);
    }    
}  // namespace vamp::planning
