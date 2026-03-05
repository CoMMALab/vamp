#pragma once

#include <cmath>
#include <random>
#include <algorithm>

namespace vamp::rng {
    template <typename Robot>  
    struct Uniform  
    {  
        using Configuration = typename Robot::Configuration;  
        
        explicit Uniform(uint32_t seed = std::random_device{}())   
            : rng_(seed), dist_(0.0f, 1.0f) {}  
    
        inline auto sample() -> Configuration  
        {  
            std::array<float, Robot::dimension> values;  
            for (auto i = 0U; i < Robot::dimension; ++i)  
            {  
                values[i] = dist_(rng_);  
            }  
            
            // Create configuration and apply VAMP's scaling  
            Configuration config(values);  
            Robot::scale_configuration(config);  
            return config;  
        }
        inline auto sample_ball(Configuration q0, float r) -> Configuration  
        {  
            std::array<float, Robot::dimension> values;  
            for (auto i = 0U; i < Robot::dimension; ++i)  
            {  
                values[i] = dist_(rng_);  
            }  
            
            // Create configuration and apply VAMP's scaling
            Configuration config(values);  
            Robot::scale_configuration(config);

            auto diff = config - q0;
            auto diff_arr = diff.to_array();
            std::array<float, Robot::dimension> diff_pos_arr;
            for (auto i = 0U; i < Robot::dimension; ++i)  
            {  
                if (i < Robot::dimension / 3) {
                    diff_pos_arr[i] = diff_arr[i];
                }
                else {
                    diff_pos_arr[i] = 0;
                }
            }
            Configuration diff_pos(diff_pos_arr);
            auto scale = diff_pos.l2_norm() < r ? diff_pos.l2_norm() : r * dist_(rng_); // multiply r with a random from 0 to 1
            auto new_config = q0 + diff_pos / diff_pos.l2_norm() * scale;

            auto config_arr = config.to_array();
            auto new_config_arr = new_config.to_array();
            std::array<float, Robot::dimension> config_final_arr;
            for (auto i = 0U; i < Robot::dimension; ++i)  
            {  
                if (i < Robot::dimension / 3) {
                    config_final_arr[i] = new_config_arr[i];
                }
                else {
                    config_final_arr[i] = config_arr[i];
                }
            }
            Configuration config_final(config_final_arr);

            return config_final;  
        }    
    
    private:  
        std::mt19937 rng_;  
        std::uniform_real_distribution<float> dist_;  
    };
}
