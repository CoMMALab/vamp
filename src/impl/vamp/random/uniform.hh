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
    
    private:  
        std::mt19937 rng_;  
        std::uniform_real_distribution<float> dist_;  
    };
}
