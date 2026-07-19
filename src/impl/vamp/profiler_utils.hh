#pragma once

#include <chrono>
#include <vamp/vector.hh>
#include <map>
#include <numeric>
#include <iomanip>
#include <algorithm>

using namespace std::chrono;
using namespace std;

namespace vamp::profiling
{
    struct Profiler {
        std::map<std::string, std::vector<double>> measurements;

        // Access operator for easy push_back: profiler["name"].push_back(value)
        std::vector<double>& operator[](const std::string& name) {
            return measurements[name];
        }

        // Calculate statistics for a measurement
        struct Stats {
            double min_val;
            double max_val;
            double avg;
            double median;
            size_t count;
        };

        Stats calculate_stats(const std::vector<double>& times) const {
            if (times.empty()) {
                return {0, 0, 0, 0, 0};
            }

            double min_val = *std::min_element(times.begin(), times.end());
            double max_val = *std::max_element(times.begin(), times.end());
            double avg = std::accumulate(times.begin(), times.end(), 0.0) / times.size();

            std::vector<double> sorted = times;
            std::sort(sorted.begin(), sorted.end());
            double median = (sorted.size() % 2 == 0)
                            ? (sorted[sorted.size() / 2 - 1] + sorted[sorted.size() / 2]) / 2.0
                            : sorted[sorted.size() / 2];

            return {min_val, max_val, avg, median, times.size()};
        }

        // Print report for all measurements
        void printReport() {
            if (measurements.empty()) {
                std::cout << "No measurements recorded.\n";
                return;
            }

            std::cout << "\n";
            std::cout << "================================\n";
            std::cout << "         PROFILER REPORT        \n";
            std::cout << "================================\n";

            for (const auto& [name, times] : measurements) {
                if (times.empty()) continue;

                Stats stats = calculate_stats(times);

                std::cout << std::left << std::setw(35) << name << " | "
                         << "count: " << std::setw(6) << stats.count
                         << " avg: " << std::fixed << std::setprecision(6) << std::setw(12) << stats.avg << " ns"
                         << " median: " << std::setw(12) << stats.median << " ns"
                         << " min: " << std::setw(12) << stats.min_val << " ns"
                         << " max: " << std::setw(12) << stats.max_val << " ns\n";
            }

            std::cout << "================================\n\n";
        }

        // Print raw measurements for a specific category
        void print_measurements(const std::string& name) const {
            auto it = measurements.find(name);
            if (it == measurements.end() || it->second.empty()) {
                std::cout << "No measurements for: " << name << "\n";
                return;
            }

            std::cout << "Measurements for " << name << ": ";
            for (double val : it->second) {
                std::cout << val << " ";
            }
            std::cout << "\n";
        }

        // Clear all measurements
        void clear() {
            measurements.clear();
        }

        // Clear a specific measurement category
        void clear(const std::string& name) {
            measurements.erase(name);
        }

        // Get the number of categories being tracked
        size_t num_categories() const {
            return measurements.size();
        }

        // Get the total number of measurements across all categories
        size_t total_measurements() const {
            size_t total = 0;
            for (const auto& [name, times] : measurements) {
                total += times.size();
            }
            return total;
        }
    };

    // Global profiler instance with accessor
    inline Profiler& get_profiler() {
        static Profiler profiler;
        return profiler;
    }
}

