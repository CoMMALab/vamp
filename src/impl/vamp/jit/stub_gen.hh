#pragma once

#include <vamp/planning/planner.hh>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace vamp::jit
{
    // Constraint kernels present in the generated robot source, mirroring the has_*
    // gates that cricket sets from the recipe keys. Gated stub entry points are only
    // emitted (and later resolved) for capabilities that are present.
    struct RobotCapabilities
    {
        bool constraints = false;
        bool com = false;
        bool closed_loops = false;
        bool lead_screw = false;
        bool twist = false;
        std::size_t n_eef = 0;
        std::size_t n_closed_loops = 0;

        [[nodiscard]] constexpr auto any() const -> bool
        {
            return constraints || com || closed_loops || lead_screw || twist;
        }
    };

    auto generate_stub_source(
        const std::string &robot_source,
        const std::string &robot_name,
        std::size_t rake,
        std::size_t resolution,
        const std::vector<vamp::planning::Planner> &planners,
        const RobotCapabilities &capabilities = {}) -> std::string;

    auto planner_symbol(vamp::planning::Planner p, std::string_view suffix) -> std::string;
    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string;
}  // namespace vamp::jit
