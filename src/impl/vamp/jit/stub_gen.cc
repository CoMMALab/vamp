#include <vamp/jit/stub_gen.hh>

#include <vamp/jit/embedded_stubs.hh>

#include <inja/inja.hpp>
#include <nlohmann/json.hpp>

#include <sstream>
#include <stdexcept>
#include <string>

namespace vamp::jit
{
    auto planner_symbol(vamp::planning::Planner p, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + std::string(vamp::planning::planner_name(p)) + "_" +
               std::string(suffix);
    }

    auto robot_symbol(const std::string &robot_name, std::string_view suffix) -> std::string
    {
        return std::string("vamp_jit_") + robot_name + "_" + std::string(suffix);
    }

    auto generate_stub_source(
        const std::string &robot_source,
        const std::string &robot_name,
        std::size_t rake,
        std::size_t resolution,
        const std::vector<vamp::planning::Planner> &planners,
        const RobotCapabilities &capabilities) -> std::string
    {
        if (robot_source.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_source");
        }

        if (robot_name.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: empty robot_name");
        }

        if (planners.empty())
        {
            throw std::runtime_error("vamp::jit::generate_stub_source: no planners requested");
        }

        std::ostringstream out;
        out << embedded::preamble << "\n" << robot_source << "\n";

        inja::Environment env;

        nlohmann::json base = {
            {"robot_name", robot_name},
            {"sym", robot_name},
            {"robot_type", "vamp::robots::" + robot_name},
            {"robot_ns", "vamp_jit_robot"},
            {"rake", rake},
            {"resolution", resolution},
            {"has_constraints", capabilities.constraints},
            {"has_com", capabilities.com},
            {"has_closed_loops", capabilities.closed_loops},
            {"has_lead_screw", capabilities.lead_screw},
            {"has_twist", capabilities.twist},
            {"has_any_constraint", capabilities.any()},
            {"num_end_effectors", capabilities.n_eef},
            {"is_flask", false},
            {"has_chart", capabilities.chart()},
        };

        out << env.render(std::string(embedded::robot_stub), base);

        // The flask sibling gets its own full robot stub (Robot::Flask under a second
        // namespace, symbols suffixed _flask) plus the phase/chart helpers; constraint
        // factories stay on the ambient robot only, mirroring the static bindings.
        nlohmann::json flask_base;
        if (capabilities.flask)
        {
            flask_base = base;
            flask_base["sym"] = robot_name + "_flask";
            flask_base["robot_type"] = "vamp::robots::" + robot_name + "::Flask";
            flask_base["robot_ns"] = "vamp_jit_flask_robot";
            flask_base["resolution"] = capabilities.flask_resolution;
            flask_base["has_constraints"] = false;
            flask_base["has_com"] = false;
            flask_base["has_closed_loops"] = false;
            flask_base["has_lead_screw"] = false;
            flask_base["has_twist"] = false;
            flask_base["has_any_constraint"] = false;
            flask_base["is_flask"] = true;

            out << env.render(std::string(embedded::robot_stub), flask_base);
            out << env.render(std::string(embedded::flask_stub), flask_base);
        }

        for (auto p : planners)
        {
            const auto &d = vamp::planning::planner_descriptor(p);
            nlohmann::json data = base;
            data["planner_name"] = std::string(d.name);
            data["planner_class"] = std::string(d.class_name);
            data["settings_class"] = std::string(d.settings_class);
            data["planner_header"] = std::string(d.header);
            data["settings_header"] = std::string(d.settings_header);
            data["planner_local_planner"] = d.local_planner;
            out << env.render(std::string(embedded::planner_stub), data);

            if (capabilities.flask)
            {
                nlohmann::json fdata = flask_base;
                fdata["planner_name"] = "flask_" + std::string(d.name);
                fdata["planner_class"] = std::string(d.class_name);
                fdata["settings_class"] = std::string(d.settings_class);
                fdata["planner_header"] = std::string(d.header);
                fdata["settings_header"] = std::string(d.settings_header);
                fdata["planner_local_planner"] = d.local_planner;
                out << env.render(std::string(embedded::planner_stub), fdata);
            }
        }

        return out.str();
    }
}  // namespace vamp::jit
