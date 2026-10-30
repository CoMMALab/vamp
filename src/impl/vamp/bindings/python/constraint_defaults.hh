#pragma once

#include <vamp/planning/constraints/settings.hh>

namespace vamp::binding
{
    // The constraint settings a Python caller gets when it passes none: the library's defaults, but with the bounded
    // least-squares step (ProjMethod::BoundedLS), where the joint limits enter the step as constraints. The C++
    // default (ConstraintSettings{}) is left as it is.
    inline auto python_constraint_settings() -> vamp::planning::constraint::ConstraintSettings
    {
        vamp::planning::constraint::ConstraintSettings settings;
        settings.method = vamp::planning::constraint::ProjMethod::BoundedLS;
        return settings;
    }
}  // namespace vamp::binding
