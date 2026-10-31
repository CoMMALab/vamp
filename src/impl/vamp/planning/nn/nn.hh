#pragma once

#include <vamp/planning/nn/kdtree.hh>

namespace vamp::planning
{
    template <typename Robot, typename Space = Robot>
    using NN = KDTree<Robot, 128, Space>;
}  // namespace vamp::planning
