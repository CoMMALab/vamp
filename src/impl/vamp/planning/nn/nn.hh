#pragma once

#include <vamp/planning/nn/kdtree.hh>

namespace vamp::planning
{
    // Nearest-neighbor structure over the states of `Space` (a robot is its own space).
    template <typename Space>
    using NN = KDTree<Space>;
}  // namespace vamp::planning
