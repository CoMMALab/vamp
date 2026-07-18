#pragma once

#include <vamp/planning/nn/kdtree.hh>

namespace vamp::planning
{
    template <typename Robot>
    using NN = KDTree<Robot>;
}  // namespace vamp::planning
