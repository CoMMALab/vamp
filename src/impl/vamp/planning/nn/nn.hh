#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include <vamp/vector.hh>
#include <vamp/vector/eigen.hh>

#include <nigh/nigh_forward.hpp>
#include <nigh/lp_space.hpp>
#include <nigh/kdtree_batch.hpp>

namespace vamp::planning
{
    namespace nn_detail
    {
        template <std::size_t dim_>
        struct FloatArray
        {
            inline static constexpr auto dim = dim_;
            float *v;

            // NOTE: This lint (google-explicit-constructor) is incorrect in this case; we want implicit
            // conversion
            inline operator Eigen::Matrix<float, dim_, 1>() const
            {
                return EigenFloatVectorMap<dim_>(v);
            }
        };

        template <std::size_t dimension>
        struct Node
        {
            std::size_t index;
            FloatArray<dimension> array;
        };

        template <std::size_t dimension>
        struct NodeKey
        {
            inline auto operator()(const Node<dimension> &node) const noexcept
                -> const FloatArray<dimension> &
            {
                return node.array;
            }
        };
    }  // namespace nn_detail
}  // namespace vamp::planning

namespace unc::robotics::nigh::metric
{
    template <std::size_t dim>
    struct Space<vamp::planning::nn_detail::FloatArray<dim>, LP<2>>
    {
        using Type = vamp::planning::nn_detail::FloatArray<dim>;
        using Metric = LP<2>;
        using Distance = float;
        static constexpr int kDimensions = dim;

        static auto isValid(const Type & /*v*/) -> bool
        {
            return true;
        }

        static auto coeff(const Type &v, std::size_t i) -> float
        {
            return *(v.v + i);
        }

        [[nodiscard]] static constexpr auto dimensions() -> unsigned
        {
            return kDimensions;
        }

        static auto distance(const Type &a, const Type &b) -> float
        {
            const auto diff = vamp::FloatVector<kDimensions>(a.v) - vamp::FloatVector<kDimensions>(b.v);
            return diff.l2_norm();
        }
    };
}  // namespace unc::robotics::nigh::metric

namespace vamp::planning
{
    // Nearest-neighbor structure over robot configurations, backed by nigh. Configurations are
    // stored by pointer: the caller keeps the storage alive and unmoved while they are in the tree.
    template <typename Robot, std::size_t batch = 128>
    struct NN
    {
        static constexpr std::size_t dimension = Robot::dimension;

        using Node = nn_detail::Node<dimension>;
        using Key = nn_detail::FloatArray<dimension>;
        using Tree = unc::robotics::nigh::Nigh<
            Node,
            unc::robotics::nigh::metric::Space<Key, unc::robotics::nigh::metric::LP<2>>,
            nn_detail::NodeKey<dimension>,
            unc::robotics::nigh::NoThreadSafety,
            unc::robotics::nigh::KDTreeBatch<batch>>;

        static constexpr float inf = std::numeric_limits<float>::infinity();

        void reserve(std::size_t) noexcept
        {
        }

        [[nodiscard]] auto size() const noexcept -> std::size_t
        {
            return tree_.size();
        }

        void clear()
        {
            tree_.clear();
        }

        void insert(std::size_t index, const float *q)
        {
            tree_.insert(Node{index, {const_cast<float *>(q)}});
        }

        [[nodiscard]] auto nearest(const float *q) const noexcept
            -> std::optional<std::pair<std::size_t, float>>
        {
            const auto result = tree_.nearest(Key{const_cast<float *>(q)});
            if (not result)
            {
                return std::nullopt;
            }

            return std::make_pair(result->first.index, result->second);
        }

        // K-nearest within an optional radius, sorted ascending by distance.
        void nearest(
            std::vector<std::pair<std::size_t, float>> &out,
            const float *q,
            std::size_t k,
            float radius = inf) const
        {
            out.clear();
            std::vector<std::pair<Node, float>> found;
            tree_.nearest(found, Key{const_cast<float *>(q)}, k, radius);
            out.reserve(found.size());
            for (const auto &[node, distance] : found)
            {
                out.emplace_back(node.index, distance);
            }
        }

    private:
        Tree tree_;
    };
}  // namespace vamp::planning
