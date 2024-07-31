#pragma once

#include <vector>
#include <array>
#include <Eigen/Geometry>

namespace Ponca {

template <typename _DataPoint, int _Dim = _DataPoint::Dim>
struct NDTreeDefaultTraits
{
    enum
    {
        MAX_DEPTH = 32,
        DIM = _Dim,
        CHILD_COUNT = 1 << DIM // 2^DIM
    };

    using DataPoint = _DataPoint;
    using IndexType = int;
    using LeafSizeType = unsigned short;

    using PointContainer = std::vector<DataPoint>;
    using IndexContainer = std::vector<IndexType>;

    using NodeIndexType = std::size_t;

    using Scalar = typename DataPoint::Scalar;
    using VectorType = typename DataPoint::VectorType;
    using AabbType = Eigen::AlignedBox<Scalar, DIM>;

    class NodeType
    {
    public:
        AabbType aabb;

        [[nodiscard]] bool is_leaf() const { return m_is_leaf; }
        void set_is_leaf(bool is_leaf) { m_is_leaf = is_leaf; }
        
        [[nodiscard]] IndexType point_count() const { return m_point_count; }
        void set_point_count(IndexType count) { m_point_count = count; }

        void configure_range(IndexType start, IndexType size, const AabbType &aabb)
        {
            this->aabb = aabb;
            if (m_is_leaf)
            {
                m_leaf.start = start;
                m_leaf.size = (LeafSizeType)size;
            }
        }

        void configure_inner(const std::array<NodeIndexType, CHILD_COUNT>& child_ids)
        {
            if (!m_is_leaf)
            {
                for (int i = 0; i < CHILD_COUNT; ++i)
                {
                    m_inner.child_ids[i] = child_ids[i];
                }
            }
        }

        AabbType getAabb() const { return aabb; }

        [[nodiscard]] IndexType leaf_start() const { return m_leaf.start; }
        [[nodiscard]] LeafSizeType leaf_size() const { return m_leaf.size; }
        [[nodiscard]] NodeIndexType inner_child_id(int index) const { return m_inner.child_ids[index]; }

    private:
        bool m_is_leaf{true};
        IndexType m_point_count{0};
        union {
            struct {
                IndexType start;
                LeafSizeType size;
            } m_leaf;
            struct {
                std::array<NodeIndexType, CHILD_COUNT> child_ids;
            } m_inner;
        };
    };

    using NodeContainer = std::vector<NodeType>;
};

} // namespace Ponca