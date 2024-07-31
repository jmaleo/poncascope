#pragma once

#include <Ponca/Common>
#include <Ponca/SpatialPartitioning>


namespace Ponca {
template <typename Traits> class NDTreeBase;

template <typename Traits>
class NDTreeQuery
{
public:
    using DataPoint  = typename Traits::DataPoint;
    using IndexType  = typename Traits::IndexType;
    using Scalar     = typename DataPoint::Scalar;
    using VectorType = typename DataPoint::VectorType;

    explicit inline NDTreeQuery(const NDTreeBase<Traits>* ndtree) : m_ndtree(ndtree), m_stack() {}

protected:
    /// \brief Init stack for a new search
    inline void reset() {
        m_stack.clear();
        m_stack.push({0,0});
    }

    const NDTreeBase<Traits>* m_ndtree { nullptr };
    Stack<IndexSquaredDistance<IndexType, Scalar>, 2 * Traits::MAX_DEPTH> m_stack;

    template<typename LeafPreparationFunctor,
             typename DescentDistanceThresholdFunctor,
             typename SkipIndexFunctor,
             typename ProcessNeighborFunctor>
    bool search_internal(const VectorType& point,
                         LeafPreparationFunctor prepareLeafTraversal,
                         DescentDistanceThresholdFunctor descentDistanceThreshold,
                         SkipIndexFunctor skipFunctor,
                         ProcessNeighborFunctor processNeighborFunctor)
    {
        const auto& nodes   = m_ndtree->node_data();
        const auto& points  = m_ndtree->point_data();
        const auto& indices = m_ndtree->index_data();

        if (nodes.empty() || points.empty() || indices.empty())
            throw std::invalid_argument("Empty NDTree");

        while(!m_stack.empty())
        {
            auto& qnode = m_stack.top();
            const auto& node = nodes[qnode.index];

            if(qnode.squared_distance < descentDistanceThreshold())
            {
                if(node.is_leaf())
                {
                    m_stack.pop();
                    IndexType start = node.leaf_start();
                    IndexType end = node.leaf_start() + node.leaf_size();
                    prepareLeafTraversal(start, end);
                    for(IndexType i=start; i<end; ++i)
                    {
                        IndexType idx = indices[i];
                        if(skipFunctor(idx)) continue;

                        Scalar d = (point - points[idx].pos()).squaredNorm();

                        if(d < descentDistanceThreshold())
                        {
                            if( processNeighborFunctor( idx, i, d )) return false;
                        }
                    }
                }
                else
                {
                    // Traverse child nodes
                    for (int i = 0; i < Traits::CHILD_COUNT; ++i)
                    {
                        IndexType child_id = node.inner_child_id(i);
                        if (child_id != (IndexType)-1)
                        {
                            const auto& child = nodes[child_id];
                            Scalar d = child.aabb.squaredExteriorDistance(point);
                            if (d < descentDistanceThreshold())
                            {
                                m_stack.push({child_id, d});
                            }
                        }
                    }
                    m_stack.pop(); // Remove current node
                }
            }
            else
            {
                m_stack.pop();
            }
        }
        return true;
    }
};
} // namespace Ponca