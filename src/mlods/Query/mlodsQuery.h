#pragma once

#include "Ponca/SpatialPartitioning"
#include "Ponca/Common"

namespace Ponca {


/// \brief Associates an index with a distance
template<typename Index, typename Scalar>
struct IndexDistance
{
    //// Index of the closest point
    Index index {-1};

    /// Distance to the closest point
    Scalar distance { std::numeric_limits<Scalar>::max() };

    /// Comparison operator based on squared_distance
    inline bool operator < (const IndexDistance& other) const
    { return distance < other.distance; }
};

template <typename Traits> class KdTreeCustom;

template <typename Traits>
class MlodsQueryBase
{
public:
    using DataPoint  = typename Traits::DataPoint;
    using IndexType  = typename Traits::IndexType;
    using Scalar     = typename DataPoint::Scalar;
    using VectorType = typename DataPoint::VectorType;
    using FitT       = typename Traits::FitT;

    explicit inline MlodsQueryBase(const KdTreeCustom<Traits>* kdtree) : m_kdtree( kdtree ), m_stack() {
        reset();
    }

protected:
    /// \brief Init stack for a new search
    inline void reset() {
        m_stack.clear();
        m_stack.push({0,0});
    }

    const KdTreeCustom<Traits>* m_kdtree { nullptr };
    Stack<IndexDistance<IndexType, Scalar>, 2 * Traits::MAX_DEPTH> m_stack;

    template<typename LeafPreparationFunctor,
            typename DescentDistanceThresholdFunctor,
            typename SkipIndexFunctor,
            typename ProcessNeighborFunctor>
    bool search_internal(const VectorType& point,
                         LeafPreparationFunctor prepareLeafTraversal,
                         DescentDistanceThresholdFunctor descentDistanceThreshold,
                         SkipIndexFunctor skipFunctor,
                         ProcessNeighborFunctor processNeighborFunctor
                         )
    {
        const auto& nodes   = m_kdtree->node_data();
        const auto& points  = m_kdtree->point_data();
        const auto& indices = m_kdtree->index_data();

        if (nodes.empty() || points.empty() || indices.empty())
            throw std::invalid_argument("Empty KdTree");
            
        while(!m_stack.empty())
        {
            auto& qnode = m_stack.top();
            const auto& node = nodes[qnode.index];

            // If the query point is inside the bounding box of the node, 
            // we need to grab the node's fit and visit its children
            if(qnode.distance < descentDistanceThreshold(node.getAabb().diagonal().norm()))
            {
                // Do something with the node's fit

                if(node.is_leaf())
                {
                    m_stack.pop();
                }
                else
                {
                    // replace the stack top by the farthest and push the closest
                    Scalar newOff = point[node.inner_split_dim()] - node.inner_split_value();
                    m_stack.push();
                    if(newOff < 0)
                    {
                        m_stack.top().index = node.inner_first_child_id();
                        qnode.index         = node.inner_first_child_id()+1;
                    }
                    else
                    {
                        m_stack.top().index = node.inner_first_child_id()+1;
                        qnode.index         = node.inner_first_child_id();
                    }
                    // Update the distances to the new nodes and the query point
                    m_stack.top().distance = (point - nodes[m_stack.top().index].getAabb().center()).norm();;
                    qnode.distance         = (point - nodes[qnode.index].getAabb().center()).norm();
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
