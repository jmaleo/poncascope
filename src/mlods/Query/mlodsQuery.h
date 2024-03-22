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

    template<typename NodeFunctor,
            typename DescentDistanceThresholdFunctor,
            typename WeightFunctor>
    // FitT
    std::vector<DataPoint> 
    search_internal_rec(const VectorType& point,
                             const IndexDistance<IndexType, Scalar>& nodeInfo,
                             NodeFunctor nodeFunctor,
                             DescentDistanceThresholdFunctor descentDistanceThreshold,
                             WeightFunctor weightFunctor
                            )
    {
        std::vector <DataPoint> res;
        // FitT fit;
        // fit.init(point);

        const auto& node = m_kdtree->node_data()[nodeInfo.index];

        Scalar distance = descentDistanceThreshold( node.getAabb().diagonal().norm() ) ;

        if (node.is_leaf()){
            Scalar weight = Scalar( 0.0 );
            for (IndexType i = node.leaf_start(); i < node.leaf_size(); ++i)
            {
                weight += weightFunctor( m_kdtree->point_data()[m_kdtree->index_data()[i]].pos() );
            }
            // auto algebraic = node.getFit().getParabolicCylinder();
            // algebraic *= weight;
            // fit += algebraic;
            // return fit;
            DataPoint current {weight * node.getFit().barycenter(), weight * node.getFit().meanNormalVector()};
            res.push_back( current );
            return res;
        }

        Scalar weight = weightFunctor ( node.getFit().barycenter() );

        if ( ! ( nodeInfo.distance <= distance ) ){
            // fit *= weight;
            // return fit;
            DataPoint current {weight * node.getFit().barycenter(), weight * node.getFit().meanNormalVector()};
            res.push_back( current );
            return res;
        }

        for (IndexType i = node.inner_first_child_id(); i < node.inner_first_child_id() + 2; ++i)
        {
            IndexDistance<IndexType, Scalar> childInfo{i, (point - m_kdtree->node_data()[i].getAabb().center()).norm()};
            Scalar gamma = nodeFunctor( nodeInfo, childInfo );
            Scalar total_weight = weight * gamma;
            // FitT fitChild = search_internal_rec(point, childInfo, nodeFunctor, descentDistanceThreshold, weightFunctor);
            // FitT fitCurrent = node.getFit();
            // fitChild *= ( Scalar( 1) - gamma );
            // fitCurrent *= total_weight;
            // fit += fitChild;
            // fit += fitCurrent;
            std::vector<DataPoint> fitChild = search_internal_rec(point, childInfo, nodeFunctor, descentDistanceThreshold, weightFunctor);
            for (auto& v : fitChild) {
                v.pos() *= ( Scalar( 1) - gamma );
                v.normal() *= ( Scalar( 1) - gamma );   
            }
            res.insert(res.end(), fitChild.begin(), fitChild.end());
            DataPoint current {total_weight * node.getFit().barycenter(), total_weight * node.getFit().meanNormalVector()};
            res.push_back( current );
        }
        // return fit;
        return res;

    }

    template<typename LeafPreparationFunctor,
            typename DescentDistanceThresholdFunctor,
            typename WeightFunctor,
            typename ProcessFitFunctor>
    bool search_internal(const VectorType& point,
                         LeafPreparationFunctor prepareLeafTraversal,
                         DescentDistanceThresholdFunctor descentDistanceThreshold,
                         WeightFunctor weightFunctor,
                         ProcessFitFunctor processFitFunctor
                         )
    {
        const auto& nodes   = m_kdtree->node_data();
        const auto& points  = m_kdtree->point_data();
        const auto& indices = m_kdtree->index_data();

        if (nodes.empty() || points.empty() || indices.empty())
            throw std::invalid_argument("Empty KdTree");

        auto nodeFunctor = [this, &nodes, &descentDistanceThreshold](const IndexDistance<IndexType, Scalar>& parent, const IndexDistance<IndexType, Scalar>& child){
            
            const auto& parent_node = nodes[parent.index];
            const auto& child_node = nodes[child.index];

            Scalar parent_threshold = descentDistanceThreshold( parent_node.getAabb().diagonal().norm() );
            Scalar child_threshold = descentDistanceThreshold( child_node.getAabb().diagonal().norm() );

            Scalar distance_to_parent = parent.distance;
            Scalar distance_to_child = child.distance;

            Scalar distance_to_parent_threshold = parent.distance - parent_threshold;
            Scalar distance_to_child_threshold = child.distance - child_threshold;

            if ( distance_to_parent_threshold <= Scalar( 0.0 ) && distance_to_child_threshold <= Scalar( 0.0 ) ){
                return Scalar( 0.0 );
            }
            if ( distance_to_parent_threshold >= Scalar( 0.0 ) && distance_to_child_threshold >= Scalar( 0.0 ) ){
                return Scalar( 1.0 );
            }

            Scalar u = distance_to_child_threshold / ( distance_to_child_threshold - distance_to_parent_threshold );
            return std::exp( - std::exp( Scalar ( 1.0 ) / ( u - Scalar ( 1.0 ) ) ) ) / std::pow( u, Scalar ( 2.0 ) );
        };


        // FitT res = search_internal_rec(point, m_stack.top(), nodeFunctor, descentDistanceThreshold, weightFunctor);
        std::vector<DataPoint> res = search_internal_rec(point, m_stack.top(), nodeFunctor, descentDistanceThreshold, weightFunctor);

        processFitFunctor( res, Scalar(1) );
        return true;
        // while(!m_stack.empty())
        // {
        //     auto& qnode = m_stack.top();
        //     const auto& node = nodes[qnode.index];

        //     // If the query point is inside the bounding box of the node, 
        //     // we need to grab the node's fit and visit its children
        //     if(qnode.distance < descentDistanceThreshold(node.getAabb().diagonal().norm()))
        //     {
        //         // Do something with the node's fit
        //         FitT nodeFit = node.getFit();
        //         processFitFunctor( nodeFit, weightFunctor( nodeFit.barycenter() ) );

        //         if(node.is_leaf())
        //         {
        //             m_stack.pop();
                    
        //             Scalar weight = Scalar( 0.0 );
        //             for (IndexType i = node.leaf_start(); i < node.leaf_size(); ++i)
        //             {
        //                 weight += weightFunctor( m_kdTree->point_data()[m_kdtree->index_data()[i]].pos() );
        //             }

        //         }
        //         else
        //         {
        //             // replace the stack top by the farthest and push the closest
        //             Scalar newOff = point[node.inner_split_dim()] - node.inner_split_value();
        //             m_stack.push();
        //             if(newOff < 0)
        //             {
        //                 m_stack.top().index = node.inner_first_child_id();
        //                 qnode.index         = node.inner_first_child_id()+1;
        //             }
        //             else
        //             {
        //                 m_stack.top().index = node.inner_first_child_id()+1;
        //                 qnode.index         = node.inner_first_child_id();
        //             }
        //             // Update the distances to the new nodes and the query point
        //             m_stack.top().distance = (point - nodes[m_stack.top().index].getAabb().center()).norm();;
        //             qnode.distance         = (point - nodes[qnode.index].getAabb().center()).norm();
        //         }
        //     }
        //     else
        //     {
        //         m_stack.pop();
        //     }
        // }
        return true;
    }
};

} // namespace Ponca
