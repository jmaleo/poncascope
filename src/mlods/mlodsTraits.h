#pragma once

#include "Ponca/Fitting"
#include <optional>
#include "../definitions.h"

using Scalar2             = double;
// using Scalar             = float;

using VectorType2         = Eigen::Matrix<Scalar2, 3,1>;
using PPAdapter2          = BlockPointAdapter<Scalar2>;

template <typename _Scalar>
using ConstWeightFunc2      = Ponca::DistWeightFunc<PPAdapter2, Ponca::ConstantWeightKernel<_Scalar> >;

template <typename Scalar>
using basket_FullyOrientedEllipsoid2DFit2 =  Ponca::Basket<PPAdapter2, ConstWeightFunc2<Scalar>, Ponca::FullyOrientedEllipsoid2DFit>;

namespace Ponca {

////////////////////////////////////////////////
//  MLoDS Traits
////////////////////////////////////////////////


/**
 * TODO : Créer des noeuds customs pour le kdTree, qui stoque en chaque noeud un FitT. 
 * Nécessite d'avoir un operator + et / pour FitT. 
 * 
 * Les fonctions dans ces noeuds customs doivent avoir en paramètre le kdTree lui-même, pour pouvoir accéder à ses données.
 * Ils seront utilisés dans un kdTree custom, qui fera un post-traitement sur les noeuds pour compute les fits. 
 * Dans une première passe, les feuilles pourrons être compute directement, mais les noeuds internes devront attendre que les feuilles soient compute.
 */

//! [CustomInnerNodeDefinition]
template < typename _FitT, typename NodeIndex, typename Index, typename Scalar, int DIM >
struct MyKdTreeInnerNode : public Ponca::KdTreeDefaultInnerNode<NodeIndex, Scalar, DIM> {

    using FitT = _FitT;

    FitT m_fit;

    bool m_fitted = false;

    // Only used if the aggregation of the fits is working.
    template <typename KdTree>
    bool fittingProcess(const KdTree& tree, const Index& childIdx)
    {   
        if ( ! tree.node_data()[childIdx].is_fitted() || ! tree.node_data()[childIdx+1].is_fitted() )
            return false;
        
        m_fit += tree.node_data()[childIdx].getFit();
        m_fit += tree.node_data()[childIdx+1].getFit();

        m_fit /= Scalar(2);

        return true;
    }

    // Only used when the aggregation of the fits is working.
    template <typename KdTree>
    void aggregate_fitting(const KdTree& tree, const Index& childIdx){
        m_fitted = fittingProcess(tree, childIdx);
    }

    template <typename KdTree>
    bool fittingProcess(const KdTree& tree, Index start, Index size)
    {

        // FitT::WeightFunction must be constantWeightKernel
        // static_assert(std::is_same_v<typename FitT::WeightFunction, ConstWeightFunc>, "WeightFunction must be constant");
        // Get Neighbors using the KdTree and start, size values

        using weightFunc = typename FitT::WeightFunction;

        // First point to init the fitting process
        auto default_point = tree.point_data()[tree.index_data()[start]].pos();
        weightFunc w = weightFunc(size); // Maybe check the instantiation of the weight function

        m_fit.setWeightFunc(w);
        m_fit.init(default_point);


        // compute the fitting process.
        Ponca::FIT_RESULT res;
        do {
            for( Index i=start; i<start+size; ++i ){
                Index realIdx = tree.index_data()[i];
                auto point = tree.point_data()[realIdx];  
                m_fit.addNeighbor(point);
            }
            res = m_fit.finalize();
        } while ( res == Ponca::NEED_OTHER_PASS);

        if ( res != Ponca::STABLE )
            return false;
        return true;
    }

    template <typename KdTree>
    void configure_fitting (const KdTree& tree, const Index& start, const Index& size)
    {
        // // // // Only used if the aggregation on the children is working.
        // Instantiate Weight Function and FitT
        // using weightFunc = typename FitT::WeightFunction;
        // weightFunc w = weightFunc(size); // Maybe check the instantiation of the weight function
        // m_fit.setWeightFunc(w);
        // auto default_point = tree.point_data()[tree.index_data()[start]].pos();
        // m_fit.init(default_point);

        // Compute Fit
        m_fitted = fittingProcess(tree, start, size);
    } 

};
//! [CustomInnerNodeDefinition]

//! [CustomLeafNodeDefinition]
template < typename _FitT, typename Index, typename Size >
struct MyKdTreeLeafNode : public Ponca::KdTreeDefaultLeafNode<Index, Size> {

    using FitT = _FitT;

    FitT m_fit;

    bool m_fitted = false;

    template <typename KdTree>
    bool fittingProcess(const KdTree& tree, Index start, Index size)
    {

        // FitT::WeightFunction must be constantWeightKernel
        // static_assert(std::is_same_v<typename FitT::WeightFunction, ConstWeightFunc>, "WeightFunction must be constant");
        // Get Neighbors using the KdTree and start, size values

        using weightFunc = typename FitT::WeightFunction;

        // First point to init the fitting process
        auto default_point = tree.point_data()[tree.index_data()[start]].pos();
        weightFunc w = weightFunc(size); // Maybe check the instantiation of the weight function

        m_fit.setWeightFunc(w);
        m_fit.init(default_point);


        // compute the fitting process.
        Ponca::FIT_RESULT res;
        do {
            for( Index i=start; i<start+size; ++i ){
                Index realIdx = tree.index_data()[i];
                auto point = tree.point_data()[realIdx];  
                m_fit.addNeighbor(point);
            }
            res = m_fit.finalize();
        } while ( res == Ponca::NEED_OTHER_PASS);

        if ( res != Ponca::STABLE )
            return false;
        return true;
    }

    template <typename KdTree>
    void configure_fitting (const KdTree& tree, Index start, Index size)
    {
        // Compute Fit
        m_fitted = fittingProcess(tree, start, size);
    } 

};
//! [CustomLeafNodeDefinition]

//! [CustomNodeDefinition]
template < typename _FitT, typename Index, typename NodeIndex, typename DataPoint, typename LeafSize = Index >
struct MyKdTreeNode : Ponca::KdTreeCustomizableNode<Index, NodeIndex, DataPoint, 
        LeafSize,
        MyKdTreeInnerNode< _FitT, NodeIndex, Index, typename DataPoint::Scalar, DataPoint::Dim >,
        MyKdTreeLeafNode< _FitT, Index, LeafSize > > {

    using Base = Ponca::KdTreeCustomizableNode< Index, NodeIndex, DataPoint, 
            LeafSize,
            MyKdTreeInnerNode< _FitT, NodeIndex, Index, typename DataPoint::Scalar, DataPoint::Dim >,
            MyKdTreeLeafNode< _FitT, Index, LeafSize > >;
    
    using FitT  = _FitT;
    using AabbType  = typename Base::AabbType;

    AabbType m_aabb{};

    template <typename KdTree>
    void configure_fitting (const KdTree& tree, Index start, Index size)
    {
        if ( Base::is_leaf() )
        {
            Base::getAsLeaf().configure_fitting(tree, start, size);
        }
        else 
        {
            Base::getAsInner().configure_fitting(tree, start, size);
        }
    }

    template <typename KdTree>
    void compute_inner_fit (const KdTree& tree)
    {
        if ( ! Base::is_leaf() )
        {
            // Compute Fit of inner node by aggregating children fits
            Base::getAsInner().aggregate_fitting(tree, Base::inner_first_child_id());
        }
    }

    void configure_range(Index start, Index size, const AabbType &aabb)
    {
        Base::configure_range(start, size, aabb);
        m_aabb = aabb;
    }

    [[nodiscard]] inline AabbType getAabb() const {
        return m_aabb;
    }

    [[nodiscard]] inline FitT getFit() const {
        if ( Base::is_leaf() )
            return Base::getAsLeaf().m_fit;
        else
            return Base::getAsInner().m_fit;
    }

    [[nodiscard]] inline bool leaf_is_fitted () const {
        return Base::getAsLeaf().m_fitted;
    }

    [[nodiscard]] inline bool inner_is_fitted () const {
        return Base::getAsInner().m_fitted;
    }

    [[nodiscard]] inline bool is_fitted () const {
        if ( Base::is_leaf() )
            return Base::getAsLeaf().m_fitted;
        else
            return Base::getAsInner().m_fitted;
    }

    [[nodiscard]] inline LeafSize getSize() const {
        if ( Base::is_leaf() )
            return Base::getAsLeaf().size;
        else
            return Base::getAsInner().size;
    }

};
//! [CustomNodeDefinition]

/*!
 * \brief The default traits type used by the kd-tree.
 *
 * \see KdTreeCustomizableNode Helper class to modify Inner/Leaf nodes without redefining a Trait class
 *
 * \tparam _NodeType Type used to store nodes, set by default to #KdTreeDefaultNode
 */
template <typename _DataPoint, typename _FitT, 
        template <typename /*FitT*/,
                  typename /*Index*/, 
                  typename /*NodeIndex*/,
                  typename /*DataPoint*/,
                  typename /*LeafSize*/>
                    typename _NodeType = MyKdTreeNode>
struct MyKdTreeTraits
{
    enum
    {
        /*!
         * \brief A compile-time constant specifying the maximum depth of the kd-tree.
         */
        MAX_DEPTH = 32,
    };

    /*!
     * \brief The type used to store point data.
     *
     * Must provide `Scalar` and `VectorType` typedefs.
     *
     * `VectorType` must provide a `squaredNorm()` function returning a `Scalar`, as well as a
     * `maxCoeff(int*)` function returning the dimension index of its largest scalar in its output
     * parameter (e.g. 0 for *x*, 1 for *y*, etc.).
     */
    using DataPoint    = _DataPoint;
    using IndexType    = int;
    using LeafSizeType = unsigned short;

    using FitT = _FitT;

    // Containers
    using PointContainer = std::vector<DataPoint>;
    using IndexContainer = std::vector<IndexType>;

    // Nodes
    using NodeIndexType = std::size_t;
    using NodeType      = _NodeType< FitT, IndexType, NodeIndexType, DataPoint, LeafSizeType >;
    using NodeContainer = std::vector<NodeType>;
};


} // namespace Ponca
