#pragma once

// #include "../definitions.h"
#include "mlodsTraits.h"
#include "Query/mlodsRangeQuery.h"
#include "mlodsWeightFunc.h"

#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

namespace Ponca {
/**
 * TODO : Créer un kdTree custom, qui stoque en chaque noeud un FitT.
 * Il a la nécesité, lors du build, d'avoir un post traitement pour compute les fits dans les noeuds.
*/

template<typename Traits>
class KdTreeCustom : public KdTreeBase<Traits> 
{

public:
    using DataPoint    = typename Traits::DataPoint; ///< DataPoint given by user via Traits
    using IndexType    = typename Traits::IndexType; ///< Type used to index points into the PointContainer
    using LeafSizeType = typename Traits::LeafSizeType; ///< Type used to store the size of leaf nodes

    using PointContainer = typename Traits::PointContainer; ///< Container for DataPoint used inside the KdTree
    using IndexContainer = typename Traits::IndexContainer; ///< Container for indices used inside the KdTree

    using NodeIndexType  = typename Traits::NodeIndexType; ///< Type used to index nodes into the NodeContainer
    using NodeType       = typename Traits::NodeType; ///< Type of nodes used inside the KdTree
    using NodeContainer  = typename Traits::NodeContainer; ///< Container for nodes used inside the KdTree

    using Scalar     = typename DataPoint::Scalar; ///< Scalar given by user via DataPoint
    using VectorType = typename DataPoint::VectorType; ///< VectorType given by user via DataPoint
    using AabbType   = typename NodeType::AabbType; ///< Bounding box type given by user via NodeType

    using FitT = typename Traits::FitT; ///< Fit type given by user via Traits

    using Base = KdTreeBase<Traits>;

    /// Default constructor creating an empty tree. \see build \see buildWithSampling
    inline KdTreeCustom(): KdTreeBase<Traits>() {}

    /// Constructor generating a tree from a custom contained type converted using DefaultConverter
    template<typename PointUserContainer>
    inline KdTreeCustom(PointUserContainer&& points):
        Base::m_points(PointContainer()),
        Base::m_nodes(NodeContainer()),
        Base::m_indices(IndexContainer()),
        Base::m_min_cell_size(64),
        Base::m_leaf_count(0)
    {
        this->build(std::forward<PointUserContainer>(points));
    };

    /// Constructor generating a tree sampled from a custom contained type converted using DefaultConverter
    template<typename PointUserContainer, typename IndexUserContainer>
    inline KdTreeCustom(PointUserContainer&& points, IndexUserContainer sampling): // PointUserContainer => Given by user, transformed to PointContainer
                                                                                 // IndexUserContainer => Given by user, transformed to IndexContainer
        Base::m_points(),
        Base::m_nodes(),
        Base::m_indices(),
        Base::m_min_cell_size(64),
        Base::m_leaf_count(0)
    {
        this->buildWithSampling(std::forward<PointUserContainer>(points), std::move(sampling));
    };

    /// Generate a tree from a custom contained type converted using DefaultConverter
    /// \tparam PointUserContainer Input point container, transformed to PointContainer
    /// \param points Input points
    template<typename PointUserContainer>
    inline void build(PointUserContainer&& points)
    {
        Base::build(std::forward<PointUserContainer>(points), Base::DefaultConverter());

        // Post traitement pour compute les fits dans les noeuds
        // TODO
        // Function name.
    }

    /// Generate a tree from a custom contained type converted using DefaultConverter
    /// \tparam PointUserContainer Input point container, transformed to PointContainer
    /// \tparam IndexUserContainer Input sampling container, transformed to IndexContainer
    /// \param points Input points
    /// \param c Cast/Convert input point type to DataType
    template<typename PointUserContainer, typename Converter>
    inline void build(PointUserContainer&& points, Converter c);

     /// Generate a tree sampled from a custom contained type converted using DefaultConverter
    /// \tparam PointUserContainer Input point, transformed to PointContainer
    /// \tparam IndexUserContainer Input sampling, transformed to IndexContainer
    /// \param points Input points
    /// \param sampling Indices of points used in the tree
    template<typename PointUserContainer, typename IndexUserContainer>
    inline void buildWithSampling(PointUserContainer&& points,
                                  IndexUserContainer sampling)
    {
        buildWithSampling(std::forward<PointUserContainer>(points), std::move(sampling), Base::DefaultConverter());
    
        // Post traitement pour compute les fits dans les noeuds
        // TODO
        // Function name.
    }

    /// Generate a tree sampled from a custom contained type converted using DefaultConverter
    /// \tparam PointUserContainer Input point, transformed to PointContainer
    /// \tparam IndexUserContainer Input sampling, transformed to IndexContainer
    /// \tparam Converter
    /// \param points Input points
    /// \param sampling Indices of points used in the tree
    /// \param c Cast/Convert input point type to DataType
    template<typename PointUserContainer, typename IndexUserContainer, typename Converter>
    inline void buildWithSampling(PointUserContainer&& points,
                                  IndexUserContainer sampling,
                                  Converter c);

    // template<typename PointUserContainer, typename IndexUserContainer>
    // inline void buildSecondPass(PointUserContainer&& points,
    //                               IndexUserContainer sampling);

    // inline bool valid() const;
    inline std::string to_string(bool verbose = false) const;

protected:
    inline void build_rec(NodeIndexType node_id, IndexType start, IndexType size, int depth);

    /// @brief  Compute the fitting for the node using it's children's fitting. 
    /// The leaf are computed in the first pass, here we compute the node's fitting by combining it's children's fitting.
    inline void build_fit_rec(NodeIndexType node_id);

public:

    FitT fit_request(const VectorType& point, Scalar radiusFactor) const
    {
        MLODSTreeRangePointQuery<Traits> query(this, radiusFactor, point);

        return query.search();
    }

    template <typename WeightParameters>
    FitT fit_request(const VectorType& point, Scalar radiusFactor, WeightParameters param) const
    {
        MLODSTreeRangePointQuery<Traits> query(this, radiusFactor, point);

        return query.search(param);
    }

};


#include "./mlods.hpp"
} // namespace Ponca