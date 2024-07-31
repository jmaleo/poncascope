#pragma once

#include "./ndtreeTraits.h"
#include "Query/ndtreeNearestQueries.h"
#include "Query/ndtreeRangeQueries.h"

#include <memory>
#include <numeric>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace Ponca {

template <typename Traits> class NDTreeBase;

template<typename DataPoint>
using Octree = NDTreeBase<NDTreeDefaultTraits<DataPoint, 3>>;

/*!
 * \brief Public interface for NDTree datastructure.
 *
 * Provides default implementation of the NDTree
 *
 * \see NDTreeDefaultTraits for the default trait interface documentation.
 * \see NDTreeBase for complete API
 */
template <typename DataPoint, int Dim>
using NDTree = NDTreeBase<NDTreeDefaultTraits<DataPoint, Dim>>;

/*!
 * \brief Customizable base class for NDTree datastructure
 *
 * \see Ponca::NDTree
 *
 * \tparam Traits Traits type providing the types and constants used by the nd-tree. Must have the
 * same interface as the default traits type.
 *
 * \see NDTreeDefaultTraits for the trait interface documentation.
 */
template <typename Traits>
class NDTreeBase
{
public:
    using DataPoint    = typename Traits::DataPoint;
    using IndexType    = typename Traits::IndexType;
    using LeafSizeType = typename Traits::LeafSizeType;

    using PointContainer = typename Traits::PointContainer;
    using IndexContainer = typename Traits::IndexContainer;

    using NodeIndexType  = typename Traits::NodeIndexType;
    using NodeType       = typename Traits::NodeType;
    using NodeContainer  = typename Traits::NodeContainer;

    using Scalar     = typename DataPoint::Scalar;
    using VectorType = typename DataPoint::VectorType;
    using AabbType   = typename Traits::AabbType;

    enum
    {
        DIM = Traits::DIM,
        CHILD_COUNT = Traits::CHILD_COUNT,
        MAX_DEPTH = Traits::MAX_DEPTH,
    };

    inline NDTreeBase();

    template<typename PointUserContainer>
    inline NDTreeBase(PointUserContainer&& points);

    inline void clear();

    struct DefaultConverter
    {
        template <typename Input>
        inline void operator()(Input&& i, PointContainer& o);
    };

    template<typename PointUserContainer>
    inline void build(PointUserContainer&& points);

    template<typename PointUserContainer, typename Converter>
    inline void build(PointUserContainer&& points, Converter c);

    template<typename PointUserContainer, typename IndexUserContainer>
    inline void buildWithSampling(PointUserContainer&& points, IndexUserContainer sampling);

    template<typename PointUserContainer, typename IndexUserContainer, typename Converter>
    inline void buildWithSampling(PointUserContainer&& points, IndexUserContainer sampling, Converter c);

    inline bool valid() const;
    inline std::string to_string(bool verbose = false) const;

    // Accessors
    inline NodeIndexType node_count() const;
    inline IndexType index_count() const;
    inline IndexType point_count() const;
    inline NodeIndexType leaf_count() const;
    inline PointContainer& point_data();
    inline const PointContainer& point_data() const;
    inline const NodeContainer& node_data() const;
    inline const IndexContainer& index_data() const;

    // Parameters
    inline LeafSizeType min_cell_size() const;
    inline void set_min_cell_size(LeafSizeType min_cell_size);

protected:
    inline void build_rec(NodeIndexType node_id, IndexType start, IndexType end, const AabbType& aabb, int level);

    PointContainer m_points;
    NodeContainer m_nodes;
    IndexContainer m_indices;
    LeafSizeType m_min_cell_size;
    NodeIndexType m_leaf_count;

public:
    NDTreeNearestPointQuery<Traits> nearest_neighbor(const VectorType& point) const
    {
        return NDTreeNearestPointQuery<Traits>(this, point);
    }

    NDTreeNearestIndexQuery<Traits> nearest_neighbor(IndexType index) const
    {
        return NDTreeNearestIndexQuery<Traits>(this, index);
    }

    NDTreeRangePointQuery<Traits> range_neighbors(const VectorType& point, Scalar r) const
    {
        return NDTreeRangePointQuery<Traits>(this, r, point);
    }

    NDTreeRangeIndexQuery<Traits> range_neighbors(IndexType index, Scalar r) const
    {
        return NDTreeRangeIndexQuery<Traits>(this, r, index);
    }
};

#include "./ndtree.hpp"

} // namespace Ponca