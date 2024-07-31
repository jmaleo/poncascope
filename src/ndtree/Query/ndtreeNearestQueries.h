#pragma once

#include "ndtreeQuery.h"
#include <Ponca/SpatialPartitioning>
#include "../Iterator/ndtreeNearestIterator.h"

namespace Ponca {

template <typename Traits,
          template <typename> typename IteratorType,
          typename QueryType>
class NDTreeNearestQueryBase : public NDTreeQuery<Traits>, public QueryType
{
public:
    using DataPoint      = typename Traits::DataPoint;
    using IndexType      = typename Traits::IndexType;
    using Scalar         = typename DataPoint::Scalar;
    using VectorType     = typename DataPoint::VectorType;
    using QueryAccelType = NDTreeQuery<Traits>;
    using Iterator       = IteratorType<typename Traits::IndexType>;

    NDTreeNearestQueryBase(const NDTreeBase<Traits>* ndtree, typename QueryType::InputType input) :
            NDTreeQuery<Traits>(ndtree), QueryType(input) {}

public:
    inline Iterator begin() {
        QueryAccelType::reset();
        QueryType::reset();
        this->search();
        return Iterator(QueryType::m_nearest);
    }
    inline Iterator end() {
        return Iterator(QueryType::m_nearest + 1);
    }

protected:
    inline void search() {
        QueryAccelType::search_internal(QueryType::getInputPosition(QueryAccelType::m_ndtree->point_data()),
                                        [](IndexType, IndexType){},
                                        [this](){return QueryType::descentDistanceThreshold();},
                                        [this](IndexType idx){return QueryType::skipIndexFunctor(idx);},
                                        [this](IndexType idx, IndexType, Scalar d)
                                        {
                                            QueryType::m_nearest = idx;
                                            QueryType::m_squared_distance = d;
                                            return false;
                                        }
        );
    }
};

template <typename Traits>
using NDTreeNearestIndexQuery = NDTreeNearestQueryBase< Traits, NDTreeNearestIterator,
                                NearestIndexQuery<typename Traits::IndexType, typename Traits::DataPoint::Scalar>>;
template <typename Traits>
using NDTreeNearestPointQuery = NDTreeNearestQueryBase< Traits, NDTreeNearestIterator,
                                NearestPointQuery<typename Traits::IndexType, typename Traits::DataPoint>>;
} // namespace Ponca