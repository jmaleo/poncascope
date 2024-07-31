#pragma once

#include "ndtreeQuery.h"
#include <Ponca/SpatialPartitioning>
#include "../Iterator/ndtreeRangeIterator.h"

namespace Ponca {

template <typename Traits,
        template <typename,typename,typename> typename IteratorType,
        typename QueryType>
class NDTreeRangeQueryBase : public NDTreeQuery<Traits>, public QueryType
{
public:
    using DataPoint      = typename Traits::DataPoint;
    using IndexType      = typename Traits::IndexType;
    using Scalar         = typename DataPoint::Scalar;
    using VectorType     = typename DataPoint::VectorType;
    using QueryAccelType = NDTreeQuery<Traits>;
    using Iterator       = IteratorType<IndexType, DataPoint, NDTreeRangeQueryBase>;

protected:
    friend Iterator;

public:
    NDTreeRangeQueryBase(const NDTreeBase<Traits>* ndtree, Scalar radius, typename QueryType::InputType input) :
            NDTreeQuery<Traits>(ndtree), QueryType(radius, input) {}

public:
    inline Iterator begin() {
        QueryAccelType::reset();
        QueryType::reset();
        Iterator it(this);
        this->advance(it);
        return it;
    }
    inline Iterator end() {
        return Iterator(this, QueryAccelType::m_ndtree->point_count());
    }

protected:
    inline void advance(Iterator& it) {
        const auto& points  = QueryAccelType::m_ndtree->point_data();
        const auto& indices = QueryAccelType::m_ndtree->index_data();
        const auto& point   = QueryType::getInputPosition(points);

        auto descentDistanceThreshold = [this](){return QueryType::descentDistanceThreshold();};
        auto skipFunctor              = [this](IndexType idx){return QueryType::skipIndexFunctor(idx);};
        auto processNeighborFunctor   = [&it](IndexType idx, IndexType i, Scalar)
        {
            it.m_index = idx;
            it.m_start = i+1;
            return true;
        };

        if (points.empty() || indices.empty())
            throw std::invalid_argument("Empty NDTree");

        for(IndexType i=it.m_start; i<it.m_end; ++i)
        {
            IndexType idx = indices[i];
            if(skipFunctor(idx)) continue;

            Scalar d = (point - points[idx].pos()).squaredNorm();
            if(d < descentDistanceThreshold())
            {
                if( processNeighborFunctor(idx, i, d) ) return;
            }
        }

        if (NDTreeQuery<Traits>::search_internal(point,
                                                 [&it](IndexType start, IndexType end)
                                                 {
                                                     it.m_start = start;
                                                     it.m_end   = end;
                                                 },
                                                 descentDistanceThreshold,
                                                 skipFunctor,
                                                 processNeighborFunctor))
            it.m_index = static_cast<IndexType>(points.size());
    }
};

template <typename Traits>
using NDTreeRangeIndexQuery = NDTreeRangeQueryBase< Traits, NDTreeRangeIterator,
        RangeIndexQuery<typename Traits::IndexType, typename Traits::DataPoint::Scalar>>;
template <typename Traits>
using NDTreeRangePointQuery = NDTreeRangeQueryBase< Traits, NDTreeRangeIterator,
        RangePointQuery<typename Traits::IndexType, typename Traits::DataPoint>>;
} // namespace Ponca