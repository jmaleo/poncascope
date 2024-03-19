/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "mlodsQuery.h"
#include "queryExtend.h"

namespace Ponca {


// This class could inherit from Iterator, to go through the nodes of the tree
template <typename Traits,
        typename QueryType>
class MLODSTreeRangeQueryBase : public MlodsQueryBase<Traits>, public QueryType
                              //public RangeIndexQuery<typename Traits::IndexType, typename Traits::DataPoint::Scalar>
{
public:
    using DataPoint      = typename Traits::DataPoint;
    using IndexType      = typename Traits::IndexType;
    using Scalar         = typename DataPoint::Scalar;
    using VectorType     = typename DataPoint::VectorType;
    using QueryAccelType = MlodsQueryBase<Traits>;
    
    using FitT           = typename Traits::FitT;

public:
    MLODSTreeRangeQueryBase(const KdTreeCustom<Traits>* kdtree, Scalar radiusFactor, typename QueryType::InputType input) :
            MlodsQueryBase<Traits>(kdtree), QueryType(radiusFactor, input){}

public:

    inline FitT search(/*Iterator& it*/){
        const auto& points  = QueryAccelType::m_kdtree->point_data();
        const auto& indices = QueryAccelType::m_kdtree->index_data();
        const auto& point   = QueryType::getInputPosition(points);
        FitT fit;

        // Redifine the following functions
        // Especially "DescentDistanceThreshold()" 
        //    which needs to take in a parameter the percentage of the bounding box for each node.
        auto descentDistanceThreshold = [this](const Scalar& diagonalNode){return QueryType::descentDistanceThreshold(diagonalNode);};
        auto skipFunctor              = [this](IndexType idx){return QueryType::skipIndexFunctor(idx);};
        auto processNeighborFunctor   = [this /*&it*/](IndexType idx, IndexType i, Scalar)
        {
            // it.m_index = idx;
            // it.m_start = i+1;
            return true;
        };

        if (points.empty() || indices.empty())
            throw std::invalid_argument("Empty KdTree");

        // Redefine the following function
        if (MlodsQueryBase<Traits>::search_internal(point,
                                                 [this /*&it*/](IndexType start, IndexType end)
                                                 {
                                                    //  it.m_start = start;
                                                    //  it.m_end   = end;
                                                 },
                                                 descentDistanceThreshold,
                                                 skipFunctor,
                                                 processNeighborFunctor))
            // it.m_index = static_cast<IndexType>(points.size());
        
        return fit;
    }
};

template <typename Traits>
using MLODSTreeRangePointQuery = MLODSTreeRangeQueryBase< Traits, MLODSRangePointQuery<typename Traits::IndexType, typename Traits::DataPoint>>;

} // namespace ponca
