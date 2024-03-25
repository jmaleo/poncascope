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
    using WeightKernel   = typename Traits::WeightKernel;
    using WeightParam    = typename WeightKernel::WeightParam;

public:
    MLODSTreeRangeQueryBase(const KdTreeCustom<Traits>* kdtree, Scalar radiusFactor, typename QueryType::InputType input) :
            MlodsQueryBase<Traits>(kdtree), QueryType(radiusFactor, input) {}

public:

    
    inline FitT search(const WeightParam& _param = WeightParam()){
        const auto& points  = QueryAccelType::m_kdtree->point_data();
        const auto& indices = QueryAccelType::m_kdtree->index_data();
        const auto& point   = QueryType::getInputPosition(points);
        FitT fit;
        fit.init(point);

        WeightKernel weightKernel (_param);
        weightKernel.init(point);

        std::cout << "Parameters of the kernel : " << std::endl;
        std::cout << "k : " << weightKernel.m_parameters.m_k << std::endl;
        std::cout << "epsilon : " << weightKernel.m_parameters.m_epsilon << std::endl;

        // Redifine the following functions
        // Especially "DescentDistanceThreshold()" 
        //    which needs to take in a parameter the percentage of the bounding box for each node.
        auto descentDistanceThreshold = [this](const Scalar& diagonalNode){return QueryType::descentDistanceThreshold(diagonalNode);};
        auto processFitFunctor        = [this, &fit](FitT nodeFit, Scalar weight)
        {   
            // auto algebraic = nodeFit.getParabolicCylinder(); 
            // nodeFit *= weight;
            fit = nodeFit;
        };

        auto weightFunctor            = [this, &weightKernel](const VectorType& _p){ // Average point of the node
            return weightKernel.w(_p);
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
                                                 weightFunctor,
                                                 processFitFunctor)){
            std::cout << "Success" << std::endl;
        }
            // it.m_index = static_cast<IndexType>(points.size());
        return fit;
    }
};

template <typename Traits>
using MLODSTreeRangePointQuery = MLODSTreeRangeQueryBase< Traits, MLODSRangePointQuery<typename Traits::IndexType, typename Traits::DataPoint>>;

} // namespace ponca
