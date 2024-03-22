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
        fit.init(point);

        // Redifine the following functions
        // Especially "DescentDistanceThreshold()" 
        //    which needs to take in a parameter the percentage of the bounding box for each node.
        auto descentDistanceThreshold = [this](const Scalar& diagonalNode){return QueryType::descentDistanceThreshold(diagonalNode);};
        // auto processFitFunctor        = [this, &fit](FitT nodeFit, Scalar weight)
        // {   
        //     auto algebraic = nodeFit.getParabolicCylinder(); 
        //     algebraic *= weight;
        //     fit += algebraic;
        // };
        auto processFitFunctor        = [this, &fit](std::vector<DataPoint> traversed_points, Scalar weight)
        {   
            // auto algebraic = nodeFit.getParabolicCylinder(); 
            // algebraic *= weight;
            // fit += algebraic;
            FIT_RESULT res;
            do {
                fit.startNewPass();
                for (const auto& p : traversed_points){
                    fit.addNeighbor(p);
                }
                res = fit.finalize();
            } while (res == NEED_OTHER_PASS);

        };
        auto weightFunctor            = [this, &point](const VectorType& averagePoint){ // Average point of the node
            Scalar weight = Scalar(1);
            Scalar base_sigma = Scalar(0.1);

            // Parameters of the gaussian mixture
            int k = 1;
            Scalar a = 1;

            for ( int i = 0; i < k; i++ ){
                Scalar sigma = std::pow( a, i ) * base_sigma;
                // Scalar num = -glm::pow(glm::l2Norm(q-p),2);
                Scalar num = -std::pow( ( averagePoint - point ).norm(), 2 );
                Scalar denom = Scalar(2) * std::pow( sigma, 2 );
                Scalar fact = ( std::pow( sigma, -3 ) * exp( num / denom ) );
                weight += fact;
            }

            return Scalar(weight);
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
