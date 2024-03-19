/*
 This Source Code Form is subject to the terms of the Mozilla Public
 License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "Ponca/SpatialPartitioning"
#include "Ponca/Common"

#include <cmath>


namespace Ponca {


/// \brief Base class for range queries
template<typename Index, typename Scalar>
struct QueryOutputIsMLODSRange : public QueryOutputBase {
    using OutputParameter = Scalar;

    inline QueryOutputIsMLODSRange(OutputParameter radiusFactor = OutputParameter(1))
            : m_radius_factor ( radiusFactor ) {}

    inline Scalar radius_factor() const { return m_radius_factor; }

    inline void set_radius_factor(Scalar radiusFactor) { m_radius_factor = radiusFactor; }

protected:
    /// \brief Reset Query for a new search
    inline void reset() { }
    /// \brief Distance threshold used during tree descent to select nodes to explore
    inline Scalar descentDistanceThreshold( const Scalar& diagonalNode ) const { return Scalar( 0.5 ) * diagonalNode * m_radius_factor; }
    Scalar m_radius_factor{1};
};


/*! \brief Base Query class combining QueryInputIsPosition and QueryOutputIsMLODSRange. */    \
template <typename Index, typename DataPoint>
struct MLODSRangePointQuery : Query<QueryInputIsPosition<DataPoint>,
                                 QueryOutputIsMLODSRange<Index, typename DataPoint::Scalar>>
{
    using Base = Query<QueryInputIsPosition<DataPoint>, QueryOutputIsMLODSRange<Index, typename DataPoint::Scalar>>;
    using Base::Base;
};

#undef DECLARE_POINT_QUERY_CLASS

} // namespace Ponca
