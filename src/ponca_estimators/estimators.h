#pragma once

#include <Eigen/Dense>
#include <Ponca/Common>
#include <Ponca/Fitting>
#include "../IO/PointCloudDiff.h"

#include "adapters/EstimatorFactory.h"
#include "adapters/declarations.h"

/// Convenience function measuring and printing the processing time of F
template <typename Functor>
void measureTime( const std::string &actionName, Functor F ){
    using namespace std::literals; // enables the usage of 24h instead of e.g. std::chrono::hours(24)

    const std::chrono::time_point<std::chrono::steady_clock> start =
            std::chrono::steady_clock::now();
    F(); // run process
    const auto end = std::chrono::steady_clock::now();
}

/// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
/// \note Functor is called only if fit is stable
template<typename Estimator, typename Functor>
void processPointCloud( Estimator& estimator, int uniqueIdx, Functor f ){
    int nvert = ponca_kdtree.index_data().size();

    if ( uniqueIdx > -1 ){
        Quantity<Scalar> q;
        estimator( uniqueIdx, q );
        f ( uniqueIdx, q );
        return;
    }
    
    #pragma omp parallel for
    for (int i = 0; i < nvert; ++i) {
        DGtal::trace.progressBar( i, nvert );    
        Quantity<Scalar> q;
        estimator(i, q);
        f (i, q);
    }
}

template<typename Estimator>
void specialUniquePointCompute ( Estimator &estimator, PointCloudDiff<Scalar>& pcDiff, int uniqueIdx ) {

    measureTime( "[Ponca] Compute special unique point using " + estimator.getName(),
                 [&uniqueIdx, &estimator, &pcDiff]() {
                         Quantity<Scalar> q;
                         estimator( uniqueIdx, q );
                         estimator.customQuery( pcDiff );
                    });
}

template<typename Estimator>
SampleVectorType evalScalarField_impl ( Estimator &estimator, const SampleMatrixType &input_pos ) {
    SampleVectorType scalarField;
    int nvert = input_pos.size();
    scalarField = SampleVectorType::Zero( nvert );

    measureTime( "[Ponca] Compute scalar field using " + estimator.getName(),
                 [&estimator, &input_pos, &scalarField, &nvert]() {

                     #pragma omp for
                     for (int i = 0 ; i < nvert ; ++i) {
                             Quantity<Scalar> q;
                             estimator( input_pos.row(i), q );
                             scalarField[i] = estimator.potential(input_pos.row(i));
                         }
                     });

    return scalarField;
}

/// Generic processing function: traverse point cloud and compute mean, first and second curvatures + their direction
/// \tparam FitT Defines the type of estimator used for computation
template<typename Estimator>
DifferentialQuantities<Scalar> estimateDifferentialQuantities_impl( Estimator &estimator, int uniqueIdx ) {

    int nvert = ponca_kdtree.index_data().size();

    VectorType defaultVectorType = VectorType(1, 0, 0); // Keep the norm to 1 to avoid numerical issues for non-stable points

    DGtal::Statistic<Scalar> statTime, statNei;
    SampleVectorType mean ( nvert ), kmin ( nvert ), kmax ( nvert ), gauss ( nvert );
    SampleMatrixType proj ( nvert, 3 ), normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 );
    std::vector <unsigned int> non_stable_idx(nvert, 0);

    measureTime( "[Ponca] Compute differential quantities using " + estimator.getName(),
                 [&uniqueIdx, &estimator, &proj, &mean, &kmin, &kmax, &gauss, &normal, &dmin, &dmax, &statTime, &statNei, &non_stable_idx]() {
                            processPointCloud(estimator, uniqueIdx,
                                [&proj, &mean, &kmin, &kmax, &gauss, &normal, &dmin, &dmax, &statTime, &statNei, &non_stable_idx]
                                (
                                    int i, Quantity<Scalar> &quantity
                                ) {
                                    statNei.addValue( quantity.neighbor_count );
                                    // statNei.addValue( fit.getNumNeighbors() );
                                    statTime.addValue( quantity.computing_time.count() );

                                    if (quantity.non_stable >= 1) {
                                        non_stable_idx[i] = 1;
                                        return;
                                    }
                                    mean  [ i ]   = quantity.mean;
                                    kmax  [ i ]   = quantity.k2;
                                    kmin  [ i ]   = quantity.k1;
                                    dmax.row( i )   = quantity.d2;
                                    dmin.row( i )   = quantity.d1;
                                    gauss [ i ]   = quantity.gauss;
                                    normal.row ( i )   = quantity.normal;
                                    proj.row( i )   = quantity.projection;
                                });
                            });

    // if there is unstable points, we don't return the results

    DifferentialQuantities<Scalar> dq = { proj, kmin, kmax, mean, gauss, dmin, dmax, normal, statNei, statTime };
    dq.setNonStableVector(non_stable_idx);
    return dq;
}

template<typename WeightFunc>
DifferentialQuantities<Scalar> estimateDifferentialQuantities( std::string name, int uniqueIdx = -1 ) {

    std::shared_ptr<EstimatorFactory< WeightFunc >> factory = getEstimatorFactory<WeightFunc>();

    auto estimator = factory->getEstimator(name);

    DifferentialQuantities<Scalar> diff = estimateDifferentialQuantities_impl( *estimator, uniqueIdx );
    diff.setOriented(estimator->isOriented());
    return diff;
}

template <typename WeightFunc>
void estimateAndProject_OnePoint( std::string name, PointCloudDiff<Scalar>& pcDiff, int uniqueIdx) {
    specialUniquePointCompute( *getEstimatorFactory<WeightFunc>()->getEstimator(name), pcDiff, uniqueIdx );
}

template <typename WeightFunc>
SampleVectorType evalScalarField( std::string name, const SampleMatrixType &input_pos) {
    return evalScalarField_impl( *getEstimatorFactory<WeightFunc>()->getEstimator(name), input_pos );
}

template<typename WeightFunc>
SampleVectorType neiRequest(int index) {
    int nvert = ponca_kdtree.index_data().size();
    SampleVectorType nei_value = SampleVectorType::Zero(nvert);

    VectorType pos = ponca_kdtree.point_data()[index].pos();
    WeightFunc w(radius);

    processNeighbors(pos, index, [&w, &pos, &nei_value, &index](int j) {
        // fit.addNeighbor(ponca_kdtree.point_data()[j]);
        if (researchType == 0) {
            nei_value[j] = (j != index) ? 1 : 2;
        } else {
            const auto &q = ponca_kdtree.point_data()[j];
            nei_value[j] = w.w(q.pos() - pos, q).first;
        }
    });

    return nei_value;
}

inline Scalar evalMeanNeighbors () {
    Scalar meanNeighbors = 0;
    int nvert = ponca_kdtree.index_data().size();
    measureTime("[Ponca] Dry fit", [&meanNeighbors, &nvert]() {
        for (int i = 0; i < nvert; ++i) {
            processNeighbors(i, [&meanNeighbors](int j) {
                meanNeighbors += 1;
            });
        }
    });
    meanNeighbors /= nvert;
    return meanNeighbors;
}

Scalar estimateRadiusFromKNN(int k) {
    Scalar radius = 0;
    int nvert = ponca_kdtree.index_data().size();
    for (int i = 0; i < nvert; ++i) {
        VectorType pos = ponca_kdtree.point_data()[i].pos();
        Scalar max_dist = 0;
        
        for (int j : ponca_kdtree.k_nearest_neighbors(i, k)){
            max_dist = std::max(max_dist, (pos - ponca_kdtree.point_data()[j].pos()).norm());
        }

        radius += max_dist;
    }
    radius /= nvert;
    return radius;
}

// GetVertexSourcePosition
inline VectorType getSourcePosition(const int index) {
    return ponca_kdtree.point_data()[index].pos();
}
