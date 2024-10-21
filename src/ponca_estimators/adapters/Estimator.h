#pragma once

#include "types.h"
#include <chrono>

#include "declarations.h"
#include "fitDefinitions.h"
#include "../../definitions.h"

/// Function to compute the neighbors of a point
template <typename Functor>
void processNeighbors(const int &idx, const Functor f){
    // VectorType pos = ponca_kdtree.point_data()[idx].pos();
    if(use_kNNGraph){
        if (researchType == 0){
            for (int j : ponca_knnGraph->k_nearest_neighbors(idx)){
                f(j);
            }
            f(idx);
        }
        else {
            for (int j : ponca_knnGraph->range_neighbors(idx, radius)){
                f(j);
            }
            f(idx);
        }
    } else {
        if (researchType == 0){
            for (int j : ponca_kdtree.k_nearest_neighbors(idx, kNN)){
                f(j);
            }
            f(idx);
        }
        else {
            for (int j : ponca_kdtree.range_neighbors(idx, radius)){
                f(j);
            }
            f(idx);
        }
    }
}

/// Function to compute the neighbors of a point
template <typename Functor>
void processNeighbors(const VectorType &pos, const int &init_idx, const Functor f){
    // VectorType pos = ponca_kdtree.point_data()[init_idx].pos();
    if(use_kNNGraph){
        if (researchType == 0){
            for (int j : ponca_knnGraph->k_nearest_neighbors(init_idx)){
                f(j);
            }
            f(init_idx);
        }
        else {
            for (int j : ponca_knnGraph->range_neighbors(init_idx, radius)){
                f(j);
            }
            f(init_idx);
        }
    } else {
        if (researchType == 0){
            for (int j : ponca_kdtree.k_nearest_neighbors(pos, kNN)){
                f(j);
            }
        }
        else {
            for (int j : ponca_kdtree.range_neighbors(pos, radius)){
                double dist = (pos - ponca_kdtree.point_data()[j].pos()).norm();
                f(j);
            }
                
        }
    }
}

template < typename _Fit > 
class InterfaceProcessHandler {

    protected : 
        using FitT = _Fit;
        using WeightFunc = typename FitT::WeightFunction;
    
    public : 

        ~InterfaceProcessHandler() = default;
        InterfaceProcessHandler() {
            init_idx = 0;
            current_pos = VectorType::Zero();
            current_fitResult = Ponca::UNDEFINED;
            current_NeighborsPos.clear();
            current_NeighborsNormal.clear();
            current_NeighborsCount = 0;
            current_NeighborsTiming = std::chrono::nanoseconds::zero();
            current_ComputeTiming = std::chrono::nanoseconds::zero();
        };

        void initHandler ( int query_idx ){
            init_idx = query_idx;
            current_pos = ponca_kdtree.point_data()[ query_idx ].pos();
        }

        void initHandler ( const VectorType &query_pos ){
            current_pos = query_pos;
            init_idx = 0;
        }

        void initEstimator( FitT &fit ) {
            fit.setWeightFunc(WeightFunc(radius));
            fit.init(current_pos);

            current_NeighborsCount = 0;
            current_NeighborsPos.clear();
            current_NeighborsNormal.clear();
        }

        virtual void applyNeighbors( FitT & fit ) = 0;

        Ponca::FIT_RESULT finalize ( FitT &fit ) {
            
            auto tCompute_start = std::chrono::high_resolution_clock::now();

            current_fitResult = fit.finalize();
            if ( current_fitResult == Ponca::STABLE ){
                current_pos = fit.project( current_pos );
            }

            auto tCompute_end = std::chrono::high_resolution_clock::now();
            current_ComputeTiming += std::chrono::duration_cast<std::chrono::nanoseconds>( tCompute_end - tCompute_start );

            return current_fitResult;
        }

        void applyFunctor( FitT &fit, Quantity<Scalar> &diffQuantity ) {
            diffQuantity.neighbor_count = current_NeighborsCount;
            diffQuantity.computing_time = current_ComputeTiming;
            diffQuantity.non_stable = (current_fitResult != Ponca::STABLE || current_NeighborsCount < 3);
            if (diffQuantity.non_stable) return;

            diffQuantity.k1 = fit.kmin();
            diffQuantity.k2 = fit.kmax();
            diffQuantity.mean = fit.kMean();
            diffQuantity.gauss = fit.GaussianCurvature();
            diffQuantity.d1 = fit.kminDirection();
            diffQuantity.d2 = fit.kmaxDirection();
            diffQuantity.normal = fit.primitiveGradient();
            diffQuantity.projection = current_pos;
        }

        virtual void customQuery ( FitT &fit, PointCloudDiff<Scalar>& pcDiff ) = 0;

        virtual Scalar potential ( FitT &fit, const VectorType& pos ) = 0;

    protected :

        int                      init_idx = 0;
        VectorType               current_pos = VectorType::Zero();
        Ponca::FIT_RESULT        current_fitResult = Ponca::UNDEFINED;
        std::vector<VectorType>  current_NeighborsPos;
        std::vector<VectorType>  current_NeighborsNormal;
        int                      current_NeighborsCount = 0;

        std::chrono::nanoseconds current_NeighborsTiming = std::chrono::nanoseconds::zero();
        std::chrono::nanoseconds current_ComputeTiming = std::chrono::nanoseconds::zero();

}; // InterfaceProcessHandler

// Generic ProcessHandler
template < typename _Fit > 
class ProcessHandler : public InterfaceProcessHandler< _Fit > {

    using Base = InterfaceProcessHandler< _Fit >;
    using FitT = typename Base::FitT;

    public :
        std::string name;

    public : 

        ProcessHandler (const std::string &name) : name(name), Base() {}
        ~ProcessHandler() = default;

        void applyNeighbors( FitT & fit ) override {
            Base::current_NeighborsCount = 0;
            fit.startNewPass();
            auto tNei_start = std::chrono::high_resolution_clock::now();

            processNeighbors(Base::current_pos, Base::init_idx, [&]( int idx ){
                fit.addNeighbor( ponca_kdtree.point_data()[idx] );
                Base::current_NeighborsCount ++;
            });

            auto tNei_end = std::chrono::high_resolution_clock::now();
            Base::current_NeighborsTiming += std::chrono::duration_cast<std::chrono::nanoseconds>( tNei_end - tNei_start );
        }

    void customQuery ( FitT &fit, PointCloudDiff<Scalar>& pcDiff ) override {
        VectorType origin = pcDiff.points.row(0);
        pcDiff.v1.row(0) = fit.kminDirection();
        pcDiff.v2.row(0) = fit.kmaxDirection();
        pcDiff.normals.row(0) = fit.primitiveGradient();
        pcDiff.mean[0] = fit.kMean();
        pcDiff.k1[0] = fit.kmin();
        pcDiff.k2[0] = fit.kmax();
        pcDiff.gauss[0] = fit.GaussianCurvature();
        pcDiff.points.row(0) = fit.project( origin );

        if (name == "Sphere" 
        || name == "UnorientedSphere" 
        || name == "APSS"
        || name == "ASO" ) {
            pcDiff.points.row(0) = fit.barycenter();
        }

        for (int i = 1 ; i < pcDiff.points.rows(); i++) {
            const VectorType current = pcDiff.points.row(i);
            const VectorType proj = fit.project( current );
            if ( ( current - proj ).norm() > 1e-6 ) {
                pcDiff.points.row(i) = proj;
            }
            else {
                pcDiff.points.row(i) = pcDiff.points.row(0);
            }
        }
    }

    Scalar potential ( FitT &fit, const VectorType& pos ) override {
        return fit.potential( pos );
    }

}; // ProcessHandler


// Specific ProcessHandler for CNC
template <>
class ProcessHandler< fit_CNC > : public InterfaceProcessHandler< fit_CNC > {

    using Base = InterfaceProcessHandler< fit_CNC >;
    using FitT = fit_CNC;

    public :
        std::string name;
    
    public : 

        ProcessHandler (const std::string &name) : name( name ), Base() {}
        ~ProcessHandler() = default;

        void initEstimator( FitT &fit ) {
            Base::initEstimator( fit );
            if (name == "Independent")
                fit.chooseIndependentGeneration();
            else if (name == "Uniform")
                fit.chooseUniformGeneration();
            else if (name == "Hexagram")
                fit.chooseHexagramGeneration();
            else if (name == "AvgHexagram")
                fit.chooseAvgHexagramGeneration();
        }

        void applyNeighbors( FitT & fit ) override {
            Base::current_NeighborsCount = 0;
            fit.startNewPass();

            auto tNei_start = std::chrono::high_resolution_clock::now();

            processNeighbors(Base::current_pos, Base::init_idx, [&]( int idx ){
                Base::current_NeighborsPos.push_back(ponca_kdtree.point_data()[idx].pos());
                Base::current_NeighborsNormal.push_back(ponca_kdtree.point_data()[idx].normal());
                Base::current_NeighborsCount ++;
            });
            fit.computeNeighbors(ponca_kdtree.point_data()[Base::init_idx], Base::current_NeighborsPos, Base::current_NeighborsNormal);
            
            auto tNei_end = std::chrono::high_resolution_clock::now();
            Base::current_NeighborsTiming += std::chrono::duration_cast<std::chrono::nanoseconds>( tNei_end - tNei_start );
        }

        Ponca::FIT_RESULT finalize ( FitT &fit ) {
            auto tCompute_start = std::chrono::high_resolution_clock::now();

            Base::current_fitResult = fit.finalize();

            Base::current_ComputeTiming += std::chrono::duration_cast<std::chrono::nanoseconds>( std::chrono::high_resolution_clock::now() - tCompute_start );

            return Base::current_fitResult;
        }

        void customQuery ( FitT &fit, PointCloudDiff<Scalar>& pcDiff ) override {
            pcDiff.v1.row(0) = fit.kminDirection();
            pcDiff.v2.row(0) = fit.kmaxDirection();
            pcDiff.normals.row(0) = fit.primitiveGradient();
            std::vector<std::array<Scalar,3>> triangles;
            fit.getTriangles(triangles);
            pcDiff.init_zeros(triangles.size());
            for (int i = 0 ; i < triangles.size(); i++) {
                pcDiff.points.row(0) = VectorType(triangles[i][0], triangles[i][1], triangles[i][2]);
            }
            for (int i = 0 ; i < 3; i++) {
                pcDiff.normals.row(i) = fit.primitiveGradient();
                pcDiff.v1.row(i) = fit.kminDirection();
                pcDiff.v2.row(i) = fit.kmaxDirection();
            }
            pcDiff.setTriangles(triangles);
        }

        Scalar potential ( FitT &fit, const VectorType& pos ) override {
            return 0;
        }

}; // ProcessHandler for CNC


class BaseEstimator {
    
    public :

        virtual ~BaseEstimator() = default;
        virtual std::string getName () const = 0;
        virtual std::string toString() const = 0;
        virtual bool isOriented() const = 0;
        
        virtual void operator() ( const int &query_idx, Quantity<Scalar> &quantity ) {
            std::cout << " Only here to check. " << std::endl;
        }

        virtual void operator() ( const VectorType &query_pos, Quantity<Scalar> &quantity ) {
            std::cout << " Only here to check. " << std::endl;
        }

        virtual void customQuery( PointCloudDiff<Scalar>& pcDiff ) = 0;

        virtual Scalar potential ( const VectorType& pos ) = 0;

}; // BaseEstimator


// The Handler store the functors, called in the main loop
template < typename _FitT >
class Estimator : public BaseEstimator {
    using Self = Estimator<_FitT>;

    protected :
        bool oriented = false;
        std::string name = "None";
        int mls_max = 1;

        _FitT end_fit;
        bool isFinished = false;

    public : 
        // Overload the FitT
        using FitT = _FitT;

        using WeightFunc = typename FitT::WeightFunction;

        Estimator() = default;
        Estimator(std::string name, bool oriented, int mls_max) : name(name), oriented(oriented), mls_max(mls_max) {}

        std::string getName () const override {
            return name;
        }

        std::string toString() const override {
            return name;
        }

        bool isOriented() const override {
            return oriented;
        }

        void operator() ( const int &query_idx, Quantity<Scalar> &quantity ) override {
            
            ProcessHandler<FitT> pHandler( name );

            int mls_current = 0; 

            // VectorType query_pos = ponca_kdtree.point_data()[ query_idx ].pos();
            pHandler.initHandler ( query_idx );

            for ( mls_current = 0 ; mls_current < mls_max ; mls_current ++ ){
                FitT fit;
                pHandler.initEstimator( fit );

                Ponca::FIT_RESULT res;
                do {
                    pHandler.applyNeighbors( fit );                
                    res = pHandler.finalize( fit );
                } while ( res == Ponca::NEED_OTHER_PASS );

                if ( mls_current == mls_max - 1 ){
                    pHandler.applyFunctor( fit, quantity );
                    isFinished = true;
                    end_fit = fit;
                }
            }
        }

        void operator() ( const VectorType &query_pos, Quantity<Scalar> &quantity ) override {

            ProcessHandler<FitT> pHandler( name );

            int mls_current = 0;

            // VectorType query_pos = ponca_kdtree.point_data()[ query_idx ].pos();
            pHandler.initHandler ( query_pos );

            for ( mls_current = 0 ; mls_current < mls_max ; mls_current ++ ){
                FitT fit;
                pHandler.initEstimator( fit );

                Ponca::FIT_RESULT res;
                do {
                    pHandler.applyNeighbors( fit );
                    res = pHandler.finalize( fit );
                } while ( res == Ponca::NEED_OTHER_PASS );

                if ( mls_current == mls_max - 1 ){
                    pHandler.applyFunctor( fit, quantity );
                    isFinished = true;
                    end_fit = fit;
                }
            }
        }

        void customQuery( PointCloudDiff<Scalar>& pcDiff ) override {
            if ( !isFinished ) return;
            ProcessHandler<FitT> pHandler( name );
            pHandler.customQuery(end_fit, pcDiff);
        }

        Scalar potential ( const VectorType& pos ) override {
            if ( !isFinished ) return 0;
            ProcessHandler<FitT> pHandler( name );
            return pHandler.potential(end_fit, pos);
        }

}; // Estimator
