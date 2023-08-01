#pragma once

#include "definitions.h"
#include "MyPointCloud.h"

#include <iostream>
#include <utility>
#include <chrono>

class PointProcessing {

    // Variables
    public:

        KdTree tree;                   /// < kdtree for nearest neighbors search

        // Options for algorithms
        int    iVertexSource  = 7;     /// < id of the selected point
        int    kNN            = 10;    /// < neighborhood size (knn)
        int    mlsIter        = 3;     /// < number of moving least squares iterations
        float NSize          = 0.25;  /// < neighborhood size (euclidean)

        // Current Point Cloud Data

    public :

        PointProcessing() {
            // Default values
            iVertexSource  = 7;
            kNN            = 10;
            mlsIter        = 3;
            NSize          = 0.25;
        }

        // Constructor
        PointProcessing(MyPointCloud &cloud) {

            // Default values
            iVertexSource  = 7;
            kNN            = 10;
            mlsIter        = 3;
            NSize          = 0.25;
        
            // build kdtree
            buildKdTree(cloud.getVertices(), cloud.getNormals(), tree);
        };

        // update the KdTree
        void update(MyPointCloud &cloud) {
            buildKdTree(cloud.getVertices(), cloud.getNormals(), tree);
        }

        // Traverse the point cloud to compute differential quantities
        template<typename FitT>
        void computeDiffQuantities(const std::string &name, MyPointCloud &cloud);

        // Compute differential quantities for a single point
        template<typename FitT>
        void computeUniquePoint(const std::string &name, MyPointCloud &cloud);

        // ColorizeKnn
        const Eigen::VectorXd colorizeKnn(MyPointCloud &cloud);

        // ColorizeEuclideanNeighborhood
        const Eigen::VectorXd colorizeEuclideanNeighborhood(MyPointCloud &cloud);

        const Eigen::VectorXd getVertexSourcePosition(){ return tree.point_data()[iVertexSource].pos(); }

private :

        template <typename Functor>
        void measureTime( const std::string &actionName, Functor F );

        // Compute differential quantities
        template<typename FitT, typename Functor>
        void processOnePoint(const int &idx, const typename FitT::WeightFunction& w, Functor f);

        template<typename FitT, typename Functor>
        void processPointCloud(const bool &unique, const typename FitT::WeightFunction& w, Functor f);

}; // class PointProcessing

#include "PointProcessing.hpp"
