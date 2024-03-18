#pragma once

#include "MyPointCloud.h"
#include "definitions.h"

#include <iostream>
#include <utility>
#include <chrono>
#include <random>

class PointProcessing {

    // Variables
    public:

        KdTree tree;                   /// < kdtree for nearest neighbors search
        MlodsTree<basket_FullyOrientedEllipsoid2DFit<ConstWeightFunc>> mlodsTree;           /// < kdtree for MLoDs for nearest neighbors search
        KnnGraph *knnGraph = nullptr;  /// < k-neighbor graph

        // Options for algorithms
        bool  useKnnGraph    = false; /// < use k-neighbor graph instead of kdtree
        int   kNN_for_graph  = 6;     /// < neighborhood size (knn) for the graph
        int   iVertexSource  = 7;     /// < id of the selected point
        int   kNN            = 10;    /// < neighborhood size (knn)
        int   mlsIter        = 3;     /// < number of moving least squares iterations
        float NSize          = 0.25;  /// < neighborhood size (euclidean)

        int researchType = 1; // 0 : k Nearest Neighbors, 1 : Euclidian Nearest Neighbors

    public :

        PointProcessing() {
            // Default values
            useKnnGraph    = false;
            kNN_for_graph  = 6;
            iVertexSource  = 7;
            kNN            = 10;
            mlsIter        = 3;
            NSize          = 0.25;

            researchType = 1;
        }

        // Constructor
        PointProcessing(MyPointCloud<Scalar> &cloud) {

            // Default values
            useKnnGraph    = false;
            kNN_for_graph  = 6;
            iVertexSource  = 7;
            kNN            = 10;
            mlsIter        = 3;
            NSize          = 0.25;

            researchType = 1;

            // build kdtree and knn graph
            update(cloud);
        };

        // update the KdTree
        void update(MyPointCloud<Scalar> &cloud) {
            measureTime( "[Ponca] Build KdTree",
                 [this, &cloud]() {
                    buildKdTree(cloud.getVertices(), cloud.getNormals(), tree);
                });
            recomputeKnnGraph();

            // build MLoDs
            measureTime( "[Ponca] Build MLoDs",
                 [this, &cloud]() {
                    buildKdTree(cloud.getVertices(), cloud.getNormals(), mlodsTree);
                });
        }

        /// @brief Measure time of a function
        /// @tparam Functor type of the function
        /// @param actionName name of the action 
        /// @param F function to measure
        template <typename Functor>
        void measureTime( const std::string &actionName, Functor F );
        
        /// Compute the kNN graph
        void recomputeKnnGraph();

        /// @brief Compute differential quantities
        /// @tparam FitT Fit Type, \see definitions.h
        /// @param name Name of the method, to be displayed in the console
        /// @param cloud Point Cloud to process
        template<typename FitT, bool isSigned = true>
        void computeDiffQuantities(const std::string &name, MyPointCloud<Scalar> &cloud);

        /// @brief Compute differential quantities
        /// @tparam FitT Fit Type, \see definitions.h
        /// @param name Name of the method, to be displayed in the console
        /// @param cloud Point Cloud to process
        void computeDiffQuantities_Triangle(const std::string &name, const int& type, MyPointCloud<Scalar> &cloud);

        /// @brief Compute differential quantities for a single point
        /// @tparam FitT Fit Type, \see definitions.h
        /// @param name Name of the method, to be displayed in the console
        /// @param cloud Point Cloud to process
        template<typename FitT>
        void computeUniquePoint(const std::string &name, MyPointCloud<Scalar> &cloud);

        /// @brief Compute triangle mesh with CNC algorithm
        /// @param name Name of the method, to be displayed in the console
        void computeUniquePoint_triangle(const std::string &name, const int& type, MyPointCloud<Scalar> &cloud);

        /// @brief Evaluate scalar field for generit FitType
        /// @tparam FitT Fit Type, \see definitions.h
        template<typename FitT, bool isSigned = true>
        SampleVectorType evalScalarField_impl(const std::string &name, const SampleMatrixType& input_pos);


        /// Dry run: loop over all vertices + run MLS loops without computation
        /// This function is useful to monitor the KdTree performances
        /// And to compute the mean number of neighbors
        void mlsDryRun();

        /// Colorize point cloud using kNN
        const SampleVectorType colorizeKnn();

        /// Colorize point cloud using euclidean distance
        template <typename WeightFunc>
        const SampleVectorType colorizeEuclideanNeighborhood();

        const SampleVectorType getVertexSourcePosition(){ return tree.point_data()[iVertexSource].pos(); }

        Scalar getMeanNeighbors() { return m_meanNeighbors; }

        //////////////////////
        // test : 
        //////////////////////

        template<typename WeightFunc>
        void computeUniquePoint_aggregation(const std::string &name, MyPointCloud<Scalar> &cloud, int nbNei);

        // Using to test mlodsTree
        const SampleVectorType colorizeCell(int cellIdx);

        // Using to test the cell computation
        Eigen::AlignedBox<Scalar, 3> computeCell(MyPointCloud<Scalar> &cloud, int cellIdx);

private :

        Scalar m_meanNeighbors = 0;

        // /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial data structure
        // /// @tparam Functor type of the function
        // /// @param idx index of the point to process
        // /// @param f function to apply on the neighbors
        // template <typename Functor>
        // void processRangeNeighbors(const int &idx, const Functor f);

        // /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial data structure
        // /// @tparam Functor type of the function
        // /// @param pos pos of the point to process
        // /// @param f function to apply on the neighbors
        // template <typename Functor>
        // void processRangeNeighbors(const VectorType &pos, const Functor f);

        /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial data structure
        /// @tparam Functor type of the function
        /// @param pos pos of the point to process
        /// @param f function to apply on the neighbors
        template <typename Functor>
        void processNeighbors(const int &idx, const Functor f);

        /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial data structure
        /// @tparam Functor type of the function
        /// @param idx index of the point to process
        /// @param f function to apply on the neighbors
        template <typename Functor>
        void processNeighbors(const VectorType &pos, const Functor f);

        /// @brief Process one point, compute fitting, and use functor to process fitting output
        /// @tparam FitT Fit Type, \see definitions.h
        /// @tparam Functor type of the function
        /// @param idx index of the point to process
        /// @param w weight function
        /// @param f function to apply on the fitting
        template<typename FitT, typename Functor>
        void processOnePoint(const int &idx, const typename FitT::WeightFunction& w, Functor f);

        /// @brief Process one point, compute fitting, and use functor to process fitting output
        /// @tparam FitT Fit Type, \see definitions.h
        /// @tparam Functor type of the function
        /// @param pos pos of the point to process
        /// @param w weight function
        /// @param f function to apply on the fitting
        template<typename FitT, typename Functor>
        void processOnePoint(const VectorType &init_pos, const typename FitT::WeightFunction& w, Functor f);

        /// @brief Process one point for CNC methods, compute fitting, and use functor to process fitting output
        /// \see definitions.h
        /// @tparam Functor type of the function
        /// @param idx index of the point to process
        /// @param w weight function
        /// @param f function to apply on the fitting
        template <typename Functor>
        void processOnePoint_Triangle(const int& idx, const int& type, Functor f);

        /// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
        /// \note Functor is called only if fit is stable
        /// @tparam FitT Fit Type, \see definitions.h
        /// @tparam Functor type of the function
        /// @param w weight function
        /// @param f function to apply on the fitting
        template<typename FitT, typename Functor>
        void processPointCloud(const bool &unique, const typename FitT::WeightFunction& w, Functor f);

        template<typename Functor>
        void processPointCloud_Triangle(const bool &unique, const int& type, Functor f);

        // /// @brief Used to compute the normal of a single point and to avoid compilation errors
        // /// @tparam FitT Fit Type, \see definitions.h
        // /// @param idx index of the point to process
        // /// @param fit fitting method
        // /// @param init initial point
        // /// @param normal output normals
        // template<typename FitT, typename WeightFunc>
        // void
        // processPointUniqueNormal(const int &idx, const FitT& fit, const VectorType& init, SampleMatrixType& normal);
        

        /// @brief Used to init the way to use computeCurvature into the fit / to dissociate classic fits and plane fits
        template<typename FitT>
        void initUseNormal (FitT &fit);
        
}; // class PointProcessing

#include "PointProcessing.hpp"
