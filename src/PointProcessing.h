#pragma once

#include "MyPointCloud.h"
#include "definitions.h"

#include <chrono>
#include <iostream>
#include <random>
#include <utility>

#include "ponca_estimators/adapters/declarations.h"

class PointProcessing {

    // Variables
public:
    // KdTree ponca_kdtree; /// < kdtree for nearest neighbors search
    // KnnGraph *ponca_knnGraph = nullptr; /// < k-neighbor graph
    // MyVoxelGrid voxelGrid; /// < voxel grid

    // Options for algorithms
    use_kNNGraph = false; /// < use k-neighbor graph instead of kdtree
    kNN_for_graph = 6; /// < neighborhood size (knn) for the graph
    iVertexSource = 7; /// < id of the selected point
    kNN = 10; /// < neighborhood size (knn)
    mls_iter = 3; /// < number of moving least squares iterations
    radius = 0.25; /// < neighborhood size (euclidean)

    use_VoxelGrid = false; /// < use voxel grid instead of kdtree
    N_voxels = 10; /// < number of cells in each direction
    resolution = 1; /// < resolution of the voxel grid


    int researchType = 1; // 0 : k Nearest Neighbors, 1 : Euclidian Nearest Neighbors

public:
    PointProcessing() {
        // Default values
        use_kNNGraph = false;
        kNN_for_graph = 6;
        iVertexSource = 7;
        kNN = 10;
        mls_iter = 3;
        radius = 0.25;

        researchType = 1;
    }

    // Constructor
    PointProcessing(PointCloudDiff<Scalar> &cloud) {

        // Default values
        use_kNNGraph = false;
        kNN_for_graph = 6;
        iVertexSource = 7;
        kNN = 10;
        mls_iter = 3;
        radius = 0.25;

        researchType = 1;

        // build kdtree and knn graph
        update(cloud);
    };

    // update the KdTree
    void update(PointCloudDiff<Scalar> &cloud) {
        measureTime("[Ponca] Build KdTree",
                    [this, &cloud]() { buildKdTree(cloud.points, cloud.normals, ponca_kdtree); });
        recomputeKnnGraph();
    }

    /// @brief Measure time of a function
    /// @tparam Functor type of the function
    /// @param actionName name of the action
    /// @param F function to measure
    template<typename Functor>
    void measureTime(const std::string &actionName, Functor F);

    /// Compute the kNN graph
    void recomputeKnnGraph();

    /// @brief Compute the VoxelGrid
    void computeVoxelGrid(PointCloudDiff<Scalar> &cloud);

    /// @brief Compute differential quantities
    /// @tparam FitT Fit Type, \see definitions.h
    /// @param name Name of the method, to be displayed in the console
    /// @param cloud Point Cloud to process
    template<typename FitT, bool isSigned = true>
    void computeDiffQuantities(const std::string &name, PointCloudDiff<Scalar> &cloud);

    /// @brief Compute differential quantities
    /// @tparam FitT Fit Type, \see definitions.h
    /// @param name Name of the method, to be displayed in the console
    /// @param cloud Point Cloud to process
    void computeDiffQuantities_Triangle(const std::string &name, const int &type, PointCloudDiff<Scalar> &cloud);

    /// @brief Compute differential quantities for a single point
    /// @tparam FitT Fit Type, \see definitions.h
    /// @param name Name of the method, to be displayed in the console
    /// @param cloud Point Cloud to process
    template<typename FitT>
    void computeUniquePoint(const std::string &name, PointCloudDiff<Scalar> &cloud);

    /// @brief Compute triangle mesh with CNC algorithm
    /// @param name Name of the method, to be displayed in the console
    void computeUniquePoint_triangle(const std::string &name, const int &type, PointCloudDiff<Scalar> &cloud);

    /// @brief Evaluate scalar field for generit FitType
    /// @tparam FitT Fit Type, \see definitions.h
    template<typename FitT, bool isSigned = true>
    SampleVectorType evalScalarField_impl(const std::string &name, const SampleMatrixType &input_pos);


    /// Dry run: loop over all vertices + run MLS loops without computation
    /// This function is useful to monitor the KdTree performances
    /// And to compute the mean number of neighbors
    void mlsDryRun();

    /// Colorize point cloud using kNN
    const SampleVectorType colorizeKnn();

    /// Colorize point cloud using euclidean distance
    template<typename WeightFunc>
    const SampleVectorType colorizeEuclideanNeighborhood();

    /// Colorize point cloud using euclidean distance for many queries
    template<typename WeightFunc>
    const SampleVectorType colorizeEuclideanNeighborhood(const std::vector<int> &vertexQueries,
                                                         const std::vector<float> &radii);

    const SampleVectorType getVertexSourcePosition() { return ponca_kdtree.point_data()[iVertexSource].pos(); }

    Scalar getMeanNeighbors() { return m_meanNeighbors; }

private:
    Scalar m_meanNeighbors = 0;

    // /// @brief Loop over all points to compute differential quantities, the method differ depending on the used
    // spatial data structure
    // /// @tparam Functor type of the function
    // /// @param idx index of the point to process
    // /// @param f function to apply on the neighbors
    // template <typename Functor>
    // void processRangeNeighbors(const int &idx, const Functor f);

    // /// @brief Loop over all points to compute differential quantities, the method differ depending on the used
    // spatial data structure
    // /// @tparam Functor type of the function
    // /// @param pos pos of the point to process
    // /// @param f function to apply on the neighbors
    // template <typename Functor>
    // void processRangeNeighbors(const VectorType &pos, const Functor f);

    /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial
    /// data structure
    /// @tparam Functor type of the function
    /// @param pos pos of the point to process
    /// @param f function to apply on the neighbors
    template<typename Functor>
    void processNeighbors(const int &idx, const Functor f);

    /// @brief Loop over all points to compute differential quantities, the method differ depending on the used spatial
    /// data structure
    /// @tparam Functor type of the function
    /// @param idx index of the point to process
    /// @param f function to apply on the neighbors
    template<typename Functor>
    void processNeighbors(const VectorType &pos, const Functor f);

    /// @brief Process one point, compute fitting, and use functor to process fitting output
    /// @tparam FitT Fit Type, \see definitions.h
    /// @tparam Functor type of the function
    /// @param idx index of the point to process
    /// @param w weight function
    /// @param f function to apply on the fitting
    template<typename FitT, typename Functor>
    void processOnePoint(const int &idx, const typename FitT::WeightFunction &w, Functor f);

    /// @brief Process one point, compute fitting, and use functor to process fitting output
    /// @tparam FitT Fit Type, \see definitions.h
    /// @tparam Functor type of the function
    /// @param pos pos of the point to process
    /// @param w weight function
    /// @param f function to apply on the fitting
    template<typename FitT, typename Functor>
    void processOnePoint(const VectorType &init_pos, const typename FitT::WeightFunction &w, Functor f);

    /// @brief Process one point for CNC methods, compute fitting, and use functor to process fitting output
    /// \see definitions.h
    /// @tparam Functor type of the function
    /// @param idx index of the point to process
    /// @param w weight function
    /// @param f function to apply on the fitting
    template<typename Functor>
    void processOnePoint_Triangle(const int &idx, const int &type, Functor f);

    /// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
    /// \note Functor is called only if fit is stable
    /// @tparam FitT Fit Type, \see definitions.h
    /// @tparam Functor type of the function
    /// @param w weight function
    /// @param f function to apply on the fitting
    template<typename FitT, typename Functor>
    void processPointCloud(const bool &unique, const typename FitT::WeightFunction &w, Functor f);

    template<typename Functor>
    void processPointCloud_Triangle(const bool &unique, const int &type, Functor f);

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
    void initUseNormal(FitT &fit);

}; // class PointProcessing

#include "PointProcessing.hpp"
