#pragma once

#include "definitions.h"
#include "ponca_estimators/estimators.h"
#include "IO/PointCloudDiff.h"

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

public:
    PointProcessing() {
        // Default values
        use_kNNGraph = false; /// < use k-neighbor graph instead of kdtree
        kNN_for_graph = 6; /// < neighborhood size (knn) for the graph
        iVertexSource = 7; /// < id of the selected point
        kNN = 10; /// < neighborhood size (knn)
        mls_iter = 3; /// < number of moving least squares iterations
        radius = 0.25; /// < neighborhood size (euclidean)

        use_VoxelGrid = false; /// < use voxel grid instead of kdtree
        N_voxels = 10; /// < number of cells in each direction
        resolution = 1; /// < resolution of the voxel grid

        researchType = 1;
    }

    // Constructor
    explicit PointProcessing(PointCloudDiff<Scalar> &cloud) {
        // Default values
        use_kNNGraph = false; /// < use k-neighbor graph instead of kdtree
        kNN_for_graph = 6; /// < neighborhood size (knn) for the graph
        iVertexSource = 7; /// < id of the selected point
        kNN = 10; /// < neighborhood size (knn)
        mls_iter = 3; /// < number of moving least squares iterations
        radius = 0.25; /// < neighborhood size (euclidean)

        use_VoxelGrid = false; /// < use voxel grid instead of kdtree
        N_voxels = 10; /// < number of cells in each direction
        resolution = 1; /// < resolution of the voxel grid

        researchType = 1;

        // build kdtree and knn graph
        update(cloud);
    };

    // update the KdTree
    void update(PointCloudDiff<Scalar> &cloud) {
        measureTime("[Ponca] Build KdTree",
                    [this, &cloud]() { buildKdTree(cloud.points, cloud.normals, ponca_kdtree); });
        recomputeKnnGraph();
    };

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
    /// @tparam WeightFunc Fit Type, \see definitions.h
    /// @param name Name of the method, to be displayed in the console
    /// @param cloud Point Cloud to process
    template<typename WeightFunc>
    static void computeDiffQuantities(const std::string &name, PointCloudDiff<Scalar> &cloud);

    /// @brief Compute differential quantities for a single point
    /// @tparam WeightFunc Fit Type, \see definitions.h
    /// @param name Name of the method, to be displayed in the console
    /// @param cloud Point Cloud to process
    template<typename WeightFunc>
    void computeUniquePoint(const std::string &name, PointCloudDiff<Scalar> &cloud);

    /// @brief Evaluate scalar field for generit FitType
    /// @tparam WeightFunc Fit Type, \see definitions.h
    template<typename WeightFunc, bool isSigned = true>
    SampleVectorType getScalarField(const std::string &name, const SampleMatrixType &input_pos);


    /// Dry run: loop over all vertices + run MLS loops without computation
    /// This function is useful to monitor the KdTree performances
    /// And to compute the mean number of neighbors
    void mlsDryRun();

    /// Colorize point cloud using euclidean distance
    template<typename WeightFunc>
    SampleVectorType colorizeNeighbors();

    template<typename WeightFunc>
    SampleVectorType colorizeNeighbors(const std::vector<int>& indices, const std::vector<float>& radii);


    VectorType getVertexSourcePosition(){
        return getSourcePosition(iVertexSource);
    }

    Scalar getMeanNeighbors() { return m_meanNeighbors; };

private:

    Scalar m_meanNeighbors = 0;

}; // class PointProcessing

#include "PointProcessing.hpp"
