#pragma once

#include "Ponca/Fitting"
#include "Ponca/SpatialPartitioning"
#include "ponca_estimators/adapters/poncaAdapters.hpp"
#include "VoxelGrid/voxelGridAdapter.h"


// Types definition

using Scalar = double;
// using Scalar             = float;
using VectorType = Eigen::Matrix<Scalar, 3, 1>;
using PPAdapter = BlockPointAdapter<Scalar>;
using KdTree = Ponca::KdTree<PPAdapter>;
using KnnGraph = Ponca::KnnGraph<PPAdapter>;

using MyQuantity = VoxelQuantity<PPAdapter>;
using MyVoxelGrid = VoxelGrid<PPAdapter, MyQuantity, VectorType>;

typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> SampleMatrixType;
typedef Eigen::Vector<Scalar, Eigen::Dynamic> SampleVectorType;
// typedef std::vector<VectorType> SampleMatrixType;
// typedef std::vector<Scalar> SampleVectorType;
