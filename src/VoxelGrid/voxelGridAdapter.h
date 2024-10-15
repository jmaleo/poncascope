//
// Created by larnalan on 14/10/24.
//

#ifndef VOXELGRIDADAPTER_H
#define VOXELGRIDADAPTER_H


#include <array>
#include <memory>
#include <Eigen/Geometry>
#include "iVoxel.h"
#include "Voxel.h"
#include "VoxelGrid.h"

struct Quantity {
    int test = 0; // just for testing
};

// Be carefull when using std::vector or Eigen::Matrix
template <typename SampleMatrixType, typename VoxelGridType, typename DataType>
void
buildVoxelGrid (const SampleMatrixType& pos, const SampleMatrixType& norm, VoxelGridType& voxelGrid, int resolution, int N) {
    using VectorType = typename DataType::VectorType;
    using Aabb = typename VoxelGridType::Aabb;

    VectorType min_bbox, max_bbox;
    Aabb bbox = Aabb(pos.colwise().minCoeff(), pos.colwise().maxCoeff());
    voxelGrid.init(bbox, N, resolution);
    for (int i = 0; i < pos.size(); i++) {

        DataType data(pos.row(i).transpose(), norm.row(i).transpose());
        voxelGrid.addData(data, i);
    }

}

#endif //VOXELGRIDADAPTER_H
