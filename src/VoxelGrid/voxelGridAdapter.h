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

template <typename SampleMatrixType, typename VoxelGridType, typename DataType>
void
buildVoxelGrid (const SampleMatrixType& pos, const SampleMatrixType& norm, VoxelGridType& voxelGrid, int resolution, int N) {
    using Aabb = typename VoxelGridType::Aabb;
    Aabb bbox;
    for (int i = 0; i < pos.size(); i++) {
        bbox.extend(pos[i]);
    }
    voxelGrid.init(bbox, N, resolution);
    for (int i = 0; i < pos.size(); i++) {
        DataType data(pos[i], norm[i]);
        voxelGrid.addData(data, i);
    }

}

#endif //VOXELGRIDADAPTER_H
