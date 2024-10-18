//
// Created by larnalan on 14/10/24.
//

#ifndef VOXELGRIDADAPTER_H
#define VOXELGRIDADAPTER_H


#include <Eigen/Geometry>
#include <array>
#include <memory>
#include "Voxel.h"
#include "VoxelGrid.h"
#include "iVoxel.h"

template<typename DataType>
class Quantity {

private:
    using VectorType = typename DataType::VectorType;
    using Scalar = typename DataType::Scalar;

    int N = 0;

public:
    VectorType barycenter;

public:
    Quantity() { barycenter = VectorType::Zero(); }

    Quantity &operator+=(const DataType &data) {
        barycenter += data.pos();
        return *this;
    }

    Quantity &operator/=(const Scalar &divisor) {
        barycenter /= divisor;
        return *this;
    }

    Quantity &operator+=(const Quantity &other) {
        barycenter += other.barycenter;
        return *this;
    }

    Quantity operator+(const Quantity &other) const {
        Quantity result = *this;
        result += other;
        return result;
    }
};

template<typename Scalar, typename VectorType>
std::pair<VectorType, VectorType> getMinMaxAsCube(const VectorType &min, const VectorType &max) {
    Scalar width = (max.x() - min.x());
    Scalar height = (max.y() - min.y());
    Scalar depth = (max.z() - min.z());

    Scalar maxDimension = std::max({width, height, depth});
    Scalar dx = (maxDimension - width) / 2;
    Scalar dy = (maxDimension - height) / 2;
    Scalar dz = (maxDimension - depth) / 2;
    VectorType min_bbox(min.x() - dx, min.y() - dy, min.z() - dz);
    VectorType max_bbox(max.x() + dx, max.y() + dy, max.z() + dz);
    return std::make_pair(min_bbox, max_bbox);
}

// Be carefull when using std::vector or Eigen::Matrix
template<typename SampleMatrixType, typename VoxelGridType, typename DataType>
void buildVoxelGrid(const SampleMatrixType &pos, const SampleMatrixType &norm, VoxelGridType &voxelGrid, int resolution,
                    int N) {
    using VectorType = typename DataType::VectorType;
    using Aabb = typename VoxelGridType::Aabb;
    using Scalar = typename DataType::Scalar;

    VectorType min_bbox, max_bbox;
    min_bbox = pos.colwise().minCoeff();
    max_bbox = pos.colwise().maxCoeff();
    auto minmax = getMinMaxAsCube<Scalar, VectorType>(min_bbox, max_bbox);
    Aabb bbox(minmax.first, minmax.second);
    voxelGrid.init(bbox, N, resolution);
    for (int i = 0; i < pos.rows(); i++) {
        DataType data(pos.row(i).transpose(), norm.row(i).transpose());
        voxelGrid.addData(data, i);
    }
    voxelGrid.computeData();
}

#endif // VOXELGRIDADAPTER_H
