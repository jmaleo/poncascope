//
// Created by larnalan on 14/10/24.
//

#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <array>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "iVoxel.h"
#include "Voxel.h"


/**
 * It stores in leaf's list the indices of the points that are in the cell
 * @tparam DataType
 * @tparam Quantity
 * @tparam Estimator
 */
template <typename DataType, typename Quantity, typename Estimator>
class VoxelGrid
{
private:
    using VectorType = typename DataType::VectorType;
    using Scalar = typename DataType::Scalar;
    using Aabb = Eigen::AlignedBox<Scalar, VectorType::RowsAtCompileTime>;
    using VoxelType = Voxel<DataType, Quantity, Estimator>;

    int N = 0;

    Aabb boundingBox;

    // This will be easy to get : children[0]->boundingBox
    // children[iX + N * (iY + N * iZ)]-> cell(iX, iY, iZ)
    std::vector<std::unique_ptr<iVoxel<DataType, Quantity, Estimator>>> children;
    std::vector<int> indices; // useless for now
    std::vector<DataType> data;

    Quantity quantity;
    int resolution = 0;
    bool leaf = false;

public:

    VoxelGrid () = default;

    /**
     * @param bbox The bounding box of the grid
     * @param N For instance, we use the N as default and the resolution as 1,
     *         the bounding box will be divided into N * N * N cells, and each cell will be a Voxel
     *         But, at the end, we may imagine to be more detailed with a resolution of 2, 3, 4, etc.
     * @param res The resolution of the grid (depth of the tree)
     */
    void init(Aabb bbox, const int N, const int res) : N(N), resolution(res) {
        this->leaf = resolution == 0;
        this->boundingBox = Aabb::Empty();
        children = std::vector<std::unique_ptr<iVoxel<DataType, Quantity, Estimator>>>(N * N * N);
        for (int i = 0; i < N * N * N; i++) {
            Aabb children_aabb = getCellBoundingBox(i);
            if (this->resolution == 1) {
                children[i] = std::make_unique<VoxelType>(children_aabb);
            }
            else {
                children[i] = std::make_unique<VoxelGrid>(children_aabb, N, resolution - 1);
            }
        }
    }

    void addData(const DataType& data, int i) {
        // Grab position
        const VectorType& pos = data.pos();
        // Get the cell index
        const int cellIdx = getCellIdx(pos);
        // Add the data to the cell
        children[cellIdx]->addData(data, i);
    }

    Aabb getBoundingBox() {
        return boundingBox;
    }

    // For debug only, get the Aabb of each cell, given a resolution
    std::vector<Aabb> getCellBoundingBoxes(int res) {
        std::vector<Aabb> cellBoundingBoxes;
        for (int i = 0; i < N * N * N; i++) {
            if (std::unique_ptr<iVoxel<DataType, Quantity, Estimator>>& cell = children[i]; res == cell->getResolution() ) {
                cellBoundingBoxes.push_back(cell->getBoundingBox());
            }
            else {
                std::vector<Aabb> childCellBoundingBoxes = static_cast<VoxelGrid*>(cell.get())->getCellBoundingBoxes(res);
                cellBoundingBoxes.insert(cellBoundingBoxes.end(), childCellBoundingBoxes.begin(), childCellBoundingBoxes.end());
            }
        }
        return cellBoundingBoxes;
    }

private:

    // Convert a position to a cell index
    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
    int getCellIdx (const VectorType& pos) {
        const int iX = ( (pos.x() - boundingBox.min().x()) / (boundingBox.max().x() - boundingBox.min().x()) ) * N;
        const int iY = ( (pos.y() - boundingBox.min().y()) / (boundingBox.max().y() - boundingBox.min().y()) ) * N;
        const int iZ = ( (pos.z() - boundingBox.min().z()) / (boundingBox.max().z() - boundingBox.min().z()) ) * N;
        return iX + N * (iY + N * iZ);
    }

    // Get Cell using an index
    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
     std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> getCell (const int& idx) {
        const std::unique_ptr<iVoxel<DataType, Quantity, Estimator>>& cell = children[idx];
        return cell;
    }

    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
    Aabb getCellBoundingBox (const int i) {
        const int iX = i % N;
        const int iY = (i / N) % N;
        const int iZ = i / (N * N);
        const Scalar dx = (boundingBox.max().x() - boundingBox.min().x()) / N;
        const Scalar dy = (boundingBox.max().y() - boundingBox.min().y()) / N;
        const Scalar dz = (boundingBox.max().z() - boundingBox.min().z()) / N;
        return Aabb(VectorType(boundingBox.min().x() + iX * dx, boundingBox.min().y() + iY * dy, boundingBox.min().z() + iZ * dz),
                    VectorType(boundingBox.min().x() + (iX + 1) * dx, boundingBox.min().y() + (iY + 1) * dy, boundingBox.min().z() + (iZ + 1) * dz));
    }

};

#endif //VOXELGRID_H
