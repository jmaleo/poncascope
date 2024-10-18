//
// Created by larnalan on 14/10/24.
//

#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <array>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Voxel.h"
#include "iVoxel.h"


/**
 * It stores in leaf's list the indices of the points that are in the cell
 * @tparam DataType
 * @tparam Quantity
 * @tparam Estimator
 */
template<typename DataType, typename Quantity, typename Estimator>
class VoxelGrid : public iVoxel<DataType, Quantity, Estimator> {
public:
    using VectorType = typename DataType::VectorType;
    using Scalar = typename DataType::Scalar;
    using Aabb = Eigen::AlignedBox<Scalar, VectorType::RowsAtCompileTime>;
    using VoxelType = Voxel<DataType, Quantity, Estimator>;
    using VoxelGridType = VoxelGrid<DataType, Quantity, Estimator>;

private:
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
    ~VoxelGrid() override = default;

    VoxelGrid() = default;

    VoxelGrid(const Aabb &bbox, const int N, const int res) { init(bbox, N, res); }

    VoxelGrid(const VoxelGrid &) = delete;
    VoxelGrid &operator=(const VoxelGrid &) = delete;

    VoxelGrid(VoxelGrid &&other) noexcept :
        children(std::move(other.children)), indices(std::move(other.indices)), data(std::move(other.data)),
        boundingBox(std::move(other.boundingBox)), N(other.N), quantity(std::move(other.quantity)),
        resolution(other.resolution), leaf(other.leaf) {}

    VoxelGrid &operator=(VoxelGrid &&other) noexcept {
        if (this != &other) {
            children = std::move(other.children);
            indices = std::move(other.indices);
            data = std::move(other.data);
            boundingBox = std::move(other.boundingBox);
            N = other.N;
            quantity = std::move(other.quantity);
            resolution = other.resolution;
            leaf = other.leaf;
        }
        return *this;
    }

    void setData(const std::vector<DataType> &data) override { std::cout << "setData" << std::endl; }

    int getResolution() override { return resolution; }

    bool isLeaf() override { return leaf; }

    bool isEmpty() override {
        for (const std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell: children) {
            if (!cell->isEmpty())
                return false;
        }
        return true;
    }

    Quantity getQuantity() override { return quantity; }

    Estimator getEstimator() override { return Estimator(); }

    std::vector<int> getIndices() override {
        std::vector<int> idx;
        for (const std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell: children) {
            std::vector<int> childIdx = cell->getIndices();
            idx.insert(idx.end(), childIdx.begin(), childIdx.end());
        }
        return idx;
    }

    /**
     * @param bbox The bounding box of the grid
     * @param N For instance, we use the N as default and the resolution as 1,
     *         the bounding box will be divided into N * N * N cells, and each cell will be a Voxel
     *         But, at the end, we may imagine to be more detailed with a resolution of 2, 3, 4, etc.
     * @param res The resolution of the grid (depth of the tree)
     */
    void init(Aabb bbox, const int N, const int res) {
        this->boundingBox = bbox;
        this->N = N;
        this->resolution = res;
        this->leaf = res == 0;
        if (!leaf) {
            children.reserve(N * N * N);
            for (int ix = 0; ix < N; ix++) {
                for (int iy = 0; iy < N; iy++) {
                    for (int iz = 0; iz < N; iz++) {
                        initializeChild(ix, iy, iz);
                    }
                }
            }
        }
    }

    void initializeChild(int ix, int iy, int iz) {
        int index = ix + N * (iy + N * iz);
        if (children.size() <= index) {
            children.resize(index + 1);
        }
        if (!children[index]) {
            Aabb childBbox = getCellBoundingBox(ix, iy, iz);
            if (resolution == 1) {
                children[index] = std::make_unique<VoxelType>(childBbox);
            } else {
                children[index] = std::make_unique<VoxelGridType>(childBbox, N, resolution - 1);
            }
        }
    }

    iVoxel<DataType, Quantity, Estimator> *getChild(int index) {
        if (index >= N * N * N)
            return nullptr;
        return children[index].get();
    }

    void addData(const DataType &data, const int i) override {
        // Grab position
        const VectorType &pos = data.pos();
        // Get the cell index
        const int cellIdx = getCellIdx(pos);

        iVoxel<DataType, Quantity, Estimator> *cell = getChild(cellIdx);
        if (cell) {
            cell->addData(data, i);
        }
    }

    void computeData() override {
        quantity = Quantity();
        int count = 0;
        for (std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell: children) {
            cell->computeData();
            if (cell->isEmpty())
                continue;
            count += 1;
            quantity += cell->getQuantity();
        }
        quantity /= count;
    }

    Aabb getBoundingBox() override { return boundingBox; }

    // For debug only, get the Aabb of each cell, given a resolution
    std::vector<Aabb> getCellBoundingBoxes(int res, bool onlyNotEmpty = false) {
        std::vector<Aabb> cellBoundingBoxes;
        for (int cellIdx = 0; cellIdx < N * N * N; cellIdx++) {
            std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell = children[cellIdx];
            if (res == cell->getResolution()) {
                if (onlyNotEmpty && cell->isEmpty()) {
                    continue;
                }
                cellBoundingBoxes.push_back(cell->getBoundingBox());
            } else {
                std::vector<Aabb> childCellBoundingBoxes =
                        static_cast<VoxelGrid *>(cell.get())->getCellBoundingBoxes(res, onlyNotEmpty);
                cellBoundingBoxes.insert(cellBoundingBoxes.end(), childCellBoundingBoxes.begin(),
                                         childCellBoundingBoxes.end());
            }
        }
        return cellBoundingBoxes;
    }

    VectorType getCenter() override {
        if (isEmpty())
            return boundingBox.center();
        return quantity.barycenter;
    }

    std::vector<VectorType> getCellCenters(int res, bool onlyNotEmpty = false) {
        std::vector<VectorType> cellCenters;
        for (int cellIdx = 0; cellIdx < N * N * N; cellIdx++) {
            std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell = children[cellIdx];
            if (res == cell->getResolution()) {
                if (onlyNotEmpty && cell->isEmpty()) {
                    continue;
                }
                cellCenters.push_back(cell->getCenter());
            } else {
                std::vector<VectorType> childCellCenters =
                        static_cast<VoxelGrid *>(cell.get())->getCellCenters(res, onlyNotEmpty);
                cellCenters.insert(cellCenters.end(), childCellCenters.begin(), childCellCenters.end());
            }
        }
        return cellCenters;
    }

private:
    // Convert a position to a cell index
    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
    int getCellIdx(const VectorType &pos) {
        int iX = std::floor((pos.x() - boundingBox.min().x()) / (boundingBox.max().x() - boundingBox.min().x()) * N);
        int iY = std::floor((pos.y() - boundingBox.min().y()) / (boundingBox.max().y() - boundingBox.min().y()) * N);
        int iZ = std::floor((pos.z() - boundingBox.min().z()) / (boundingBox.max().z() - boundingBox.min().z()) * N);
        return iX + N * (iY + N * iZ);
    }

    // Get Cell using an index
    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
    std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> getCell(const int &idx) {
        const std::unique_ptr<iVoxel<DataType, Quantity, Estimator>> &cell = children[idx];
        return cell;
    }

    // [TODO] Need to be careful with the indices (do we need to use floor or ceil ?)
    Aabb getCellBoundingBox(const int ix, const int iy, const int iz) {
        const Scalar dx = (boundingBox.max().x() - boundingBox.min().x()) / N;
        const Scalar dy = (boundingBox.max().y() - boundingBox.min().y()) / N;
        const Scalar dz = (boundingBox.max().z() - boundingBox.min().z()) / N;
        return Aabb(VectorType(boundingBox.min().x() + ix * dx, boundingBox.min().y() + iy * dy,
                               boundingBox.min().z() + iz * dz),
                    VectorType(boundingBox.min().x() + (ix + 1) * dx, boundingBox.min().y() + (iy + 1) * dy,
                               boundingBox.min().z() + (iz + 1) * dz));
    }
};

#endif // VOXELGRID_H
