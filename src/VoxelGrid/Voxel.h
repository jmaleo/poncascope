//
// Created by larnalan on 14/10/24.
//

#ifndef VOXEL_H
#define VOXEL_H


#include "iVoxel.h"

template <typename DataType, typename Quantity, typename Estimator>
class Voxel : public iVoxel<DataType, Quantity, Estimator> {
    using VectorType = typename DataType::VectorType;
    using Scalar = typename DataType::Scalar;
    using Aabb = Eigen::AlignedBox<Scalar, VectorType::RowsAtCompileTime>;

private:
    std::vector<int> indices;
    Aabb boundingBox;
    Quantity quantity;
    Estimator estimator;
    bool leaf = true;
    int resolution = 0;
    std::vector<DataType> data;

public:
    Voxel() = default;

    explicit Voxel (Aabb bbox) : boundingBox(bbox) {}

    std::vector<int> getIndices() override {
        return indices;
    }

    Quantity getQuantity() override {
        return quantity;
    }

    Estimator getEstimator() override {
        return estimator;
    }

    Aabb getBoundingBox() override {
        return boundingBox;
    }

    int getResolution() override {
        return resolution;
    }

    void setData(const std::vector<DataType>& data) override {
        this->data = data;
    }

    void addData(const DataType& data, const int i) override {
        this->indices.push_back(i);
    }

    bool isLeaf() override {
        return leaf;
    }

    bool isEmpty() override {
        return indices.empty();
    }

    ~Voxel() override = default;
};

#endif //VOXEL_H
