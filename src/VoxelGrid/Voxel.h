//
// Created by larnalan on 14/10/24.
//

#ifndef VOXEL_H
#define VOXEL_H


#include "iVoxel.h"

template<typename DataType, typename Quantity, typename Estimator>
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

    explicit Voxel(Aabb bbox) : boundingBox(bbox) { quantity = Quantity(); }

    std::vector<int> getIndices() override { return indices; }

    Quantity getQuantity() override { return quantity; }

    Estimator getEstimator() override { return estimator; }

    Aabb getBoundingBox() override { return boundingBox; }

    int getResolution() override { return resolution; }

    VectorType getCenter() override {
        if (isEmpty())
            return boundingBox.center();
        return quantity.barycenter;
    }

    void setData(const std::vector<DataType> &data) override { this->data = data; }

    /**
     * [TODO] Implement the use of Estimator
     */
    void addData(const DataType &data, const int i) override {
        this->indices.push_back(i);
        this->data.push_back(data);
        // At the end, we only add the points' data, and use a "compute" function that compute the quantity using the
        // estimator
        this->quantity += data;
    }

    /**
     * [TODO] Implement the use of Estimator
     */
    void computeData() override {
        if (isEmpty())
            return;
        quantity /= indices.size();
    }

    bool isLeaf() override { return leaf; }

    bool isEmpty() override { return indices.empty(); }

    ~Voxel() override = default;
};

#endif // VOXEL_H
