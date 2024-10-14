//
// Created by larnalan on 14/10/24.
//

#ifndef IVOXEL_H
#define IVOXEL_H

#include <vector>

template <typename DataType, typename Quantity, typename Estimator>
class iVoxel
{
public:
    using VectorType = typename DataType::VectorType;
    using Scalar = typename DataType::Scalar;
    using Aabb = Eigen::AlignedBox<Scalar, VectorType::RowsAtCompileTime>;

    virtual ~iVoxel() = 0;

    virtual std::vector<int> getIndices() = 0;
    virtual Quantity getQuantity() = 0;
    virtual Estimator getEstimator() = 0;
    virtual Aabb getBoundingBox() = 0;


    virtual void setData(std::vector<DataType> data) = 0;

    // If the voxel is a grid, this will call the grid and add the data to the correct cell
    virtual void addData(const DataType& data, int i) = 0;

    virtual int getResolution() = 0;
    virtual bool isLeaf() = 0;
    virtual bool isEmpty() = 0;

};

#endif //IVOXEL_H
