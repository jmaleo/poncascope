//MIT License
//
//Copyright (c) 2021 Ponca Development Group
//
//        Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//        copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//        copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//        AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.

/// \file This file contains structures mapping Polyscope data representation to Ponca concepts
/// \author Nicolas Mellado <nmellado0@gmail.com>

#pragma once

#include "../../definitions.h"


#include <Eigen/Core>
#include <Ponca/SpatialPartitioning>


/// Map a block to a Ponca point
template <typename _Scalar>
struct BlockPointAdapter {
public:
    enum {Dim = 3};
    using Scalar     = _Scalar;
    using VectorType = Eigen::Matrix<Scalar, Dim, 1>;
    using MatrixType = Eigen::Matrix<Scalar, Dim, Dim>;

    /// \brief Map a vector as ponca Point
    PONCA_MULTIARCH inline BlockPointAdapter(VectorType v, VectorType n)
            : m_pos (v), m_nor (n) {}

    PONCA_MULTIARCH inline VectorType pos()    const { return m_pos; }
    PONCA_MULTIARCH inline VectorType normal() const { return m_nor; }

private:
    VectorType m_pos;
    VectorType m_nor;
};

template<typename KdTreeType, typename SampleMatrixType>
void
buildKdTree(const SampleMatrixType& cloudV, const SampleMatrixType& cloudN, KdTreeType& tree){
    std::vector<int> ids(cloudV.size());
    std::iota(ids.begin(), ids.end(), 0);

    using VN = std::pair<const SampleMatrixType&, const SampleMatrixType&>;

    tree.buildWithSampling(VN(cloudV, cloudN),
                           ids,
                           [](VN bufs, typename KdTreeType::PointContainer &out) {
                               int s = bufs.first.size();
                               out.reserve(s);
                               for (int i = 0; i != s; ++i)
                                   out.push_back(typename KdTreeType::DataPoint(bufs.first.row(i),
                                                                                bufs.second.row(i)));
                           });
}