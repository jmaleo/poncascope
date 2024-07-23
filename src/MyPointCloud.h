#pragma once

#include <Eigen/Dense>
#include "definitions.h"

template <typename _Scalar>
class DiffQuantities {

    using Scalar = _Scalar;

    public:

        DiffQuantities(){
            m_vertices = SampleMatrixType::Zero(0, 0);
            m_normals = SampleMatrixType::Zero(0, 0);
            m_kMinDir = SampleMatrixType::Zero(0, 0);
            m_kMaxDir = SampleMatrixType::Zero(0, 0);
            m_kMin = SampleVectorType::Zero(0);
            m_kMax = SampleVectorType::Zero(0);
            m_kMean = SampleVectorType::Zero(0);
            m_shapeIndex = SampleVectorType::Zero(0);
        }

        DiffQuantities(const SampleMatrixType &vertices, const SampleMatrixType &normals, const SampleMatrixType &kMinDir, const SampleMatrixType &kMaxDir, const SampleVectorType &kMin, const SampleVectorType &kMax, const SampleVectorType &kMean, const SampleVectorType &shapeIndex){
            m_vertices = vertices;
            m_normals = normals;
            m_kMinDir = kMinDir;
            m_kMaxDir = kMaxDir;
            m_kMin = kMin;
            m_kMax = kMax;
            m_kMean = kMean;
            m_shapeIndex = shapeIndex;
        }

        DiffQuantities(const SampleMatrixType &vertices, const SampleMatrixType &normals){
            m_vertices = vertices;
            m_normals = normals;
        }

        ~DiffQuantities(){
            m_vertices.resize(0, 0);
            m_normals.resize(0, 0);
            m_kMinDir.resize(0, 0);
            m_kMaxDir.resize(0, 0);
            m_kMin.resize(0);
            m_kMax.resize(0);
            m_kMean.resize(0);
            m_shapeIndex.resize(0);
        }

        void clear(){
            m_vertices.resize(0, 0);
            m_normals.resize(0, 0);
            m_kMinDir.resize(0, 0);
            m_kMaxDir.resize(0, 0);
            m_kMin.resize(0);
            m_kMax.resize(0);
            m_kMean.resize(0);
            m_shapeIndex.resize(0);
        }
    
        const SampleMatrixType & getVertices(){
            return m_vertices;
        }

        const SampleMatrixType & getNormals(){
            return m_normals;
        }

        const SampleMatrixType & getKMinDir(){
            return m_kMinDir;
        }

        const SampleMatrixType & getKMaxDir(){
            return m_kMaxDir;
        }

        const SampleVectorType & getKMin(){
            return m_kMin;
        }

        const SampleVectorType & getKMax(){
            return m_kMax;
        }

        const SampleVectorType & getKMean(){
            return m_kMean;
        }

        const SampleVectorType & getShapeIndex(){
            return m_shapeIndex;
        }

        void setD1(const SampleMatrixType &d1){
            m_kMinDir = d1;
        }

        void setD2(const SampleMatrixType &d2){
            m_kMaxDir = d2;
        }

        const SampleMatrixType getByName (const std::string &name){
            if (name == "Projections") return m_vertices;
            if (name == "Normals") return m_normals;
            if (name == "Min curvature direction") return m_kMinDir;
            if (name == "Max curvature direction") return m_kMaxDir;
            if (name == "Min curvature") return m_kMin;
            if (name == "Max curvature") return m_kMax;
            if (name == "Mean curvature") return m_kMean;
            if (name == "Shape index") return m_shapeIndex;
            return m_vertices;
        }

    private:

        SampleMatrixType m_vertices;
        SampleMatrixType m_normals;
        SampleMatrixType m_kMinDir;
        SampleMatrixType m_kMaxDir;
        SampleVectorType m_kMin;
        SampleVectorType m_kMax;
        SampleVectorType m_kMean;
        SampleVectorType m_shapeIndex;

}; // class DiffQuantities

template <typename _Scalar>
class MyPointCloud {

    using Scalar = _Scalar;

    public:

        MyPointCloud(){
            m_vertices = SampleMatrixType::Zero(0, 0);
            m_normals = SampleMatrixType::Zero(0, 0);
            m_size = 0;
        }

        MyPointCloud(SampleMatrixType & vertices, SampleMatrixType & normals){
            // Assert for the size of the matrix
            assert(vertices.rows() == normals.rows());
            assert(vertices.cols() == 3);
            assert(normals.cols() == 3);
            m_vertices = vertices;
            m_normals = normals;
            m_size = vertices.rows();
            updateBoundingBox();
        }

        ~MyPointCloud(){
            m_diffQuantities.clear();
            m_vertices.resize(0, 0);
            m_normals.resize(0, 0);
        }

        void clear(){
            m_diffQuantities.clear();
            m_vertices.resize(0, 0);
            m_normals.resize(0, 0);
        }

        void setDiffQuantities(const DiffQuantities<Scalar> & quantities){
            m_diffQuantities = quantities;
        }

        void setVertices(SampleMatrixType &points){
            // Assert for the size of the matrix
            assert(points.rows() == m_size);
            assert(points.cols() == 3);
            m_vertices = points;
            updateBoundingBox();
        }

        void setNormals(SampleMatrixType &normals){
            // Assert for the size of the matrix
            assert(normals.rows() == m_size);
            assert(normals.cols() == 3);
            m_normals = normals;
        }

        DiffQuantities<Scalar> & getDiffQuantities(){
            return m_diffQuantities;
        }

        void setTriangles(std::vector<std::array<Scalar, 3>> &triangles){
            m_triangles = triangles;
        }

        std::vector<std::array<Scalar, 3>> & getTriangles(){
            return m_triangles;
        }

        SampleMatrixType getVertices(){
            return m_vertices;
        }

        SampleMatrixType getNormals(){
            return m_normals;
        }

        int getSize(){
            return m_size;
        }

        std::pair<SampleMatrixType, SampleVectorType> getNonZeros( SampleVectorType &values ){
            std::vector<VectorType> nonZeros;
            std::vector<Scalar> nonZerosValues;
            for (int i = 0; i < m_size; ++i) {
                if (values(i) != 0){
                    nonZeros.push_back(m_vertices.row(i));
                    nonZerosValues.push_back(values(i));
                }
            }
            // convert to matrix
            SampleMatrixType nonZerosMatrix(nonZeros.size(), 3);
            SampleVectorType nonZerosValuesVector(nonZerosValues.size());
            for (int i = 0; i < nonZeros.size(); ++i) {
                nonZerosMatrix.row(i) = nonZeros[i];
                nonZerosValuesVector(i) = nonZerosValues[i];
            }
            return std::make_pair(nonZerosMatrix, nonZerosValuesVector);
        }

        void addNoise (const Scalar &sigma_pos, const Scalar &sigma_norm){

            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<Scalar> dist_pos(0.0, sigma_pos);
            std::normal_distribution<Scalar> dist_normal(0.0, sigma_norm);

            for (int i = 0; i < m_size; ++i) {
                for (int j = 0; j < m_vertices.cols(); ++j) {
                    m_vertices(i, j) += dist_pos(gen);
                    m_normals(i, j) += dist_normal(gen);
                }
            }
        }

        SampleVectorType getMin(){
            return m_min;
        }

        SampleVectorType getMax(){
            return m_max;
        }

    private:

        SampleMatrixType m_vertices;
        SampleMatrixType m_normals;
        DiffQuantities<Scalar>  m_diffQuantities;
        int m_size;

        SampleVectorType m_min;
        SampleVectorType m_max;

        std::vector<std::array<Scalar, 3>> m_triangles;


        void updateBoundingBox(){
            m_min = m_vertices.colwise().minCoeff();
            m_max = m_vertices.colwise().maxCoeff();
        }

}; // class MyPointCloud
