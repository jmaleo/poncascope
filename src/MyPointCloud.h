#pragma once

#include <Eigen/Dense>

template <typename _Scalar>
class DiffQuantities {

    using Scalar =  _Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;
    typedef Eigen::Vector<Scalar, Eigen::Dynamic>                 VectorType;


    public:

        DiffQuantities(){
            m_vertices = MatrixType::Zero(0, 0);
            m_normals = MatrixType::Zero(0, 0);
            m_kMinDir = MatrixType::Zero(0, 0);
            m_kMaxDir = MatrixType::Zero(0, 0);
            m_kMin = VectorType::Zero(0);
            m_kMax = VectorType::Zero(0);
            m_kMean = VectorType::Zero(0);
            m_shapeIndex = VectorType::Zero(0);
        }

        DiffQuantities(const MatrixType &vertices, const MatrixType &normals, const MatrixType &kMinDir, const MatrixType &kMaxDir, const VectorType &kMin, const VectorType &kMax, const VectorType &kMean, const VectorType &shapeIndex){
            m_vertices = vertices;
            m_normals = normals;
            m_kMinDir = kMinDir;
            m_kMaxDir = kMaxDir;
            m_kMin = kMin;
            m_kMax = kMax;
            m_kMean = kMean;
            m_shapeIndex = shapeIndex;
        }

        DiffQuantities(const MatrixType &vertices, const MatrixType &normals){
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
    
        const MatrixType & getVertices(){
            return m_vertices;
        }

        const MatrixType & getNormals(){
            return m_normals;
        }

        const MatrixType & getKMinDir(){
            return m_kMinDir;
        }

        const MatrixType & getKMaxDir(){
            return m_kMaxDir;
        }

        const VectorType & getKMin(){
            return m_kMin;
        }

        const VectorType & getKMax(){
            return m_kMax;
        }

        const VectorType & getKMean(){
            return m_kMean;
        }

        const VectorType & getShapeIndex(){
            return m_shapeIndex;
        }

        const MatrixType getByName (const std::string &name){
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

        MatrixType m_vertices;
        MatrixType m_normals;
        MatrixType m_kMinDir;
        MatrixType m_kMaxDir;
        VectorType m_kMin;
        VectorType m_kMax;
        VectorType m_kMean;
        VectorType m_shapeIndex;

}; // class DiffQuantities

template <typename _Scalar>
class MyPointCloud {

    using Scalar = _Scalar;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixType;
    typedef Eigen::Vector<Scalar, Eigen::Dynamic>                 VectorType;

    public:

        MyPointCloud(){
            m_vertices = MatrixType::Zero(0, 0);
            m_normals = MatrixType::Zero(0, 0);
            m_size = 0;
        }

        MyPointCloud(MatrixType & vertices, MatrixType & normals){
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

        void setVertices(MatrixType &points){
            // Assert for the size of the matrix
            assert(points.rows() == m_size);
            assert(points.cols() == 3);
            m_vertices = points;
            updateBoundingBox();
        }

        void setNormals(MatrixType &normals){
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

        MatrixType getVertices(){
            return m_vertices;
        }

        MatrixType getNormals(){
            return m_normals;
        }

        int getSize(){
            return m_size;
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

        VectorType getMin(){
            return m_min;
        }

        VectorType getMax(){
            return m_max;
        }

    private:

        MatrixType m_vertices;
        MatrixType m_normals;
        DiffQuantities<Scalar>  m_diffQuantities;
        int m_size;

        VectorType m_min;
        VectorType m_max;

        std::vector<std::array<Scalar, 3>> m_triangles;


        void updateBoundingBox(){
            m_min = m_vertices.colwise().minCoeff();
            m_max = m_vertices.colwise().maxCoeff();
        }

}; // class MyPointCloud
