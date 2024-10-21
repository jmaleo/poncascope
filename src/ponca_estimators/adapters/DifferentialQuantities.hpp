#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <DGtal/math/Statistic.h>


template <typename _Scalar>
struct Quantity {
    using Scalar = _Scalar;
    using VectorType = Eigen::Matrix<Scalar, 3, 1>;

public :
    Scalar k1 = 0; 
    Scalar k2 = 0; 
    Scalar mean = 0; 
    Scalar gauss = 0;
    Scalar neighbor_count = 0;
    VectorType d1 = VectorType(1, 0, 0);
    VectorType d2 = VectorType(1, 0, 0);
    VectorType normal = VectorType(1, 0, 0);
    VectorType projection = VectorType::Zero();

    int non_stable = 0;
    std::chrono::nanoseconds computing_time = std::chrono::nanoseconds(0);

    Quantity() = default; 

    Quantity(const Quantity& q) {
        this->k1 = q.k1;
        this->k2 = q.k2;
        this->mean = q.mean;
        this->gauss = q.gauss;
        this->d1 = q.d1;
        this->d2 = q.d2;

        this->neighbor_count = q.neighbor_count;

        this->projection = q.projection;
        this->normal = q.normal;

        this->non_stable = q.non_stable;
        this->computing_time = q.computing_time;
    }

};


/// Store the differential quantities
template <typename _Scalar>
struct DifferentialQuantities {
public:
    enum {Dim = 3};
    using Scalar     = _Scalar;

    DifferentialQuantities()  = default;
    ~DifferentialQuantities() = default;

    inline DifferentialQuantities(SampleMatrixType position, SampleVectorType k1, SampleVectorType k2, SampleVectorType mean, SampleVectorType gauss, SampleMatrixType d1, SampleMatrixType d2, SampleMatrixType normal, DGtal::Statistic<Scalar> statNei, DGtal::Statistic<Scalar> statTime) {
        m_k1 = std::move(k1);
        m_k2 = std::move(k2);
        m_mean = std::move(mean);
        m_gauss = std::move(gauss);
        m_d1 = std::move(d1);
        m_d2 = std::move(d2);
        m_normal = std::move(normal);
        m_statNeighbors = statNei;
        m_statTimings = statTime;
        m_position = std::move(position);
        m_nonstable = 0;
        computeShapeIndex();
    }

    inline void computeShapeIndex() {
        m_shapeIndex = SampleVectorType(m_k1.size());
        for (int i = 0; i < m_k1.size(); i++) {
            _Scalar sIndex       = ( 2.0 / M_PI ) * std::atan( ( m_k1[ i ] + m_k2[ i ] ) / ( m_k1 [ i ] - m_k2[ i ] ) );
            m_shapeIndex[i] = sIndex;
        }
    }

    inline void setNonStableVector(std::vector<unsigned int> non_stables ) { 
        m_non_stable_ratio = non_stables; 
        m_nonstable = std::count_if(non_stables.begin(), non_stables.end(), [](Scalar x) { return x == 1; });
    }

    [[nodiscard]] int getNumNonstable() const { return m_nonstable; }

    inline Scalar getNonStableRatio () const { return Scalar( m_nonstable ) / Scalar( m_position.size() ); }

    std::vector<unsigned int> getNonStableVector() { return m_non_stable_ratio;}

    [[nodiscard]] inline SampleVectorType k1()     const { return m_k1; }
    [[nodiscard]] inline SampleVectorType k2()     const { return m_k2; }
    [[nodiscard]] inline SampleVectorType mean()   const { return m_mean; }
    [[nodiscard]] inline SampleVectorType gauss()  const { return m_gauss; }
    [[nodiscard]] inline SampleVectorType shapeIndex() const { return m_shapeIndex; }
    [[nodiscard]] inline SampleMatrixType d1()     const { return m_d1; }
    [[nodiscard]] inline SampleMatrixType d2()     const { return m_d2; }
    [[nodiscard]] inline SampleMatrixType normal() const { return m_normal; }
    [[nodiscard]] inline SampleMatrixType position() const { return m_position; }

    inline void setOriented(const bool oriented) { m_oriented = oriented; }
    [[nodiscard]] inline bool isOriented() const { return m_oriented; }

    inline DGtal::Statistic<Scalar> statNeighbors() const { return m_statNeighbors; }
    inline DGtal::Statistic<Scalar> statTimings()   const { return m_statTimings; }

    SampleMatrixType getByName(const std::string& propertyName) {
        if (propertyName == "Min curvature") {
            return m_k1;
        } else if (propertyName == "Max curvature") {
            return m_k2;
        } else if (propertyName == "Mean curvature") {
            return m_mean;
        } else if (propertyName == "Gaussian curvature") {
            return m_gauss;
        } else if (propertyName == "ShapeIndex") {
            return m_shapeIndex;
        } else if (propertyName == "Min curvature direction") {
            return m_d1;
        } else if (propertyName == "Max curvature direction") {
            return m_d2;
        } else if (propertyName == "Normals") {
            return m_normal;
        } else {
            std::cerr << "Error: property name not found" << std::endl;
            return SampleVectorType(1, 0);
        }
    }

private:
    SampleVectorType m_k1, m_k2, m_mean, m_gauss, m_shapeIndex;
    SampleMatrixType m_d1, m_d2, m_normal, m_position;

    DGtal::Statistic<Scalar> m_statNeighbors;
    DGtal::Statistic<Scalar> m_statTimings;

    int m_nonstable = 0;
    std::vector<unsigned int> m_non_stable_ratio;

    bool m_oriented = true;
};