#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <random>

#include <Eigen/Dense>
#include <DGtal/math/Statistic.h>

#include "../definitions.h"
#include "../ponca_estimators/adapters/DifferentialQuantities.hpp"
#include "Readers.h"
#include "Writters.h"

unsigned int seed = 0;
std::random_device random_seed; 
std::default_random_engine generator;

template <typename _Scalar>
struct PointCloudDiff
{
  /// Types
  // typedef Eigen::VectorX<_Scalar>      DenseVector;
  // typedef Eigen::MatrixX<_Scalar>      DenseMatrix;
  // typedef Eigen::Vector<_Scalar, 3>    VectorType;
  // typedef Eigen::Matrix<_Scalar, 3, 3> Tensor;

  // VectorTypeCloudDiff()  = default;
  ~PointCloudDiff() = default;

  explicit PointCloudDiff (const std::string& name) {
    this->name = name;
    errorNormal = DGtal::Statistic<_Scalar> (true);
    errorPos = DGtal::Statistic<_Scalar> (true);
    errorMean = DGtal::Statistic<_Scalar> (true);
    errorGauss = DGtal::Statistic<_Scalar> (true);
    errorK1 = DGtal::Statistic<_Scalar> (true);
    errorK2 = DGtal::Statistic<_Scalar> (true);
    errorD1 = DGtal::Statistic<_Scalar> (true);
    errorD2 = DGtal::Statistic<_Scalar> (true);
    errorShapeIndex = DGtal::Statistic<_Scalar> (true);
    statTimings = DGtal::Statistic<_Scalar> (true);
    statNeighbors = DGtal::Statistic<_Scalar> (true);
  }

  PointCloudDiff (const std::string& name, const SampleMatrixType& points, const SampleMatrixType& normals) {
    this->name = name;
    if ( points.size() != normals.size() )
    {
      std::cerr << "Error: points and normals have different sizes" << std::endl;
      return;
    }
    this->points = points;
    this->normals = normals;
    errorNormal = DGtal::Statistic<_Scalar> (true);
    errorPos = DGtal::Statistic<_Scalar> (true);
    errorMean = DGtal::Statistic<_Scalar> (true);
    errorGauss = DGtal::Statistic<_Scalar> (true);
    errorK1 = DGtal::Statistic<_Scalar> (true);
    errorK2 = DGtal::Statistic<_Scalar> (true);
    errorD1 = DGtal::Statistic<_Scalar> (true);
    errorD2 = DGtal::Statistic<_Scalar> (true);
    errorShapeIndex = DGtal::Statistic<_Scalar> (true);
    statTimings = DGtal::Statistic<_Scalar> (true);
    statNeighbors = DGtal::Statistic<_Scalar> (true);
    compute_bbox();
  }

  void compute_bbox() {
    bbox_min = VectorType::Ones() * std::numeric_limits<_Scalar>::max();
    bbox_max = VectorType::Ones() * std::numeric_limits<_Scalar>::lowest();
    for (int i = 0 ; i < points.rows(); i++) {
      VectorType point = points.row(i);
      for (int j = 0; j < 3; j++) {
        bbox_min[j] = std::min(bbox_min[j], point[j]);
        bbox_max[j] = std::max(bbox_max[j], point[j]);
      }
    }
  }

  VectorType getMin() {
    return bbox_min;
  }

  VectorType getMax() {
    return bbox_max;
  }

  void resize (const int size )
  {
    points = SampleMatrixType( size, 3 );
    normals = SampleMatrixType( size, 3 );
    gauss = SampleVectorType( size );
    mean = SampleVectorType( size );
    k1 = SampleVectorType( size );
    k2 = SampleVectorType( size );
    v1 = SampleMatrixType( size, 3 );
    v2 = SampleMatrixType( size, 3 );
    non_stable_idx = SampleVectorType( size );
  }

  void init_zeros ( const int size )
  {
    points = SampleMatrixType::Zero( size, 3 );
    normals = SampleMatrixType::Zero( size, 3 );
    gauss = SampleVectorType::Zero( size );
    mean = SampleVectorType::Zero( size );
    k1 = SampleVectorType::Zero( size );
    k2 = SampleVectorType::Zero( size );
    v1 = SampleMatrixType::Zero( size, 3 );
    v2 = SampleMatrixType::Zero( size, 3 );
    non_stable_idx = SampleVectorType::Zero( size );
  }

  PointCloudDiff( const PointCloudDiff & apc ) = default;

  void loadVectorTypeCloud ( std::string & input )
  {
    IO::loadPointCloud<PointCloudDiff<_Scalar>, _Scalar>( *this, input );
  }

  void saveVectorTypeCloud ( std::string & filename )
  {
    IO::savePointCloud<PointCloudDiff<_Scalar>>( *this, filename );
  }
  
  void saveVectorTypeCloudAsPositions ( std::string & filename )
  {
    IO::savePointCloudAsPositions<PointCloudDiff<_Scalar>>( *this, filename );
  }

  void saveVectorTypeCloudAsErrors ( std::string & filename )
  {
    IO::savePointCloudAsErrors<PointCloudDiff<_Scalar>, _Scalar>( *this, filename );
  }

  std::string getStats ( std::string & name, _Scalar radius, _Scalar noise_position, _Scalar noise_normal, _Scalar flip_ratio, bool absolute_err = true )
  {
    return IO::getStats<PointCloudDiff<_Scalar>>( *this, name, radius, noise_position, noise_normal, flip_ratio, absolute_err );
  }

  void addOutliers( _Scalar ratio, _Scalar noise_position )
  {
    generator.seed( ( seed == 0 ) ? random_seed() : seed );
    std::uniform_real_distribution<_Scalar> dis( 0, 1 );
    std::normal_distribution<_Scalar> dis_noise( 0, noise_position );
    for ( int i = 0 ; i < points.rows(); i++ )
    {
      VectorType p = points.row(i);
      if (dis(generator) < ratio){
        p[ 0 ] += dis_noise( generator );
        p[ 1 ] += dis_noise( generator );
        p[ 2 ] += dis_noise( generator );
      }
    }
  }

  void flipNormal(_Scalar ratio){
    generator.seed( ( seed == 0 ) ? random_seed() : seed );
    std::uniform_real_distribution<_Scalar> dis( 0, 1 );
    for ( int i = 0 ; i < points.rows(); i++ )
    {
      VectorType n = normals.row(i);
      if (dis(generator) < ratio){
        n[ 0 ] *= -1;
        n[ 1 ] *= -1;
        n[ 2 ] *= -1;
      }
    }
  }

  void addNoisePosition( _Scalar noise_position )
  {
    generator.seed( ( seed == 0 ) ? random_seed() : seed );
    std::normal_distribution<_Scalar> dis( 0, noise_position );
    for ( int i = 0 ; i < points.rows(); i++ )
    {
      VectorType v = points.row(i);
      v[ 0 ] += dis( generator );
      v[ 1 ] += dis( generator );
      v[ 2 ] += dis( generator );
    }
  }

  void addNoiseNormal( _Scalar noise_normal )
  {
    generator.seed( ( seed == 0 ) ? random_seed() : seed );
    std::normal_distribution<_Scalar> dis( 0, noise_normal );
    for ( int i = 0 ; i < normals.rows(); i++ )
    {
      VectorType n = normals.row(i);
      n[ 0 ] += dis( generator );
      n[ 1 ] += dis( generator );
      n[ 2 ] += dis( generator );
      _Scalar norm = sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
      n[0] /= norm;
      n[1] /= norm;
      n[2] /= norm;
    }
  }

  void check_k1_k2 ()
  {
    #pragma omp parallel for
    for ( int i = 0; i < points.size(); i++ )
    {
      if ( k1[i]  >  k2[i] )
      {
        std::swap (k1[i], k2[i]);
        const VectorType tmp = v1.row(i);
        v1.row(i) = v2.row(i);
        v2.row(i) = tmp;
      }
    }
  }

  void switch_to_abs ()
  {
    #pragma omp parallel for
    for ( int i = 0; i < points.rows(); i++ )
    {
      // particular treatment for kmean and kgauss
      mean[i] = std::abs(mean[i]);
      gauss[i] = std::abs(gauss[i]);

      k1[i] = std::abs(k1[i]);
      k2[i] = std::abs(k2[i]);

      if ( k1[i]  >  k2[i] )
      {
        std::swap(k1[i], k2[i]);
        const VectorType tmp = v1.row(i);
        v1.row(i) = v2.row(i);
        v2.row(i) = tmp;
      }

    }
  }

  _Scalar squared_error( _Scalar error ) {
    return std::pow( ( error ), 2 );
  }

  _Scalar abs_error( _Scalar error ) {
    return std::fabs( error );
  }

  template <typename Functor>
  static _Scalar compute_scalar_error ( _Scalar val, _Scalar other, Functor metric ) {
    return metric( val - other );
  }

  template <typename Functor>
  _Scalar compute_radian_error ( VectorType vec, VectorType other, Functor metric, bool oriented = false ) {
    _Scalar dot_prod = vec.dot ( other );
    if ( ! oriented && dot_prod < 0. ){
      dot_prod = vec.dot ( -other );
    }
    dot_prod = ( dot_prod > 1. ) ? 1 : dot_prod;
    dot_prod = ( dot_prod < -1. ) ? 1 : dot_prod;

    return std::acos( dot_prod / ( vec.norm() * other.norm() ) ); // radian
  }

  template <typename Functor>
  _Scalar compute_vector_error( VectorType vec, VectorType other, Functor metric, bool oriented = false ) {
    _Scalar radian_err = compute_radian_error ( vec, other, metric, oriented );
    return metric ( radian_err * 180.0 / M_PI ); // degree
  }

  template <typename Functor>
  static _Scalar compute_VectorType_error ( VectorType p, VectorType other, Functor metric ) {
    return metric( ( p - other ).norm() );
  }

  void correct_vectors ( int idx ) {
    if ( v1.row( idx ).norm() < 1e-6 )
    {
      v1.row( idx ) = VectorType( 1, 0, 0 );
      v2.row( idx ) = VectorType( 0, 1, 0 );
    }
    if ( normals.row( idx ).norm() < 1e-6 )
    {
      normals.row( idx ) = VectorType( 1, 0, 0 );
    }
  }

  template <typename Functor>
  void compare_at ( const PointCloudDiff & other, int i, Functor metric ){

    errorPos.addValue( compute_VectorType_error( points.row(i), other.points.row(i), metric ) );
    errorMean.addValue( compute_scalar_error( mean[i], other.mean[i], metric ) );
    errorGauss.addValue( compute_scalar_error( gauss[i], other.gauss[i], metric ) );

    errorK1.addValue( compute_scalar_error( k1[i], other.k1[i], metric ) );
    errorK2.addValue( compute_scalar_error( k2[i], other.k2[i], metric ) );

    errorNormal.addValue( compute_vector_error( normals.row(i), other.normals.row(i), metric, oriented ) );

    if ( std::fabs( other.k1[i] - other.k2[i] ) > 1e-6 )
    {
      errorD1.addValue( compute_vector_error( v1.row(i), other.v1.row(i), metric ) );
      errorD2.addValue( compute_vector_error( v2.row(i), other.v2.row(i), metric ) );

      _Scalar sIndex       = ( 2.0 / M_PI ) * std::atan( ( k1[i] + k2[i] ) / ( k1[i] - k2[i] ) );
      _Scalar sIndex_other = ( 2.0 / M_PI ) * std::atan( ( other.k1[i] + other.k2[i] ) / ( other.k1[i] - other.k2[i] ) );
      errorShapeIndex.addValue( compute_scalar_error( sIndex, sIndex_other, metric ) );
    }
    else
    {
      errorD1.addValue( 0. );
      errorD2.addValue( 0. );
      errorShapeIndex.addValue( 0. );
    }
  } 

  template <typename Functor>
  void estimate_at ( int i, Functor metric ){

    errorPos.addValue( 0 );
    errorMean.addValue( metric( metric(mean[i]) ));
    errorGauss.addValue( metric(gauss[i]) );

    errorK1.addValue( metric(k1[i]) );
    errorK2.addValue( metric(k2[i]) );

    errorNormal.addValue( 0 );

    errorD1.addValue( 0 );
    errorD2.addValue( 0 );
    _Scalar sIndex       = ( 2.0 / M_PI ) * std::atan( ( k1[i] + k2[i] ) / ( k1[i] - k2[i] ) );
    if ( std::fabs( k1[i] - k2[i] ) > 1e-6 ){
      errorShapeIndex.addValue( metric( sIndex ) );
    }
  } 

  void compute_unique_error ( const PointCloudDiff & other, int unique_idx, bool absolute_err = true ){
      if ( non_stable_idx[ unique_idx ] == 1 )
      {
        statNeighbors.addValue( 1 );
        return;
      }
      correct_vectors( unique_idx );
      compare_at( other, unique_idx, [this, &absolute_err] ( _Scalar error ) { return ( absolute_err ) ? abs_error ( error ) : squared_error ( error ); } );
  }

  void compare( const PointCloudDiff & other, bool absolute_err = true, const bool statsAsEstimations = false )
  {
    for ( int i = 0; i < points.rows(); i++ )
    {
      if ( non_stable_idx.rows() > 0 && non_stable_idx[i] == static_cast<Scalar>(1) )
      {
        statNeighbors.addValue( 1 );
        continue;
      }
      correct_vectors( i );
      if ( statsAsEstimations )
      {
        estimate_at( i, 
          [this, &absolute_err] ( _Scalar estim ) 
          { 
            return ( absolute_err ) ? abs_error ( estim ) : estim; 
          } 
        );
      }
      else {
        compare_at( other, i, 
          [this, &absolute_err] ( _Scalar error ) 
          { 
            return ( absolute_err ) ? abs_error ( error ) : squared_error ( error ); 
          } 
        );
      }
      
    }
  }

  void setDifferentialQuantities(const DifferentialQuantities<_Scalar>& diffQuantities){
    k1 = diffQuantities.k1();
    k2 = diffQuantities.k2();
    mean = diffQuantities.mean();
    gauss = diffQuantities.gauss();
    v1 = diffQuantities.d1();
    v2 = diffQuantities.d2();
    normals = diffQuantities.normal();
    statNeighbors = diffQuantities.statNeighbors();
    statTimings = diffQuantities.statTimings();
    non_stable_idx = diffQuantities.getNonStableVector();
  }

  void setTriangles(std::vector<std::array<_Scalar, 3>> &triangles){
    m_triangles = triangles;
  }

  std::vector<std::array<_Scalar, 3>> & getTriangles(){
    return m_triangles;
  }

  SampleMatrixType getByName(const std::string& propertyName) {
    if (propertyName == "Min curvature") {
      return k1;
    } else if (propertyName == "Max curvature") {
      return k2;
    } else if (propertyName == "Mean curvature") {
      return mean;
    } else if (propertyName == "Gaussian curvature") {
      return gauss;
    } else if (propertyName == "Min curvature direction") {
      return v1;
    } else if (propertyName == "Max curvature direction") {
      return v2;
    } else if (propertyName == "Normals") {
      return normals;
    } else if (propertyName == "ShapeIndex") {
      return shapeIndex;
    } else {
      std::cerr << "Error: property name not found" << std::endl;
      return SampleMatrixType(1);
    }
  }

  [[nodiscard]] std::pair<SampleMatrixType, SampleVectorType> getNonZeros(const SampleVectorType& nei_val, const int iVertexSource) const {
    int nb_non_zeros = 0;
    for (int i = 0; i < nei_val.size(); i++) {
      if (nei_val[i] != 0) {
        nb_non_zeros++;
      }
    }

    SampleVectorType nei_value(nb_non_zeros);
    SampleMatrixType nei_pos(nb_non_zeros, 3);

    int idx = 0;
    for (int i = 0; i < nei_val.size(); i++) {
      if (nei_val[i] != 0) {
        nei_value[idx] = nei_val[i];
        nei_pos.row(idx) = points.row(i);
        idx++;
      }
    }
    return std::make_pair(nei_pos, nei_value);

  }

  const DifferentialQuantities<_Scalar> & getDiffQuantities() {
    return estimatedDiffQuantities;
  }

  bool oriented = true;

  std::string name;

  DifferentialQuantities<_Scalar> estimatedDiffQuantities;

  VectorType bbox_min = VectorType::Zero();
  VectorType bbox_max = VectorType::Zero();

  SampleMatrixType points;
  SampleMatrixType normals;
  SampleVectorType gauss, mean, k1, k2;
  SampleMatrixType v1;
  SampleMatrixType v2;
  SampleVectorType shapeIndex;
  SampleVectorType non_stable_idx;

  std::vector<std::array<_Scalar, 3>> m_triangles;

  // For error stats
  
  DGtal::Statistic<_Scalar> errorShapeIndex; // To TEST

  DGtal::Statistic<_Scalar> errorPos;
  DGtal::Statistic<_Scalar> errorNormal;
  DGtal::Statistic<_Scalar> errorMean;
  DGtal::Statistic<_Scalar> errorGauss;
  DGtal::Statistic<_Scalar> errorK1;
  DGtal::Statistic<_Scalar> errorK2;
  DGtal::Statistic<_Scalar> errorD1;
  DGtal::Statistic<_Scalar> errorD2;
  DGtal::Statistic<_Scalar> statTimings;
  DGtal::Statistic<_Scalar> statNeighbors;
};
