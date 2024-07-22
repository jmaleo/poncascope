#pragma once

#include "Ponca/Fitting"
#include "Ponca/SpatialPartitioning"
#include "poncaAdapters.hpp"


// Types definition

using Scalar             = double;
// using Scalar             = float;

using VectorType         = Eigen::Matrix<Scalar, 3,1>;
using PPAdapter          = BlockPointAdapter<Scalar>;
using KdTree             = Ponca::KdTree<PPAdapter>;
using KnnGraph           = Ponca::KnnGraph<PPAdapter>;

typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>  SampleMatrixType;
typedef Eigen::Vector<Scalar, Eigen::Dynamic>                  SampleVectorType;


// Weighting functions
using SmoothWeightFunc     = Ponca::DistWeightFunc<PPAdapter, Ponca::SmoothWeightKernel<Scalar> >;
using ConstWeightFunc      = Ponca::DistWeightFunc<PPAdapter, Ponca::ConstantWeightKernel<Scalar> >;
using WendlandWeightFunc   = Ponca::DistWeightFunc<PPAdapter, Ponca::WendlandWeightKernel<Scalar> >;
using SingularWeightFunc   = Ponca::DistWeightFunc<PPAdapter, Ponca::SingularWeightKernel<Scalar> >;
using varifoldWeightFunc   = Ponca::DistWeightFunc<PPAdapter, Ponca::VarifoldWeightKernel<Scalar> >;

// Don't use in global to avoid the case where scale2D requires a ddf valide for the mlsSphereFit derivative
// using CompactExpWeightFunc = Ponca::DistWeightFunc<PPAdapter, Ponca::CompactExpWeightKernel<Scalar> >;


// No templated Fitting methods

using basket_triangleGeneration          =  Ponca::Basket<PPAdapter, ConstWeightFunc, Ponca::TriangleGeneration>; 
using basket_dryFit                      =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::DryFit>;

template <typename WeightFunc>
using basket_MeanFit                     =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::MeanCurvatureFit>;

template <typename WeightFunc>
using basket_Covariance2DFit             = Ponca::Basket<PPAdapter, WeightFunc, Ponca::Covariance2DFit>;

template <typename WeightFunc>
using basket_NormalCovariance2DFit       = Ponca::Basket<PPAdapter, WeightFunc, Ponca::NormalCovariance2DFit>;

template <typename WeightFunc>
using basket_NormalCovariance3DFit       = Ponca::Basket<PPAdapter, WeightFunc, Ponca::NormalCovariance3DFit>;

template<typename WeightFunc>
using basket_ShapeOperator2DFit          = Ponca::Basket<PPAdapter, WeightFunc, Ponca::ShapeOperator2DFit>;

// Templated ( for the weight kernel ) Fitting methods
template <typename WeightFunc>
using basket_FullyOrientedCylinderFit    =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::FullyOrientedParabolicCylinderFit>;

template <typename WeightFunc>
using basket_BaseCylinderFit             =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::BaseParabolicCylinderFit>;

template <typename WeightFunc>
using basket_BaseOrientedCylinderFit     =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::BaseOrientedParabolicCylinderFit>;

template <typename WeightFunc>
using basket_FullyOrientedEllipsoid2DFit =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::FullyOrientedEllipsoid2DFit>;

template <typename WeightFunc>
using basket_BaseEllipsoid2DFit          =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::BaseEllipsoid2DFit>;

template <typename WeightFunc>
using basket_BaseOrientedEllipsoid2DFit  =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::BaseOrientedEllipsoid2DFit>;

template <typename WeightFunc>
using basket_AlgebraicShapeOperatorFit   =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedSphereFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::OrientedSphereDer, Ponca::MlsSphereFitDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// template <typename WeightFunc>
// using basket_AlgebraicPointSetSurfaceFit =  Ponca::BasketDiff<
//                                             Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedSphereFit>,
//                                             Ponca::DiffType::FitSpaceDer,
//                                             Ponca::OrientedSphereDer,
//                                             Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// template <typename WeightFunc>
// using basket_UnorientedSphereFit =  Ponca::BasketDiff<
//                                             Ponca::Basket<PPAdapter, WeightFunc, Ponca::UnorientedSphereFit>,
//                                             Ponca::DiffType::FitSpaceDer,
//                                             Ponca::UnorientedSphereDer,
//                                             Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// template <typename WeightFunc>
// using basket_SphereFit =  Ponca::BasketDiff<
//                                             Ponca::Basket<PPAdapter, WeightFunc, Ponca::SphereFit>,
//                                             Ponca::DiffType::FitSpaceDer,
//                                             Ponca::SphereFitDer,
//                                             Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

template <typename WeightFunc>
using basket_UnorientedSphereFit         = Ponca::Basket<PPAdapter, WeightFunc, Ponca::SimpleUnorientedSphereFit>;
                                            
template <typename WeightFunc>
using basket_SphereFit                   = Ponca::Basket<PPAdapter, WeightFunc, Ponca::SimpleSphereFit>;

template <typename WeightFunc>
using basket_AlgebraicPointSetSurfaceFit =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::SimpleOrientedSphereFit>;

template <typename WeightFunc>
using basket_planeFit                    =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, WeightFunc, Ponca::CovariancePlaneFit>,
                                                Ponca::DiffType::FitSpaceDer,
                                                Ponca::CovariancePlaneDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

template <typename WeightFunc>
using basket_meanPlaneFit                    =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, WeightFunc, Ponca::MeanPlaneFit>,
                                                Ponca::DiffType::FitSpaceDer,
                                                Ponca::MeanPlaneDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

template <typename WeightFunc>
using basket_ellipsoidFit                =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedEllipsoidFit>,
                                                Ponca::DiffType::FitSpaceDer, Ponca::OrientedEllipsoidDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;
                                                
template <typename WeightFunc>
using basket_hyperboloidFit              =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedHyperboloid_ACP_Fit>;

template <typename WeightFunc>
using basket_hyperboloidFit_Diff         =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedHyperboloid_ACP_Fit>,
                                                Ponca::DiffType::FitSpaceDer, Ponca::OrientedEllipsoidDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;


template <typename WeightFunc>
using basket_orientedWaveJets            =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedWaveJetsFit>;

template <typename WeightFunc>
using basket_waveJets                    =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::WaveJetsFit>;

template <typename WeightFunc>
using basket_mongePatchFit               =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::MongePatchFit>;

template <typename WeightFunc>
using basket_orientedMongePatchFit       =  Ponca::Basket<PPAdapter, WeightFunc, Ponca::OrientedMongePatchFit>;

using basket_varifold                    =  Ponca::Basket<PPAdapter, varifoldWeightFunc, Ponca::Varifold>;