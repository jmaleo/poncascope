#pragma once

#include "Ponca/Fitting"
#include "Ponca/SpatialPartitioning"
#include "poncaAdapters.hpp"


// Types definition

// using Scalar             = double;
using Scalar             = float;

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
using CompactExpWeightFunc = Ponca::DistWeightFunc<PPAdapter, Ponca::CompactExpWeightKernel<Scalar> >;

// Fitting methods
using basket_dryFit                      =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::DryFit>;

using basket_FullyOrientedCylinderFit    =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedParabolicCylinderFit>;
using basket_BaseCylinderFit             =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseParabolicCylinderFit>;
using basket_BaseOrientedCylinderFit     =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedParabolicCylinderFit>;
using basket_NearOrientedCylinderFit     =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedParabolicCylinderFit>;

using basket_FullyOrientedEllipsoid2DFit =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedEllipsoid2DFit>;
using basket_BaseEllipsoid2DFit          =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseEllipsoid2DFit>;
using basket_BaseOrientedEllipsoid2DFit  =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedEllipsoid2DFit>;
using basket_NearOrientedEllipsoid2DFit  =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedEllipsoid2DFit>;

using basket_AlgebraicShapeOperatorFit   =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedSphereFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::OrientedSphereDer, Ponca::MlsSphereFitDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// using basket_AlgebraicShapeOperatorFit   =  Ponca::BasketDiff<
//                                             Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedSphereFit>,
//                                             Ponca::DiffType::FitSpaceDer,
//                                             Ponca::OrientedSphereDer, Ponca::MlsSphereFitDerTest>;

using basket_AlgebraicPointSetSurfaceFit =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedSphereFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::OrientedSphereDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;
using basket_planeFit                    =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::CovariancePlaneFit>,
                                                Ponca::DiffType::FitSpaceDer,
                                                Ponca::CovariancePlaneDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;
using basket_ellipsoidFit                =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedEllipsoidFit>,
                                                Ponca::DiffType::FitSpaceDer, Ponca::OrientedEllipsoidDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;
// using basket_ellipsoidFit                =  Ponca::BasketDiff<
//                                                 Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedEllipsoidFit>,
//                                                 Ponca::DiffType::FitSpaceDer, Ponca::OrientedEllipsoidDer>;


using basket_hyperboloidFit              =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedHyperboloid_ACP_Fit>;
// Same as ellipsoidFit
using basket_hyperboloidFit_Diff         =  Ponca::BasketDiff<
                                                Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedHyperboloid_ACP_Fit>,
                                                Ponca::DiffType::FitSpaceDer, Ponca::OrientedEllipsoidDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// Trying to generate triangles
using basket_orientedWaveJets            =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedWaveJetsFit>;
using basket_waveJets                    =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::WaveJetsFit>;

using basket_triangleGeneration          =  Ponca::Basket<PPAdapter, ConstWeightFunc, Ponca::TriangleGeneration>; 


// // Fitting methods
// template <typename WeightFunc>
// struct FittingMethods{
//     // Define the type of the fitting method in order to use it in the main
//     typedef  Ponca::Basket<PPAdapter, WeightFunc, Ponca::FullyOrientedParabolicCylinderFit>  basket_test  ;

// }; // end of struct FittingMethods
