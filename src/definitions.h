#pragma once

#include "Ponca/Fitting"
#include "Ponca/SpatialPartitioning"
#include "poncaAdapters.hpp"

// Types definition

using Scalar             = double;
using VectorType         = Eigen::Matrix<Scalar, 3,1>;
using PPAdapter          = BlockPointAdapter<Scalar>;
using KdTree             = Ponca::KdTree<PPAdapter>;
using KnnGraph           = Ponca::KnnGraph<PPAdapter>;

// Weighting functions

using SmoothWeightFunc   = Ponca::DistWeightFunc<PPAdapter, Ponca::SmoothWeightKernel<Scalar> >;
using ConstWeightFunc    = Ponca::DistWeightFunc<PPAdapter, Ponca::ConstantWeightKernel<Scalar> >;

// Fitting methods

using basket_dryFit                      =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::DryFit>;

using basket_FullyOrientedCylinderFit    =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedParabolicCylinderFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;


using basket_BaseCylinderFit             =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseParabolicCylinderFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalCovarianceCurvatureEstimator>;

using basket_BaseOrientedCylinderFit     =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedParabolicCylinderFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalCovarianceCurvatureEstimator>;

using basket_NearOrientedCylinderFit     =  Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedParabolicCylinderFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

using basket_FullyOrientedEllipsoid2DFit =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedEllipsoid2DFit>;

// Fixed
using basket_FullyOrientedEllipsoid2DFitTest = Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedEllipsoid2DFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

using basket_BaseEllipsoid2DFit          =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseEllipsoid2DFit>;

using basket_BaseEllipsoid2DFitTest = Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseEllipsoid2DFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;


using basket_BaseOrientedEllipsoid2DFit  =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedEllipsoid2DFit>;

using basket_BaseOrientedEllipsoid2DFitTest = Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedEllipsoid2DFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

using basket_NearOrientedEllipsoid2DFit  =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedEllipsoid2DFit>;

using basket_NearOrientedEllipsoid2DFitTest = Ponca::BasketDiff<
                                            Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedEllipsoid2DFit>,
                                            Ponca::DiffType::FitSpaceDer,
                                            Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;


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
                                                Ponca::DiffType::FitSpaceDer,
                                                Ponca::CurvatureEstimatorBase, Ponca::NormalDerivativesCurvatureEstimator>;

// Trying to generate triangles
using basket_triangleGeneration          =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::TriangleGeneration>;

using basket_orientedWaveJets            =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedWaveJetsFit>;

using basket_waveJets                    =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::WaveJetsFit>;
