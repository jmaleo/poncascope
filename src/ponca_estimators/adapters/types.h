#pragma once

#include <Eigen/Dense>
#include <Ponca/Common>
#include <Ponca/Fitting>

#include "../../definitions.h"
#include "poncaAdapters.hpp"

// Weighting functions
using SmoothWeightFunc      = Ponca::DistWeightFunc<PPAdapter, Ponca::SmoothWeightKernel<Scalar> >;
using ConstWeightFunc       = Ponca::DistWeightFunc<PPAdapter, Ponca::ConstantWeightKernel<Scalar> >;
using WendlandWeightFunc    = Ponca::DistWeightFunc<PPAdapter, Ponca::WendlandWeightKernel<Scalar> >;
using SingularWeightFunc    = Ponca::DistWeightFunc<PPAdapter, Ponca::SingularWeightKernel<Scalar> >;
using VarifoldWeightFunc    = Ponca::DistWeightFunc<PPAdapter, Ponca::VarifoldWeightKernel<Scalar> >;
using ExponentialWeightFunc = Ponca::DistWeightFunc<PPAdapter, Ponca::ExponentialWeightKernel<Scalar> >;
// Don't use in global to avoid the case where scale2D requires a ddf valide for the mlsSphereFit derivative
// using CompactExpWeightFunc = Ponca::DistWeightFunc<PPAdapter, Ponca::CompactExpWeightKernel<Scalar> >;
