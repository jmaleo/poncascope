#pragma once

#include <map>
#include <unordered_map>
#include <string>
#include <functional>
#include <memory>
#include "Estimator.h"

template <typename WeightFunc>
class EstimatorFactory {
public:

    EstimatorFactory() {
        
        // estimators ORIENTED
        estimatorMap["MeanPLANE"] =        std::make_shared<Estimator<fit_MeanPLANE<WeightFunc>>>("MeanPLANE", true, mls_iter);
        estimatorMap["ASO"] =              std::make_shared<Estimator<fit_ASO<WeightFunc>>>("ASO", true, mls_iter);
        estimatorMap["APSS"] =             std::make_shared<Estimator<fit_APSS<WeightFunc>>>("APSS", true, mls_iter);
        estimatorMap["OrientedPC-MLS"] =  std::make_shared<Estimator<fit_BOCylinder<WeightFunc>>>("OrientendPC-MLS", true, mls_iter);
        estimatorMap["Direct2-Monge"] =    std::make_shared<Estimator<fit_FO2D<WeightFunc>>>("Direct2-Monge", true, mls_iter);
        estimatorMap["Ellipsoid"] =        std::make_shared<Estimator<fit_Ellipsoid<WeightFunc>>>("Ellipsoid", true, mls_iter);
        estimatorMap["Varifolds"] =        std::make_shared<Estimator<fit_Varifolds>>("Varifolds", true, 1); // To be fixed in Ponca for multiple iterations
        estimatorMap["VarifoldsMeanPlane"] = std::make_shared<Estimator<fit_VarifoldsMeanPlane>>("VarifoldsMeanPlane", true, 1); // To be fixed in Ponca for multiple iterations
        estimatorMap["Oriented2-Monge"] =  std::make_shared<Estimator<fit_OrientedMonge<WeightFunc>>>("Oriented2-Monge", true, mls_iter);
        estimatorMap["ShapeOperator"] =    std::make_shared<Estimator<fit_ShapeOperator<WeightFunc>>>("ShapeOperator", true, 1);
        
        // estimators UNORIENTED
        estimatorMap["PCA"] =              std::make_shared<Estimator<fit_PCA<WeightFunc>>>("PCA", false, mls_iter);
        estimatorMap["Sphere"] =           std::make_shared<Estimator<fit_Sphere<WeightFunc>>>("Sphere", false, mls_iter);
        estimatorMap["UnorientedSphere"] = std::make_shared<Estimator<fit_UnorientedSphere<WeightFunc>>>("UnorientedSphere", false, mls_iter);
        estimatorMap["PC-MLS"] =           std::make_shared<Estimator<fit_BCylinder<WeightFunc>>>("PC-MLS", false, mls_iter);
        estimatorMap["2-Monge"] =          std::make_shared<Estimator<fit_Monge<WeightFunc>>>("2-Monge", false, mls_iter);
        estimatorMap["Cov2D"] =            std::make_shared<Estimator<fit_Cov2D<WeightFunc>>>("Cov2D", false, 1);
        estimatorMap["NormCov2D"] =        std::make_shared<Estimator<fit_NormCov2D<WeightFunc>>>("NormCov2D", false, 1);
        estimatorMap["NormCov3D"] =        std::make_shared<Estimator<fit_NormCov3D<WeightFunc>>>("NormCov3D", false, 1);
        estimatorMap["Mean"] =             std::make_shared<Estimator<fit_MeanCurvate<WeightFunc>>>("Mean", false, 1);
        estimatorMap["3DQuadric"] =        std::make_shared<Estimator<fit_quadric<WeightFunc>>>("3DQuadric", false, mls_iter);

        // Special estimators ORIENTED
        estimatorMap["OrientedWaveJets"] = std::make_shared<Estimator<fit_OrientedWaveJets<WeightFunc>>>("OrientedWaveJets", true, 1);
        estimatorMap["Independent"] =      std::make_shared<Estimator<fit_CNC>>("Independent", true, 1);
        estimatorMap["Uniform"] =          std::make_shared<Estimator<fit_CNC>>("Uniform", true, 1);
        estimatorMap["AvgHexagram"] =      std::make_shared<Estimator<fit_CNC>>("AvgHexagram", true, 1);
        estimatorMap["Hexagram"] =         std::make_shared<Estimator<fit_CNC>>("Hexagram", true, 1);
        
        // Special estimators UNORIENTED
        estimatorMap["WaveJets"] =         std::make_shared<Estimator<fit_WaveJets<WeightFunc>>>("WaveJets", false, 1);
    }

    std::shared_ptr< BaseEstimator > getEstimator(const std::string& name) {
        if (estimatorMap.find(name) != estimatorMap.end()) {
            return estimatorMap[name];
        } else {
            throw std::runtime_error("Estimator type not found");
        }
    }

private:

    std::map<std::string, std::shared_ptr< BaseEstimator > > estimatorMap;
};

template <typename WeightFunc>
std::shared_ptr<EstimatorFactory< WeightFunc > > getEstimatorFactory() {
    static auto factory = std::make_shared<EstimatorFactory< WeightFunc >>();
    return factory;
}
