#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"

// #include <igl/readOBJ.h>
// #include <igl/per_vertex_normals.h>


// #include <Ponca/Fitting>
// #include <Ponca/SpatialPartitioning>
// #include "poncaAdapters.hpp"
// #include <Eigen/Dense>

#include <iostream>
#include <utility>
#include <chrono>
// #include "generators.hpp"

// // Types definition
// using Scalar             = double;
// using VectorType         = Eigen::Matrix<Scalar, 3,1>;
// using PPAdapter          = BlockPointAdapter<Scalar>;
// using KdTree             = Ponca::KdTree<PPAdapter>;
// using SmoothWeightFunc   = Ponca::DistWeightFunc<PPAdapter, Ponca::SmoothWeightKernel<Scalar> >;
// using ConstWeightFunc    = Ponca::DistWeightFunc<PPAdapter, Ponca::ConstantWeightKernel<Scalar> >;

// // Variables
// Eigen::MatrixXd cloudV, cloudN;
// KdTree tree;
// polyscope::PointCloud* cloud = nullptr;
// polyscope::PointCloud* cloudCube = nullptr;
// polyscope::PointCloud* cloudCubeProj = nullptr;
// polyscope::PointCloud* cloudCurvature = nullptr;

// std::vector <VectorType> cubePoints;
// std::vector <VectorType> cubeProjPoints;
// std::vector <VectorType> cubeProjNorms;


// std::vector <VectorType> cubeProjPlanePoints;
// std::vector <glm::vec3> colorsPlane;

// Eigen::MatrixXd curvaturePoints;
// std::vector <Scalar> curvatures;

// // Options for algorithms
// int iVertexSource  = 7;     /// < id of the selected point
// int kNN            = 10;    /// < neighborhood size (knn)
// float NSize        = 0.25;   /// < neighborhood size (euclidean)
// int mlsIter        = 3;     /// < number of moving least squares iterations
// Scalar pointRadius = 0.005; /// < display radius of the point cloud
// double noise_sigma = 0.0;

// // Options for cylinder test
// bool m_cylinder = true;

// double a_cylinder  = -0.3;

// double bx_cylinder  = 0.2;
// double bz_cylinder  = 0.1;

// double c_a = 0.6;
// double cx_cylinder  = -0.8;
// double cz_cylinder  = 0.2;

// int x_cylinder     = 40;
// int z_cylinder     = 40;

// // Options for ellipsoid test
// Eigen::Matrix3d c_ellipsoid;
// int theta_ellipsoid = 40;
// int phi_ellipsoid   = 40;

// // KdTree tree_cylinder;
// std::pair<Eigen::MatrixXd, Eigen::MatrixXd>  parabolic_pair;

// Eigen::MatrixXd ellipsoid_verts;
// Eigen::MatrixXd ellipsoid_norms;

// // Metrics 
// bool m_metrics = false;
// std::array<double, 3> metrics;
// std::array<double, 3> ellipsoidMetrics;
// std::array<double, 3> cylinderMetrics;

// // Fitting methods
// using ellipsoidFit =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedEllipsoidFit>;
// using cylinderFit =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::OrientedCylinderFit>;
// using basket_FullyOrientedCylinder =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedParabolicCylinderFit>;
// using basket_BaseCylinder =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseParabolicCylinderFit>;
// using basket_BaseOrientedCylinder =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedParabolicCylinderFit>;
// using basket_NearOrientedCylinder=  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedParabolicCylinderFit>;

// using basket_FullyOrientedEllipsoid2D =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedEllipsoid2DFit>;
// using basket_BaseEllipsoid2D =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseEllipsoid2DFit>;
// using basket_BaseOrientedEllipsoid2D =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::BaseOrientedEllipsoid2DFit>;
// using basket_NearOrientedEllipsoid2D=  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::NearOrientedEllipsoid2DFit>;

// using basket_mongePatchFit = Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::MongePatchFit>;
                        
// /// Convenience function measuring and printing the processing time of F
// template <typename Functor>
// void measureTime( const std::string &actionName, Functor F ){
//     using namespace std::literals; // enables the usage of 24h instead of e.g. std::chrono::hours(24)

//     const std::chrono::time_point<std::chrono::steady_clock> start =
//             std::chrono::steady_clock::now();
//     F(); // run process
//     const auto end = std::chrono::steady_clock::now();
//     std::cout << actionName << " in " << (end - start) / 1ms << "ms.\n";
// }

// /// Show in polyscope the euclidean neighborhood of the selected point (iVertexSource), with smooth weighting function
// void colorizeEuclideanNeighborhood() {
//     int nvert = tree.index_data().size();
//     Eigen::VectorXd closest ( nvert );
//     closest.setZero();

//     SmoothWeightFunc w(NSize );

//     closest(iVertexSource) = 2;
//     const auto &p = tree.point_data()[iVertexSource];
//     for (int j : tree.range_neighbors(iVertexSource, NSize)){
//         const auto &q = tree.point_data()[j];
//         closest(j) = w.w( q.pos() - p.pos(), q ).first;
//     }
//     cloud->addScalarQuantity(  "range neighborhood", closest);
// }

// /// Show in polyscope the knn neighborhood of the selected point (iVertexSource)
// void colorizeKnn() {
//     int nvert = tree.index_data().size();
//     Eigen::VectorXd closest ( nvert );
//     closest.setZero();

//     closest(iVertexSource) = 2;
//     for (int j : tree.k_nearest_neighbors(iVertexSource, kNN)){
//         closest(j) = 1;
//     }
//     cloud->addScalarQuantity(  "knn neighborhood", closest);
// }

// /// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
// /// \note Functor is called only if fit is stable
// template<typename FitT, typename Functor>
// void processPointCloud(KdTree tree, const typename FitT::WeightFunction& w, Functor f){

//     int nvert = tree.index_data().size();

// #pragma omp parallel for
//     for (int i = 0; i < nvert; ++i) {
//         VectorType pos = tree.point_data()[i].pos();

//         for( int mm = 0; mm < mlsIter; ++mm) {
//             FitT fit;
//             fit.setWeightFunc(w);
//             fit.init( pos );

//             Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(i, NSize), tree.point_data() );
//             if (res == Ponca::STABLE){
//                 pos = fit.project( pos );
//                 if ( mm == mlsIter -1 ) // last mls step, calling functor
//                     f(i, fit, pos);
//             }
//             else {
//                 std::cerr << "Warning: fit " << i << " is not stable" << std::endl;
//             }

//         }
//     }
// }

// /// Generic processing function: traverse point cloud and compute mean, first and second curvatures + their direction
// /// \tparam FitT Defines the type of estimator used for computation
// template<typename FitT>
// void estimateDifferentialQuantities_impl(const std::string& name) {
//     int nvert = tree.index_data().size();
//     Eigen::VectorXd mean ( nvert ), kmin ( nvert ), kmax ( nvert );
//     Eigen::MatrixXd normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

//     measureTime( "[Ponca] Compute differential quantities using " + name,
//                  [&mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]() {
//         processPointCloud<FitT>(tree, SmoothWeightFunc(NSize),
//                                 [&mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]
//                                 ( int i, const FitT& fit, const VectorType& mlsPos){

//             mean(i) = fit.kMean();
            
//             kmax(i) = fit.kmax();
//             kmin(i) = fit.kmin();

//             normal.row( i ) = fit.primitiveGradient();
//             dmin.row( i )   = fit.kminDirection();
//             dmax.row( i )   = fit.kmaxDirection();

//             proj.row( i )   = mlsPos - tree.point_data()[i].pos();
//         });
//     });

//     measureTime( "[Polyscope] Update differential quantities",
//                  [&name, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]() {
//                      cloud->addScalarQuantity(name + " - Mean Curvature", mean)->setMapRange({-10,10});
//                      cloud->addScalarQuantity(name + " - K1", kmin)->setMapRange({-10,10});
//                      cloud->addScalarQuantity(name + " - K2", kmax)->setMapRange({-10,10});

//                      cloud->addVectorQuantity(name + " - normal", normal)->setVectorLengthScale(
//                              Scalar(2) * pointRadius);
//                     //  cloud->addVectorQuantity(name + " - K1 direction", dmin)->setVectorLengthScale(
//                     //          Scalar(2) * pointRadius);
//                     //  cloud->addVectorQuantity(name + " - K2 direction", dmax)->setVectorLengthScale(
//                     //          Scalar(2) * pointRadius);
//                      cloud->addVectorQuantity(name + " - projection", proj, polyscope::VectorType::AMBIENT);
//                  });
// }




// /// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
// /// \note Functor is called only if fit is stable
// template<typename FitT, typename Functor>
// void processUniquePoint(KdTree tree, int select, const typename FitT::WeightFunction& w, Functor f){

//     int nvert = tree.index_data().size();
//     VectorType pos = tree.point_data()[select].pos();

//     for( int mm = 0; mm < mlsIter; ++mm) {
//         FitT fit;
//         fit.setWeightFunc(w);
//         fit.init( pos );

//         Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(select, NSize), tree.point_data() );
        
//         if (res == Ponca::STABLE){
//             pos = fit.project( pos );
//             if ( mm == mlsIter -1 ) // last mls step, calling functor
//                 f(select, fit, pos);
//         }
//         else {
//             std::cerr << "Warning: fit " << select << " is not stable" << std::endl;
//         }
//     }
//     std::cout << "Done..." << std::endl;
// }



// void cleaning_point_cloud () {
//     if (cloudCurvature != nullptr)
//         cloudCurvature->remove();
//     if (cloudCubeProj != nullptr)
//         cloudCubeProj->remove();
//     if (cloudCube != nullptr)
//         cloudCube->remove();
//     if (cubePoints.size() != 0)
//         cubePoints.clear();
//     if (cubeProjPlanePoints.size() != 0)
//         cubeProjPlanePoints.clear();
//     if (cubeProjPoints.size() != 0)
//         cubeProjPoints.clear();
//     if (cubeProjNorms.size() != 0)
//         cubeProjNorms.clear();
//     if (colorsPlane.size() != 0)
//         colorsPlane.clear();
//     if (curvatures.size() != 0)
//         curvatures.clear();
// }

// void generate_sphere_curvature (VectorType pos, VectorType norm, Scalar curvature ){
//     double radius = 1.0/curvature;
//     // Normalize the normal vector
//     norm.normalize();
//     VectorType center = pos + radius * norm * (-1.0);
//     int n = 1000;
//     curvaturePoints = Eigen::MatrixXd(1, 3);
//     curvaturePoints.row(0) = center;
//     // curvaturePoints = create_sphere (center, radius, n);
//     cloudCurvature = polyscope::registerPointCloud("curvature sphere", curvaturePoints);    
//     cloudCurvature->setPointRadius(radius, false);
//     cloudCurvature->setTransparency(0.5);
// }

// template <typename fitting_procedure>
// void cloud_fitting_all(const std::string &actionName){
//     int nvert = tree.index_data().size();
//     Eigen::VectorXd alpha(nvert);
//     Eigen::VectorXd curvature(nvert);
//     Eigen::MatrixXd norms(nvert, 3);
//     Eigen::MatrixXd proj(nvert, 3);

//     Eigen::VectorXd gaussian_curve(nvert);
//     Eigen::VectorXd mean_curve(nvert);
//     Eigen::VectorXd max_curve(nvert);
//     Eigen::VectorXd min_curve(nvert);

//     // Put zero in all matrix curve and curvature
//     alpha.setZero();
//     curvature.setZero();
//     gaussian_curve.setZero();
//     mean_curve.setZero();
//     max_curve.setZero();
//     min_curve.setZero();


//     measureTime( actionName, [&proj, &alpha, &curvature, &norms, &gaussian_curve, &mean_curve, &max_curve, &min_curve]() {
//         processPointCloud<fitting_procedure>(tree, SmoothWeightFunc(NSize),
//             [&proj, &alpha, &curvature, &norms, &gaussian_curve, &mean_curve, &max_curve, &min_curve]
//             ( int i, const fitting_procedure& fit, const VectorType& pos){
//                 proj.row(i) = pos;
//                 norms.row(i) = fit.primitiveGradient(pos);
//                 curvature(i) = fit.curvature_k(pos);
//                 if constexpr (std::is_same <fitting_procedure, basket_FullyOrientedCylinder>::value
//                         || std::is_same <fitting_procedure, basket_BaseCylinder>::value
//                         || std::is_same <fitting_procedure, basket_BaseOrientedCylinder>::value
//                         || std::is_same <fitting_procedure, basket_NearOrientedCylinder>::value
//                         || std::is_same <fitting_procedure, cylinderFit>::value){
//                             // alpha(i) = fit.alpha_curvature();
//                             gaussian_curve(i) = fit.GaussianCurvature();
//                             mean_curve(i) = fit.kMean();
//                             max_curve(i) = fit.kmax();
//                             min_curve(i) = fit.kmin();
//                 }
//                 });
//     });
    
//     cloudCubeProj = polyscope::registerPointCloud("cube proj cylinder", proj);

//     cloudCubeProj->addVectorQuantity("normals", norms);

//     cloudCubeProj->addScalarQuantity("curvature", curvature);
//     cloudCubeProj->addScalarQuantity("alpha", alpha);
//     cloudCubeProj->addScalarQuantity("gaussian", gaussian_curve);
//     cloudCubeProj->addScalarQuantity("mean", mean_curve);
//     cloudCubeProj->addScalarQuantity("k1", max_curve);
//     cloudCubeProj->addScalarQuantity("k2", min_curve);
//     return;
// }

// template <typename fitting_procedure>
// void cloud_fitting_unique(int select){
//     VectorType pos = tree.point_data()[select].pos();
//     cubePoints = create_cube (pos);
//     int nvert = cubePoints.size();
//     Eigen::VectorXd alpha(nvert);
//     Eigen::VectorXd curvature(nvert);
//     Eigen::MatrixXd norms(nvert, 3);
//     Eigen::MatrixXd proj(nvert, 3);

//     // Put zero in all matrix curve and curvature 
//     alpha.setZero();
//     curvature.setZero();
//     proj.setZero();
//     norms.setZero();

//     processUniquePoint<fitting_procedure>(tree, select, SmoothWeightFunc(NSize),
//                             [&proj, &alpha, &curvature, &norms]
//                             ( int , const fitting_procedure& fit, const VectorType& pos){
//                             	    #pragma omp parallel for
//                                     for (int k = 0; k < cubePoints.size(); k++){
//                                         VectorType p = fit.project(cubePoints[k]);
//                                         if (p != cubePoints[k]){
//                                             proj.row(k) = p;
//                                             norms.row(k) = fit.primitiveGradient(p);
//                                             curvature(k) = fit.curvature_k(p);
//                                         }
//                                         if constexpr (std::is_same <fitting_procedure, basket_FullyOrientedCylinder>::value
//                                                 || std::is_same <fitting_procedure, basket_BaseCylinder>::value
//                                                 || std::is_same <fitting_procedure, basket_BaseOrientedCylinder>::value
//                                                 || std::is_same <fitting_procedure, basket_NearOrientedCylinder>::value){
//                                                     alpha(k) = fit.alpha_curvature();
//                                         }
//                                     }
//                                     generate_sphere_curvature(pos, fit.primitiveGradient(pos), fit.curvature_k(pos));
//                                 }
//                              );

//     cloudCube =     polyscope::registerPointCloud("cube", cubePoints);
//     cloudCube->setEnabled(false);
//     cloudCubeProj = polyscope::registerPointCloud("cube proj cylinder", proj);

//     cloudCubeProj->addVectorQuantity("normals", norms);

//     cloudCubeProj->addScalarQuantity("curvature", curvature);
//     cloudCubeProj->addScalarQuantity("alpha", alpha);
// }

// template <typename fitting_procedure>
// void cloud_fitting(const std::string &actionName, int select) {
//     using orientedCylinderFit =  Ponca::Basket<PPAdapter, SmoothWeightFunc, Ponca::FullyOrientedParabolicCylinderFit>;
//     cleaning_point_cloud();
//     if (select == -1){
//         cloud_fitting_all<fitting_procedure>(actionName);
//     }
//     else {
//         cloud_fitting_unique<fitting_procedure>(select);
//     }
// }

// template<typename VectorType>
// std::pair<Eigen::MatrixXd, Eigen::MatrixXd> create_parabolic_cylinder(double x_min, double x_max, double z_min, double z_max, int nb_x, int nb_z) {

//     Eigen::MatrixXd parabolic_verts = Eigen::MatrixXd(nb_x * nb_z, 3);
//     Eigen::MatrixXd parabolic_norms = Eigen::MatrixXd(nb_x * nb_z, 3);

//     double dx = (x_max - x_min) / (nb_x - 1);
//     double dz = (z_max - z_min) / (nb_z - 1);

//     for (int i = 0; i < nb_x; ++i) {
//         for (int j = 0; j < nb_z; ++j) {
//             double x = x_min + i * dx;
//             double z = z_min + j * dz;
//             double y = a_cylinder + bx_cylinder * x + bz_cylinder * z + c_a * (cx_cylinder * x + cz_cylinder * z)*(cx_cylinder * x + cz_cylinder * z);

//             parabolic_verts(i * nb_z + j, 0) =  x;
//             parabolic_verts(i * nb_z + j, 1) =  y;
//             parabolic_verts(i * nb_z + j, 2) =  z;

//             parabolic_norms(i * nb_z + j, 0) =  bx_cylinder + 2 * c_a * (cx_cylinder  * cx_cylinder * x + cx_cylinder * cz_cylinder * z);
//             parabolic_norms(i * nb_z + j, 1) =  -1;
//             parabolic_norms(i * nb_z + j, 2) =  bz_cylinder  + 2 * c_a * (cz_cylinder * cz_cylinder * z + cx_cylinder * cz_cylinder * x);
//         }
//     }
//     return std::make_pair(parabolic_verts, parabolic_norms);
// }

// void generate_parabolic_cylinder(double x_min, double x_max, double z_min, double z_max, int nb_x, int nb_z) {
//     if (cloud != nullptr)
//         cloud->remove();
//     polyscope::removePointCloud("cloud", false);

//     Eigen::MatrixXd curvature; 
//     Eigen::MatrixXd k1; 
//     Eigen::MatrixXd k2;
//     Eigen::MatrixXd mean;  
    
//     parabolic_pair = create_parabolic_cylinder<VectorType>(x_min, x_max, z_min, z_max, nb_x, nb_z);

//     parabolic_pair.first = add_noise(parabolic_pair.first, noise_sigma);
    
//     cloud = polyscope::registerPointCloud("cloud", parabolic_pair.first);
//     cloud->addVectorQuantity("normals", parabolic_pair.second);
//     buildKdTree(parabolic_pair.first, parabolic_pair.second, tree);
// }


// void loadObject (std::string filename) {
//     if (cloud != nullptr)
//         cloud->remove();
    
//     Eigen::MatrixXi meshF;
//     igl::readOBJ(filename, cloudV, meshF);
//     igl::per_vertex_normals(cloudV, meshF, cloudN);

//     // Check if normals have been properly loaded
//     int nbUnitNormal = cloudN.rowwise().squaredNorm().sum();
//     if ( nbUnitNormal != cloudV.rows() ) {
//         std::cerr << "[libIGL] An error occurred when computing the normal vectors from the mesh. Aborting..."
//                   << std::endl;
//         exit (EXIT_FAILURE);
//     }

//     cloudV = add_noise(cloudV, noise_sigma);

//     cloud = polyscope::registerPointCloud("cloud", cloudV);
//     cloud->addVectorQuantity("normals", cloudN);

//     buildKdTree(cloudV, cloudN, tree);
// }

// void callback_ellipsoid_fitting() {
//     ImGui::Separator();

//     ImGui::Text("2D ellipsoid fitting methods");

//     if (ImGui::Button ("Base 2D Ellipsoid"))
//         cloud_fitting<basket_BaseEllipsoid2D>("[PONCA] Base 2D Ellipsoid", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Base ell"))
//         cloud_fitting<basket_BaseEllipsoid2D>("[PONCA] Base 2D Ellipsoid", -1);

//     if (ImGui::Button ("Fully oriented 2D Ellipsoid"))
//         cloud_fitting<basket_FullyOrientedEllipsoid2D>("[PONCA] Fully Oriented 2D Ellipsoid", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Fully ell"))
//         cloud_fitting<basket_FullyOrientedEllipsoid2D>("[PONCA] Fully Oriented 2D Ellipsoid", -1);

//     if (ImGui::Button ("Base oriented 2D Ellipsoid"))
//         cloud_fitting<basket_BaseOrientedEllipsoid2D>("[PONCA] Base Oriented 2D Ellipsoid", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Base-oriented ell"))
//         cloud_fitting<basket_BaseOrientedEllipsoid2D>("[PONCA] Base Oriented 2D Ellipsoid", -1);

//     if (ImGui::Button ("Near oriented 2D Ellipsoid"))
//         cloud_fitting<basket_NearOrientedEllipsoid2D>("[PONCA] Near Oriented 2D Ellipsoid", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Near-oriented ell"))
//         cloud_fitting<basket_NearOrientedEllipsoid2D>("[PONCA] Near Oriented 2D Ellipsoid", -1);
// }

// void callback_cylinder_fitting() {


//     ImGui::Separator();

//     if (ImGui::Button ("Oriented 3D Ellipsoid on point"))
//         cloud_fitting<ellipsoidFit>("[PONCA] Ellipsoid 3D", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all"))
//         cloud_fitting<ellipsoidFit>("[PONCA] Ellipsoid 3D", -1);

//     ImGui::Text("2D cylindrical fitting methods");

//     if (ImGui::Button ("Base"))
//         cloud_fitting<basket_BaseCylinder>("[PONCA] Base Cylinder", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Base"))
//         cloud_fitting<basket_BaseCylinder>("[PONCA] Base Cylinder", -1);

//     if (ImGui::Button ("Fully oriented"))
//         cloud_fitting<basket_FullyOrientedCylinder>("[PONCA] Fully Oriented Cylinder", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Fully"))
//         cloud_fitting<basket_FullyOrientedCylinder>("[PONCA] Fully Oriented Cylinder", -1);

//     if (ImGui::Button ("Base oriented"))
//         cloud_fitting<basket_BaseOrientedCylinder>("[PONCA] Base Oriented Cylinder", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Base-oriented"))
//         cloud_fitting<basket_BaseOrientedCylinder>("[PONCA] Base Oriented Cylinder", -1);

//     if (ImGui::Button ("Near oriented"))
//         cloud_fitting<basket_NearOrientedCylinder>("[PONCA] Near Oriented Cylinder", iVertexSource);
//     ImGui::SameLine();
//     if (ImGui::Button ("all Near-oriented"))
//         cloud_fitting<basket_NearOrientedCylinder>("[PONCA] Near Oriented Cylinder", -1);
// }

// void callback_cylinder_tests () {

//     if (m_cylinder){
//         ImGui::Separator();
//         ImGui::Text("Parameters of the parabolic-cylindrical shape");
//         ImGui::Text("u_c + u_l^T q + a * (u_q^T q)^2");
//         if (ImGui::InputDouble("u_c parameter", &a_cylinder, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);

//         if (ImGui::InputDouble("u_lx parameter", &bx_cylinder, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);
//         ImGui::SameLine();
//         if (ImGui::InputDouble("u_lz parameter", &bz_cylinder, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);

//         if (ImGui::InputDouble("a parameter", &c_a, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);

//         if (ImGui::InputDouble("u_qx parameter", &cx_cylinder, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);
//         ImGui::SameLine();
//         if (ImGui::InputDouble("u_qz parameter", &cz_cylinder, -2, 2)) generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);

//         if (ImGui::InputInt("x_number", &x_cylinder, 0, 60 )      ||
//             ImGui::InputInt("z_number", &z_cylinder, 0, 60 ))
//                 generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);
//     }

//     ImGui::Separator();

//     callback_cylinder_fitting();

//     callback_ellipsoid_fitting();

//     if (ImGui::Button("Cylinder")){
//         cloud_fitting<cylinderFit>("[PONCA] Cylinder Fit", iVertexSource);
//     }
//     ImGui::SameLine();
//     if (ImGui::Button("all Cylinder")){
//         cloud_fitting<cylinderFit>("[PONCA] Cylinder Fit", -1);
//     }
// }

// /// Define Polyscope callbacks
// void callback() {

//     ImGui::PushItemWidth(100);

//     ImGui::InputDouble("Noise sigma", &noise_sigma, 0.0, 1.0);

//     if (ImGui::Button ("Generate Armadillo")) {
//         loadObject("assets/armadillo.obj");
//         m_cylinder = false;
//         m_metrics = false;
//     }
//     ImGui::SameLine();
//     if (ImGui::Button ("Generate Bunny")) {
//         loadObject("assets/bunny.obj");
//         m_cylinder = false;
//         m_metrics = false;
//     }
//     ImGui::SameLine();
//     if (ImGui::Button ("Generate Cylinder")) {
//         generate_parabolic_cylinder(-1, 1, -1, 1,x_cylinder, z_cylinder);
//         m_cylinder = true;
//         m_metrics = false;
//     }

//     ImGui::Separator();

//     ImGui::Text("Neighborhood collection");

//     ImGui::InputInt("k-neighborhood size", &kNN);
//     ImGui::InputFloat("neighborhood size", &NSize);
//     ImGui::InputInt("source vertex", &iVertexSource);
//     ImGui::InputInt("Nb MLS Iterations", &mlsIter);
//     ImGui::SameLine();
//     if (ImGui::Button("show knn")) colorizeKnn();
//     ImGui::SameLine();
//     if (ImGui::Button("show euclidean nei")) colorizeEuclideanNeighborhood();

//     ImGui::Separator();

//     ImGui::Text("Differential estimators");
//     if (ImGui::Button("Dry Run"))  mlsDryRun();
//     ImGui::SameLine();
//     if (ImGui::Button("Plane (PCA)")) estimateDifferentialQuantitiesWithPlane();
//     ImGui::SameLine();
//     if (ImGui::Button("APSS")) estimateDifferentialQuantitiesWithAPSS();
//     ImGui::SameLine();
//     if (ImGui::Button("ASO")) estimateDifferentialQuantitiesWithASO();

//     if (ImGui::Button("Ellipsoid")) estimateDifferentialQuantitiesWithEllipsoid();
// //    ImGui::SameLine();
//    if (ImGui::Button("Base Cylinder")) estimateDifferentialQuantitiesWithBaseCylinder();
//    ImGui::SameLine();
//    if (ImGui::Button("Base Ellipsoid")) estimateDifferentialQuantitiesWithBaseEllipsoid();

//    if (ImGui::Button("Base oriented Cylinder")) estimateDifferentialQuantitiesWithBaseOrientedCylinder();
//     ImGui::SameLine();
//    if (ImGui::Button("Base oriented Ellipsoid")) estimateDifferentialQuantitiesWithBaseOrientedEllipsoid();

//    if (ImGui::Button("Fully oriented Cylinder")) estimateDifferentialQuantitiesWithFullyOrientedCylinder();
//     ImGui::SameLine();
//     if (ImGui::Button("Fully oriented Ellipsoid")) estimateDifferentialQuantitiesWithFullyOrientedEllipsoid();

//    if (ImGui::Button("Near oriented Cylinder")) estimateDifferentialQuantitiesWithNearOrientedCylinder();
//     ImGui::SameLine();
//     if (ImGui::Button("Near oriented Ellipsoid")) estimateDifferentialQuantitiesWithNearOrientedEllipsoid();

//     if (ImGui::Button("Cylinder diff")) estimateDifferentialQuantitiesWithCylinder();


//     callback_cylinder_tests();

//     ImGui::PopItemWidth();

// }

#include "defines.h"

std::unique_ptr<GUI> gui;

void callback(){
    gui->mainCallBack();
}

int main(int argc, char **argv) {

    // Options
    polyscope::options::autocenterStructures = false;
    polyscope::options::programName = "poncascope";
    polyscope::view::windowWidth = 1024;
    polyscope::view::windowHeight = 1024;
    polyscope::options::groundPlaneEnabled = false;
    polyscope::view::bgColor = std::array<float, 4> {0.185, 0.185, 0.185, 0};


    // Initialize polyscope
    polyscope::init();

    gui = std::make_unique<GUI>(GUI());
    // Load a mesh
    // generate_parabolic_cylinder(-1, 1, -1, 1, x_cylinder, z_cylinder);

    // cloud->setPointRadius(pointRadius);

    // Add the callback
    polyscope::state::userCallback = callback;

    // Show the gui
    polyscope::show();

    return EXIT_SUCCESS;
}
