// GUI.h
#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include "../IO/CloudGeneration.h"
#include "../MyPointCloud.h"

#include "../PointProcessing.h"

#include "../../deps/imgui_filedialog.h"
#include <polyscope/messages.h>
#include <polyscope/pick.h>
#include <polyscope/point_cloud.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>

class GUI {

public:
    GUI() {
        // Initialize fileDialogue
        dialogInfo.title = "Choose File";
        dialogInfo.type = ImGuiFileDialogType_OpenFile;
        dialogInfo.directoryPath = std::filesystem::current_path();

        // Initialize polyscope
        selectedQuantities.resize(6, 0);

        // Initialize the point cloud
        selectedFile = assetsDir + "armadillo.obj";

        // Initialize the item selected method
        item_selected_method = 0;
    }

    GUI(const std::string &input_file) {
        // Initialize fileDialogue
        dialogInfo.title = "Choose File";
        dialogInfo.type = ImGuiFileDialogType_OpenFile;
        dialogInfo.directoryPath = std::filesystem::current_path();

        // Initialize polyscope
        selectedQuantities.resize(6, 0);

        // Initialize the point cloud
        selectedFile = input_file;

        // Initialize the item selected method
        item_selected_method = 0;
    }

    void init() {
        pointProcessing.measureTime("[Generation] Load object",
                                    [this]() { loadObject(mainCloud, selectedFile, pointNoise, normalNoise); });

        pointProcessing.update(mainCloud);
        polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
        addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
        remove_clouds();

        if (fastMode) {
            polyscope_mainCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
        }

        if (pointRadius > 0.0f)
            polyscope_mainCloud->setPointRadius(pointRadius);
    }

    void setProperty(const std::string &prop) {
        for (int i = 0; i < quantityNames.size(); i++) {
            if (quantityNames[i] == prop)
                selectedQuantities[i] = 1;
            else
                selectedQuantities[i] = 0;
        }
    }

    void setMethod(const std::string &met) {
        int i = 0;
        for (const auto &method: methods) {
            if (method == met) {
                item_selected_method = i;
                break;
            }
            i++;
        }
    }

    void setRadius(const float &r) { pointProcessing.NSize = r; }

    void setkNN(const int &k) { pointProcessing.kNN = k; }

    void setMLSIter(const int &mls) { pointProcessing.mlsIter = mls; }

    void setKernel(const std::string &kernel) {
        if (kernel == "Constant")
            weightFuncType = 0;
        else if (kernel == "Smooth")
            weightFuncType = 1;
        else if (kernel == "Wendland")
            weightFuncType = 2;
        else if (kernel == "Singular")
            weightFuncType = 3;
        else
            weightFuncType = 1;
    }

    void setFastMode() { fastMode = true; }

    void setVertexSource(const int &i) { pointProcessing.iVertexSource = i; }

    void setVertexRadius(const float &r) { pointRadius = r; }

    void setNoise(const float &noisePosition, const float &noiseNormal) {
        pointNoise = noisePosition;
        normalNoise = noiseNormal;
    }

    void mainCallBack();

    void remove_clouds() {
        for (polyscope::PointCloud *pc: polyscope_projectionClouds) {
            // delete the point cloud
            polyscope::removeStructure(pc->name, false);
        }

        for (polyscope::PointCloud *pc: polyscope_uniqueClouds) {
            // delete the point cloud
            polyscope::removeStructure(pc->name, false);
        }

        polyscope_projectionClouds.clear();
        polyscope_uniqueClouds.clear();
    }

    void remove_meshs() {
        for (polyscope::SurfaceMesh *sm: polyscope_meshs) {
            // delete the point cloud
            polyscope::removeStructure(sm, false);
        }
        polyscope_meshs.clear();

        // for (std::string slice_struct_name : polyscope_slices){
        //     // delete the surface mesh
        //     polyscope::removeStructure(slice_struct_name, false);
        // }
        polyscope::removeStructure(slicerName, false);
        // polyscope_slices.clear();
    }

    template<typename WeightFunc>
    void functionWithKernel(SampleMatrixType &values, const std::string &propertyName,
                            const std::vector<int> &vertexQueries, const std::vector<float> &radii) {
        if (propertyName == "Neighbors") {
            values = pointProcessing.colorizeEuclideanNeighborhood<WeightFunc>(vertexQueries, radii);
        } else {
            methodWithKernel<WeightFunc>();
            values = mainCloud.getDiffQuantities().getByName(propertyName);
        }
        std::cout << "[Property] " << propertyName << " computed" << std::endl;
    };

    void oneShotCallBack(const std::string &output_file, const std::string &propertyName,
                         const std::vector<int> &vertexQueries, const std::vector<float> &radii, float minBound,
                         float maxBound) {
        // one shot computing :
        offline_computing = true;

        SampleMatrixType values;

        if (propertyName != "") {
            if (propertyName == "real-normals") {
                values = mainCloud.getNormals();
            } else {
                switch (weightFuncType) {
                    case 0:
                        functionWithKernel<ConstWeightFunc>(values, propertyName, vertexQueries, radii);
                        break;
                        // methodWithKernel<SmoothWeightFunc>(); break;
                    case 2:
                        functionWithKernel<WendlandWeightFunc>(values, propertyName, vertexQueries, radii);
                        break;
                        // methodWithKernel<WendlandWeightFunc>(); break;
                    case 3:
                        functionWithKernel<SingularWeightFunc>(values, propertyName, vertexQueries, radii);
                        break;
                        // methodWithKernel<SingularWeightFunc>(); break;
                    default:
                        functionWithKernel<SmoothWeightFunc>(values, propertyName, vertexQueries, radii);
                        break;
                        // methodWithKernel<SmoothWeightFunc>(); break;
                }
                // const SampleMatrixType& values = mainCloud.getDiffQuantities().getByName(propertyName);
            }

            std::string colormap = "coolwarm";
            if (propertyName == "Neighbors") {
                colormap = "turbo";
                minBound = 0.0f;
                maxBound = 1.0f;
            }
            if (propertyName == "Shape index") {
                colormap = "viridis";
                minBound = -1.0f;
                maxBound = 1.0f;
            }

            if (values.cols() == 1) {
                // Make values beeing a vector
                SampleVectorType valuesVec = values.col(0);
                auto quantity = polyscope_mainCloud->addScalarQuantity(propertyName, valuesVec);
                // Set bound [-5, 5] for the scalar quantity
                quantity->setMapRange(std::pair<double, double>(minBound, maxBound));
                quantity->setColorMap(colormap);
                quantity->setEnabled(true);
            } else {
                auto quantity = polyscope_mainCloud->addVectorQuantity(propertyName, values);
                quantity->setEnabled(true);
            }
        }

        // Set SSAA anti-aliasing
        polyscope::options::ssaaFactor = 4;

        // take a screenshot
        polyscope::screenshot(output_file);

        exit(0);
    }

private:
    // ArgParser

private:
    bool offline_computing = false;

    bool cloudNeedsUpdate = false;

    // FILEDIALOGUE
    bool fileDialogOpen = false;
    ImFileDialogInfo dialogInfo;

    // State of the radio button (selection of a file or an implicit function)*
    int radioButtonCloudGeneration = 0;
    bool displayImplicitParameters = false;
    bool isCylinder = 0;
    bool isSinus = 0;

    // Point Cloud Informations
    std::string mainCloudName = "mainCloud";
    std::string assetsDir = "assets/";
    std::string selectedFile = "";
    int selectedFileIndex = -1;

    // Slicer Informations
    std::string slicerName = "slicer";
    bool isHDSlicer = false;
    float slice = 0.0f;
    int axis = 0;

    int displayVoxelResolution = 0;
    bool noEmptyVoxels = false;

    bool fastMode = false;
    Scalar pointRadius = 0.005; /// < display radius of the point cloud

    polyscope::PointCloud *polyscope_mainCloud;
    std::vector<polyscope::PointCloud *> polyscope_projectionClouds;
    std::vector<polyscope::PointCloud *> polyscope_uniqueClouds;
    std::vector<polyscope::SurfaceMesh *> polyscope_meshs;
    // std::vector<polyscope::SurfaceMesh*> polyscope_slices;

    CylinderGenerator cylinderGenerator;
    SinusGenerator sinusGenerator;
    MyPointCloud<Scalar> mainCloud;
    MyPointCloud<Scalar> tempCloud;

    PointProcessing pointProcessing;

    int item_selected_method = 0;
    // const char* methods[22] = { "Plane (PCA)", "Plane (mean)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC
    // hexa", "CNC avg hexa", "WaveJets", "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "B0 Cylinder", "B
    // Cylinder", "FO Cylinder", "Varifold", "Monge patch", "Oriented Monge patch", "Unoriented sphere", "Sphere fit" };
    const char *methods[31] = {"Plane (PCA)",
                               "Plane (mean)",
                               "APSS",
                               "ASO",
                               "CNC uniform",
                               "CNC independent",
                               "CNC hexa",
                               "CNC avg hexa",
                               "WaveJets",
                               "oriented WaveJets",
                               "Ellipsoid 3D",
                               "FO2D",
                               "B02D",
                               "B2D",
                               "B0 Cylinder",
                               "B Cylinder",
                               "FO Cylinder",
                               "VarifoldCov",
                               "VarifoldMean",
                               "Monge patch",
                               "Oriented Monge patch",
                               "Unoriented sphere",
                               "Sphere fit",
                               "Cov2D",
                               "NormCov2D",
                               "NormCov3D",
                               "ShapeOperator2D",
                               "Mean",
                               "PSS",
                               "Quadric",
                               "normalW"};


    float pointNoise = 0.0f;
    float normalNoise = 0.0f;

    void cloudGeneration();

    void generationFromFile();

    void fileResearch();

    void generationFromImplicit();

    void cylinderParameters();

    void sinusParameters();

    /// Save the camera settings to a file called "camera_settings.txt"
    /// in the current working directory
    void saveCameraSettings();

    void picking();

private:
    // Point Cloud Processing

    // quantities to display
    // 0 : display projections
    // 1 : display normals
    // 2 : display min curvature direction
    // 3 : display max curvature direction
    // 4 : display min curvature
    // 5 : display max curvature
    // 6 : display mean curvature
    std::vector<int> selectedQuantities; // Initialiser avec 6 éléments, tous à '0'
    std::vector<std::string> quantityNames = {/*"Projections",*/ "Normals",
                                              "Min curvature direction",
                                              "Max curvature direction",
                                              "Min curvature",
                                              "Max curvature",
                                              "Mean curvature"};

    std::string lastDryRun = "";
    std::string methodName = "";

    bool all_computed = false;
    bool unique_computed = false;
    bool displayProjectedPointCloud = false;

    int weightFuncType = 1; // 0 : constant, 1 : smooth, 2 : wendland, 3 : singular

    template<typename WeightFunc>
    void methodWithKernel() {

        //  { "Plane (PCA)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC hexa", "CNC avg hexa", "WaveJets",
        //  "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "NO2D", "B0 Cylinder", "B Cylinder", "NO
        //  Cylinder", "FO Cylinder" };

        switch (item_selected_method) {
            case (0):
                methodForCloudComputing<basket_planeFit<WeightFunc>, false>("Plane (PCA)");
                break;
            case (1):
                methodForCloudComputing<basket_meanPlaneFit<WeightFunc>>("Plane (mean)");
                break;
            case (2):
                methodForCloudComputing<basket_AlgebraicPointSetSurfaceFit<WeightFunc>>("APSS");
                break;
            case (3):
                methodForCloudComputing<basket_AlgebraicShapeOperatorFit<WeightFunc>>("ASO", false);
                break;
            case (4):
                methodForCloudComputing_OnlyTriangle("CNC uniform", 1);
                break;
            case (5):
                methodForCloudComputing_OnlyTriangle("CNC independent", 2);
                break;
            case (6):
                methodForCloudComputing_OnlyTriangle("CNC HexagramGeneration", 3);
                break;
            case (7):
                methodForCloudComputing_OnlyTriangle("CNC AvgHexagramGeneration", 4);
                break;
            case (8):
                methodForCloudComputing<basket_waveJets<WeightFunc>, false>("WaveJets", false);
                break;
            case (9):
                methodForCloudComputing<basket_orientedWaveJets<WeightFunc>>("oriented WaveJets", false);
                break;
            case (10):
                methodForCloudComputing<basket_ellipsoidFit<WeightFunc>>("Ellipsoid 3D");
                break;
            case (11):
                methodForCloudComputing<basket_FullyOrientedEllipsoid2DFit<WeightFunc>>("FO Ellipsoid2D");
                break;
            case (12):
                methodForCloudComputing<basket_BaseOrientedEllipsoid2DFit<WeightFunc>>("BO Ellipsoid2D");
                break;
            case (13):
                methodForCloudComputing<basket_BaseEllipsoid2DFit<WeightFunc>, false>("B Ellipsoid2D");
                break;
            // case (14) : methodForCloudComputing<basket_NearOrientedEllipsoid2DFit<WeightFunc >>("NO Ellipsoid2D");
            // break;
            case (14):
                methodForCloudComputing<basket_BaseOrientedCylinderFit<WeightFunc>>("BO Cylinder");
                break;
            case (15):
                methodForCloudComputing<basket_BaseCylinderFit<WeightFunc>, false>("B Cylinder");
                break;
            // case (17) : methodForCloudComputing<basket_NearOrientedCylinderFit<WeightFunc >>("NO Cylinder"); break;
            case (16):
                methodForCloudComputing<basket_FullyOrientedCylinderFit<WeightFunc>>("FO Cylinder");
                break;
            case (17):
                methodForCloudComputing<basket_varifoldsCovPlane>("VarifoldsCov", false);
                break;
            case (18):
                methodForCloudComputing<basket_varifoldsMeanPlane>("VarifoldsMean", false);
                break;
            case (19):
                methodForCloudComputing<basket_mongePatchFit<WeightFunc>, false>("Monge patch");
                break;
            case (20):
                methodForCloudComputing<basket_orientedMongePatchFit<WeightFunc>>("Oriented Monge patch");
                break;
            case (21):
                methodForCloudComputing<basket_UnorientedSphereFit<WeightFunc>, false>("Unoriented sphere");
                break;
            case (22):
                methodForCloudComputing<basket_SphereFit<WeightFunc>, false>("Sphere fit");
                break;
            case (23):
                methodForCloudComputing<basket_Covariance2DFit<WeightFunc>, false>("Cov2D");
                break;
            case (24):
                methodForCloudComputing<basket_NormalCovariance2DFit<WeightFunc>, false>("NormCov2D");
                break;
            case (25):
                methodForCloudComputing<basket_NormalCovariance3DFit<WeightFunc>, false>("NormCov3D");
                break;
            case (26):
                methodForCloudComputing<basket_ShapeOperator2DFit<WeightFunc>>("ShapeOperator2D");
                break;
            case (27):
                methodForCloudComputing<basket_MeanFit<WeightFunc>, false>("Mean");
                break;
            case (28):
                methodForCloudComputing<basket_pss<WeightFunc>>("PSS");
                break;
            case (29):
                methodForCloudComputing<basket_quadricFit<WeightFunc>, false>("Quadric");
                break;
            case (30):
                methodForCloudComputing<basket_normalWeingartenFit<WeightFunc>>("normalW");
                break;
            default:
                break;
        }
    }

    template<typename WeightFunc>
    SampleVectorType sliceWithKernel(const SampleMatrixType &vertices) {

        //  { "Plane (PCA)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC hexa", "CNC avg hexa", "WaveJets",
        //  "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "NO2D", "B0 Cylinder", "B Cylinder", "NO
        //  Cylinder", "FO Cylinder" };

        switch (item_selected_method) {
            case (0):
                return pointProcessing.evalScalarField_impl<basket_planeFit<WeightFunc>, false>("Plane (PCA)",
                                                                                                vertices);
            case (1):
                return pointProcessing.evalScalarField_impl<basket_meanPlaneFit<WeightFunc>>("Plane (mean)", vertices);
            case (2):
                return pointProcessing.evalScalarField_impl<basket_AlgebraicPointSetSurfaceFit<WeightFunc>>("APSS",
                                                                                                            vertices);
            case (3):
                return pointProcessing.evalScalarField_impl<basket_AlgebraicShapeOperatorFit<WeightFunc>>("ASO",
                                                                                                          vertices);
            case (4):
                return SampleVectorType::Zero(vertices.rows()); // CNC uniform
            case (5):
                return SampleVectorType::Zero(vertices.rows()); // CNC independent
            case (6):
                return SampleVectorType::Zero(vertices.rows()); // CNC hexa
            case (7):
                return SampleVectorType::Zero(vertices.rows()); // CNC avg hexa
            case (8):
                return SampleVectorType::Zero(vertices.rows()); // WaveJets
            case (9):
                return SampleVectorType::Zero(vertices.rows()); // oriented WaveJets
            case (10):
                return pointProcessing.evalScalarField_impl<basket_ellipsoidFit<WeightFunc>>("Ellipsoid 3D", vertices);
            case (11):
                return pointProcessing.evalScalarField_impl<basket_FullyOrientedEllipsoid2DFit<WeightFunc>>(
                        "FO Ellipsoid2D", vertices);
            case (12):
                return pointProcessing.evalScalarField_impl<basket_BaseOrientedEllipsoid2DFit<WeightFunc>>(
                        "BO Ellipsoid2D", vertices);
            case (13):
                return pointProcessing.evalScalarField_impl<basket_BaseEllipsoid2DFit<WeightFunc>, false>(
                        "B Ellipsoid2D", vertices);
            // case (14) : return pointProcessing.evalScalarField_impl<basket_NearOrientedEllipsoid2DFit<WeightFunc
            // >>("NO Ellipsoid2D", vertices);
            case (14):
                return pointProcessing.evalScalarField_impl<basket_BaseOrientedCylinderFit<WeightFunc>>("BO Cylinder",
                                                                                                        vertices);
            case (15):
                return pointProcessing.evalScalarField_impl<basket_BaseCylinderFit<WeightFunc>, false>("B Cylinder",
                                                                                                       vertices);
            // case (17) : return pointProcessing.evalScalarField_impl<basket_NearOrientedCylinderFit<WeightFunc >>("NO
            // Cylinder", vertices);
            case (16):
                return pointProcessing.evalScalarField_impl<basket_FullyOrientedCylinderFit<WeightFunc>>("FO Cylinder",
                                                                                                         vertices);
            case (17):
                return SampleVectorType::Zero(vertices.rows()); // VarifoldCov
            case (18):
                return SampleVectorType::Zero(vertices.rows()); // VarifoldMean
            case (19):
                return pointProcessing.evalScalarField_impl<basket_mongePatchFit<WeightFunc>, false>("Monge patch",
                                                                                                     vertices);
            case (20):
                return pointProcessing.evalScalarField_impl<basket_orientedMongePatchFit<WeightFunc>>(
                        "Oriented Monge patch", vertices);
            case (21):
                return pointProcessing.evalScalarField_impl<basket_UnorientedSphereFit<WeightFunc>, false>(
                        "Unoriented sphere", vertices);
            case (22):
                return pointProcessing.evalScalarField_impl<basket_SphereFit<WeightFunc>, false>("Sphere fit",
                                                                                                 vertices);
            case (23):
                return pointProcessing.evalScalarField_impl<basket_Covariance2DFit<WeightFunc>, false>("Cov2D",
                                                                                                       vertices);
            case (24):
                return pointProcessing.evalScalarField_impl<basket_NormalCovariance2DFit<WeightFunc>, false>(
                        "NormCov2D", vertices);
            case (25):
                return SampleVectorType::Zero(vertices.rows()); // NormCov3D
            case (26):
                return pointProcessing.evalScalarField_impl<basket_ShapeOperator2DFit<WeightFunc>, false>(
                        "ShapeOperator2D", vertices);
            case (27):
                return SampleVectorType::Zero(vertices.rows()); // MeanFit
            case (28):
                return SampleVectorType::Zero(vertices.rows()); // PSS
            case (29):
                return pointProcessing.evalScalarField_impl<basket_quadricFit<WeightFunc>, false>("Quadric", vertices);
            case (30):
                return pointProcessing.evalScalarField_impl<basket_normalWeingartenFit<WeightFunc>>("normalW",
                                                                                                    vertices);
            default:
                break;
        }
        return SampleVectorType::Zero(vertices.rows());
    }

    void cloudComputing();

    void cloudComputingUpdateAll();

    void cloudComputingUpdateUnique();

    void cloudComputingSlices();

    template<typename FitT, bool isSigned = true>
    void methodForCloudComputing(const std::string &metName, bool unique = true);

    void methodForCloudComputing_OnlyTriangle(const std::string &metName, const int &type);

    void quantitiesParameters();

    void cloudComputingParameters();

    void addQuantities(polyscope::PointCloud *pc, const std::string &name, const SampleMatrixType &values);


}; // class GUI

#include "GUI.cpp"
