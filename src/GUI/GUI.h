// GUI.h
#pragma once

#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include "../IO/CloudGeneration.h"
#include "../IO/PointCloudDiff.h"

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
        polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.points);
        addQuantities(polyscope_mainCloud, "real normals", mainCloud.normals);
        remove_clouds();

        if (fastMode) {
            polyscope_mainCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
        }
        std::cout << "init done" << std::endl;
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

    void setRadius(const float &r) { radius = r; }

    void setkNN(const int &k) { kNN = k; }

    void setMLSIter(const int &mls) { mls_iter = mls; }

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

    void setVertexSource(const int &i) { iVertexSource = i; }

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
            values = pointProcessing.colorizeNeighbors<WeightFunc>(vertexQueries, radii);
        } else {
            pointProcessing.computeDiffQuantities<WeightFunc>(methodName, mainCloud);
            auto diffQuantities = mainCloud.getDiffQuantities();
            values = diffQuantities.getByName(propertyName);
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
                values = mainCloud.normals;
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

            if ( values.cols() == 1 ) {
                // Make values beeing a vector
                auto quantity = polyscope_mainCloud->addScalarQuantity(propertyName, static_cast<Eigen::VectorXd>(values));
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
    std::string assetsDir = "../assets/";
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
    PointCloudDiff<Scalar> mainCloud = PointCloudDiff<Scalar>("MainCloud");
    PointCloudDiff<Scalar> tempCloud = PointCloudDiff<Scalar>("TempCloud");

    PointProcessing pointProcessing;

    int item_selected_method = 0;
    const char *methods[26] = {
        "MeanPLANE",
        "APSS",
        "ASO",
        "OrientedPC-MLS",
        "Direct2-Monge",
        "Ellipsoid",
        "Varifolds",
        "VarifoldsMeanPlane",
        "Oriented2-Monge",
        "ShapeOperator",
        "PCA",
        "Sphere",
        "UnorientedSphere",
        "PC-MLS",
        "2-Monge",
        "Cov2D",
        "NormCov2D",
        "NormCov3D",
        "Mean",
        "3DQuadric",
        "OrientedWaveJets",
        "Independent",
        "Uniform",
        "AvgHexagram",
        "Hexagram",
        "WaveJets",
    };


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

    void cloudComputing();

    void cloudComputingUpdateAll();

    void cloudComputingUpdateUnique();

    void cloudComputingSlices();

    template<typename FitT, bool isSigned = true>
    void methodForCloudComputing(const std::string &metName, bool unique = true);

    void methodForCloudComputing_OnlyTriangle(const std::string &metName, const int &type);

    void quantitiesParameters();

    void cloudComputingParameters();

    void addQuantities(polyscope::PointCloud *pc, const std::string &name, const SampleMatrixType &values) {
        if (values.cols() == 1) {
            auto quantity = pc->addScalarQuantity(name, values.col(0));
            if (name != "knn" && name != "euclidean nei" && name != "ShapeIndex") {
                quantity->setMapRange(std::pair<double, double>(-5, 5));
                quantity->setColorMap("coolwarm");
            } else {
                if (name == "ShapeIndex") {
                    quantity->setMapRange(std::pair<double, double>(-1, 1));
                    quantity->setColorMap("viridis");
                } else {
                    quantity->setColorMap("turbo");
                    quantity->setEnabled(true);
                }
            }
        } else {
            pc->addVectorQuantity(name, values);
        }
    }

    template <typename WeightFunc>
    SampleVectorType sliceWithKernel (const SampleMatrixType& pos) {
        std::string currentSelectedMethod = std::string(methods[item_selected_method]);
        SampleVectorType values = pointProcessing.getScalarField<WeightFunc>(currentSelectedMethod, pos);
        return values;
    }

    template<typename WeightFunc>
    void methodWithKernel() {
        std::string currentSelectedMethod = std::string(methods[item_selected_method]);
        if ( currentSelectedMethod == "AvgHexagram"
        || currentSelectedMethod == "Hexagram"
        || currentSelectedMethod == "Uniform"
        || currentSelectedMethod == "Independent" ) {
            methodForCloudComputing_OnlyTriangle(currentSelectedMethod, 0);
        } else {
            methodForCloudComputing<WeightFunc>(currentSelectedMethod);
        }
    }

    static bool isQuantityMatrix(const std::string &name) {
        return name == "Min curvature direction" || name == "Max curvature direction" || name == "Normals";
    }

}; // class GUI

#include "GUI.cpp"
