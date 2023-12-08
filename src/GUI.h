// GUI.h
#pragma once

#include <fstream>
#include <iostream>
#include <filesystem>
#include <mutex>
#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "MyPointCloud.h"
#include "CloudGeneration.h"
#include "PointProcessing.h"
#include "imgui_filedialog.h"
#include <Eigen/Dense>
#include "imgui_filedialog.h"

class GUI {

    public:

        GUI(){
            // Initialize fileDialogue
            dialogInfo.title = "Choose File";
            dialogInfo.type = ImGuiFileDialogType_OpenFile;
            dialogInfo.directoryPath = std::filesystem::current_path();

            // Initialize polyscope
            selectedQuantities.resize(6, 0);

            // Initialize the point cloud
            selectedFile = assetsDir + "armadillo.obj";

            pointProcessing.measureTime("[Generation] Load object", [this](){
                loadObject(mainCloud, selectedFile, 0.0f, 0.0f);
            });
            
            pointProcessing.update(mainCloud);
            polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
            addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
            remove_clouds();
        }

        void mainCallBack();

        void remove_clouds(){
            for (polyscope::PointCloud* pc : polyscope_projectionClouds){
                // delete the point cloud
                polyscope::removeStructure(pc->name, false);
            }
            for (polyscope::PointCloud* pc : polyscope_uniqueClouds){
                // delete the point cloud
                polyscope::removeStructure(pc->name, false);
            }
            polyscope_projectionClouds.clear();
            polyscope_uniqueClouds.clear();
        }

        void remove_meshs(){
            for (polyscope::SurfaceMesh* sm : polyscope_meshs){
                // delete the point cloud
                polyscope::removeStructure(sm->name, false);
            }
            polyscope_meshs.clear();
        }


    private: 

        bool cloudNeedsUpdate = false;

        // FILEDIALOGUE
        bool fileDialogOpen = false;
        ImFileDialogInfo dialogInfo;

        // State of the radio button (selection of a file or an implicit function)*
        int radioButtonCloudGeneration = 0;
        bool displayImplicitParameters = false;
        bool isCylinder = 0;

        // Point Cloud Informations
        std::string mainCloudName = "mainCloud";
        std::string assetsDir = "assets/";
        std::string selectedFile = "";
        int selectedFileIndex = -1;

        Scalar pointRadius    = 0.005; /// < display radius of the point cloud
        
        polyscope::PointCloud* polyscope_mainCloud;
        std::vector<polyscope::PointCloud*> polyscope_projectionClouds;
        std::vector<polyscope::PointCloud*> polyscope_uniqueClouds;
        std::vector<polyscope::SurfaceMesh*> polyscope_meshs;

        CylinderGenerator cylinderGenerator;
        MyPointCloud<Scalar> mainCloud;
        MyPointCloud<Scalar> tempCloud;
        
        PointProcessing pointProcessing;
    
        float pointNoise = 0.0f;
        float normalNoise = 0.0f;

        void cloudGeneration();

        void generationFromFile();

        void fileResearch ();

        void generationFromImplicit();

        void cylinderParameters();

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
        std::vector<std::string> quantityNames = {/*"Projections",*/"Normals", "Min curvature direction", "Max curvature direction", "Min curvature", "Max curvature", "Mean curvature"};
        
        std::string lastDryRun = "";
        std::string methodName = "";

        bool all_computed = false;
        bool unique_computed = false;
        bool displayProjectedPointCloud = false;

        int weightFuncType = 1;

        template<typename WeightFunc>
        void
        methodWithKernel(){

            methodForCloudComputing<basket_planeFit<WeightFunc >>("Plane (PCA)");

            methodForCloudComputing<basket_AlgebraicPointSetSurfaceFit<WeightFunc >>("APSS");

            methodForCloudComputing<basket_AlgebraicShapeOperatorFit<WeightFunc >>("ASO", false);

            // methodForCloudComputing<basket_waveJets<WeightFunc >>("WaveJets", false);

            methodForCloudComputing<basket_orientedWaveJets<WeightFunc >>("oriented WaveJets", false);

            methodForCloudComputing<basket_ellipsoidFit<WeightFunc >>("Ellipsoid 3D");
            // //Same as ellipsoidFit
            // methodForCloudComputing<basket_hyperboloidFit_Diff<WeightFunc >>("Hyperboloid 3D basket Diff");

            // methodForCloudComputing<basket_hyperboloidFit<WeightFunc >>("Hyperboloid 3D ACP");

            methodForCloudComputing<basket_FullyOrientedEllipsoid2DFit<WeightFunc >>("FO Ellipsoid2D");
            methodForCloudComputing<basket_BaseOrientedEllipsoid2DFit<WeightFunc >>("B0 Ellipsoid2D");
            methodForCloudComputing<basket_BaseEllipsoid2DFit<WeightFunc >>("B Ellipsoid2D");
            methodForCloudComputing<basket_NearOrientedEllipsoid2DFit<WeightFunc >>("N0 Ellipsoid2D");

            // methodForCloudComputing<basket_BaseOrientedCylinderFit<WeightFunc >>("B0 Cylinder");
            // methodForCloudComputing<basket_BaseCylinderFit<WeightFunc >>("B Cylinder");
            // methodForCloudComputing<basket_NearOrientedCylinderFit<WeightFunc >>("NO Cylinder");
            // methodForCloudComputing<basket_FullyOrientedCylinderFit<WeightFunc >>("FO Cylinder");

        }

        void cloudComputing();

        void cloudComputingUpdateAll();

        void cloudComputingUpdateUnique();

        template <typename FitT>
        void methodForCloudComputing(const std::string &metName, bool unique=true);

        void methodForCloudComputing_OnlyTriangle(const std::string &metName, const int& type);

        void quantitiesParameters();

        void cloudComputingParameters();

        void addQuantities(polyscope::PointCloud *pc, const std::string &name, const SampleMatrixType &values);


}; // class GUI

#include "GUI.cpp"
