// GUI.h
#pragma once

#include <fstream>
#include <iostream>
#include <filesystem>
#include <mutex>
#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "MyPointCloud.h"
#include "CloudGeneration.h"
#include "PointProcessing.h"
#include "imgui_filedialog.h"
#include <Eigen/Dense>

class GUI {

    public:

        GUI(){
            // Initialize fileDialogue
            dialogInfo.title = "Choose File";
            dialogInfo.type = ImGuiFileDialogType_OpenFile;
            dialogInfo.directoryPath = std::filesystem::current_path();

            // Initialize polyscope
            selectedQuantities.resize(7, 1);

            // Initialize the point cloud
            selectedFile = assetsDir + "armadillo.obj";

            pointProcessing.measureTime("[Generation] Load object", [this](){
                loadObject(mainCloud, assetsDir + "armadillo.obj", 0.0f, 0.0f);
            });
            
            pointProcessing.update(mainCloud);
            polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
            addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
        }

        void mainCallBack();


    private: 

        bool cloudNeedsUpdate = false;

        // FILEDIALOGUE
        bool fileDialogOpen = false;
        ImFileDialogInfo dialogInfo;

        // State of the radio button (selection of a file or an implicit function)
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

        CylinderGenerator cylinderGenerator;
        MyPointCloud mainCloud;
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
        std::vector<std::string> quantityNames = {"Projections", "Normals", "Min curvature direction", "Max curvature direction", "Min curvature", "Max curvature", "Mean curvature"};
        std::vector<int> selectedQuantities; // Initialize to 1 all quantities
        
        std::string lastDryRun = "";

        bool all_computed = false;
        std::string methodName = "";

        void cloudComputing();

        void cloudComputingUpdateAll();

        template <typename FitT>
        void methodForCloudComputing(const std::string &metName);

        void cloudComputingParameters();

        void addQuantities(polyscope::PointCloud *pc, const std::string &name, const Eigen::MatrixXd &values);


}; // class GUI

#include "GUI.cpp"
