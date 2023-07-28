// GUI.hpp
#pragma once

#include <fstream>
#include <iostream>
#include <filesystem>
#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"
#include "MyPointCloud.h"
#include "CloudGeneration.h"
#include "PointProcessing.h"
#include <Eigen/Dense>

class GUI {

    public:

        GUI(){
            selectedQuantities.resize(6, 0);
            loadObject(mainCloud, assetsDir + "armadillo.obj", 0.0f, 0.0f);
            pointProcessing.update(mainCloud);
            polyscopeClouds.push_back(polyscope::registerPointCloud("mainCloud", mainCloud.getVertices()));
        }

        void mainCallBack();


    private: 
        // First one : main cloud, second one : projection cloud, others : temporary clouds
        std::vector<polyscope::PointCloud*> polyscopeClouds; 

        MyPointCloud mainCloud;
        CylinderGenerator cylinderGenerator;
        PointProcessing pointProcessing;

        bool cloudNeedsUpdate = false;

    private:

        Scalar pointRadius    = 0.005; /// < display radius of the point cloud

        // State of the radio button (selection of a file or an implicit function)
        int radioButtonCloudGeneration = 0;
        std::string assetsDir = "assets/";
        std::string selectedFile = "";
        int selectedFileIndex = -1;
        bool displayImplicitParameters = false;
        bool isCylinder = 0;

        float pointNoise = 0.0f;
        float normalNoise = 0.0f;

        void cloudGeneration();

        void generationFromFile();

        void generationFromImplicit();

        void cylinderParameters();

        // Selection of quantities to display
        // 0 : display projections
        // 0 : display normals
        // 1 : display min curvature direction
        // 2 : display max curvature direction
        // 3 : display min curvature
        // 4 : display max curvature
        // 5 : display mean curvature
        std::vector<int> selectedQuantities; // Initialiser avec 6 éléments, tous à '0'
        bool displayProjectedPointCloud = false;

        std::vector<std::string> quantityNames = {/*"Projections",*/"Normals", "Min curvature direction", "Max curvature direction", "Min curvature", "Max curvature", "Mean curvature"};

        void quantitiesParameters();

        // Make computation of the cloud

        bool computed = false;
        std::string methodName = "";

        void cloudComputing();

        void cloudComputingParameters();

        void addQuantities(int num_pc, const std::string &name, const Eigen::MatrixXd &values);


}; // class GUI

#include "GUI.hpp";
