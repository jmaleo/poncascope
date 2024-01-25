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

            // Initialize the item selected method
            item_selected_method = 0;
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

        // Slicer Informations
        std::string slicerName = "slicer";
        bool isHDSlicer = false;
        float slice = 0.0f;
        int axis = 0;

        Scalar pointRadius    = 0.005; /// < display radius of the point cloud
        
        polyscope::PointCloud* polyscope_mainCloud;
        std::vector<polyscope::PointCloud*> polyscope_projectionClouds;
        std::vector<polyscope::PointCloud*> polyscope_uniqueClouds;
        std::vector<polyscope::SurfaceMesh*> polyscope_meshs;
        // std::vector<polyscope::SurfaceMesh*> polyscope_slices;

        CylinderGenerator cylinderGenerator;
        MyPointCloud<Scalar> mainCloud;
        MyPointCloud<Scalar> tempCloud;
        
        PointProcessing pointProcessing;

        int item_selected_method = 0;
        const char* methods[23] = { "Plane (PCA)", "Plane (mean)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC hexa", "CNC avg hexa", "WaveJets", "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "NO2D", "B0 Cylinder", "B Cylinder", "NO Cylinder", "FO Cylinder", "Varifold", "Monge patch", "Oriented Monge patch", "Unoriented sphere" };

    
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

            //  { "Plane (PCA)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC hexa", "CNC avg hexa", "WaveJets", "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "NO2D", "B0 Cylinder", "B Cylinder", "NO Cylinder", "FO Cylinder" };
            
            switch (item_selected_method) {
                case (0)  : methodForCloudComputing<basket_planeFit<WeightFunc >, false>("Plane (PCA)"); break;
                case (1)  : methodForCloudComputing<basket_meanPlaneFit<WeightFunc >>("Plane (mean)"); break;
                case (2)  : methodForCloudComputing<basket_AlgebraicPointSetSurfaceFit<WeightFunc >>("APSS"); break;
                case (3)  : methodForCloudComputing<basket_AlgebraicShapeOperatorFit<WeightFunc >>("ASO", false); break;
                // case (4)  : methodForCloudComputing<basket_CNC_uniform<WeightFunc >>("CNC uniform"); break;
                // case (5)  : methodForCloudComputing<basket_CNC_independent<WeightFunc >>("CNC independent"); break;
                // case (6)  : methodForCloudComputing<basket_CNC_hexa<WeightFunc >>("CNC hexa"); break;
                // case (7)  : methodForCloudComputing<basket_CNC_avg_hexa<WeightFunc >>("CNC avg hexa"); break;
                case (8)  : methodForCloudComputing<basket_waveJets<WeightFunc >, false>("WaveJets", false); break;
                case (9)  : methodForCloudComputing<basket_orientedWaveJets<WeightFunc >>("oriented WaveJets", false); break;
                case (10)  : methodForCloudComputing<basket_ellipsoidFit<WeightFunc >>("Ellipsoid 3D"); break;
                case (11) : methodForCloudComputing<basket_FullyOrientedEllipsoid2DFit<WeightFunc >>("FO Ellipsoid2D"); break;
                case (12) : methodForCloudComputing<basket_BaseOrientedEllipsoid2DFit<WeightFunc >>("BO Ellipsoid2D"); break;
                case (13) : methodForCloudComputing<basket_BaseEllipsoid2DFit<WeightFunc >, false>("B Ellipsoid2D"); break;
                case (14) : methodForCloudComputing<basket_NearOrientedEllipsoid2DFit<WeightFunc >>("NO Ellipsoid2D"); break;
                case (15) : methodForCloudComputing<basket_BaseOrientedCylinderFit<WeightFunc >>("BO Cylinder"); break;
                case (16) : methodForCloudComputing<basket_BaseCylinderFit<WeightFunc >, false>("B Cylinder"); break;
                case (17) : methodForCloudComputing<basket_NearOrientedCylinderFit<WeightFunc >>("NO Cylinder"); break;
                case (18) : methodForCloudComputing<basket_FullyOrientedCylinderFit<WeightFunc >>("FO Cylinder"); break;
                case (19) : methodForCloudComputing<basket_varifold>("Varifold", false); break;
                case (20) : methodForCloudComputing<basket_mongePatchFit<WeightFunc>, false>("Monge patch"); break;
                case (21) : methodForCloudComputing<basket_orientedMongePatchFit<WeightFunc>>("Oriented Monge patch"); break;
                case (22) : methodForCloudComputing<basket_UnorientedSphereFit<WeightFunc>, false>("Unoriented sphere"); break;
                default : break; 
            }
        }

        template<typename WeightFunc>
        SampleVectorType
        sliceWithKernel(const SampleMatrixType& vertices){

            //  { "Plane (PCA)", "APSS", "ASO", "CNC uniform", "CNC independent", "CNC hexa", "CNC avg hexa", "WaveJets", "oriented WaveJets", "Ellipsoid 3D", "FO2D", "B02D", "B2D", "NO2D", "B0 Cylinder", "B Cylinder", "NO Cylinder", "FO Cylinder" };
            
            switch (item_selected_method) {
                case (0)  : return pointProcessing.evalScalarField_impl<basket_planeFit<WeightFunc >, false>("Plane (PCA)", vertices);
                case (1)  : return pointProcessing.evalScalarField_impl<basket_meanPlaneFit<WeightFunc >>("Plane (mean)", vertices);
                case (2)  : return pointProcessing.evalScalarField_impl<basket_AlgebraicPointSetSurfaceFit<WeightFunc >>("APSS", vertices);
                case (3)  : return pointProcessing.evalScalarField_impl<basket_AlgebraicShapeOperatorFit<WeightFunc >>("ASO", vertices);
                case (4)  : return SampleVectorType::Zero(vertices.rows()); // CNC uniform
                case (5)  : return SampleVectorType::Zero(vertices.rows()); // CNC independent
                case (6)  : return SampleVectorType::Zero(vertices.rows()); // CNC hexa
                case (7)  : return SampleVectorType::Zero(vertices.rows()); // CNC avg hexa
                case (8)  : return SampleVectorType::Zero(vertices.rows()); // WaveJets
                case (9)  : return SampleVectorType::Zero(vertices.rows()); // oriented WaveJets
                case (10)  : return pointProcessing.evalScalarField_impl<basket_ellipsoidFit<WeightFunc >>("Ellipsoid 3D", vertices);
                case (11) : return pointProcessing.evalScalarField_impl<basket_FullyOrientedEllipsoid2DFit<WeightFunc >>("FO Ellipsoid2D", vertices);
                case (12) : return pointProcessing.evalScalarField_impl<basket_BaseOrientedEllipsoid2DFit<WeightFunc >>("BO Ellipsoid2D", vertices);
                case (13) : return pointProcessing.evalScalarField_impl<basket_BaseEllipsoid2DFit<WeightFunc >, false>("B Ellipsoid2D", vertices);
                case (14) : return pointProcessing.evalScalarField_impl<basket_NearOrientedEllipsoid2DFit<WeightFunc >>("NO Ellipsoid2D", vertices);
                case (15) : return pointProcessing.evalScalarField_impl<basket_BaseOrientedCylinderFit<WeightFunc >>("BO Cylinder", vertices);
                case (16) : return pointProcessing.evalScalarField_impl<basket_BaseCylinderFit<WeightFunc >, false>("B Cylinder", vertices);
                case (17) : return pointProcessing.evalScalarField_impl<basket_NearOrientedCylinderFit<WeightFunc >>("NO Cylinder", vertices);
                case (18) : return pointProcessing.evalScalarField_impl<basket_FullyOrientedCylinderFit<WeightFunc >>("FO Cylinder", vertices);
                case (19) : return SampleVectorType::Zero(vertices.rows()); // Varifold
                case (20) : return pointProcessing.evalScalarField_impl<basket_mongePatchFit<WeightFunc>, false>("Monge patch", vertices);
                case (21) : return pointProcessing.evalScalarField_impl<basket_orientedMongePatchFit<WeightFunc>>("Oriented Monge patch", vertices);
                case (22) : return pointProcessing.evalScalarField_impl<basket_UnorientedSphereFit<WeightFunc>, false>("Unoriented sphere", vertices);
                default : break; 
            }
            return SampleVectorType::Zero(vertices.rows());
        }

        void cloudComputing();

        void cloudComputingUpdateAll();

        void cloudComputingUpdateUnique();

        void cloudComputingSlices();

        template <typename FitT, bool isSigned = true>
        void methodForCloudComputing(const std::string &metName, bool unique=true);

        void methodForCloudComputing_OnlyTriangle(const std::string &metName, const int& type);

        void quantitiesParameters();

        void cloudComputingParameters();

        void addQuantities(polyscope::PointCloud *pc, const std::string &name, const SampleMatrixType &values);



}; // class GUI

#include "GUI.cpp"
