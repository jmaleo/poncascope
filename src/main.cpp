#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"

#include <iostream>
#include <utility>
#include <chrono>
#include "defines.h"

#include "CLI11.hpp"

// gui stuff as a global variable to be able to call it from the callback
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
    polyscope::view::bgColor = std::array<float, 4> {0.053, 0.053, 0.053, 0};

    CLI::App app{"Poncascope"};

    std::string cam_file = "";
    app.add_option("-c,--camera", cam_file, "Enter the view settings of a camera, stored as a json file");

    std::string input_file = "";
    app.add_option("-i, --input", input_file, "The input file to visualize, as .ply, .obj, .pts");

    std::string output_file = "";
    app.add_option("-o, --output", output_file, "The output file to save the image.");

    std::string method = "";
    app.add_option("-m, --method", method, "Estimator to use. Available methods : Plane (PCA), Plane (mean), APSS, ASO, CNC uniform, CNC independent, CNC hexa, CNC avg hexa, WaveJets, oriented WaveJets, Ellipsoid 3D, FO2D, B02D, B2D, NO2D, B0 Cylinder, B Cylinder, NO Cylinder, FO Cylinder, Varifold, Monge patch, Oriented Monge patch, Unoriented sphere");

    std::string kernel = "Smooth";
    app.add_option("-k, --kernel", kernel, "Kernel to use, default Smooth. Available kernels : Smooth, Constant, Wendland, Singular");

    std::string metric = "Mean curvature";
    app.add_option("--metric", metric, "Metric to use, default Mean curvature => ( Projections, Normals, Mean curvature, Min curvature, Max curvature, Min curvature direction, Max curvature direction, Shape index )");

    float radius = 0.1;
    app.add_option("-r, --radius", radius, "Radius of the neighborhood, default 0.1.");

    float mlsIter = 1;
    app.add_option("--mls", mlsIter, "Number of iterations for the moving least squares, default 1.");   

    float minBound = -5.0;
    app.add_option("--minBound", minBound, "Minimum bound of the visualization, default -5.");

    float maxBound = 5.0;
    app.add_option("--maxBound", maxBound, "Maximum bound of the visualization, default 5.");

    CLI11_PARSE(app, argc, argv);

    // Initialize polyscope
    polyscope::init();

    // Instantiate the GUI
    if (input_file != "")
        gui = std::make_unique<GUI>(GUI(input_file));
    else
        gui = std::make_unique<GUI>(GUI());

    // Add the callback
    polyscope::state::userCallback = callback;

    if (cam_file != ""){
        // Open the camera file
        std::ifstream file(cam_file);
        if (!file.is_open()){
            polyscope::error("Could not open the camera file");
            return EXIT_FAILURE;
        }
        // read the camera file to a string
        std::string cam_json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        // close the file
        file.close();
        // Set the view from the json file
        if (cam_json.empty())
            polyscope::error("The camera file is empty");
        else 
            polyscope::view::setViewFromJson(cam_json, false);
    }

    gui->setRadius (radius);
    gui->setMLSIter(mlsIter);
    gui->setKernel(kernel);
    gui->setMethod(method);
    gui->setCategory(metric);

    if (output_file != "")
        gui->oneShotCallBack(output_file, metric, minBound, maxBound);

    // Show the gui
    polyscope::show();


    return EXIT_SUCCESS;
}
