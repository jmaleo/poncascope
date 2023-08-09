#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/point_cloud.h"

#include <iostream>
#include <utility>
#include <chrono>
#include "defines.h"

// gui stuff as a global variable to be able to call it from the callback
std::unique_ptr<GUI> gui;

void display_Helper() {
    std::cout << "Usage: poncascope [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help           show this help message and exit" << std::endl;
    std::cout << "  -f, --file           specify the file to load at startup" << std::endl;
    std::cout << "  -g, --gui            open the GUI at startup" << std::endl;
}

void parser(int argc, char **argv){
    if (argc == 1) return;
    std::vector<std::string> balises = std::vector<std::string>(argv + 1, argv + argc);
    for (int i = 0; i < balises.size(); i++) {
        std::string balise = balises[i];
        if (balise == "-h" || balise == "--help") {
            display_Helper();
            exit(EXIT_SUCCESS);
        }
        else if (balise == "-f" || balise == "--file") {
            if (i+1 >= balises.size()) {
                std::cerr << "Error: missing file name" << std::endl;
                exit(EXIT_FAILURE);
            }
            else {
                std::string file = balises[i+1];
                i++;
                std::cout << "Loading file: " << file << std::endl;
                gui->setSelectedFile(file);
            }
        }
        else if (balise == "-g" || balise == "--gui") {
            gui->setFileDialogOpen();
        }
        else {
            std::cerr << "Error: unknown option " << balise << std::endl;
            display_Helper();
            exit(EXIT_FAILURE);
        }
    }
}

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

    // Instantiate the GUI
    gui = std::make_unique<GUI>(GUI());

    // Parse command line options
    parser(argc, argv);

    // Init the GUI
    gui->init();

    // Add the callback
    polyscope::state::userCallback = callback;

    // Show the gui
    polyscope::show();

    return EXIT_SUCCESS;
}
