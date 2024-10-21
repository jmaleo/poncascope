#include "polyscope/curve_network.h"
void GUI::mainCallBack() {
    // Create a window
    ImGui::PushItemWidth(100);
    // The size of the window, the position is set by default
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    // Camera settings :
    saveCameraSettings();

    cloudGeneration();

    if (isCylinder) {
        cylinderParameters();
    }
    if (isSinus) {
        sinusParameters();
    }
    ImGui::Separator();

    if (cloudNeedsUpdate) {
        pointProcessing.update(mainCloud);
        pointProcessing.measureTime("[Polyscope] Update current main cloud", [this]() {
            polyscope::removeStructure(mainCloudName, false);
            polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.points);
            addQuantities(polyscope_mainCloud, "real normals", mainCloud.normals);
            // Remove other clouds
            remove_clouds();
            remove_meshs();
        });
        lastDryRun = "";
        cloudNeedsUpdate = false;
    }

    quantitiesParameters();

    ImGui::Separator();

    picking();

    cloudComputing();

}

//////////////////////////////////////////////////////////////////////
////                           SETTINGS                           ////
//////////////////////////////////////////////////////////////////////

void GUI::saveCameraSettings() {
    // Save the camera settings
    if (ImGui::Button("Save camera settings")) {
        std::string base_name = "cameraSettings";
        std::string num = "0";
        std::string extension = ".json";
        std::string view = polyscope::view::getViewAsJson();
        // While it exists a file with the same name, add a number at the end of the name
        while (std::filesystem::exists(base_name + num + extension))
            num = std::to_string(std::stoi(num) + 1);
        std::ofstream file(base_name + num + extension);
        file << view;
        file.close();
    }
}

//////////////////////////////////////////////////////////////////////
////                       CLOUD GENERATING                       ////
//////////////////////////////////////////////////////////////////////

void GUI::fileResearch() {
    if (FileDialog(&fileDialogOpen, &dialogInfo)) {
        // L'utilisateur a sélectionné un fichier et a cliqué sur "Open".
        // Le chemin du fichier sélectionné est dans dialogInfo.resultPath.
        fileDialogOpen = false; // Ferme la boîte de dialogue pour la prochaine fois
        selectedFile = dialogInfo.resultPath.string();
        selectedFileIndex = -1;
    }

    // open = false;
    return;
}


void GUI::cloudGeneration() {
    // Point noise and normal noise selection with sliders
    ImGui::SliderFloat("Point noise", &pointNoise, 0.0f, 5.0f);
    ImGui::SameLine();
    ImGui::SliderFloat("Normal noise", &normalNoise, 0.0f, 5.0f);

    // Display the 2 radios buttons, one for file selection and the other for implicit function
    ImGui::RadioButton("File selection", &radioButtonCloudGeneration, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Implicit function", &radioButtonCloudGeneration, 1);

    // If the user selected the file selection
    if (radioButtonCloudGeneration == 0) {
        generationFromFile();
    }
    // If the user selected the implicit function
    else if (radioButtonCloudGeneration == 1) {
        selectedFile = "";
        generationFromImplicit();
    }
}

void GUI::generationFromFile() {
    // Display all the existing file in the assets directory
    std::vector<std::string> fileNames;
    for (const auto &entry: std::filesystem::directory_iterator(assetsDir)) {
        // Take only the .obj and .ply files, and only the name of the file
        if (entry.path().extension() == ".obj" || entry.path().extension() == ".ply" ||
            entry.path().extension() == ".pts") {
            fileNames.push_back(entry.path().filename().string());
        }
    }

    std::vector<const char *> fileNamesCStr;
    for (const auto &str: fileNames) {
        fileNamesCStr.push_back(str.c_str());
    }

    // Display the list of files
    ImGui::ListBox("Files", &selectedFileIndex, fileNamesCStr.data(), fileNamesCStr.size());

    ImGui::SameLine();

    if (ImGui::Button("File research")) {
        dialogInfo.title = "Choose File";
        dialogInfo.type = ImGuiFileDialogType_OpenFile;
        dialogInfo.directoryPath = std::filesystem::current_path();
        fileDialogOpen = true;
    }

    fileResearch();

    // If the user selected a file, display the name of the file
    if (selectedFileIndex != -1) {
        selectedFile = assetsDir + fileNames[selectedFileIndex];
    }

    // Display the selected file if there is one
    if (selectedFile != "") {
        ImGui::Text(selectedFile.c_str());
    }

    // generation
    if (ImGui::Button("Generate") && selectedFile != "") {
        // Reset the cloud
        mainCloud = PointCloudDiff<Scalar>("mainCloud");
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Load object",
                                    [this]() { loadObject(mainCloud, selectedFile, pointNoise, normalNoise); });
    }
}

void GUI::generationFromImplicit() {
    // Display the 3 input fields for the implicit function
    if (ImGui::Button("Cylinder")) {
        displayImplicitParameters = true;
        isCylinder = true;
        isSinus = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate cylinder", [this]() {
            cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
        });
    }

    ImGui::SameLine();
    if (ImGui::Button("Sinus")) {
        displayImplicitParameters = true;
        isCylinder = false;
        isSinus = true;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate sinus",
                                    [this]() { sinusGenerator.generateSinus(mainCloud, pointNoise, normalNoise); });
    }

    if (ImGui::Button("Tube")) {
        displayImplicitParameters = false;
        isCylinder = false;
        isSinus = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate tube", [this]() {
            create_tube(mainCloud);
            mainCloud.addNoiseNormal(normalNoise);
            mainCloud.addNoisePosition(pointNoise);
        });
    }

    ImGui::SameLine();

    if (ImGui::Button("Sphere")) {
        displayImplicitParameters = false;
        isCylinder = false;
        isSinus = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate sphere", [this]() {
            create_sphere(mainCloud);
            mainCloud.addNoiseNormal(normalNoise);
            mainCloud.addNoisePosition(pointNoise);
        });
    }
}

void GUI::cylinderParameters() {
    // Set the initial position and size of the window
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2 - 150, ImGui::GetIO().DisplaySize.y / 2 - 100),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    // Create a window for the cylinder parameters, no close button
    ImGui::Begin("Cylinder parameters", NULL);

    ImGui::Text("Parameters of the parabolic-cylindrical shape");
    ImGui::Text("u_c + u_l^T q + a * (u_q^T q)^2");

    bool modification = false;
    if (ImGui::SliderFloat("u_c", &cylinderGenerator.a_cylinder, -2.0, 2.0))
        modification = true;

    ImGui::Columns(2, "mycolumns"); // 2-Column layout
    ImGui::Separator();
    if (ImGui::SliderFloat("u_lx", &cylinderGenerator.bx_cylinder, -2, 2))
        modification = true;
    ImGui::NextColumn();
    if (ImGui::SliderFloat("u_lz", &cylinderGenerator.bz_cylinder, -2, 2))
        modification = true;
    ImGui::Columns(1); // End of 2-Column layout

    if (ImGui::SliderFloat("a", &cylinderGenerator.c_a, -2, 2))
        modification = true;

    ImGui::Columns(2, "mycolumns"); // 2-Column layout
    ImGui::Separator();
    if (ImGui::SliderFloat("u_qx", &cylinderGenerator.cx_cylinder, -2, 2))
        modification = true;
    ImGui::NextColumn();
    if (ImGui::SliderFloat("u_qz", &cylinderGenerator.cz_cylinder, -2, 2))
        modification = true;
    ImGui::Columns(1); // End of 2-Column layout

    if (ImGui::SliderInt("x_number", &cylinderGenerator.x_cylinder, 0, 60) ||
        ImGui::SliderInt("z_number", &cylinderGenerator.z_cylinder, 0, 60))
        modification = true;

    if (modification) {
        cloudNeedsUpdate = true;
        cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
    }

    ImGui::End();
}

void GUI::sinusParameters() {
    // Set the initial position and size of the window
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2 - 150, ImGui::GetIO().DisplaySize.y / 2 - 100),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    // Create a window for the cylinder parameters, no close button
    ImGui::Begin("Sinus parameters", NULL);

    ImGui::Text("Parameters of the sinusoidal shape");

    bool modification = false;
    if (ImGui::SliderFloat("Amplitude", &sinusGenerator.h_sinus, -1.0, 1.0))
        modification = true;

    if (ImGui::SliderFloat("Frequency", &sinusGenerator.f_sinus, -20.0, 20.0))
        modification = true;

    if (ImGui::SliderFloat("Phase", &sinusGenerator.p_sinus, -5.0, 5.0))
        modification = true;

    if (ImGui::SliderFloat("Amplitude2", &sinusGenerator.h_sinus2, -1.0, 1.0))
        modification = true;

    if (ImGui::SliderFloat("Frequency2", &sinusGenerator.f_sinus2, -20.0, 20.0))
        modification = true;

    if (ImGui::SliderFloat("Phase2", &sinusGenerator.p_sinus2, -5.0, 5.0))
        modification = true;

    if (ImGui::SliderInt("x_number", &sinusGenerator.x_sinus, 30, 120) ||
        ImGui::SliderInt("z_number", &sinusGenerator.z_sinus, 30, 120))
        modification = true;

    if (ImGui::Button("Automatic ;)")) {
        sinusGenerator.automatic_sinus = !sinusGenerator.automatic_sinus;
    }

    if (sinusGenerator.automatic_sinus) {
        // Update the phase sinus during time
        sinusGenerator.p_sinus += 0.01;
        if (sinusGenerator.p_sinus > 2 * M_PI)
            sinusGenerator.p_sinus = 0;
        modification = true;
    }

    if (modification) {
        cloudNeedsUpdate = true;
        sinusGenerator.generateSinus(mainCloud, pointNoise, normalNoise);
    }

    if (ImGui::Button("Save pts")) {
        sinusGenerator.saveSinus(mainCloud);
    }

    ImGui::End();
}


void GUI::picking() {
    const ImGuiIO &io = ImGui::GetIO();
    // If the user clicked on the screen, while maintaining the shift key, we pick the point
    if (io.MouseClicked[0] && io.KeyShift) {
        const glm::vec2 screenCoords{io.MousePos.x, io.MousePos.y};
        const auto [fst, snd] =
                polyscope::pick::evaluatePickQuery(screenCoords.x, screenCoords.y);

        if (fst != nullptr)
            iVertexSource = snd;
    }
}

/////////////////////////////////////////////////////////////////////
////                       CLOUD COMPUTING                       ////
/////////////////////////////////////////////////////////////////////

void GUI::cloudComputingUpdateAll() {
    if (!all_computed)
        return;

    pointProcessing.measureTime("[Polyscope] Update diff quantities and projection", [this]() {
        for (int i = 0; i < selectedQuantities.size(); ++i) {
            if (selectedQuantities[i]) {
                std::string completeName = "[" + methodName + "] " + quantityNames[i];
                auto diffQuantities = mainCloud.getDiffQuantities();
                addQuantities(polyscope_mainCloud, completeName,
                              diffQuantities.getByName(quantityNames[i]));
            }
        }
        if (displayProjectedPointCloud) {
            std::string cloudName = "[" + methodName + "] " + "projection";

            // Find if the point cloud already exists
            for (int i = 0; i < polyscope_projectionClouds.size(); ++i) {
                if (polyscope_projectionClouds[i]->name == cloudName) {
                    // remove the projected point cloud
                    polyscope_projectionClouds.erase(polyscope_projectionClouds.begin() + i);
                    polyscope::removeStructure(cloudName, false);
                    break;
                }
            }
            polyscope::PointCloud *newCloud =
                    polyscope::registerPointCloud(cloudName, mainCloud.getDiffQuantities().position());
            polyscope_projectionClouds.push_back(newCloud);
            addQuantities(newCloud, "normals", mainCloud.getDiffQuantities().normal());
        }

        // Add the shapeIndex to the main cloud
        std::string completeName = "[" + methodName + "] " + "shape index";
        auto diffQuantities = mainCloud.getDiffQuantities();
        SampleVectorType shapeIndex = diffQuantities.shapeIndex();
        addQuantities(polyscope_mainCloud, completeName, shapeIndex);
    });

    all_computed = false;
    methodName = "";
}

void GUI::cloudComputingUpdateUnique() {
    if (!unique_computed)
        return;


    std::cout << "Cloud computing update unique" << std::endl;

    pointProcessing.measureTime("[Polyscope] Update unique projection", [this]() {
        std::string cloudName = "[" + methodName + "] " + "unique";
        for (int i = 0; i < polyscope_uniqueClouds.size(); ++i) {
            if (polyscope_uniqueClouds[i]->name == cloudName) {
                // remove the unique proj point cloud
                polyscope_uniqueClouds.erase(polyscope_uniqueClouds.begin() + i);
                polyscope::removeStructure(cloudName, false);
                break;
            }
        }
        // check if the methodName contains "CNC"
        if (methodName.find("Hexagram") != std::string::npos
        || methodName.find("Uniform") != std::string::npos 
        || methodName.find("Independent") != std::string::npos 
        || methodName.find("AvgHexagram") != std::string::npos ) {
            // Create a new surface mesh

            // std::vector for indices
            std::vector<std::array<size_t, 3>> indices(tempCloud.getTriangles().size());
            for (size_t i = 0; i < tempCloud.getTriangles().size() / 3; ++i) {
                indices[i] = {3 * i, 3 * i + 1, 3 * i + 2};
            }
            // Create a new surface mesh
            std::string meshName = "[" + methodName + "] " + "mesh";
            polyscope::SurfaceMesh *mesh = polyscope::registerSurfaceMesh(meshName, tempCloud.getTriangles(), indices);
            // Add quantities to the mesh
            mesh->addVertexVectorQuantity("normals", tempCloud.normals);
            mesh->addVertexVectorQuantity("d1", tempCloud.v1);
            mesh->addVertexVectorQuantity("d2", tempCloud.v2);
            polyscope_meshs.push_back(mesh);
        } else {
            if ( methodName.find("PCA") != std::string::npos
            || methodName.find("MeanPLANE") != std::string::npos ) {
                std::cout << "Computing plane 1" << std::endl;
                VectorType origin = tempCloud.points.row(0);
                VectorType normal = tempCloud.normals.row(0);
                VectorType minDir = tempCloud.v1.row(0);
                VectorType maxDir = tempCloud.v2.row(0);
                std::cout << "Computing plane 2" << std::endl;
                std::pair< std::vector<VectorType>, std::vector<std::array<size_t, 3> > > faces_idx = generatePlane(tempCloud.points, normal, minDir, maxDir);
                // Create a new surface mesh
                std::string meshName = "[" + methodName + "] " + "mesh";
                polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh(meshName, faces_idx.first, faces_idx.second);
                std::cout << "Computing plane 3" << std::endl;
                // Add quantities only to the first vertex (or face)
                std::vector<VectorType> normals(faces_idx.first.size(), VectorType::Zero());
                std::vector<VectorType> d1(faces_idx.first.size(), VectorType::Zero());
                std::vector<VectorType> d2(faces_idx.first.size(), VectorType::Zero());
                normals[0] = normal;
                d1[0] = minDir;
                d2[0] = maxDir;
                mesh->addVertexVectorQuantity("normals", normals);
                mesh->addVertexVectorQuantity("d1", d1);
                mesh->addVertexVectorQuantity("d2", d2);
                polyscope_meshs.push_back(mesh);
            } else {
                if ( methodName.find("Sphere") != std::string::npos || methodName.find("APSS") != std::string::npos || methodName.find("UnorientedSphere") != std::string::npos ){
                    Scalar radius = Scalar(1) / std::abs( tempCloud.mean[0] );
                    std::vector<VectorType> vertices(1, tempCloud.points.row(0));
                    SampleMatrixType normals(1, 3); normals.row(0) = tempCloud.normals.row(0);
                    SampleMatrixType d1(1, 3); d1.row(0) = tempCloud.v1.row(0);
                    SampleMatrixType d2(1, 3); d2.row(0) = tempCloud.v2.row(0);
                    SampleVectorType radii = SampleVectorType::Ones(1) * radius;
                    // Create simple point cloud with this unique point
                    std::string cloudName = "[" + methodName + "] " + "unique";
                    polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, vertices);
                    polyscope_uniqueClouds.push_back(newCloud);
                    addQuantities(newCloud, "normals", normals);
                    addQuantities(newCloud, "d1", d1);
                    addQuantities(newCloud, "d2", d2);
                    addQuantities(newCloud, "radius", radii);
                    newCloud->setPointRadiusQuantity("radius", false);
                } 
                else{
                    // Create a new point cloud
                    polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, tempCloud.points);
                    polyscope_uniqueClouds.push_back(newCloud);
                    addQuantities(newCloud, "normals", tempCloud.normals);
                    addQuantities(newCloud, "d1", tempCloud.v1);
                    addQuantities(newCloud, "d2", tempCloud.v2);
                }
            }
        }
    });

    unique_computed = false;
    methodName = "";
}

void GUI::cloudComputingSlices() {
    ImGui::Text("Potential slices");

    ImGui::Checkbox("HD", &isHDSlicer);
    ImGui::SameLine();
    ImGui::Text("Axis :");
    ImGui::SameLine();
    ImGui::RadioButton("X", &axis, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Y", &axis, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Z", &axis, 2);

    if (ImGui::SliderFloat("Slice", &slice, 0.0f, 1.0f)) {
        VectorType min = mainCloud.getMin();
        VectorType max = mainCloud.getMax();
        Scalar dist = (min - max).norm();
        min -= 0.1 * dist * VectorType::Ones();
        max += 0.1 * dist * VectorType::Ones();

        std::pair<std::vector<VectorType>, std::vector<std::array<size_t, 4>>> pair_slice =
                create_frame(min, max, isHDSlicer ? 256 : 64, axis, slice);

        SampleMatrixType positions = SampleMatrixType::Zero(pair_slice.first.size(), 1);
        for (size_t i = 0; i < pair_slice.first.size(); ++i) {
            positions.row(i) = pair_slice.first[i];
        }

        // SampleVectorType values = SampleVectorType::Zero(pair_slice.first.rows());
        SampleVectorType values;
        switch (weightFuncType) {
            case 0:
                values = sliceWithKernel<SmoothWeightFunc>(positions);
                break;
            case 1:
                values = sliceWithKernel<ConstWeightFunc>(positions);
                break;
            case 2:
                values = sliceWithKernel<WendlandWeightFunc>(positions);
                break;
            case 3:
                values = sliceWithKernel<SingularWeightFunc>(positions);
                break;
            case 4:
                values = sliceWithKernel<ExponentialWeightFunc>(positions);
                break;
            default:
                values = sliceWithKernel<SmoothWeightFunc>(positions);
                break;
        }

        // display the slices as default for now
        polyscope::SurfaceMesh *mesh = polyscope::registerSurfaceMesh(slicerName, pair_slice.first, pair_slice.second);
        mesh->addVertexScalarQuantity("Potential", values)->setEnabled(true);
        // polyscope_slices.push_back(mesh);
        // polyscope_slices.push_back(slicerName);
    }
}

template<typename FitT, bool isSigned = true>
void GUI::methodForCloudComputing(const std::string &metName, bool unique) {
    // std::string buttonName_all = "Compute " + metName;
    // std::string buttonName_unique = metName + " for selected";

    std::string buttonName_all = "Compute quantities";
    std::string buttonName_unique = "Only for the selected";
    if (offline_computing || ImGui::Button(buttonName_all.c_str())) {
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities<FitT>(metName, mainCloud);
    }

    if (!unique || offline_computing)
        return;

    ImGui::SameLine();
    if (ImGui::Button(buttonName_unique.c_str())) {
        // Compute the distance between points for the cube, by taking 1/50 of the maximum distance between points of
        // the mainCloud
        Scalar dist = (mainCloud.getMin() - mainCloud.getMax()).norm() / 50.0;
        create_cube(tempCloud, pointProcessing.getVertexSourcePosition(), dist);
        methodName = metName;
        std::cout << "Unique point computation" << std::endl;
        pointProcessing.computeUniquePoint<FitT>(metName, tempCloud);
        std::cout << "Unique point computed" << std::endl;
        unique_computed = true;
    }
}

void GUI::methodForCloudComputing_OnlyTriangle(const std::string &metName, const int &type) {
    // std::string buttonName_all = "Compute " + metName;
    // std::string buttonName_unique = metName + " for selected";

    std::string buttonName_all = "Compute quantities";
    std::string buttonName_unique = "Only for the selected";

    if (offline_computing || ImGui::Button(buttonName_all.c_str())) {
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities<ConstWeightFunc>(metName, mainCloud);
    }

    if (offline_computing)
        return;

    ImGui::SameLine();

    if (ImGui::Button(buttonName_unique.c_str())) {
        methodName = metName;
        std::cout << "Unique point computation TRIANGLE" << std::endl;
        pointProcessing.computeUniquePoint<ConstWeightFunc>(metName, tempCloud);
        std::cout << "Unique point computed" << std::endl;
        if (tempCloud.getTriangles().size() == 0) {
            std::cerr << "Error: computeUniquePointTriangle returned an empty matrix" << std::endl;
            return;
        }
        unique_computed = true;
    }
}

void GUI::cloudComputing() {
    cloudComputingParameters();

    ImGui::Text("Differential estimators");

    if (ImGui::Button("Dry Run")) {
        pointProcessing.mlsDryRun();
        lastDryRun = "Last time run : " + std::to_string(pointProcessing.getMeanNeighbors()) +
                     " number of neighbors (mean) \n";
    }
    ImGui::SameLine();
    ImGui::Text(lastDryRun.c_str());

    ImGui::SetItemDefaultFocus();
    ImGui::SetNextItemWidth(200);
    ImGui::Combo("Fitting Methods", &item_selected_method, methods, IM_ARRAYSIZE(methods));

    switch (weightFuncType) {
        case 0:
            methodWithKernel<ConstWeightFunc>();
            break;
        case 1:
            methodWithKernel<SmoothWeightFunc>();
            break;
        case 2:
            methodWithKernel<WendlandWeightFunc>();
            break;
        case 3:
            methodWithKernel<SingularWeightFunc>();
            break;
        case 4:
            methodWithKernel<ExponentialWeightFunc>();
            break;
        default:
            methodWithKernel<SmoothWeightFunc>();
            break;
    }

    cloudComputingUpdateAll();
    cloudComputingUpdateUnique();

    ImGui::Separator();

    cloudComputingSlices();
}

void GUI::cloudComputingParameters() {
    ImGui::Text("Neighborhood collection");
    ImGui::SameLine();
    if (ImGui::Checkbox("Use Voxelgrid", &use_VoxelGrid)) {
        pointProcessing.computeVoxelGrid(mainCloud);
    }

    if (use_VoxelGrid) {
        ImGui::SameLine();
        ImGui::Checkbox("No empty cells", &noEmptyVoxels);
        if (ImGui::InputInt("Resolution", &resolution, 1, 200)) {
            pointProcessing.computeVoxelGrid(mainCloud);
        }
        if (ImGui::InputInt("Number of cells in each direction", &N_voxels, 1, 200)) {
            pointProcessing.computeVoxelGrid(mainCloud);
        }

        if (ImGui::InputInt("Show Voxels Resolution", &displayVoxelResolution, 0, 200)) {
            displayVoxelResolution = std::max(0, displayVoxelResolution);
            displayVoxelResolution = std::min(displayVoxelResolution, resolution);
        }
        if (ImGui::Button("Show VoxelGrid")) {
            if (displayVoxelResolution > resolution)
                displayVoxelResolution = resolution;
            using Aabb = MyVoxelGrid::Aabb;
            std::vector<Aabb> bboxes = voxelGrid.getCellBoundingBoxes(
                    displayVoxelResolution, noEmptyVoxels); // True to only keep non empty cells [TODO] not working
            // Create a new mesh using only lines
            std::vector<VectorType> vertices;
            std::vector<std::array<int, 2>> indices;
            createVoxels(bboxes, vertices, indices);
            // Delete older voxel grid if it exists
            polyscope::removeStructure("VoxelGrid", false);
            polyscope::registerCurveNetwork("VoxelGrid", vertices, indices);

            std::vector<VectorType> centers =
                    voxelGrid.getCellCenters(displayVoxelResolution, noEmptyVoxels);
            polyscope::registerPointCloud("VoxelGrid centers", centers);
            // Add some quantities next time
        }

    } else {
        if (ImGui::Checkbox("Use KnnGraph", &use_kNNGraph))
            pointProcessing.recomputeKnnGraph();
        if (ImGui::InputInt("kNN for graph", &kNN_for_graph))
            pointProcessing.recomputeKnnGraph();
        // Add radio buttons to select the research type
        ImGui::RadioButton("kNN", &researchType, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Euclidian Nearest", &researchType, 1);

        ImGui::SameLine();
        if (researchType == 0) {
            if (ImGui::Button("show neighbors"))
                addQuantities(polyscope_mainCloud, "knn", pointProcessing.colorizeNeighbors<ConstWeightFunc>());
        } else {
            if (ImGui::Button("show neighbors")) {
                switch (weightFuncType) {
                    case 0:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<ConstWeightFunc>());
                    break;
                    case 1:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<SmoothWeightFunc>());
                    break;
                    case 2:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<WendlandWeightFunc>());
                    break;
                    case 3:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<SingularWeightFunc>());
                    break;
                    case 4:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<ExponentialWeightFunc>());
                    break;
                    default:
                        addQuantities(polyscope_mainCloud, "euclidean nei",
                                      pointProcessing.colorizeNeighbors<SmoothWeightFunc>());
                    break;
                }
            }
        }
        ImGui::SameLine();
        if (researchType == 0) {
            if (ImGui::Button("pc neighbors")) {
                SampleVectorType neighbor_values = pointProcessing.colorizeNeighbors<ConstWeightFunc>();
                std::pair<SampleMatrixType, SampleVectorType> res_nei =
                        mainCloud.getNonZeros(neighbor_values, iVertexSource);
                std::string cloudName = "neighborhood";
                // Create a new point cloud
                polyscope::PointCloud *newCloud = polyscope::registerPointCloud(cloudName, res_nei.first);
                polyscope_uniqueClouds.push_back(newCloud);
                addQuantities(newCloud, "neighbors", res_nei.second);
            }
        } else {
            if (ImGui::Button("pc neighbors")) {
                SampleVectorType neighbor_values;
                switch (weightFuncType) {
                    case 0:
                        neighbor_values = pointProcessing.colorizeNeighbors<ConstWeightFunc>();
                        break;
                    case 1:
                        neighbor_values = pointProcessing.colorizeNeighbors<SmoothWeightFunc>();
                        break;
                    case 2:
                        neighbor_values = pointProcessing.colorizeNeighbors<WendlandWeightFunc>();
                        break;
                    case 3:
                        neighbor_values = pointProcessing.colorizeNeighbors<SingularWeightFunc>();
                        break;
                    case 4:
                        neighbor_values = pointProcessing.colorizeNeighbors<ExponentialWeightFunc>();
                        break;
                    default:
                        neighbor_values = pointProcessing.colorizeNeighbors<SmoothWeightFunc>();
                        break;
                }
                const std::pair<SampleMatrixType, SampleVectorType> res_nei =
                        mainCloud.getNonZeros(neighbor_values, iVertexSource);
                std::string cloudName = "neighborhood";
                // Create a new point cloud
                polyscope::PointCloud *newCloud = polyscope::registerPointCloud(cloudName, res_nei.first);
                polyscope_uniqueClouds.push_back(newCloud);
                addQuantities(newCloud, "neighbors", res_nei.second);
            }
        }

        if (researchType == 0) {
            ImGui::InputInt("k-neighborhood size", &kNN);
        } else {
            ImGui::InputFloat("neighborhood size", &radius);
        }

        ImGui::InputInt("source vertex", &iVertexSource);
        ImGui::InputInt("Nb MLS Iterations", &mls_iter);

        // Add radio buttons to select the weight function
        ImGui::RadioButton("Constant", &weightFuncType, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Smooth", &weightFuncType, 1);
        ImGui::SameLine();
        ImGui::RadioButton("Wendland", &weightFuncType, 2);
        ImGui::SameLine();
        ImGui::RadioButton("Singular", &weightFuncType, 3);
        ImGui::SameLine();
        ImGui::RadioButton("Exponential", &weightFuncType, 4);

        ImGui::TextColored(ImVec4(1, 1, 0, 1), "Warning: Only Smooth Kernel.");

        ImGui::Separator();
    }
}

void GUI::quantitiesParameters() {
    // Set of check boxes for the quantities to display (proj, normal, dir min curv, dir max curv, min curv, max curv,
    // mean curv) Also another combo boxe to display the projected point cloud
    for (int i = 0; i < selectedQuantities.size(); ++i) {
        ImGui::Checkbox(quantityNames[i].c_str(), (bool *) &selectedQuantities[i]);
        if (i % 2 == 0)
            ImGui::SameLine();
    }
    ImGui::Separator();
    ImGui::Checkbox("Projected point cloud", &displayProjectedPointCloud);
}
