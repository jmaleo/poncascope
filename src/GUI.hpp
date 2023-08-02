void GUI::mainCallBack(){

    // Create a window
    ImGui::PushItemWidth(100);
    // The size of the window, the position is set by default
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    cloudGeneration();
    if (isCylinder){
        cylinderParameters();
    }

    if (cloudNeedsUpdate){
        pointProcessing.update(mainCloud);
        polyscope::removeStructure(mainCloudName, false);
        polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
        addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
        // Remove other clouds
        remove_clouds();
        lastDryRun = "";
        cloudNeedsUpdate = false;
    }

    ImGui::Separator();

    quantitiesParameters();

    ImGui::Separator();

    cloudComputing();

    ImGui::Separator();

}

void GUI::cloudGeneration(){

    // Point noise and normal noise selection with sliders
    ImGui::SliderFloat("Point noise",  &pointNoise, 0.0f, 5.0f);
    ImGui::SameLine();
    ImGui::SliderFloat("Normal noise", &normalNoise, 0.0f, 5.0f);

    // Display the 2 radios buttons, one for file selection and the other for implicit function
    ImGui::RadioButton("File selection", &radioButtonCloudGeneration, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Implicit function", &radioButtonCloudGeneration, 1);

    // If the user selected the file selection
    if(radioButtonCloudGeneration == 0){
        generationFromFile();
    }
    // If the user selected the implicit function
    else if(radioButtonCloudGeneration == 1){
        selectedFile = "";
        generationFromImplicit();
    }

}

void GUI::generationFromFile(){

    // Display all the existing file in the assets directory
    std::vector<std::string> fileNames;
    for (const auto & entry : std::filesystem::directory_iterator(assetsDir)){
        // Take only the .obj and .ply files, and only the name of the file
        if(entry.path().extension() == ".obj" || entry.path().extension() == ".ply" || entry.path().extension() == ".pts"){
            fileNames.push_back(entry.path().filename().string());
        }
    }
    std::vector<const char*> fileNamesCStr;
    for(const auto& str : fileNames) {
        fileNamesCStr.push_back(str.c_str());
    }

    // Display the list of files
    ImGui::ListBox("Files", &selectedFileIndex, fileNamesCStr.data(), fileNamesCStr.size());

    // If the user selected a file, display the name of the file
    if(selectedFileIndex != -1){
        selectedFile = assetsDir + fileNames[selectedFileIndex];
    }

    // Display the selected file if there is one
    if(selectedFile != ""){
        ImGui::Text(selectedFile.c_str());
    }

    // generation
    if(ImGui::Button("Generate") && selectedFile != ""){
        displayImplicitParameters = false;
        isCylinder = false;
        cloudNeedsUpdate = true;
        loadObject(mainCloud, selectedFile, pointNoise, normalNoise);
    }
}

void GUI::generationFromImplicit() {
    // Display the 3 input fields for the implicit function
    if (ImGui::Button("Cylinder")){
        displayImplicitParameters = true;
        isCylinder = true;
        cloudNeedsUpdate = true;
        cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
    }
}

void GUI::cylinderParameters(){
    // Set the initial position and size of the window
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2 - 150, ImGui::GetIO().DisplaySize.y/2 - 100), ImGuiCond_FirstUseEver);
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

    if (ImGui::SliderInt("x_number", &cylinderGenerator.x_cylinder, 0, 60 )      ||
        ImGui::SliderInt("z_number", &cylinderGenerator.z_cylinder, 0, 60 ))
            modification = true;

    if (modification){
        cloudNeedsUpdate = true;
        cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
    }

    ImGui::End();
}

void GUI::quantitiesParameters() {
    // Set of check boxes for the quantities to display (proj, normal, dir min curv, dir max curv, min curv, max curv, mean curv)
    // Also another combo boxe to display the projected point cloud
    for (int i = 0; i < selectedQuantities.size(); ++i) {
        ImGui::Checkbox(quantityNames[i].c_str(), (bool*)&selectedQuantities[i]);
    }
    ImGui::Separator();
    ImGui::Checkbox ("Projected point cloud", &displayProjectedPointCloud);
}

void GUI::cloudComputingUpdateAll (){
    if (!all_computed) return;

    pointProcessing.measureTime("[Polyscope] Update diff quantities and projection", [this](){
        
        for (int i = 0; i < selectedQuantities.size(); ++i) {
            if (selectedQuantities[i]){
                std::string completeName = "[" + methodName + "] "+ quantityNames[i];
                addQuantities(polyscope_mainCloud,completeName, mainCloud.getDiffQuantities().getByName(quantityNames[i]));
            }
        }
        if (displayProjectedPointCloud){
            std::string cloudName = "[" + methodName + "] " + "projection";

            // Find if the point cloud already exists
            for (int i = 0; i < polyscope_projectionClouds.size(); ++i) {
                if (polyscope_projectionClouds[i]->name == cloudName){
                    // remove the projected point cloud
                    polyscope_projectionClouds.erase(polyscope_projectionClouds.begin() + i);
                    polyscope::removeStructure(cloudName, false);
                    break;
                }
            }
            polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, mainCloud.getDiffQuantities().getVertices());
            polyscope_projectionClouds.push_back(newCloud);
            addQuantities(newCloud, "normals", mainCloud.getDiffQuantities().getNormals());
        }

    });

    all_computed = false;
    methodName = "";
}


void GUI::cloudComputingUpdateUnique (){
    if (!unique_computed) return;


    pointProcessing.measureTime("[Polyscope] Update unique projection", [this](){
    
        std::string cloudName = "[" + methodName + "] " + "unique";
        for (int i = 0; i < polyscope_uniqueClouds.size(); ++i) {
            if (polyscope_uniqueClouds[i]->name == cloudName){
                // remove the unique proj point cloud
                polyscope_uniqueClouds.erase(polyscope_uniqueClouds.begin() + i);
                polyscope::removeStructure(cloudName, false);
                break;
            }
        }
        polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, tempCloud.getDiffQuantities().getVertices());
        polyscope_uniqueClouds.push_back(newCloud);
        addQuantities(newCloud, "normals", tempCloud.getDiffQuantities().getNormals());
    
    });

    unique_computed = false;
    methodName = "";
}


template <typename FitT>
void GUI::methodForCloudComputing(const std::string& metName){
    std::string buttonName_all = "Compute " + metName;
    std::string buttonName_unique = metName + " for selected";
    if (ImGui::Button(buttonName_all.c_str())){
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities<FitT>(metName, mainCloud);
    }
    ImGui::SameLine();
    if (ImGui::Button(buttonName_unique.c_str())){
        // Compute the distance between points for the cube, by taking 1/50 of the maximum distance between points of the mainCloud
        double dist = (mainCloud.getMin() - mainCloud.getMax()).norm() / 50.0;
        create_cube(tempCloud, pointProcessing.getVertexSourcePosition(), dist);
        methodName = metName;
        unique_computed = true;
        pointProcessing.computeUniquePoint<FitT>(metName, tempCloud);
    }

}

void GUI::cloudComputing(){

    cloudComputingParameters();

    if (ImGui::Button("Dry Fit All")){
        pointProcessing.mlsDryRun();
        lastDryRun = "Last time run : " + std::to_string(pointProcessing.getMeanNeighbors()) + " number of neighbors (mean) \n";
    }
    ImGui::SameLine();
    ImGui::Text(lastDryRun.c_str());

    methodForCloudComputing<basket_AlgebraicPointSetSurfaceFit>("APSS");

    methodForCloudComputing<basket_ellipsoidFit>("Ellipsoid 3D");

    methodForCloudComputing<basket_FullyOrientedCylinderFit>("Fully Oriented Cylinder");

    methodForCloudComputing<basket_cylinderFit>("Cylinder");

    cloudComputingUpdateAll();
    cloudComputingUpdateUnique();

}

void GUI::cloudComputingParameters(){

    ImGui::Text("Neighborhood collection");
    ImGui::SameLine();
    if(ImGui::Checkbox("Use KnnGraph", &pointProcessing.useKnnGraph))
        pointProcessing.recomputeKnnGraph();

    ImGui::InputInt("k-neighborhood size", &pointProcessing.kNN);
    ImGui::InputFloat("neighborhood size", &pointProcessing.NSize);
    ImGui::InputInt("source vertex", &pointProcessing.iVertexSource);
    ImGui::InputInt("Nb MLS Iterations", &pointProcessing.mlsIter);
    ImGui::SameLine();
    if (ImGui::Button("show knn")) addQuantities(polyscope_mainCloud, "knn", pointProcessing.colorizeKnn(mainCloud));
    ImGui::SameLine();
    if (ImGui::Button("show euclidean nei")) addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood(mainCloud));

    ImGui::Separator();

}

void GUI::addQuantities(polyscope::PointCloud *pc, const std::string &name, const Eigen::MatrixXd &values){
    if (values.cols() == 1){
        // Make values beeing a vector
        Eigen::VectorXd valuesVec = values.col(0);
        auto quantity = pc->addScalarQuantity(name, valuesVec);
        // Set bound [-5, 5] for the scalar quantity
        quantity->setMapRange(std::pair<double,double>(-5,5));
        quantity->setColorMap("coolwarm");
    }
    else 
        pc->addVectorQuantity(name, values);
}