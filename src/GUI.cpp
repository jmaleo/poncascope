void GUI::mainCallBack(){

    // Create a window
    ImGui::PushItemWidth(100);
    // The size of the window, the position is set by default
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    // Camera settings :
    saveCameraSettings();

    cloudGeneration();
    if (isCylinder){
        cylinderParameters();
    }
    ImGui::Separator();

    if (cloudNeedsUpdate){
        pointProcessing.update(mainCloud);
        pointProcessing.measureTime("[Polyscope] Update current main cloud", [this](){
            polyscope::removeStructure(mainCloudName, false);
            polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
            addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
            // Remove other clouds
            remove_clouds();
            remove_meshs();
        });
        lastDryRun = "";
        cloudNeedsUpdate = false;
    }

    quantitiesParameters();

    ImGui::Separator();

    cloudComputing();

}

//////////////////////////////////////////////////////////////////////
////                           SETTINGS                           ////
//////////////////////////////////////////////////////////////////////

void GUI::saveCameraSettings(){
    // Save the camera settings
    if (ImGui::Button("Save camera settings")){
        std::string base_name = "cameraSettings";
        std::string num = "0";
        std::string extension = ".json";
        std::string view = polyscope::view::getViewAsJson();
        // While it exists a file with the same name, add a number at the end of the name
        while (std::filesystem::exists(base_name + num + extension)) num = std::to_string(std::stoi(num) + 1);
        std::ofstream file(base_name + num + extension);
        file << view;
        file.close();
    }
}

//////////////////////////////////////////////////////////////////////
////                       CLOUD GENERATING                       ////
//////////////////////////////////////////////////////////////////////

void GUI::fileResearch(){

    if (FileDialog(&fileDialogOpen, &dialogInfo))
    {
        // L'utilisateur a sélectionné un fichier et a cliqué sur "Open".
        // Le chemin du fichier sélectionné est dans dialogInfo.resultPath.
        fileDialogOpen = false;  // Ferme la boîte de dialogue pour la prochaine fois
        selectedFile = dialogInfo.resultPath.string();
        selectedFileIndex = -1;
    }

    // open = false;
    return;
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

    ImGui::SameLine();
    if (ImGui::Button("File research")){
        dialogInfo.title = "Choose File";
        dialogInfo.type = ImGuiFileDialogType_OpenFile;
        dialogInfo.directoryPath = std::filesystem::current_path();
        fileDialogOpen = true;
    }
    fileResearch();

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
        cloudNeedsUpdate = true;
        
        pointProcessing.measureTime("[Generation] Load object", [this](){
            loadObject(mainCloud, selectedFile, pointNoise, normalNoise);
        });
        
    }
}

void GUI::generationFromImplicit() {
    // Display the 3 input fields for the implicit function
    if (ImGui::Button("Cylinder")){
        displayImplicitParameters = true;
        isCylinder = true;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate cylinder", [this](){
            cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
        });

    }

    if (ImGui::Button("Tube")){
        displayImplicitParameters = false;
        isCylinder = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate tube", [this](){
            create_tube(mainCloud);
            mainCloud.addNoise(pointNoise, normalNoise);
        });
    }

    ImGui::SameLine();

    if (ImGui::Button("Sphere")){
        displayImplicitParameters = false;
        isCylinder = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate sphere", [this](){
            create_sphere(mainCloud);
            mainCloud.addNoise(pointNoise, normalNoise);
        });
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


/////////////////////////////////////////////////////////////////////
////                       CLOUD COMPUTING                       ////
/////////////////////////////////////////////////////////////////////

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

        // Add the shapeIndex to the main cloud
        std::string completeName = "[" + methodName + "] "+ "shape index";
        auto quantity = polyscope_mainCloud->addScalarQuantity(completeName, mainCloud.getDiffQuantities().getShapeIndex());
        quantity->setMapRange(std::pair<double,double>(-1,1));

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
        // check if the methodName contains "CNC"
        if (methodName.find("CNC") != std::string::npos){
            // Create a new surface mesh
            
            // std::vector for indices
            std::vector<std::array<size_t, 3>> indices(tempCloud.getTriangles().size());
            for (size_t i = 0; i < tempCloud.getTriangles().size()/3; ++i){
                indices[i] = {3*i, 3*i+1, 3*i+2};
            }
            // Create a new surface mesh
            std::string meshName = "[" + methodName + "] " + "mesh";
            polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh(meshName, tempCloud.getTriangles(), indices);
            // Add quantities to the mesh
            mesh->addVertexVectorQuantity("normals", tempCloud.getDiffQuantities().getNormals());
            mesh->addVertexVectorQuantity("d1", tempCloud.getDiffQuantities().getKMinDir());
            mesh->addVertexVectorQuantity("d2", tempCloud.getDiffQuantities().getKMaxDir());
            polyscope_meshs.push_back(mesh);
        }
        else {
            // Create a new point cloud
            polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, tempCloud.getDiffQuantities().getVertices());
            polyscope_uniqueClouds.push_back(newCloud);
            addQuantities(newCloud, "normals", tempCloud.getDiffQuantities().getNormals());
            addQuantities(newCloud, "d1", tempCloud.getDiffQuantities().getKMinDir());
            addQuantities(newCloud, "d2", tempCloud.getDiffQuantities().getKMaxDir());
        } 
    });

    unique_computed = false;
    methodName = "";
}

void GUI::cloudComputingSlices(){
    ImGui::Text("Potential slices");

    ImGui::Checkbox("HD", &isHDSlicer); ImGui::SameLine();
    ImGui::Text("Axis :"); ImGui::SameLine();
    ImGui::RadioButton("X", &axis, 0); ImGui::SameLine();
    ImGui::RadioButton("Y", &axis, 1); ImGui::SameLine();
    ImGui::RadioButton("Z", &axis, 2);

    if (ImGui::SliderFloat("Slice", &slice, 0.0f, 1.0f)){

        VectorType min = mainCloud.getMin();
        VectorType max = mainCloud.getMax();
        Scalar dist = (min - max).norm();
        min -= 0.1*dist*VectorType::Ones();
        max += 0.1*dist*VectorType::Ones();

        std::pair<SampleMatrixType, std::vector<std::array<size_t,4>>> pair_slice = create_frame(min, max, isHDSlicer?256:64, axis, slice);

        // SampleVectorType values = SampleVectorType::Zero(pair_slice.first.rows());
        SampleVectorType values;
        switch (weightFuncType){
        case 0 : 
            values = sliceWithKernel<SmoothWeightFunc>(pair_slice.first);
            break;
        case 1 : 
            values = sliceWithKernel<ConstWeightFunc>(pair_slice.first);
            break;
        case 2 : 
            values = sliceWithKernel<WendlandWeightFunc>(pair_slice.first);
            break;
        case 3 : 
            values = sliceWithKernel<SingularWeightFunc>(pair_slice.first);
            break;
        default : 
            values = sliceWithKernel<SmoothWeightFunc>(pair_slice.first);
            break;
        }

        // display the slices as default for now
        polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh(slicerName, pair_slice.first, pair_slice.second);
        mesh->addVertexScalarQuantity("Potential", values)->setEnabled(true);
        // polyscope_slices.push_back(mesh);
        // polyscope_slices.push_back(slicerName);
    }

}

template <typename FitT, bool isSigned = true>
void GUI::methodForCloudComputing(const std::string& metName, bool unique){
    // std::string buttonName_all = "Compute " + metName;
    // std::string buttonName_unique = metName + " for selected";

    std::string buttonName_all = "Compute quantities";
    std::string buttonName_unique = "Only for the selected";
    if ( offline_computing || ImGui::Button(buttonName_all.c_str()) ){
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities<FitT, isSigned>(metName, mainCloud);
    }
    
    if (!unique || offline_computing) return;

    ImGui::SameLine();
    if (ImGui::Button(buttonName_unique.c_str())){
        // Compute the distance between points for the cube, by taking 1/50 of the maximum distance between points of the mainCloud
        Scalar dist = (mainCloud.getMin() - mainCloud.getMax()).norm() / 50.0;
        create_cube(tempCloud, pointProcessing.getVertexSourcePosition(), dist);
        methodName = metName;
        pointProcessing.computeUniquePoint<FitT>(metName, tempCloud);
        unique_computed = true;
    }

    std::string buttonName_unique_aggregation = "Test aggregation";

    if (metName == "FO Ellipsoid2D"){
        ImGui::InputInt("Number of neighbors", &nb_neighbors_for_unique);
        ImGui::SameLine();
        if (ImGui::Button(buttonName_unique_aggregation.c_str())){
            Scalar dist = (mainCloud.getMin() - mainCloud.getMax()).norm() / 50.0;
            create_cube(tempCloud, pointProcessing.getVertexSourcePosition(), dist);
            methodName = metName + " aggregation";
            switch (weightFuncType){
                case 0 : 
                    pointProcessing.computeUniquePoint_aggregation<ConstWeightFunc>(metName, tempCloud, nb_neighbors_for_unique); break;
                case 1 : 
                    pointProcessing.computeUniquePoint_aggregation<SmoothWeightFunc>(metName, tempCloud,nb_neighbors_for_unique); break;
                case 2 : 
                    pointProcessing.computeUniquePoint_aggregation<WendlandWeightFunc>(metName, tempCloud,nb_neighbors_for_unique); break;
                case 3 : 
                    pointProcessing.computeUniquePoint_aggregation<SingularWeightFunc>(metName, tempCloud,nb_neighbors_for_unique); break;
                default : 
                    pointProcessing.computeUniquePoint_aggregation<SmoothWeightFunc>(metName, tempCloud,nb_neighbors_for_unique); break;
            }
            unique_computed = true;

        }
    }

}

void GUI::methodForCloudComputing_OnlyTriangle(const std::string &metName, const int& type){
    // std::string buttonName_all = "Compute " + metName;
    // std::string buttonName_unique = metName + " for selected";

    std::string buttonName_all = "Compute quantities";
    std::string buttonName_unique = "Only for the selected";

    if ( offline_computing || ImGui::Button (buttonName_all.c_str()) ){
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities_Triangle(metName,type, mainCloud);
    }

    if (offline_computing) return;

    ImGui::SameLine();

    if (ImGui::Button(buttonName_unique.c_str())){
        methodName = metName;

        pointProcessing.computeUniquePoint_triangle(metName, type, tempCloud);

        if (tempCloud.getTriangles().size() == 0) {
            std::cerr << "Error: computeUniquePointTriangle returned an empty matrix" << std::endl;
            return;
        }
        unique_computed = true;
    }
}

void GUI::cloudComputing(){

    cloudComputingParameters();

    ImGui::Text("Differential estimators");

    if (ImGui::Button("Dry Run")){
        pointProcessing.mlsDryRun();
        lastDryRun = "Last time run : " + std::to_string(pointProcessing.getMeanNeighbors()) + " number of neighbors (mean) \n";
    }
    ImGui::SameLine();
    ImGui::Text(lastDryRun.c_str());

    ImGui::SetItemDefaultFocus();
    ImGui::SetNextItemWidth(200);
    ImGui::Combo ("Fitting Methods", &item_selected_method, methods, IM_ARRAYSIZE(methods));

    switch (weightFuncType){
        case 0 : 
            methodWithKernel<ConstWeightFunc>();
            break;
        case 1 : 
            methodWithKernel<SmoothWeightFunc>();
            break;
        case 2 : 
            methodWithKernel<WendlandWeightFunc>();
            break;
        case 3 : 
            methodWithKernel<SingularWeightFunc>();
            break;
        default : 
            methodWithKernel<SmoothWeightFunc>();
            break;
    }

    cloudComputingUpdateAll();
    cloudComputingUpdateUnique();

    ImGui::Separator();

    cloudComputingSlices();
}

void GUI::cloudComputingParameters(){

    ImGui::Text("Neighborhood collection");
    ImGui::SameLine();
    if(ImGui::Checkbox("Use KnnGraph", &pointProcessing.useKnnGraph))
        pointProcessing.recomputeKnnGraph();
    if ( ImGui::InputInt("kNN for graph", &pointProcessing.kNN_for_graph) ) 
        pointProcessing.recomputeKnnGraph();

    // Add radio buttons to select the research type
    ImGui::RadioButton("kNN", &pointProcessing.researchType, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Euclidian Nearest", &pointProcessing.researchType, 1);

    ImGui::SameLine();
    if (pointProcessing.researchType == 0){
        if (ImGui::Button("show neighbors")) addQuantities(polyscope_mainCloud, "knn", pointProcessing.colorizeKnn());
    }
    else {
        if (ImGui::Button("show neighbors")) {
            switch (weightFuncType){
                case 0 : 
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<ConstWeightFunc>());
                    break;
                case 1 :
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<SmoothWeightFunc>());
                    break;
                case 2 :
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<WendlandWeightFunc>());
                    break;
                case 3 :
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<SingularWeightFunc>());
                    break;
                default : 
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<SmoothWeightFunc>());
                    break;
            }
        }
    }

    if ( pointProcessing.researchType == 0 ) {
        ImGui::InputInt("k-neighborhood size", &pointProcessing.kNN);
    }
    else {
        ImGui::InputFloat("neighborhood size", &pointProcessing.NSize);
    }

    ImGui::InputInt("source vertex", &pointProcessing.iVertexSource);
    ImGui::InputInt("Nb MLS Iterations", &pointProcessing.mlsIter);

    // Add radio buttons to select the weight function
    ImGui::RadioButton("Constant", &weightFuncType, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Smooth", &weightFuncType, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Wendland", &weightFuncType, 2);
    ImGui::SameLine();
    ImGui::RadioButton("Singular", &weightFuncType, 3);

    ImGui::Separator();

    ///////// 
    // TEST
    /////////

    ImGui::InputInt("Cell try",&cellIdx);
    if (cellIdx > pointProcessing.mlodsTree.node_count()) cellIdx = pointProcessing.mlodsTree.node_count();

    ImGui::SameLine();
    if (ImGui::Button("Test cell")){
        // Get cell Aabb
        Eigen::AlignedBox<Scalar, 3> aabb = pointProcessing.computeCell(mainCloud, cellIdx);

        // Create a new mesh using the Aabb to show the bounding box
        std::vector<VectorType> vertices;
        std::vector<std::array<size_t, 8>> edges;
        // Vertices.push_back corner by corner
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::BottomLeftFloor));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::BottomRightFloor));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::TopRightFloor));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::TopLeftFloor));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::BottomLeftCeil));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::BottomRightCeil));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::TopRightCeil));
        vertices.push_back(aabb.corner(Eigen::AlignedBox<Scalar, 3>::TopLeftCeil));
        
        edges.push_back({0,1,2,3,4,5,6,7});
        // Create a new surface mesh
        std::string meshName = "Current Cell";
        auto mesh = polyscope::registerHexMesh(meshName, vertices, edges);
        mesh->setTransparency(0.15);
        mesh->setEdgeWidth(2);

        methodName = "Cell test";
        all_computed = true;
    }

    ImGui::InputFloat("radius factor", &pointProcessing.radiusFactor);
    if (pointProcessing.radiusFactor < 1.0) pointProcessing.radiusFactor = 1.0;

    ImGui::SameLine();
    
    if (ImGui::Button("Test MLODS on selected")){
        pointProcessing.computeMLODS(mainCloud);
        methodName = "MLODS";
        all_computed = true;
    }

    ImGui::Separator();
    ///////// 
    // TEST
    /////////

}

void GUI::addQuantities(polyscope::PointCloud *pc, const std::string &name, const SampleMatrixType &values){
    if (values.cols() == 1){
        // Make values beeing a vector
        SampleVectorType valuesVec = values.col(0);
        auto quantity = pc->addScalarQuantity(name, valuesVec);
        // Set bound [-5, 5] for the scalar quantity
        if (name != "knn" && name != "euclidean nei"){
            quantity->setMapRange(std::pair<double,double>(-5,5));
            quantity->setColorMap("coolwarm");
        }
        else {
            quantity->setColorMap("turbo");
            quantity->setEnabled(true);
        }
    }
    else 
        pc->addVectorQuantity(name, values);
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
