void GUI::mainCallBack(){

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

    picking();

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

    ImGui::SameLine();
    // generation with a mesh to compute the normals
    if(ImGui::Button("Generate with normals") && selectedFile != ""){
        cloudNeedsUpdate = true;
        pointProcessing.measureTime("[Generation] Load object with normals", [this](){
            loadObject(mainCloud, selectedFile, pointNoise, normalNoise);
            std::string selectedFileOBJ = selectedFile;
            std::string selectedFileRotation = selectedFile;
            // Replace the extension by .obj
            selectedFileOBJ.replace(selectedFileOBJ.end()-3, selectedFileOBJ.end(), "obj");
            selectedFileRotation.replace(selectedFileRotation.end()-3, selectedFileRotation.end(), "txt");
            
            
            Mesh_test originalOBG = Mesh_test(selectedFileOBJ);
            originalOBG.rotateTranslate(selectedFileRotation);

            // Add the original mesh to the polyscope
            polyscope::registerSurfaceMesh("original", originalOBG.getVertices(), originalOBG.getIndices());

            normalsFromMesh(mainCloud, originalOBG, pointNoise, normalNoise);
        });
    }
}

void GUI::generationFromImplicit() {
    // Display the 3 input fields for the implicit function
    if (ImGui::Button("Cylinder")){
        displayImplicitParameters = true;
        isCylinder = true;
        isSinus = false;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate cylinder", [this](){
            cylinderGenerator.generateCylinder(mainCloud, pointNoise, normalNoise);
        });

    }

    ImGui::SameLine();
    if (ImGui::Button("Sinus")) {
        displayImplicitParameters = true;
        isCylinder = false;
        isSinus = true;
        cloudNeedsUpdate = true;

        pointProcessing.measureTime("[Generation] Generate sinus", [this](){
            sinusGenerator.generateSinus(mainCloud, pointNoise, normalNoise);
        });
    }

    if (ImGui::Button("Tube")){
        displayImplicitParameters = false;
        isCylinder = false;
        isSinus = false;
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
        isSinus = false;
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

void GUI::sinusParameters(){
    // Set the initial position and size of the window
    ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x/2 - 150, ImGui::GetIO().DisplaySize.y/2 - 100), ImGuiCond_FirstUseEver);
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
    
    if (ImGui::SliderInt("x_number", &sinusGenerator.x_sinus, 30, 120 )      ||
        ImGui::SliderInt("z_number", &sinusGenerator.z_sinus, 30, 120 ))
            modification = true;

    if ( ImGui::Button("Automatic ;)") ){
        sinusGenerator.automatic_sinus = ! sinusGenerator.automatic_sinus;
    }

    if (sinusGenerator.automatic_sinus){
        // Update the phase sinus during time
        sinusGenerator.p_sinus += 0.01;
        if (sinusGenerator.p_sinus > 2*M_PI) sinusGenerator.p_sinus = 0;
        modification = true;
    }

    if (modification){
        cloudNeedsUpdate = true;
        sinusGenerator.generateSinus(mainCloud, pointNoise, normalNoise);
    }

    if (ImGui::Button("Save pts")){
        sinusGenerator.saveSinus(mainCloud);
    }

    ImGui::End();
}


void GUI::picking() {
    ImGuiIO& io = ImGui::GetIO();
    // If the user clicked on the screen, while maintaining the shift key, we pick the point
    if (io.MouseClicked[0] && io.KeyShift){
      glm::vec2 screenCoords{io.MousePos.x, io.MousePos.y};
      std::pair<polyscope::Structure*, size_t> pickPair =
          polyscope::pick::evaluatePickQuery(screenCoords.x, screenCoords.y);

      if (pickPair.first != nullptr) 
        pointProcessing.iVertexSource = pickPair.second ;
      }
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
            if ( methodName.find( "PCA" ) != methodName.find( "MeanPLANE" ) ){
                VectorType origin = tempCloud.getDiffQuantities().getVertices().row(0);
                VectorType normal = tempCloud.getDiffQuantities().getNormals().row(0);
                VectorType minDir = tempCloud.getDiffQuantities().getKMinDir().row(0);
                VectorType maxDir = tempCloud.getDiffQuantities().getKMaxDir().row(0);
                std::pair< std::vector<VectorType>, std::vector<std::array<size_t, 3> > > faces_idx = generatePlane(tempCloud.getDiffQuantities().getVertices(), normal, minDir, maxDir);
                // Create a new surface mesh
                std::string meshName = "[" + methodName + "] " + "mesh";
                polyscope::SurfaceMesh* mesh = polyscope::registerSurfaceMesh(meshName, faces_idx.first, faces_idx.second);
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
            }
            else {
                if ( methodName.find("Sphere") != std::string::npos || methodName.find("APSS") != std::string::npos || methodName.find("UnorientedSphere") != std::string::npos ){
                    Scalar radius = Scalar(1) / std::abs( tempCloud.getDiffQuantities().getKMean().row(0)[0] );
                    std::vector<VectorType> vertices(1, tempCloud.getDiffQuantities().getVertices().row(0));
                    SampleMatrixType normals(1, 3); normals.row(0) = tempCloud.getDiffQuantities().getNormals().row(0);
                    SampleMatrixType d1(1, 3); d1.row(0) = tempCloud.getDiffQuantities().getKMinDir().row(0);
                    SampleMatrixType d2(1, 3); d2.row(0) = tempCloud.getDiffQuantities().getKMaxDir().row(0);
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
                    polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, tempCloud.getDiffQuantities().getVertices());
                    polyscope_uniqueClouds.push_back(newCloud);
                    addQuantities(newCloud, "normals", tempCloud.getDiffQuantities().getNormals());
                    addQuantities(newCloud, "d1", tempCloud.getDiffQuantities().getKMinDir());
                    addQuantities(newCloud, "d2", tempCloud.getDiffQuantities().getKMaxDir());
                }
            }
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
        case 4 : 
            values = sliceWithKernel<ExponentialWeightFunc>(pair_slice.first);
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
        case 4 : 
            methodWithKernel<ExponentialWeightFunc>();
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
                case 4 : 
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<ExponentialWeightFunc>());
                    break;
                default : 
                    addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood<SmoothWeightFunc>());
                    break;
            }
        }
    }
    ImGui::SameLine();
    if (pointProcessing.researchType == 0){
        if (ImGui::Button("pc neighbors")) {
            SampleVectorType neighbor_values= pointProcessing.colorizeKnn();
            std::pair<SampleMatrixType, SampleVectorType> res_nei = mainCloud.getNonZeros(neighbor_values, pointProcessing.iVertexSource);
            std::string cloudName = "neighborhood";
            // Create a new point cloud
            polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, res_nei.first);
            polyscope_uniqueClouds.push_back(newCloud);
            addQuantities(newCloud, "neighbors", res_nei.second);
        }
    }
    else {
        if (ImGui::Button("pc neighbors")) {
            SampleVectorType neighbor_values;
            switch (weightFuncType){
                case 0 : 
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<ConstWeightFunc>();
                    break;
                case 1 :
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<SmoothWeightFunc>();
                    break;
                case 2 :
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<WendlandWeightFunc>();
                    break;
                case 3 :
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<SingularWeightFunc>();
                    break;
                case 4 : 
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<ExponentialWeightFunc>();
                    break;
                default : 
                    neighbor_values = pointProcessing.colorizeEuclideanNeighborhood<SmoothWeightFunc>();
                    break;
            }
            std::pair<SampleMatrixType, SampleVectorType> res_nei = mainCloud.getNonZeros(neighbor_values, pointProcessing.iVertexSource);
            std::string cloudName = "neighborhood";
            // Create a new point cloud
            polyscope::PointCloud* newCloud = polyscope::registerPointCloud(cloudName, res_nei.first);
            polyscope_uniqueClouds.push_back(newCloud);
            addQuantities(newCloud, "neighbors", res_nei.second);
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
    ImGui::SameLine();
    ImGui::RadioButton("Exponential", &weightFuncType, 4);

    ImGui::Separator();

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
