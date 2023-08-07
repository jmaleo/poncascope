void GUI::mainCallBack(){

    // Create a window
    ImGui::PushItemWidth(100);
    // The size of the window, the position is set by default
    ImGui::SetNextWindowSize(ImVec2(300, 600), ImGuiCond_FirstUseEver);

    cloudGeneration();
    ImGui::Separator();

    if (cloudNeedsUpdate){
        pointProcessing.update(mainCloud);
        pointProcessing.measureTime("[Polyscope] Update current main cloud", [this](){
            polyscope::removeStructure(mainCloudName, false);
            polyscope_mainCloud = polyscope::registerPointCloud(mainCloudName, mainCloud.getVertices());
            addQuantities(polyscope_mainCloud, "real normals", mainCloud.getNormals());
        });
        lastDryRun = "";
        cloudNeedsUpdate = false;
    }

    cloudComputing();

}

void GUI::cloudGeneration(){

    // Point noise and normal noise selection with sliders
    ImGui::SliderFloat("Point noise",  &pointNoise, 0.0f, 5.0f);
    ImGui::SameLine();
    ImGui::SliderFloat("Normal noise", &normalNoise, 0.0f, 5.0f);

    // If the user selected the file selection
    generationFromFile();


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

void GUI::cloudComputingUpdateAll (){
    if (!all_computed) return;

    pointProcessing.measureTime("[Polyscope] Update diff quantities and projection", [this](){
        
        for (int i = 0; i < selectedQuantities.size(); ++i) {
            if (selectedQuantities[i]){
                std::string completeName = "[" + methodName + "] "+ quantityNames[i];
                addQuantities(polyscope_mainCloud,completeName, mainCloud.getDiffQuantities().getByName(quantityNames[i]));
            }
        }
    });

    all_computed = false;
    methodName = "";
}


template <typename FitT>
void GUI::methodForCloudComputing(const std::string& metName){
    std::string buttonName_all = metName;
    if (ImGui::Button(buttonName_all.c_str())){
        methodName = metName;
        all_computed = true;
        pointProcessing.computeDiffQuantities<FitT>(metName, mainCloud);
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

    methodForCloudComputing<basket_planeFit>("Plane (PCA)");

    ImGui::SameLine();

    methodForCloudComputing<basket_AlgebraicPointSetSurfaceFit>("APSS");

    ImGui::SameLine();

    methodForCloudComputing<basket_AlgebraicShapeOperatorFit>("ASO");

    cloudComputingUpdateAll();
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
    if (ImGui::Button("show knn")) addQuantities(polyscope_mainCloud, "knn", pointProcessing.colorizeKnn());
    ImGui::SameLine();
    if (ImGui::Button("show euclidean nei")) addQuantities(polyscope_mainCloud, "euclidean nei", pointProcessing.colorizeEuclideanNeighborhood());

    ImGui::Separator();

}

void GUI::addQuantities(polyscope::PointCloud *pc, const std::string &name, const Eigen::MatrixXd &values){
    if (values.cols() == 1){
        // Make values beeing a vector
        Eigen::VectorXd valuesVec = values.col(0);
        auto quantity = pc->addScalarQuantity(name, valuesVec);
        // Set bound [-5, 5] for the scalar quantity
        if (name != "knn" && name != "euclidean nei"){
            quantity->setMapRange(std::pair<double,double>(-5,5));
            quantity->setColorMap("coolwarm");
        }
        else {
            quantity->setColorMap("turbo");
        }
    }
    else 
        pc->addVectorQuantity(name, values);
}

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
