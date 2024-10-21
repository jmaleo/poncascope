template<typename Functor>
void PointProcessing::measureTime(const std::string &actionName, Functor F) {
    using namespace std::literals; // enables the usage of 24h instead of e.g. std::chrono::hours(24)
    const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    F(); // run process
    const auto end = std::chrono::steady_clock::now();
    std::cout << actionName << " in " << (end - start) / 1ms << "ms.\n";
}

/// @brief Compute differential quantities
/// @tparam WeightFunc Fit Type, \see definitions.h
/// @param name Name of the method, to be displayed in the console
/// @param cloud Point Cloud to process
template<typename WeightFunc>
void PointProcessing::computeDiffQuantities(const std::string &name, PointCloudDiff<Scalar> &cloud) {
    const DifferentialQuantities<Scalar> res = estimateDifferentialQuantities<WeightFunc>(name);
    cloud.estimatedDiffQuantities = res;
}

template<typename WeightFunc>
void PointProcessing::computeUniquePoint(const std::string &name, PointCloudDiff<Scalar> &cloud) {
    estimateAndProject_OnePoint<WeightFunc>(name, cloud, iVertexSource);
}

template<typename WeightFunc>
SampleVectorType PointProcessing::colorizeNeighbors() {
    return neiRequest<WeightFunc>( iVertexSource );
}

template<typename WeightFunc>
SampleVectorType PointProcessing::colorizeNeighbors(const std::vector<int>& indices, const std::vector<float>& radii) {
    Scalar old_radius = radius;
    int old_iVertexSource = iVertexSource;
    SampleVectorType colors = SampleVectorType::Zero( ponca_kdtree.index_data().size() );
    for (int i = 0; i < indices.size(); i++) {
        iVertexSource = indices[i];
        radius = radii[i];
        const SampleVectorType color = neiRequest<WeightFunc>(iVertexSource);
        colors += color;
    }
    iVertexSource = old_iVertexSource;
    radius = old_radius;
    return colors;
}

void PointProcessing::recomputeKnnGraph() {
    if (use_kNNGraph) {
        measureTime("[Ponca] Build KnnGraph", [this]() {
            delete ponca_knnGraph;
            ponca_knnGraph = new KnnGraph(ponca_kdtree, kNN_for_graph);
        });
    }
}

void PointProcessing::computeVoxelGrid(PointCloudDiff<Scalar> &cloud) {
    if (!use_VoxelGrid)
        return;
    std::cout << "Building the voxel grid" << std::endl;
    voxelGrid = MyVoxelGrid();
    const SampleMatrixType points = cloud.points;
    const SampleMatrixType normals = cloud.normals;
    buildVoxelGrid<SampleMatrixType, MyVoxelGrid, PPAdapter>(points, normals, voxelGrid, resolution, N_voxels);
    std::cout << "Successfully build" << std::endl;
}

void PointProcessing::mlsDryRun() {
    m_meanNeighbors = evalMeanNeighbors();
}

template<typename WeightFunc, bool isSigned>
SampleVectorType PointProcessing::getScalarField(const std::string &name, const SampleMatrixType &input_pos) {
    SampleVectorType scalarField = evalScalarField<WeightFunc>(name, input_pos);
    return scalarField;
}
