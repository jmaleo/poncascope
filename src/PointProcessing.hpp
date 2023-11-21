template <typename Functor>
void 
PointProcessing::measureTime( const std::string &actionName, Functor F ){
    using namespace std::literals; // enables the usage of 24h instead of e.g. std::chrono::hours(24)
    const std::chrono::time_point<std::chrono::steady_clock> start =
            std::chrono::steady_clock::now();
    F(); // run process
    const auto end = std::chrono::steady_clock::now();
    std::cout << actionName << " in " << (end - start) / 1ms << "ms.\n";
}

template <typename Functor>
void PointProcessing::processRangeNeighbors(const int &idx, const Functor f){
    if(useKnnGraph)
        for (int j : knnGraph->range_neighbors(idx, NSize)){
            f(j);
        }
    else
        for (int j : tree.range_neighbors(idx, NSize)){
            f(j);
        }
}

template<typename FitT, typename Functor>
void PointProcessing::processOnePoint(const int &idx, const typename FitT::WeightFunction& w, Functor f){
        VectorType pos = tree.point_data()[idx].pos();
        for( int mm = 0; mm < mlsIter; ++mm) {
            FitT fit;
            fit.setWeightFunc(w);
            fit.init( pos );
            
            // Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(idx, NSize), tree.point_data() );
            
            // Loop until fit not need other pass, same process as fit.computeWithIds
            Ponca::FIT_RESULT res;
            do {
                fit.startNewPass();
                processRangeNeighbors(idx, [this, &fit](int j) {
                    fit.addNeighbor(tree.point_data()[j]);
                });
                res = fit.finalize();
            } while (res == Ponca::NEED_OTHER_PASS);
            
            if (res == Ponca::STABLE){

                pos = fit.project( pos );
                if ( mm == mlsIter -1 ) // last mls step, calling functor
                    f(idx, fit, pos);
            }
            else {
                std::cerr << "[Ponca][Warning] fit " << idx << " is not stable" << std::endl;
            }
        }
}

template <typename Functor>
void 
PointProcessing::processOnePoint_Triangle(const int& idx, const int& type, Functor f){

    VectorType pos = tree.point_data()[idx].pos();
    basket_triangleGeneration fit;
    fit.init( pos );

    if (type == 2 ){
        fit.chooseIndependentGeneration(nb_max_triangles);
    }
    else if (type == 3) {
        fit.chooseHexagramGeneration(avg_normals_weight);
    }
    else if (type == 4) {
        fit.chooseAvgHexagramGeneration(avg_normals_weight);
    }
    else {
        fit.chooseUniformGeneration(nb_max_triangles);
    }

    // Position of each nei points
    std::vector<VectorType> neiPos;
    // Normal of each nei points
    std::vector<VectorType> neiNormal;

    // Loop on neighbors
    if (useKnnGraph) {
        for (int j : knnGraph->range_neighbors(idx, NSize)){
            std::cout << "[PONCA] KnnGraph used with range_neighbors" << std::endl;
            neiPos.push_back(tree.point_data()[j].pos());
            neiNormal.push_back(tree.point_data()[j].normal());
        }
    }
    else {
        for (int j : tree.k_nearest_neighbors(idx, kNN)){
            neiPos.push_back(tree.point_data()[j].pos());
            neiNormal.push_back(tree.point_data()[j].normal());
        }
    }

    Ponca::FIT_RESULT res;

    fit.startNewPass();

    fit.computeNeighbors(tree.point_data()[idx], neiPos, neiNormal);

    res = fit.finalize();

    if (res == Ponca::STABLE){
        f(idx, fit, pos);
    }
    else {
        std::cerr << "[Ponca][Warning] fit " << idx << " is not stable" << std::endl;
    }
}

template<typename Functor>
void 
PointProcessing::processPointCloud_Triangle(const bool &unique, const int& type, Functor f)
{
    if (unique) {
        processOnePoint_Triangle( iVertexSource, type, f);
    }
    else {
        int nvert = tree.index_data().size();
        std::cout << "nvert: " << nvert << std::endl;
        // Traverse point cloud
        #pragma omp parallel for
        for (int i = 0; i < nvert; ++i) {
            processOnePoint_Triangle( i, type, f);
        }
    }
}

template<typename FitT, typename Functor>
void PointProcessing::processPointCloud(const bool &unique, const typename FitT::WeightFunction& w, Functor f){

    if (unique) {
        processOnePoint<FitT, Functor>( iVertexSource, w, f );
    }
    else {
        int nvert = tree.index_data().size();
        std::cout << "nvert: " << nvert << std::endl;
        // Traverse point cloud
        #pragma omp parallel for
        for (int i = 0; i < nvert; ++i) {
            processOnePoint<FitT, Functor>(i, w, f);
        }
    }
}

template<typename FitT>
void
PointProcessing::computeDiffQuantities(const std::string &name, MyPointCloud &cloud) {
    
    int nvert = tree.index_data().size();

    // Allocate memory
    Eigen::VectorXd mean ( nvert ), kmin ( nvert ), kmax ( nvert );
    Eigen::MatrixXd normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    measureTime( "[Ponca] Compute differential quantities using " + name,
                 [this, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]() {
                    processPointCloud<FitT>(false, SmoothWeightFunc(NSize),
                                [this, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]
                                ( int i, const FitT& fit, const VectorType& mlsPos){

                                    mean(i) = fit.kMean();
                                    
                                    kmax(i) = fit.kmax();
                                    kmin(i) = fit.kmin();

                                    normal.row( i ) = fit.primitiveGradient();
                                    dmin.row( i )   = fit.kminDirection();
                                    dmax.row( i )   = fit.kmaxDirection();

                                    // proj.row( i )   = mlsPos - tree.point_data()[i].pos();
                                    proj.row( i )   = mlsPos;
                                });
                    });
    
    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities(proj, normal,dmin, dmax, kmin, kmax, mean));
}

void
PointProcessing::computeDiffQuantities_Triangle(const std::string &name, const int& type, MyPointCloud &cloud) {
    
    int nvert = tree.index_data().size();

    // Allocate memory
    Eigen::VectorXd mean ( nvert ), kmin ( nvert ), kmax ( nvert );
    Eigen::MatrixXd normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    proj.setZero();
    normal.setZero();

    measureTime( "[Ponca] Compute differential quantities using " + name,
                 [this, &type, &mean, &kmin, &kmax, &dmin, &dmax]() {
                    processPointCloud_Triangle(false, type,
                                [this, &mean, &kmin, &kmax, &dmin, &dmax]
                                ( int i, basket_triangleGeneration& fit, const VectorType& mlsPos){

                                    mean(i) = fit.kMean();
                                    
                                    kmax(i) = fit.kmax();
                                    kmin(i) = fit.kmin();

                                    dmin.row( i )   = fit.kminDirection();
                                    dmax.row( i )   = fit.kmaxDirection();

                                });
                    });
    
    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities(proj, normal,dmin, dmax, kmin, kmax, mean));
}

// concept ConceptFitT = requires (ConceptFitT fit, VectorType init) {
//     { fit.primitiveGradient(init) } -> std::same_as<Eigen::MatrixXd>;
// };

template<typename FitT>
void
PointProcessing::processPointUniqueNormal(const int &idx, const FitT& fit, const VectorType& init, Eigen::MatrixXd& normal)
{
    normal.row(idx) = fit.primitiveGradient(init);
}

template<>
void
PointProcessing::processPointUniqueNormal<basket_AlgebraicShapeOperatorFit>(const int &idx, const basket_AlgebraicShapeOperatorFit& fit, const VectorType& init, Eigen::MatrixXd& normal)
{
    // Do nothing because ASO does not have a primitive gradient (VectorType pos) function.
}

template<typename FitT>
void
PointProcessing::computeUniquePoint(const std::string &name, MyPointCloud &cloud)
{
    int nvert = cloud.getSize();

    // Allocate memory
    Eigen::VectorXd mean ( nvert ), kmin ( nvert ), kmax ( nvert );
    Eigen::MatrixXd normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    // set Zeros 
    mean.setZero();
    kmin.setZero();
    kmax.setZero();
    normal.setZero();
    dmin.setZero();
    dmax.setZero();
    proj.setZero();


    measureTime( "[Ponca] Compute differential quantities using " + name,
                [this, &cloud, &nvert, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]() {
                    processPointCloud<FitT>(true, SmoothWeightFunc(NSize),
                                [this, &cloud, &nvert, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]
                                ( int i, const FitT& fit, const VectorType& mlsPos){
                                    
                        
                                    for (int k = 0; k < nvert; k++){
                                        VectorType init = cloud.getVertices().row(k);
                                        VectorType projPoint = fit.project(init);
                                        if (projPoint != init) {
                                            processPointUniqueNormal<FitT>(k, fit, init, normal);
                                            proj.row( k )   = projPoint;
                                        }
                                    }

                                    
                                });
                    });
    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities(proj, normal, dmin, dmax, kmin, kmax, mean));
}

void PointProcessing::computeUniquePoint_triangle(const std::string &name, const int& type, MyPointCloud &cloud){
    // Used for the use of triangles
    int nb_t = 0; 
    std::vector<std::array<double, 3>> triangles;

    measureTime( "[Ponca] Compute differential quantities using " + name,
                [this, &type, &nb_t, &triangles]() {
                    processPointCloud_Triangle(true, type,
                                [this, &nb_t, &triangles]
                                ( int i, basket_triangleGeneration& fit, const VectorType& mlsPos){
                                    nb_t = fit.getNumTriangles();
                                    fit.getTriangles(triangles);
                                });
                    });

    cloud.setTriangles(triangles);

    std::cout << "Number of triangles " << nb_t << std::endl;
}


const Eigen::VectorXd PointProcessing::colorizeKnn() {

    int nvert = tree.index_data().size();
    Eigen::VectorXd closest ( nvert );
    closest.setZero();

    closest(iVertexSource) = 2;
    if (useKnnGraph) {
        for (int j : knnGraph->range_neighbors(iVertexSource, NSize)){
            std::cout << "[PONCA] KnnGraph used with range_neighbors" << std::endl;
            closest(j) = 1;
        }
    }
    else {
        for (int j : tree.k_nearest_neighbors(iVertexSource, kNN)){
            closest(j) = 1;
        }
    }
    
    return closest;
}

const Eigen::VectorXd PointProcessing::colorizeEuclideanNeighborhood() {

    int nvert = tree.index_data().size();
    Eigen::VectorXd closest ( nvert );
    closest.setZero();

    SmoothWeightFunc w(NSize);

    closest(iVertexSource) = 2;
    const auto &p = tree.point_data()[iVertexSource];
    processRangeNeighbors(iVertexSource, [this, w,p,&closest](int j){
        const auto &q = tree.point_data()[j];
        closest(j) = w.w( q.pos() - p.pos(), q ).first;
    });
    return closest;
}

void PointProcessing::recomputeKnnGraph() {
    if(useKnnGraph) {
        measureTime("[Ponca] Build KnnGraph", [this]() {
            delete knnGraph;
            knnGraph = new KnnGraph(tree, kNN);
        });
    }
}

void PointProcessing::mlsDryRun() {
    int nvert = tree.index_data().size();
    m_meanNeighbors = Scalar(0);

    // Allocate memory
    measureTime( "[Ponca] Compute differential quantities using dry fit",
                 [this]() {
                    processPointCloud<basket_dryFit>(false, SmoothWeightFunc(NSize),
                                [this]
                                ( int, const basket_dryFit& fit, const VectorType&){
                                    m_meanNeighbors += fit.getNumNeighbors();
                    });
                });

    m_meanNeighbors /= nvert;    
}
