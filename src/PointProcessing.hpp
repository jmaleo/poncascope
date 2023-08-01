
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
                std::cerr << "Warning: fit " << idx << " is not stable" << std::endl;
            }
        }
}

/// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
/// \note Functor is called only if fit is stable
template<typename FitT, typename Functor>
void PointProcessing::processPointCloud(const bool &unique, const typename FitT::WeightFunction& w, Functor f){

    if (unique) {
        processOnePoint<FitT, Functor>( iVertexSource, w, f );
    }
    else {
        int nvert = tree.index_data().size();
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
                                            normal.row( k ) = fit.primitiveGradient(init);
                                            proj.row( k )   = projPoint;
                                        }
                                    }
                                });
                    });

    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities(proj, normal,dmin, dmax, kmin, kmax, mean));
}


/// Colorize point cloud using kNN
const Eigen::VectorXd PointProcessing::colorizeKnn(MyPointCloud &cloud) {

    int nvert = tree.index_data().size();
    Eigen::VectorXd closest ( nvert );
    closest.setZero();

    closest(iVertexSource) = 2;
    processRangeNeighbors(iVertexSource, [&closest](int j){
        closest(j) = 1;
    });
    
    return closest;
}

// Colorize point cloud using euclidean distance
const Eigen::VectorXd PointProcessing::colorizeEuclideanNeighborhood(MyPointCloud &cloud) {

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