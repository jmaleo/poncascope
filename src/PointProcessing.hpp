
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

/// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
/// \note Functor is called only if fit is stable
template<typename FitT, typename Functor>
void PointProcessing::processPointCloud(const typename FitT::WeightFunction& w, Functor f){

    int nvert = tree.index_data().size();


    // Traverse point cloud
    #pragma omp parallel for
    for (int i = 0; i < nvert; ++i) {

        VectorType pos = tree.point_data()[i].pos();
        for( int mm = 0; mm < mlsIter; ++mm) {
            FitT fit;
            fit.setWeightFunc(w);
            fit.init( pos );
            
            Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(i, NSize), tree.point_data() );
            if (res == Ponca::STABLE){

                pos = fit.project( pos );
                if ( mm == mlsIter -1 ) // last mls step, calling functor
                    f(i, fit, pos);
            }
            else {
                std::cerr << "Warning: fit " << i << " is not stable" << std::endl;
            }

        }
    }
}

template<typename FitT>
void
PointProcessing::computeDiffQuantities(const std::string &name, MyPointCloud &cloud) {
    int nvert = tree.index_data().size();
    Eigen::VectorXd mean ( nvert ), kmin ( nvert ), kmax ( nvert );
    Eigen::MatrixXd normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    measureTime( "[Ponca] Compute differential quantities using " + name,
                 [this, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj]() {
                    processPointCloud<FitT>(SmoothWeightFunc(NSize),
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
    // Compute unique point
}


/// Colorize point cloud using kNN
const Eigen::VectorXd PointProcessing::colorizeKnn(MyPointCloud &cloud) {

    int nvert = tree.index_data().size();
    Eigen::VectorXd closest ( nvert );
    closest.setZero();

    closest(iVertexSource) = 2;
    for (int j : tree.k_nearest_neighbors(iVertexSource, kNN)){
        closest(j) = 1;
    }
    
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
    for (int j : tree.range_neighbors(iVertexSource, NSize)){
        const auto &q = tree.point_data()[j];
        closest(j) = w.w( q.pos() - p.pos(), q ).first;
    }
    return closest;
}