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
    VectorType pos = tree.point_data()[idx].pos();
    if(useKnnGraph)
        for (int j : knnGraph->range_neighbors(idx, NSize)){
            f(j);
        }
    else
        for (int j : tree.range_neighbors(pos, NSize)){
            f(j);
        }
}

template <typename Functor>
void PointProcessing::processRangeNeighbors(const VectorType &pos, const Functor f){
    for (int j : tree.range_neighbors(pos, NSize)){
        f(j);
    }
}

template<typename FitT>
void PointProcessing::initUseNormal (FitT &fit){
    // Nothing if not a plane fit
}

// template<>
// void PointProcessing::initUseNormal(basket_planeFit<SmoothWeightFunc> &fit){
//     fit.setUseNormal(false);
// }

template<typename FitT, typename Functor>
void PointProcessing::processOnePoint (const VectorType &init_pos, const typename FitT::WeightFunction& w, Functor f){
    
    VectorType pos = init_pos;
    
    for( int mm = 0; mm < mlsIter; ++mm) {
        FitT fit;
        fit.setWeightFunc(w);
        fit.init( pos );
        // initUseNormal(fit);
        
        // Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(idx, NSize), tree.point_data() );
        
        // Loop until fit not need other pass, same process as fit.computeWithIds
        Ponca::FIT_RESULT res;
        do {
            fit.startNewPass();
            processRangeNeighbors(init_pos, [this, &fit](int j) {
                fit.addNeighbor(tree.point_data()[j]);
            });
            res = fit.finalize();
        } while (res == Ponca::NEED_OTHER_PASS);
        
        if (res == Ponca::STABLE){

            pos = fit.project( pos );
            if ( mm == mlsIter -1 ) // last mls step, calling functor
                f(init_pos, fit, pos);
        }
        // else {
        //     std::cerr << "[Ponca][Warning] fit on pos " << init_pos(0) << " " <<  init_pos(1) << " " <<  init_pos(2) << " is not stable" << std::endl;
        // }
    }
}

template<typename FitT, typename Functor>
void PointProcessing::processOnePoint(const int &idx, const typename FitT::WeightFunction& w, Functor f){
    
    VectorType pos = tree.point_data()[idx].pos();
    
    for( int mm = 0; mm < mlsIter; ++mm) {
        FitT fit;
        fit.setWeightFunc(w);
        fit.init( pos );

        // initUseNormal(fit);
        
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
    // use a ptr of m_randomG for the init

    fit.init( pos );

    if (type == 2 ){
        fit.chooseIndependentGeneration();
    }
    else if (type == 3) {
        fit.chooseHexagramGeneration();
    }
    else if (type == 4) {
        fit.chooseAvgHexagramGeneration();
    }
    else {
        fit.chooseUniformGeneration();
    }

    // Position of each nei points
    std::vector<VectorType> neiPos;
    // Normal of each nei points
    std::vector<VectorType> neiNormal;

    // Loop on neighbors
    // if (useKnnGraph) {
    //     for (int j : knnGraph->range_neighbors(idx, NSize)){
    //         neiPos.push_back(tree.point_data()[j].pos());
    //         neiNormal.push_back(tree.point_data()[j].normal());
    //     }
    // }
    // else {
    //     for (int j : tree.range_neighbors(idx, NSize)){
    //         neiPos.push_back(tree.point_data()[j].pos());
    //         neiNormal.push_back(tree.point_data()[j].normal());
    //     }
    // }

    for (int j : tree.k_nearest_neighbors(pos, kNN)){
        neiPos.push_back(tree.point_data()[j].pos());
        neiNormal.push_back(tree.point_data()[j].normal());
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
        // #pragma omp parallel for
        for (int i = 0; i < nvert; ++i) {
            processOnePoint<FitT, Functor>(i, w, f);
        }
    }
}

template<typename FitT, bool isSigned = true>
void
PointProcessing::computeDiffQuantities(const std::string &name, MyPointCloud<Scalar> &cloud) {
    
    using weightFunc = typename FitT::WeightFunction;

    int nvert = tree.index_data().size();

    // Allocate memory
    SampleVectorType mean ( nvert ), kmin ( nvert ), kmax ( nvert ), shapeIndex( nvert );
    SampleMatrixType normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    measureTime( "[Ponca] Compute differential quantities using " + name,
                 [this, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj, &shapeIndex]() {
                    processPointCloud<FitT>(false, weightFunc(NSize),
                                [this, &mean, &kmin, &kmax, &normal, &dmin, &dmax, &proj, &shapeIndex]
                                ( int i, const FitT& fit, const VectorType& mlsPos){

                                    mean(i) = isSigned ? fit.kMean() : std::abs(fit.kMean());
                                    
                                    kmax(i) = isSigned ? fit.kmax() : std::abs(fit.kmax());
                                    kmin(i) = isSigned ? fit.kmin() : std::abs(fit.kmin());

                                    normal.row( i ) = fit.primitiveGradient();
                                    // normalize normal
                                    normal.row( i ) /= normal.row( i ).norm();

                                    dmin.row( i )   = fit.kminDirection();
                                    dmax.row( i )   = fit.kmaxDirection();

                                    // proj.row( i )   = mlsPos - tree.point_data()[i].pos();
                                    proj.row( i )   = mlsPos;

                                    shapeIndex(i) = (2.0 / M_PI) * std::atan( ( kmin(i) + kmax(i) ) / ( kmin(i) - kmax(i) ) );
                                });
                    });
    
    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities<Scalar>(proj, normal,dmin, dmax, kmin, kmax, mean, shapeIndex));
}

void
PointProcessing::computeDiffQuantities_Triangle(const std::string &name, const int& type, MyPointCloud<Scalar> &cloud) {
    
    int nvert = tree.index_data().size();

    // Allocate memory
    SampleVectorType mean ( nvert ), kmin ( nvert ), kmax ( nvert ), shapeIndex( nvert );;
    SampleMatrixType normal( nvert, 3 ), dmin( nvert, 3 ), dmax( nvert, 3 ), proj( nvert, 3 );

    proj.setZero();
    normal.setZero();

    measureTime( "[Ponca] Compute differential quantities using " + name,
                 [this, &type, &mean, &kmin, &kmax, &dmin, &dmax, &shapeIndex]() {
                    processPointCloud_Triangle(false, type,
                                [this, &mean, &kmin, &kmax, &dmin, &dmax, &shapeIndex]
                                ( int i, basket_triangleGeneration& fit, const VectorType& mlsPos){

                                    mean(i) = fit.kMean();
                                    
                                    kmax(i) = fit.kmax();
                                    kmin(i) = fit.kmin();

                                    dmin.row( i )   = fit.kminDirection();
                                    dmax.row( i )   = fit.kmaxDirection();
                                    
                                    shapeIndex(i) = (2.0 / M_PI) * std::atan( ( fit.kmin() + fit.kmax() ) / ( fit.kmin() - fit.kmax() ) );
                                });
                    });
    
    // Add differential quantities to the cloud
    cloud.setDiffQuantities(DiffQuantities<Scalar>(proj, normal,dmin, dmax, kmin, kmax, mean, shapeIndex));
}

// concept ConceptFitT = requires (ConceptFitT fit, VectorType init) {
//     { fit.primitiveGradient(init) } -> std::same_as<SampleMatrixType>;
// };

// template<typename FitT>
// void PointProcessing::processPointUniqueNormal(const int &idx, const FitT& fit, const VectorType& init, SampleMatrixType& normal)
// {
//         normal.row(idx) = fit.primitiveGradient(init);
// }

// template <>
// void PointProcessing::processPointUniqueNormal<basket_AlgebraicShapeOperatorFit>(const int &idx, const basket_AlgebraicShapeOperatorFit<WeightFunc>& fit, const VectorType& init, SampleMatrixType& normal)
// {
//     // Do nothing for ASO
// }


template<typename FitT>
void
PointProcessing::computeUniquePoint(const std::string &name, MyPointCloud<Scalar> &cloud)
{
    using weightFunc = typename FitT::WeightFunction;

    int nvert = cloud.getSize();

    // Allocate memory
    SampleMatrixType normal( nvert, 3 ), proj( nvert, 3 ), D1 (nvert, 3), D2 (nvert, 3);

    // set Zeros 
    normal.setZero();
    proj.setZero();
    D1.setZero();
    D2.setZero();


    measureTime( "[Ponca] Compute differential quantities using " + name,
                [this, &cloud, &nvert, &normal, &proj, &D1, &D2]() {
                    processPointCloud<FitT>(true, weightFunc(NSize),
                                [this, &cloud, &nvert, &normal, &proj, &D1, &D2]
                                ( int i, const FitT& fit, const VectorType& mlsPos){
                                    
                                    D1.row( 0 ) = fit.kminDirection();
                                    D2.row( 0 ) = fit.kmaxDirection();
                                    if ( ! std::is_same<FitT, basket_AlgebraicShapeOperatorFit<weightFunc>>::value)
                                        normal.row( 0 ) = fit.primitiveGradient();
                                            
                        
                                    for (int k = 0; k < nvert; k++){
                                        VectorType init = cloud.getVertices().row(k);
                                        VectorType projPoint = fit.project(init);
                                        if (projPoint != init) {
                                            // if ( ! std::is_same<FitT, basket_AlgebraicShapeOperatorFit<weightFunc>>::value)
                                            //     normal.row( k ) = fit.primitiveGradient();
                                            // processPointUniqueNormal<FitT>(k, fit, init, normal);
                                            proj.row( k )   = projPoint;
                                        }
                                    }

                                    
                                });
                    });
    // Add differential quantities to the cloud
    
    /////////////////////// TEST ///////////////////////
    DiffQuantities<Scalar> test (proj, normal);
    test.setD1(D1);
    test.setD2(D2);
    cloud.setDiffQuantities(test);

    // cloud.setDiffQuantities(DiffQuantities<Scalar>(proj, normal));
}

void PointProcessing::computeUniquePoint_triangle(const std::string &name, const int& type, MyPointCloud<Scalar> &cloud){
    // Used for the use of triangles
    int nb_t = 0; 
    std::vector<std::array<Scalar, 3>> triangles;

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


const SampleVectorType PointProcessing::colorizeKnn() {

    int nvert = tree.index_data().size();
    SampleVectorType closest ( nvert );
    closest.setZero();

    closest(iVertexSource) = 2;
    processRangeNeighbors(iVertexSource, [&closest](int j){
        closest(j) = 1;
    });
    
    return closest;
}

template <typename WeightFunc>
const SampleVectorType PointProcessing::colorizeEuclideanNeighborhood() {

    int nvert = tree.index_data().size();
    SampleVectorType closest ( nvert );
    closest.setZero();

    WeightFunc w(NSize);

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

    Eigen::VectorX<int> numNeighbors(nvert);
    numNeighbors.setZero();


    m_meanNeighbors = Scalar(0);
    // Allocate memory
    measureTime( "[Ponca] Compute differential quantities using dry fit",
                 [this, &numNeighbors]() {
                    processPointCloud<basket_dryFit>(false, SmoothWeightFunc(NSize),
                                [this, &numNeighbors]
                                ( int i, const basket_dryFit& fit, const VectorType&){
                                    numNeighbors[i] = fit.getNumNeighbors();
                    });
                });

    for (int i = 0; i < nvert; ++i)
        m_meanNeighbors += numNeighbors[i];
    m_meanNeighbors /= nvert;    
}

template<typename FitT, bool isSigned = true>
SampleVectorType PointProcessing::evalScalarField_impl(const std::string &name, const SampleMatrixType& input_pos)
{
    using weightFunc = typename FitT::WeightFunction;

    int nvert = input_pos.rows();

    // Allocate memory
    SampleVectorType potential = SampleVectorType::Zero( nvert );

    measureTime( "[Ponca] Compute potentials quantities using " + name,
        [this, &nvert, &input_pos, &potential]() {
            #pragma omp parallel for
            for (int i = 0; i < nvert; ++i) {
                processOnePoint<FitT>(input_pos.row(i), weightFunc(NSize),
                            [this, &i, &potential]
                            ( const VectorType& pos, const FitT& fit, const VectorType& mlsPos){
                                potential(i) = isSigned ? fit.potential(pos) : std::abs(fit.potential(pos));
                });
            }
        });

    return potential;
}