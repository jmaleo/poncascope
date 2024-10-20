template<typename Functor>
void PointProcessing::measureTime(const std::string &actionName, Functor F) {
    using namespace std::literals; // enables the usage of 24h instead of e.g. std::chrono::hours(24)
    const std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    F(); // run process
    const auto end = std::chrono::steady_clock::now();
    std::cout << actionName << " in " << (end - start) / 1ms << "ms.\n";
}

template<typename FitT, typename Functor>
void PointProcessing::processOnePoint(const VectorType &init_pos, const typename FitT::WeightFunction &w, Functor f) {
    VectorType pos = init_pos;

    for (int mm = 0; mm < mlsIter; ++mm) {
        FitT fit;
        fit.setWeightFunc(w);
        fit.init(pos);
        // initUseNormal(fit);

        // Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(idx, NSize), tree.point_data() );

        // Loop until fit not need other pass, same process as fit.computeWithIds
        Ponca::FIT_RESULT res;
        do {
            fit.startNewPass();
            processNeighbors(init_pos, [this, &fit](int j) { fit.addNeighbor(tree.point_data()[j]); });
            res = fit.finalize();
        } while (res == Ponca::NEED_OTHER_PASS);

        if (res == Ponca::STABLE) {
            pos = fit.project(pos);
            if (mm == mlsIter - 1) // last mls step, calling functor
                f(init_pos, fit, pos);
        }
        // else {
        //     std::cerr << "[Ponca][Warning] fit on pos " << init_pos(0) << " " <<  init_pos(1) << " " <<  init_pos(2)
        //     << " is not stable" << std::endl;
        // }
    }
}

template<typename FitT, typename Functor>
void PointProcessing::processOnePoint(const int &idx, const typename FitT::WeightFunction &w, Functor f) {
    VectorType pos = tree.point_data()[idx].pos();

    for (int mm = 0; mm < mlsIter; ++mm) {
        FitT fit;
        fit.setWeightFunc(w);
        fit.init(pos);

        // initUseNormal(fit);

        // Ponca::FIT_RESULT res = fit.computeWithIds(tree.range_neighbors(idx, NSize), tree.point_data() );

        // Loop until fit not need other pass, same process as fit.computeWithIds
        Ponca::FIT_RESULT res;
        do {
            fit.startNewPass();
            processNeighbors(idx, [this, &fit](int j) { fit.addNeighbor(tree.point_data()[j]); });
            res = fit.finalize();
        } while (res == Ponca::NEED_OTHER_PASS);

        if (res == Ponca::STABLE) {
            pos = fit.project(pos);
            if (mm == mlsIter - 1) // last mls step, calling functor
                f(idx, fit, pos);
        } else {
            std::cerr << "[Ponca][Warning] fit " << idx << " is not stable" << std::endl;
        }
    }
}

template<typename Functor>
void PointProcessing::processOnePoint_Triangle(const int &idx, const int &type, Functor f) {
    VectorType pos = tree.point_data()[idx].pos();
    basket_triangleGeneration fit;
    // use a ptr of m_randomG for the init

    fit.init(pos);

    if (type == 2) {
        fit.chooseIndependentGeneration();
    } else if (type == 3) {
        fit.chooseHexagramGeneration();
    } else if (type == 4) {
        fit.chooseAvgHexagramGeneration();
    } else {
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

    processNeighbors(idx, [this, &neiPos, &neiNormal](int j) {
        neiPos.push_back(tree.point_data()[j].pos());
        neiNormal.push_back(tree.point_data()[j].normal());
    });

    // for (int j : tree.k_nearest_neighbors(pos, kNN)){
    //     neiPos.push_back(tree.point_data()[j].pos());
    //     neiNormal.push_back(tree.point_data()[j].normal());
    // }
    Ponca::FIT_RESULT res;

    fit.startNewPass();

    fit.computeNeighbors(tree.point_data()[idx], neiPos, neiNormal);

    res = fit.finalize();

    if (res == Ponca::STABLE) {
        f(idx, fit, pos);
    } else {
        std::cerr << "[Ponca][Warning] fit " << idx << " is not stable" << std::endl;
    }
}

/// Generic processing function: traverse point cloud, compute fitting, and use functor to process fitting output
/// \note Functor is called only if fit is stable
template<typename Estimator, typename Functor>
void processPointCloud( Estimator& estimator, int uniqueIdx, Functor f ){
    int nvert = ponca_kdtree.index_data().size();

    if ( uniqueIdx > -1 ){
        Quantity<Scalar> q;
        estimator( uniqueIdx, q );
        f ( uniqueIdx, q );
        // Print results of the quantity
        // std::cout << "Unique point " << idx << ":\n";
        // std::cout << "Mean: " << q.mean << "\n";
        // std::cout << "Kmin: " << q.k1 << "\n";
        // std::cout << "Kmax: " << q.k2 << "\n";
        // std::cout << "Gauss: " << q.gauss << "\n";
        return;
    }

#pragma omp parallel for
    for (int i = 0; i < nvert; ++i) {
        DGtal::trace.progressBar( i, nvert );
        Quantity<Scalar> q;
        estimator(i, q);
        f (i, q);
    }
}

/// Generic processing function: traverse point cloud and compute mean, first and second curvatures + their direction
/// \tparam FitT Defines the type of estimator used for computation
template<typename Estimator>
DifferentialQuantities<Scalar> estimateDifferentialQuantities_impl( Estimator &estimator, int uniqueIdx ) {

    int nvert = ponca_kdtree.index_data().size();

    VectorType defaultVectorType = VectorType(1, 0, 0); // Keep the norm to 1 to avoid numerical issues for non-stable points

    DGtal::Statistic<Scalar> statTime, statNei;
    std::vector<Scalar> mean ( nvert ), kmin ( nvert ), kmax ( nvert ), gauss ( nvert );
    std::vector<Eigen::Vector<Scalar, 3>> proj ( nvert ), normal( nvert, defaultVectorType ), dmin( nvert, defaultVectorType ), dmax( nvert,defaultVectorType );
    std::vector <unsigned int> non_stable_idx(nvert, 0);

    measureTime( "[Ponca] Compute differential quantities using " + estimator.getName(),
                 [&uniqueIdx, &estimator, &proj, &mean, &kmin, &kmax, &gauss, &normal, &dmin, &dmax, &statTime, &statNei, &non_stable_idx]() {
                            processPointCloud(estimator, uniqueIdx,
                                [&proj, &mean, &kmin, &kmax, &gauss, &normal, &dmin, &dmax, &statTime, &statNei, &non_stable_idx]
                                (
                                    int i, Quantity<Scalar> &quantity
                                ) {
                                    statNei.addValue( quantity.neighbor_count );
                                    // statNei.addValue( fit.getNumNeighbors() );
                                    statTime.addValue( quantity.computing_time.count() );

                                    if (quantity.non_stable >= 1) {
                                        non_stable_idx[i] = 1;
                                        return;
                                    }
                                    mean  [ i ]   = quantity.mean;
                                    kmax  [ i ]   = quantity.k2;
                                    kmin  [ i ]   = quantity.k1;
                                    dmax  [ i ]   = quantity.d2;
                                    dmin  [ i ]   = quantity.d1;
                                    gauss [ i ]   = quantity.gauss;
                                    normal[ i ]   = quantity.normal;
                                    proj  [ i ]   = quantity.projection;
                                });
                            });

    // if there is unstable points, we don't return the results

    DifferentialQuantities<Scalar> dq = { proj, kmin, kmax, mean, gauss, dmin, dmax, normal, statNei, statTime };
    dq.setNonStableVector(non_stable_idx);
    return dq;
}

template<typename WeightFunc>
DifferentialQuantities<Scalar> estimateDifferentialQuantities( std::string name, int uniqueIdx = -1 ) {

    std::shared_ptr<EstimatorFactory< WeightFunc >> factory = getEstimatorFactory<WeightFunc>();

    auto estimator = factory->getEstimator(name);

    DifferentialQuantities<Scalar> diff = estimateDifferentialQuantities_impl( *estimator, uniqueIdx );
    diff.setOriented(estimator->isOriented());
    return diff;
}

template<typename FitT>
void PointProcessing::computeUniquePoint(const std::string &name, MyPointCloud<Scalar> &cloud) {
    using weightFunc = typename FitT::WeightFunction;

    int nvert = cloud.getSize();

    // Allocate memory
    SampleMatrixType normal(nvert, 3), proj(nvert, 3), D1(nvert, 3), D2(nvert, 3);

    // set Zeros
    normal.setZero();
    proj.setZero();
    D1.setZero();
    D2.setZero();


    measureTime("[Ponca] Compute differential quantities using " + name, [this, &cloud, &nvert, &normal, &proj, &D1,
                                                                          &D2]() {
        processPointCloud<FitT>(
                true, weightFunc(NSize),
                [this, &cloud, &nvert, &normal, &proj, &D1, &D2](int i, const FitT &fit, const VectorType &mlsPos) {
                    D1.row(0) = fit.kminDirection();
                    D2.row(0) = fit.kmaxDirection();
                    proj.row(0) = fit.project(getVertexSourcePosition());
                    if (!std::is_same<FitT, basket_AlgebraicShapeOperatorFit<weightFunc>>::value)
                        normal.row(0) = fit.primitiveGradient();


                    for (int k = 1; k < nvert; k++) {
                        VectorType init = cloud.getVertices().row(k);
                        VectorType projPoint = fit.project(init);
                        if ((init - projPoint).norm() > 1e-6) {
                            // if ( ! std::is_same<FitT, basket_AlgebraicShapeOperatorFit<weightFunc>>::value)
                            //     normal.row( k ) = fit.primitiveGradient();
                            // processPointUniqueNormal<FitT>(k, fit, init, normal);
                            proj.row(k) = projPoint;
                        } else {
                            proj.row(k) = proj.row(0);
                        }
                    }
                });
    });
    // Add differential quantities to the cloud

    /////////////////////// TEST ///////////////////////
    DiffQuantities<Scalar> test(proj, normal);
    test.setD1(D1);
    test.setD2(D2);
    cloud.setDiffQuantities(test);

    // cloud.setDiffQuantities(DiffQuantities<Scalar>(proj, normal));
}

void PointProcessing::computeUniquePoint_triangle(const std::string &name, const int &type,
                                                  MyPointCloud<Scalar> &cloud) {
    // Used for the use of triangles
    int nb_t = 0;
    std::vector<std::array<Scalar, 3>> triangles;

    // init position
    VectorType pos = tree.point_data()[iVertexSource].pos();
    for (int i = 0; i < 3; i++) {
        triangles.push_back({pos(0), pos(1), pos(2)});
    }

    VectorType normal = VectorType::Zero(3);
    VectorType D1 = VectorType::Zero(3);
    VectorType D2 = VectorType::Zero(3);

    measureTime("[Ponca] Compute differential quantities using " + name, [this, &type, &nb_t, &triangles, &D1, &D2,
                                                                          &normal]() {
        processPointCloud_Triangle(true, type,
                                   [this, &nb_t, &triangles, &D1, &D2, &normal](int i, basket_triangleGeneration &fit,
                                                                                const VectorType &mlsPos) {
                                       nb_t = fit.getNumTriangles();
                                       fit.getTriangles(triangles);
                                       normal = fit.primitiveGradient();
                                       D1 = fit.kminDirection();
                                       D2 = fit.kmaxDirection();
                                   });
    });
    // Add a triangle to the cloud, corresponding to the triangle of the selected point


    int size = triangles.size();

    SampleMatrixType vertices = SampleMatrixType::Zero(size, 3);
    SampleMatrixType normals = SampleMatrixType::Zero(size, 3);
    SampleMatrixType D1s = SampleMatrixType::Zero(size, 3);
    SampleMatrixType D2s = SampleMatrixType::Zero(size, 3);

    // Create the point clouds
    int i = 0;
    for (const std::array<Scalar, 3> point: triangles) {
        vertices.row(i) = VectorType(point[0], point[1], point[2]);
        i++;
    }

    // Only put the normal and directions of the selected point's triangle
    for (int i = 0; i < 3; i++) {
        normals.row(i) = normal;
        D1s.row(i) = D1;
        D2s.row(i) = D2;
    }

    cloud = MyPointCloud<Scalar>(vertices, normals);

    cloud.setTriangles(triangles);
    DiffQuantities<Scalar> test(vertices, normals);
    test.setD1(D1s);
    test.setD2(D2s);
    cloud.setDiffQuantities(test);

    std::cout << "Number of triangles " << nb_t << std::endl;
}


const SampleVectorType PointProcessing::colorizeKnn() {
    int nvert = tree.index_data().size();
    SampleVectorType closest(nvert);
    closest.setZero();

    processNeighbors(iVertexSource, [&closest](int j) { closest(j) = 1; });
    // closest(iVertexSource) = 2;

    return closest;
}

template<typename WeightFunc>
const SampleVectorType PointProcessing::colorizeEuclideanNeighborhood() {
    int nvert = tree.index_data().size();
    SampleVectorType closest(nvert);
    closest.setZero();

    WeightFunc w(NSize);

    const auto &p = tree.point_data()[iVertexSource];
    processNeighbors(iVertexSource, [this, w, p, &closest](int j) {
        const auto &q = tree.point_data()[j];
        closest(j) = w.w(q.pos() - p.pos(), q).first;
    });
    // closest(iVertexSource) = 2;

    return closest;
}

template<typename WeightFunc>
const SampleVectorType PointProcessing::colorizeEuclideanNeighborhood(const std::vector<int> &vertexQueries,
                                                                      const std::vector<float> &radii) {
    int nvert = tree.index_data().size();
    SampleVectorType closest(nvert);
    closest.setZero();

    // check that the size of the two vectors are the same
    if (vertexQueries.size() != radii.size()) {
        std::cerr << "[Ponca][Error] The size of the vertexQueries and radii vectors must be the same" << std::endl;
        return closest;
    }

    for (int idx = 0; idx < vertexQueries.size(); idx++) {
        int queryIdx = vertexQueries[idx];
        float queryRadius = radii[idx];
        WeightFunc w(queryRadius);
        const auto &p = tree.point_data()[queryIdx];
        processNeighbors(queryIdx, [this, w, p, &closest](int j) {
            const auto &q = tree.point_data()[j];
            float weight = w.w(q.pos() - p.pos(), q).first;
            if (weight > closest(j))
                closest(j) = weight;
        });
    }

    return closest;
}

void PointProcessing::recomputeKnnGraph() {
    if (useKnnGraph) {
        measureTime("[Ponca] Build KnnGraph", [this]() {
            delete knnGraph;
            knnGraph = new KnnGraph(tree, kNN_for_graph);
        });
    }
}

inline void PointProcessing::computeVoxelGrid(MyPointCloud<Scalar> &cloud) {
    if (!useVoxelGrid)
        return;
    std::cout << "Building the voxel grid" << std::endl;
    voxelGrid = MyVoxelGrid();
    const SampleMatrixType points = cloud.getVertices();
    const SampleMatrixType normals = cloud.getNormals();
    buildVoxelGrid<SampleMatrixType, MyVoxelGrid, PPAdapter>(points, normals, voxelGrid, resolution, N);
    std::cout << "Successfully build" << std::endl;
}

void PointProcessing::mlsDryRun() {
    int nvert = tree.index_data().size();

    Eigen::VectorX<int> numNeighbors(nvert);
    numNeighbors.setZero();


    m_meanNeighbors = Scalar(0);
    // Allocate memory
    measureTime("[Ponca] Compute differential quantities using dry fit", [this, &numNeighbors]() {
        processPointCloud<basket_dryFit>(false, SmoothWeightFunc(NSize),
                                         [this, &numNeighbors](int i, const basket_dryFit &fit, const VectorType &) {
                                             numNeighbors[i] = fit.getNumNeighbors();
                                         });
    });

    for (int i = 0; i < nvert; ++i)
        m_meanNeighbors += numNeighbors[i];
    m_meanNeighbors /= nvert;
}

template<typename FitT, bool isSigned = true>
SampleVectorType PointProcessing::evalScalarField_impl(const std::string &name, const SampleMatrixType &input_pos) {
    using weightFunc = typename FitT::WeightFunction;

    int nvert = input_pos.rows();

    // Allocate memory
    SampleVectorType potential = SampleVectorType::Zero(nvert);

    measureTime("[Ponca] Compute potentials quantities using " + name, [this, &nvert, &input_pos, &potential]() {
#pragma omp parallel for
        for (int i = 0; i < nvert; ++i) {
            processOnePoint<FitT>(
                    input_pos.row(i), weightFunc(NSize),
                    [this, &i, &potential](const VectorType &pos, const FitT &fit, const VectorType &mlsPos) {
                        potential(i) = isSigned ? fit.potential(pos) : std::abs(fit.potential(pos));
                    });
        }
    });

    return potential;
}
