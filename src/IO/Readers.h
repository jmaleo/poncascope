#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <set>

#include <Eigen/Dense>
#include <igl/read_triangle_mesh.h>
#include <igl/readPLY.h>
#include <igl/per_vertex_normals.h>
#include "../../deps/happly.h"

namespace IO
{

template<typename _Scalar>
bool loadMeshWithLibigl(const std::string& filename, Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>& V, Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>& N, Eigen::MatrixXi& F) {
    Eigen::MatrixXi cloudE;
    Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> cloudUV;
    try {
        if (!igl::readPLY(filename, V, F, cloudE, N, cloudUV)) {

            std::cerr << "Erreur lors du chargement du fichier avec libigl" << std::endl;
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Exception lors du chargement avec libigl: " << e.what() << std::endl;
        return false;
    }
}

template<typename _Scalar>
void loadPLYPointCloud (std::string& filename, Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloudV, Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloudN){
    using Point = Eigen::Matrix<_Scalar, 3, 1>;
    happly::PLYData plyIn(filename);

    std::vector <_Scalar> x = plyIn.getElement("vertex").getProperty<_Scalar >("x");
    std::vector <_Scalar> y = plyIn.getElement("vertex").getProperty< _Scalar >("y");
    std::vector <_Scalar> z = plyIn.getElement("vertex").getProperty< _Scalar >("z");
    std::vector <_Scalar> nx = plyIn.getElement("vertex").getProperty< _Scalar >("nx");
    std::vector <_Scalar> ny = plyIn.getElement("vertex").getProperty< _Scalar >("ny");
    std::vector <_Scalar> nz = plyIn.getElement("vertex").getProperty< _Scalar >("nz");
    
    cloudV = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>(x.size(), 3);
    cloudN = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>(x.size(), 3);

    #pragma omp parallel for
    for (int i = 0; i < x.size(); ++i){
        cloudV.row(i) = Point(x[i], y[i], z[i]);
        cloudN.row(i) = Point(nx[i], ny[i], nz[i]);
    }
}

template <typename PointCloudDiff, typename _Scalar>
void loadPointCloud_other_than_PTS( PointCloudDiff& pointCloud,  std::string & input ){

    using Point = Eigen::Matrix<_Scalar, 3, 1>;
    Eigen::MatrixXi meshF;
    Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> cloudV, cloudN;

    if (input.substr(input.find_last_of(".") + 1) == "ply"){
        if (! loadMeshWithLibigl (input, cloudV, cloudN, meshF) ) {
            std::cout << "[libIGL] Loading the PLY mesh with libIGL failed. Trying to load the point cloud using happly..." << std::endl;
            loadPLYPointCloud(input, cloudV, cloudN);
        }
    }
    else {
        igl::read_triangle_mesh(input, cloudV, meshF);
    }
    if (cloudN.rows() == 0)
        igl::per_vertex_normals(cloudV, meshF, cloudN);

    // Check if there is mesh 
    if ( meshF.rows() == 0 && cloudN.rows() == 0 ) {
        std::cout << "[libIGL] The mesh is empty. Aborting..." << std::endl;
        // exit (EXIT_FAILURE);
    }

    // Check if normals have been properly loaded
    int nbUnitNormal = cloudN.rowwise().squaredNorm().sum();
    // if ( meshF.rows() != 0 && nbUnitNormal != cloudV.rows() ) {
    if ( meshF.rows() != 0 && cloudN.rows() == 0 ) {
        std::cout << "[libIGL] An error occurred when computing the normal vectors from the mesh. Aborting..." << std::endl;
        // exit (EXIT_FAILURE);
    }

    // Check if the number of vertices and normals are consistent
    if ( cloudV.rows() != cloudN.rows() ) {
        std::cout << "[libIGL] The number of vertices and normals are not consistent. Aborting..." << std::endl;
        // exit (EXIT_FAILURE);
    }
    pointCloud.init_zeros(cloudV.rows());

    pointCloud.points = cloudV;
    pointCloud.normals = cloudN;
}


template <typename _Scalar>
void processHeader( std::map < std::string, _Scalar > & header_idx, std::string & line )
{
    if (line.find("#") != std::string::npos) {
        std::stringstream linestream(line);
        std::string header;
        linestream >> header;
        int idx = 0;
        while (linestream >> header) {
            header_idx[header] = idx++;
        }
    } else {
        header_idx = {{"x", 0}, {"y", 1}, {"z", 2}, {"nx", 3}, {"ny", 4}, {"nz", 5}};
    }

    // std::cout << "Header: " << std::endl;
    // for (const auto& h : header_idx) {
    //     std::cout << h.first << " : " << h.second << std::endl;
    // }
}

template <typename PointCloudDiff, typename _Scalar>
bool processLine ( PointCloudDiff& pointCloud, const int line_idx, const std::map < std::string, _Scalar > & header_idx, std::string & line ) {

    using Point = Eigen::Matrix<_Scalar, 3, 1>;
    if (line.find("#") != std::string::npos) {
        std::cout << "Skipping # line" << std::endl;
        return false;
    }

    std::vector<_Scalar> line_values(header_idx.size(), 0.0);
    std::stringstream linestream(line);
    std::string token;
    int idx = 0;

    _Scalar epsilon = _Scalar(150);

    while (linestream >> token) {
        for (const auto& pair : header_idx) {
            if (pair.second == idx) {
                size_t find_exp = token.find("e");
                if (find_exp != std::string::npos) {
                    int exp = std::stoi(token.substr(find_exp + 1));
                    if ( exp > std::numeric_limits<_Scalar>::max_exponent10 ) {
                        line_values[idx] = std::numeric_limits<_Scalar>::max();
                    }
                    if ( exp < std::numeric_limits<_Scalar>::min_exponent10 ) {
                        line_values[idx] = std::numeric_limits<_Scalar>::min();
                    }
                }
                else {
                    line_values[idx] = std::stod(token);
                }
                break;
            }
        }
        idx++;
    }

    if ( header_idx.find("k1") != header_idx.end() ) {
        _Scalar val_k1 = line_values[header_idx.at("k1")];
        _Scalar val_k2 = line_values[header_idx.at("k2")];
        if ( val_k1 > epsilon || val_k1 < -epsilon || val_k2 > epsilon || val_k2 < -epsilon ) {
            return false;
        }
    }

    pointCloud.points.row(line_idx) = Point(line_values[header_idx.at("x")],
                           line_values[header_idx.at("y")],
                           line_values[header_idx.at("z")]);

    if (header_idx.find("nx") != header_idx.end())
        pointCloud.normals.row(line_idx) = Point(line_values[header_idx.at("nx")],
                                line_values[header_idx.at("ny")],
                                line_values[header_idx.at("nz")]);

    if (header_idx.find("Gauss_Curvature") != header_idx.end() &&
        header_idx.find("Mean_Curvature") != header_idx.end() &&
        header_idx.find("k1") != header_idx.end() &&
        header_idx.find("k2") != header_idx.end() &&
        header_idx.find("d1x") != header_idx.end() &&
        header_idx.find("d2x") != header_idx.end()) {
            pointCloud.gauss[line_idx] = line_values[header_idx.at("Gauss_Curvature")];
            pointCloud.mean[line_idx] = line_values[header_idx.at("Mean_Curvature")];
            pointCloud.k1[line_idx] = line_values[header_idx.at("k1")];
            pointCloud.k2[line_idx] = line_values[header_idx.at("k2")];
            pointCloud.v1.row(line_idx) = Point(line_values[header_idx.at("d1x")],
                            line_values[header_idx.at("d1y")],
                            line_values[header_idx.at("d1z")]);
            pointCloud.v2.row(line_idx) = Point(line_values[header_idx.at("d2x")],
                            line_values[header_idx.at("d2y")],
                            line_values[header_idx.at("d2z")]);
    }

    return true;
}

int pointCount (std::string & input){
    int nVert = 0;
    std::string line;
    std::ifstream ifs( input, std::ifstream::in );
    do {
        if (line.find("#") != std::string::npos) continue;
        nVert++;
    } while ( std::getline( ifs, line ) );
    ifs.close();
    return nVert;
}

template <typename PointCloudDiff, typename _Scalar>
void loadPointCloud( PointCloudDiff& pointCloud, std::string & input )
{
    using Point = Eigen::Matrix<_Scalar, 3, 1>;
    std::string extension = input.substr(input.find_last_of(".") + 1);

    if ( extension != "pts"
        && extension != "xyz") {
        loadPointCloud_other_than_PTS<PointCloudDiff, _Scalar>(pointCloud, input);
        return;
    }
    int nVert = pointCount(input);
    pointCloud.init_zeros(nVert);

    std::map < std::string, _Scalar > header_idx;
    std::ifstream ifs( input, std::ifstream::in );
    // If first line is a header (starts with #), read it
    std::string line;
    int line_idx = 0;
    if ( std::getline( ifs, line ) )
    {
        processHeader<_Scalar>( header_idx, line );
    }
    do {
        if ( processLine<PointCloudDiff, _Scalar>( pointCloud, line_idx, header_idx, line ) ) 
            line_idx++; 
    } while ( std::getline( ifs, line ) );
    
    ifs.close();
    std::cout << "Read: {} points " << nVert << std::endl;
}

} // namespace IO
