#pragma once

#include <igl/read_triangle_mesh.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/per_vertex_normals.h>
#include "MyPointCloud.h"
#include "definitions.h"

SampleMatrixType rescalePoints (SampleMatrixType &vertices){
    VectorType baryCenter = VectorType::Zero();
    // Compute barycenter
    for (int i = 0; i < vertices.rows(); ++i){
        baryCenter += vertices.row(i);
    }
    baryCenter /= Scalar(vertices.rows());

    // Compute max distance
    VectorType maxDist({0.0, 0.0, 0.0});
    for (int i = 0; i < vertices.rows(); ++i){
        VectorType current = VectorType(vertices.row(i)[0], vertices.row(i)[1], vertices.row(i)[2]) - baryCenter;
        maxDist = maxDist.cwiseMax(current.cwiseAbs());
    }
    Scalar maxDistNorm = maxDist.maxCoeff();

    // Rescale
    SampleMatrixType rescaledVertices = SampleMatrixType::Zero(vertices.rows(), 3);
    for (int i = 0; i < vertices.rows(); ++i){
        VectorType current = VectorType(vertices.row(i)[0], vertices.row(i)[1], vertices.row(i)[2]);
        rescaledVertices.row(i) = (current - baryCenter) / maxDistNorm;
    }
    return rescaledVertices;
}

void loadPTSObject (MyPointCloud<Scalar> &cloud, std::string filename, Scalar sigma_pos, Scalar sigma_normal){

    SampleMatrixType cloudV, cloudN;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << filename << std::endl;
        return;
    }

    std::string line;
    // Read the first line (header)
    std::getline(file, line);
    std::istringstream iss(line);
    std::vector<std::string> header;
    std::string word;
    while (iss >> word) {
        header.push_back(word);
    }

    // Find the indices of the x, y, z, nx, ny, nz in the header
    std::map<std::string, int> indices;
    for (int i = 0; i < header.size(); ++i) {
        if (header[i] == "x" || header[i] == "y" || header[i] == "z" || header[i] == "nx" || header[i] == "ny" || header[i] == "nz") {
            indices[header[i]] = i - 1;
        }
    }

    // Read the rest of the file
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<Scalar> values(header.size());
        for (int i = 0; i < header.size(); ++i) {
            iss >> values[i];
        }

        // Add the read values to the matrices
        cloudV.conservativeResize(cloudV.rows() + 1, 3);
        cloudV.row(cloudV.rows() - 1) << values[indices["x"]], values[indices["y"]], values[indices["z"]];

        cloudN.conservativeResize(cloudN.rows() + 1, 3);
        cloudN.row(cloudN.rows() - 1) << values[indices["nx"]], values[indices["ny"]], values[indices["nz"]];
    }

    file.close();

    std::cout << "cloudV: " << cloudV.rows() << " " << cloudV.cols() << std::endl;

    cloud = MyPointCloud(cloudV, cloudN);
    cloud.addNoise(sigma_pos, sigma_normal);
}


void loadObject (MyPointCloud<Scalar> &cloud, std::string filename, Scalar sigma_pos, Scalar sigma_normal) {

    Eigen::MatrixXi meshF;
    SampleMatrixType cloudV, cloudN;

    if (filename.substr(filename.find_last_of(".") + 1) == "pts") {
        loadPTSObject(cloud, filename, sigma_pos, sigma_normal);
        return;
    }
    else {
        if (filename.substr(filename.find_last_of(".") + 1) == "ply"){
            Eigen::MatrixXi cloudE;
            SampleMatrixType cloudUV;
            igl::readPLY(filename, cloudV, meshF, cloudE, cloudN, cloudUV);
        }
        else {
            igl::read_triangle_mesh(filename, cloudV, meshF);
        }
        if (cloudN.rows() == 0)
            igl::per_vertex_normals(cloudV, meshF, cloudN);
    }
    // Check if there is mesh 
    if ( meshF.rows() == 0 && cloudN.rows() == 0 ) {
        std::cerr << "[libIGL] The mesh is empty. Aborting..." << std::endl;
        exit (EXIT_FAILURE);
    }

    // Check if normals have been properly loaded
    int nbUnitNormal = cloudN.rowwise().squaredNorm().sum();
    // if ( meshF.rows() != 0 && nbUnitNormal != cloudV.rows() ) {
    if ( meshF.rows() != 0 && cloudN.rows() == 0 ) {
        std::cerr << "[libIGL] An error occurred when computing the normal vectors from the mesh. Aborting..."
                  << std::endl;
        exit (EXIT_FAILURE);
    }

    SampleMatrixType resca = rescalePoints(cloudV);
    cloud = MyPointCloud(resca, cloudN);

    cloud.addNoise(sigma_pos, sigma_normal);
}

void create_tube(MyPointCloud<Scalar> &cloud) {

    int size = 1000;

    SampleMatrixType cloudV(size, 3);
    SampleMatrixType cloudN(size, 3);
    cloudN.setZero();

    Scalar a = 4; // radius in the x-direction
    Scalar b = 4;   // radius in the y-direction
    Scalar length = 20; // length of the tube
    
    for(int i = 0; i < size; ++i) {
        Scalar u = ((Scalar) rand() / (RAND_MAX)) * 2 * M_PI; // azimuthal angle
        Scalar z = ((Scalar) rand() / (RAND_MAX)) * length - length/2; // z-coordinate
        
        // Vertex calculation
        cloudV.row(i) = VectorType(a * cos(u), b * sin(u), z);
        
        // Normal calculation
        cloudN.row(i) = VectorType(cos(u) / a, sin(u) / b, 0);
        
        // Normalize the normal vector
        cloudN.row(i) = cloudN.row(i) / cloudN.row(i).norm();
    }

    cloud = MyPointCloud(cloudV, cloudN);
}

void create_sphere(MyPointCloud<Scalar> &cloud) {

    int size = 1000;

    SampleMatrixType cloudV(size, 3);
    SampleMatrixType cloudN(size, 3);
    cloudN.setZero();

    Scalar r = 5.0; // radius of the sphere

    for(int i = 0; i < size; ++i) {
        Scalar theta = ((Scalar) rand() / (RAND_MAX)) * 2 * M_PI; // azimuthal angle
        Scalar phi = ((Scalar) rand() / (RAND_MAX)) * M_PI; // polar angle
        
        // Vertex calculation
        cloudV.row(i) = VectorType(
            r * sin(phi) * cos(theta), // x
            r * sin(phi) * sin(theta), // y
            r * cos(phi)               // z
        );
        
        // Normal calculation
        cloudN.row(i) = cloudV.row(i) / r; // since the sphere is centered at origin, the normal is the position vector normalized
    }

    cloud = MyPointCloud(cloudV, cloudN);
}

void create_cube (MyPointCloud<Scalar> &cloud, const SampleVectorType &pos, const Scalar &dist = 0.1) {
    int size = 20 * 20 * 20;
    SampleMatrixType cloudV(size, 3);
    SampleMatrixType cloudN(size, 3);
    cloudN.setZero();

    int index = 0;
    for (int i = -10 ; i < 10; i++ ){
        for (int j = -10; j < 10; j++){
            for (int k = -10; k < 10; k ++){
                cloudV.row(index) = pos + dist * VectorType(i, j, k);
                index ++;
            }
        }
    }

    cloud = MyPointCloud(cloudV, cloudN);
}

class CylinderGenerator {

    public:
        CylinderGenerator() = default;

        void generateCylinder(MyPointCloud<Scalar> &cloud, Scalar sigma_pos, Scalar sigma_normal) {

            SampleMatrixType parabolic_verts = SampleMatrixType(x_cylinder * z_cylinder, 3);
            SampleMatrixType parabolic_norms = SampleMatrixType(x_cylinder * z_cylinder, 3);

            Scalar dx = (2.0f) / (x_cylinder - 1);
            Scalar dz = (2.0f) / (z_cylinder - 1);

            for (int i = 0; i < x_cylinder; ++i) {
                for (int j = 0; j < z_cylinder; ++j) {
                    Scalar x = -1 + i * dx;
                    Scalar z = -1 + j * dz;
                    Scalar y = a_cylinder + bx_cylinder * x + bz_cylinder * z + c_a * (cx_cylinder * x + cz_cylinder * z)*(cx_cylinder * x + cz_cylinder * z);

                    parabolic_verts(i * z_cylinder + j, 0) =  x;
                    parabolic_verts(i * z_cylinder + j, 1) =  y;
                    parabolic_verts(i * z_cylinder + j, 2) =  z;

                    parabolic_norms(i * z_cylinder + j, 0) =  bx_cylinder + 2 * c_a * (cx_cylinder  * cx_cylinder * x + cx_cylinder * cz_cylinder * z);
                    parabolic_norms(i * z_cylinder + j, 1) =  -1;
                    parabolic_norms(i * z_cylinder + j, 2) =  bz_cylinder  + 2 * c_a * (cz_cylinder * cz_cylinder * z + cx_cylinder * cz_cylinder * x);
                }
            }
            cloud = MyPointCloud(parabolic_verts, parabolic_norms);
            cloud.addNoise(sigma_pos, sigma_normal);
        }

    public:
        
        // Parameters for the parabolic cylinder public for easy access and modification

        float a_cylinder  = -0.3;

        float bx_cylinder  = 0.2;
        float bz_cylinder  = 0.1;

        float c_a = 0.6;
        float cx_cylinder  = -0.8;
        float cz_cylinder  = 0.2;

        int x_cylinder      = 40;
        int z_cylinder      = 40;


}; // class cylinderGenerator
