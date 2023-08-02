#pragma once

#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/per_vertex_normals.h>
#include "MyPointCloud.h"

void loadPTSObject (MyPointCloud &cloud, std::string filename, float sigma_pos, float sigma_normal){

    Eigen::MatrixXd cloudV, cloudN;

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
            indices[header[i]] = i-1;
        }
    }

    // Read the rest of the file
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> values(header.size());
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

    cloud = MyPointCloud(cloudV, cloudN);
    cloud.addNoise(sigma_pos, sigma_normal);
}


void loadObject (MyPointCloud &cloud, std::string filename, float sigma_pos, float sigma_normal) {

    Eigen::MatrixXi meshF;
    Eigen::MatrixXd cloudV, cloudN;

    // If filename end with .ply, load the file as a PLY file
    if ( filename.substr(filename.find_last_of(".") + 1) == "ply" ) {
        igl::readPLY(filename, cloudV, meshF);
    }
    else if ( filename.substr(filename.find_last_of(".") + 1) == "obj" ) {
        igl::readOBJ(filename, cloudV, meshF);
    }
    else if (filename.substr(filename.find_last_of(".") + 1) == "pts") {
        loadPTSObject(cloud, filename, sigma_pos, sigma_normal);
        return;
    }
    igl::per_vertex_normals(cloudV, meshF, cloudN);

    // Check if normals have been properly loaded
    int nbUnitNormal = cloudN.rowwise().squaredNorm().sum();
    if ( nbUnitNormal != cloudV.rows() ) {
        std::cerr << "[libIGL] An error occurred when computing the normal vectors from the mesh. Aborting..."
                  << std::endl;
        exit (EXIT_FAILURE);
    }

    cloud = MyPointCloud(cloudV, cloudN);

    cloud.addNoise(sigma_pos, sigma_normal);
}

void create_cube (MyPointCloud &cloud, const Eigen::VectorXd &pos, const double &dist = 0.1) {
    int size = 20 * 20 * 20;
    Eigen::MatrixXd cloudV(size, 3);
    Eigen::MatrixXd cloudN(size, 3);
    cloudN.setZero();

    int index = 0;
    for (int i = -10 ; i < 10; i++ ){
        for (int j = -10; j < 10; j++){
            for (int k = -10; k < 10; k ++){
                cloudV.row(index) = pos + dist * Eigen::Vector3d(i, j, k);
                index ++;
            }
        }
    }

    cloud = MyPointCloud(cloudV, cloudN);
}

class CylinderGenerator {

    public:
        CylinderGenerator() = default;

        void generateCylinder(MyPointCloud &cloud, float sigma_pos, float sigma_normal) {

            Eigen::MatrixXd parabolic_verts = Eigen::MatrixXd(x_cylinder * z_cylinder, 3);
            Eigen::MatrixXd parabolic_norms = Eigen::MatrixXd(x_cylinder * z_cylinder, 3);

            float dx = (2.0f) / (x_cylinder - 1);
            float dz = (2.0f) / (z_cylinder - 1);

            for (int i = 0; i < x_cylinder; ++i) {
                for (int j = 0; j < z_cylinder; ++j) {
                    float x = -1 + i * dx;
                    float z = -1 + j * dz;
                    float y = a_cylinder + bx_cylinder * x + bz_cylinder * z + c_a * (cx_cylinder * x + cz_cylinder * z)*(cx_cylinder * x + cz_cylinder * z);

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