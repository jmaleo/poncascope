#pragma once

#include <igl/read_triangle_mesh.h>
#include <igl/readOBJ.h>
#include <igl/readPLY.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>

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
    std::vector<VectorType> cloudVVec, cloudNVec;

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

        cloudVVec.push_back( VectorType( values[ indices[ "x" ] ], values[ indices[ "y" ] ], values[ indices[ "z" ] ] ) );

        cloudNVec.push_back( VectorType( values[ indices[ "nx" ] ], values[ indices[ "ny" ] ], values[ indices[ "nz" ] ] ) );
    }

    file.close();
    // Cast cloudVVec and cloudNVec to Eigen::Matrix
    cloudV = SampleMatrixType( cloudVVec.size(), 3 );
    cloudN = SampleMatrixType( cloudNVec.size(), 3 );

    #pragma omp parallel for
    for ( int i = 0; i < cloudVVec.size(); ++i ) {
        cloudV.row( i ) = cloudVVec[ i ];
        cloudN.row( i ) = cloudNVec[ i ];
    }

    cloud = MyPointCloud<Scalar>(cloudV, cloudN);
    cloud.addNoise(sigma_pos, sigma_normal);
}

void loadXYZObject (MyPointCloud<Scalar> &cloud, std::string &filename, Scalar sigma_pos, Scalar sigma_normal){

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
    std::string word;

    // Find the indices of the x, y, z, nx, ny, nz in the header
    std::map<std::string, int> indices;

    indices["x"] = 0;
    indices["y"] = 1;
    indices["z"] = 2;

    // Read the rest of the file
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<Scalar> values(3);
        for (int i = 0; i < 3; ++i) {
            iss >> values[i];
        }

        // Add the read values to the matrices
        cloudV.conservativeResize(cloudV.rows() + 1, 3);
        cloudV.row(cloudV.rows() - 1) << values[indices["x"]], values[indices["y"]], values[indices["z"]];

        cloudN.conservativeResize(cloudN.rows() + 1, 3);
        cloudN.row(cloudN.rows() - 1) << 0, 0, 0;
    }

    file.close();

    std::cout << "cloudV: " << cloudV.rows() << " " << cloudV.cols() << std::endl;

    cloud = MyPointCloud<Scalar>(cloudV, cloudN);
    cloud.addNoise(sigma_pos, sigma_normal);
}

void loadObject (MyPointCloud<Scalar> &cloud, std::string filename, Scalar sigma_pos, Scalar sigma_normal) {

    Eigen::MatrixXi meshF;
    SampleMatrixType cloudV, cloudN;

    if (filename.substr(filename.find_last_of(".") + 1) == "pts") {
        loadPTSObject(cloud, filename, sigma_pos, sigma_normal);
        return;
    }
    if (filename.substr(filename.find_last_of(".") + 1) == "xyz") {
        loadXYZObject(cloud, filename, sigma_pos, sigma_normal);
        return;
    }
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

    // SampleMatrixType resca = rescalePoints(cloudV);
    cloud = MyPointCloud<Scalar>(cloudV, cloudN);

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

    cloud = MyPointCloud<Scalar>(cloudV, cloudN);
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

    cloud = MyPointCloud<Scalar>(cloudV, cloudN);
}

void create_cube (MyPointCloud<Scalar> &cloud, const SampleVectorType &pos, const Scalar &dist = 0.1) {
    int size = 20 * 20 * 20;
    SampleMatrixType cloudV(size+1, 3);
    SampleMatrixType cloudN(size+1, 3);
    cloudN.setZero();

    cloudV.row(0) = pos;
    cloudN.row(0) = VectorType(0, 0, 1);

    int index = 1;
    for (int i = -10 ; i < 10; i++ ){
        for (int j = -10; j < 10; j++){
            for (int k = -10; k < 10; k ++){
                cloudV.row(index) = pos + dist * VectorType(i, j, k);
                index ++;
            }
        }
    }

    cloud = MyPointCloud<Scalar>(cloudV, cloudN);
}

std::pair<SampleMatrixType, std::vector<std::array<size_t,4>>> create_frame(const VectorType &lowerBound, const VectorType &upperBound, size_t nbSteps, size_t axis, Scalar slice=0.0){
    size_t sliceid = static_cast<size_t>(std::floor(slice*nbSteps));
  
    auto dim1 = (axis+1)%3;
    auto dim2 = (axis+2)%3;
    
    Scalar du = (upperBound[dim1]-lowerBound[dim1])/Scalar(nbSteps);
    Scalar dv = (upperBound[dim2]-lowerBound[dim2])/Scalar(nbSteps);
    Scalar dw = (upperBound[axis]-lowerBound[axis])/Scalar(nbSteps);
    
    Scalar u = lowerBound[dim1];
    Scalar v = lowerBound[dim2];
    Scalar w = lowerBound[axis] + sliceid*dw;
    
    VectorType p;
    VectorType vu,vv;
    switch (axis) {
        case 0: p=VectorType(w,u,v); vu=VectorType(0,du,0); vv=VectorType(0,0,dv);break;
        case 1: p=VectorType(u,w,v); vu=VectorType(du,0,0); vv=VectorType(0,0,dv);break;
        case 2: p=VectorType(u,v,w); vu=VectorType(du,0,0); vv=VectorType(0,dv,0);break;
    }
    
    //   std::vector<VectorType> vertices(nbSteps*nbSteps);
    
    SampleMatrixType vertices (nbSteps*nbSteps,3);

    SampleVectorType values = SampleVectorType::Zero(nbSteps*nbSteps);

    std::vector<std::array<size_t,4>> faces;
    faces.reserve(nbSteps*nbSteps);
    std::array<size_t,4> face;
    
    //Regular grid construction
    for(size_t id=0; id < nbSteps*nbSteps; ++id)
    {
        auto i = id % nbSteps;
        auto j = id / nbSteps;
        p = lowerBound + i*vu + j*vv;
        p[axis] += sliceid*dw;
        vertices.row(id) = p;
        face = { id, id+1, id+1+nbSteps, id+nbSteps };
        if (((i+1) < nbSteps) && ((j+1)<nbSteps))
        faces.push_back(face);
    }
    return std::make_pair(vertices, faces);
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
            cloud = MyPointCloud<Scalar>(parabolic_verts, parabolic_norms);
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

class SinusGenerator {

    using MatrixType = Eigen::Matrix<Scalar, 3, 3>;
    // using VectorType = Eigen::Matrix<Scalar, 3, 1>;

    public:
        SinusGenerator() = default;

        MatrixType computeLocalFrame(const VectorType &n) {
            MatrixType B;
            VectorType u = n.cross(VectorType(1, 0, 0));
            if (u.norm() < 1e-6)
                u = n.cross(VectorType(0, 1, 0));
            u.normalize();
            VectorType v = n.cross(u);
            v.normalize();
            B << u, v, n;
            return B;
        }

        VectorType worldToLocalFrame(const MatrixType& B, const VectorType& origin, const VectorType& _q, bool ignoreTranslation) const {
            if (ignoreTranslation)
                return B.transpose() * _q;
            else
                return B.transpose() * (_q - origin);
        }

        VectorType localFrameToWorld(const MatrixType& B, const VectorType& origin, const VectorType& _lq, bool ignoreTranslation) const {
            if (ignoreTranslation)
                return B * _lq;
            else
                return B * _lq + origin;
        }

        std::pair<Scalar, VectorType> computeSecondSinus(Scalar pos_x, Scalar pos_z) {
            Scalar y = h_sinus2 * sin( f_sinus2 * pos_x + p_sinus2 );

            VectorType norm = VectorType(h_sinus2 * f_sinus2 * cos(f_sinus2 * pos_x + p_sinus2), -1, 0);
            return std::make_pair(y, norm);
        }

        std::pair<Scalar, VectorType> computeFirstSinus(Scalar pos_x, Scalar pos_z) {
            Scalar y = h_sinus * sin(f_sinus * pos_x + p_sinus);

            VectorType norm = VectorType(h_sinus * f_sinus * cos(f_sinus * pos_x + p_sinus), -1, 0);
            return std::make_pair(y, norm);
        }

        void generateSinus(MyPointCloud<Scalar> &cloud, Scalar sigma_pos, Scalar sigma_normal) {
            SampleMatrixType sinus_verts = SampleMatrixType(x_sinus * z_sinus, 3);
            SampleMatrixType sinus_norms = SampleMatrixType(x_sinus * z_sinus, 3);

            Scalar dx = (4.0f) / (x_sinus - 1);
            Scalar dz = (2.0f) / (z_sinus - 1);
            
            for (int i = 0; i < x_sinus; ++i) {
                for (int j = 0; j < z_sinus; ++j) {
                    Scalar x = -1 + i * dx;
                    Scalar z = -1 + j * dz;
                    std::pair<Scalar, VectorType> vert_norm = computeFirstSinus(x, z);
                    std::pair<Scalar, VectorType> vert_norm_sin_2 = computeSecondSinus(x, z);

                    Scalar sin_1 = vert_norm.first;
                    Scalar sin_2 = vert_norm_sin_2.first;

                    VectorType norm_sin1 = vert_norm.second;
                    VectorType norm_sin2 = vert_norm_sin_2.second;

                    Scalar sin = sin_1 + sin_2;
                    VectorType norm = vert_norm_sin_2.second + vert_norm.second ;
                    norm.normalize();

                    sinus_verts.row(i * z_sinus + j) = VectorType (x, sin, z);
                    sinus_norms.row(i * z_sinus + j) = norm; 
                }
            }
            cloud = MyPointCloud<Scalar>(sinus_verts, sinus_norms);
            cloud.addNoise(sigma_pos, sigma_normal);
        }

    public:
        
        // Parameters for the sinus public for easy access and modification

        float h_sinus  = 0.3; // amplitude
        float f_sinus  = 0.5; // frequency on x
        float p_sinus  = 0.5; // phase shift

        float h_sinus2  = 0.3; // amplitude
        float f_sinus2  = 0.5; // frequency on x
        float p_sinus2  = 0.5; // phase shift

        int x_sinus      = 60;
        int z_sinus      = 60;

        bool sinus2onX = false ;

        bool automatic_sinus = false;

}; // class sinusGenerator

class Mesh_test {

    public : 

        Mesh_test(std::string &filename){
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
        }

        void rotateTranslate(std::string &filenameRotation){
            std::ifstream file(filenameRotation);
            if (!file.is_open()) {
                std::cerr << "The translation / rotation file : " << filenameRotation << " doen't exist."<< std::endl;
                return;
            }

            // Format :  rotation x y z
            //           translation x y z 
            
            VectorType rotation, translation;

            std::string line;
            for (int line_idx = 0; line_idx < 2; ++line_idx){
                std::getline(file, line);
                std::istringstream iss(line);
                std::vector<Scalar> values(3);
                for (int i = 0; i < 3; ++i) {
                    iss >> values[i];
                }
                if (line_idx == 0)
                    rotation = VectorType(values[0], values[1], values[2]);
                else
                    translation = VectorType(values[0], values[1], values[2]);
            }

            // Rotation (degree) to radian
            rotation = rotation * M_PI / 180.0;
            
            Eigen::Matrix<Scalar, 3, 3> rot, rotX, rotY, rotZ;
            rotX << 1, 0, 0, 0, cos(rotation[0]), -sin(rotation[0]), 0, sin(rotation[0]), cos(rotation[0]);
            rotY << cos(rotation[1]), 0, sin(rotation[1]), 0, 1, 0, -sin(rotation[1]), 0, cos(rotation[1]);
            rotZ << cos(rotation[2]), -sin(rotation[2]), 0, sin(rotation[2]),  cos(rotation[2]), 0, 0, 0, 1;
            rot = rotX.transpose() * rotY.transpose() * rotZ.transpose();

            for (int i = 0; i < cloudV.rows(); ++i){
                cloudV.row(i) = rot.transpose() * cloudV.row(i).transpose() + translation;
                cloudN.row(i) = rot.transpose() * cloudN.row(i).transpose();
            }
        }

        const Eigen::MatrixXi getIndices() {
            return meshF;
        }

        const SampleMatrixType getVertices () {
            return cloudV;
        }

        const SampleMatrixType getNormals() {
            return cloudN;
        }

        private :

        Eigen::MatrixXi meshF;
        SampleMatrixType cloudV, cloudN;
}; // class Mesh_test

/***
 * Cloud is already loaded, we just need to open the file, 
 * load the normals from the mesh and add the corresponding normal to each point of the Cloud.
 * The normals are computed using per face normals.
 */
void normalsFromMesh (MyPointCloud<Scalar> &cloud, Mesh_test& originalMesh, Scalar sigma_pos, Scalar sigma_normal){

    const Eigen::MatrixXi meshF = originalMesh.getIndices();
    const SampleMatrixType cloudV = originalMesh.getVertices();
    const SampleMatrixType cloudN = originalMesh.getNormals();

    SampleMatrixType cloudN_new = cloud.getNormals();

    VectorType point_i;
    VectorType normal_i = VectorType::Zero();
    for (int i = 0; i < cloud.getSize(); ++i){
        point_i = cloud.getVertices().row(i);
        // Check the nearest point / face to the point_i in the meshF
        double minDist = std::numeric_limits<double>::max();
        for (int j = 0; j < meshF.rows(); ++j){
            VectorType face = cloudV.row(meshF(j, 0)) + cloudV.row(meshF(j, 1)) + cloudV.row(meshF(j, 2));
            face /= 3.0;
            double dist = (point_i - face).norm();
            if (dist < minDist){
                minDist = dist;
                normal_i = cloudN.row(meshF(j, 0)) + cloudN.row(meshF(j, 1)) + cloudN.row(meshF(j, 2));
                normal_i.normalize();
            }
        }
        cloudN_new.row(i) = normal_i;
    }

    cloud.setNormals(cloudN_new);

    // SampleMatrixType resca = rescalePoints(cloudV);
    // cloud = MyPointCloud<Scalar>(cloudV, cloudN);

    // cloud.addNoise(sigma_pos, sigma_normal);
    
}
