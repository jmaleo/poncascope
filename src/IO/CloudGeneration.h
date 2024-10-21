#pragma once

#include "../IO/Readers.h"
#include "../IO/Writters.h"
#include "../IO/PointCloudDiff.h"

#include "../definitions.h"

inline std::pair< std::vector<VectorType>, std::vector<std::array<size_t, 3> >> generatePlane(const SampleMatrixType& vertices, VectorType & normal, VectorType& d1, VectorType& d2){
    std::vector<std::array<size_t, 3>> faces (3);
    faces[0] = {0, 0, 0};
    faces[1] = {1, 2, 3};
    faces[2] = {1, 3, 4};
    
    VectorType origin = vertices.row(0);

    // compute the bounding box from vertices
    VectorType min = vertices.colwise().minCoeff();
    VectorType max = vertices.colwise().maxCoeff();
    Scalar min_dist = (max - min).norm() / 4.0;

    std::vector<VectorType> farestPoints;
    farestPoints.push_back(origin);
    farestPoints.push_back(origin + min_dist * d1);
    farestPoints.push_back(origin + min_dist * d2);
    farestPoints.push_back(origin - min_dist * d1);
    farestPoints.push_back(origin - min_dist * d2);
    

    return std::make_pair(farestPoints, faces);
}

inline SampleMatrixType applyCentering(const SampleMatrixType &cloudV) {
    auto cloud_out = SampleMatrixType(cloudV.rows(), 3);

    VectorType min = {std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max(),
                      std::numeric_limits<Scalar>::max()};
    VectorType max = {std::numeric_limits<Scalar>::min(), std::numeric_limits<Scalar>::min(),
                      std::numeric_limits<Scalar>::min()};

    for (int i = 0; i < cloudV.rows(); i++) {
        VectorType p = cloudV.row(i);
        for (auto j = 0u; j < 3; ++j) {
            if (p[j] < min[j])
                min[j] = p[j];
            if (p[j] > max[j])
                max[j] = p[j];
        }
    }

    VectorType barycenter = (min + max) / 2.0;
#pragma omp parallel for
    for (int i = 0; i < cloud_out.rows(); ++i) {
        const VectorType& current = cloudV.row(i);
        cloud_out.row(i) = current - barycenter;
    }

    std::cout << "Cloud is centered. Barycenter: " << barycenter.transpose() << std::endl;

    return cloud_out;
}

inline SampleMatrixType rescalePoints(SampleMatrixType &vertices) {
    VectorType baryCenter = VectorType::Zero();
    // Compute barycenter
    for (int i = 0; i < vertices.rows(); i++) {
        VectorType vertice = vertices.row(i);
        baryCenter += vertice;
    }
    baryCenter /= static_cast<Scalar>(vertices.rows());

    // Compute max distance
    VectorType maxDist({0.0, 0.0, 0.0});
    for (int i = 0; i < vertices.rows(); i++) {
        VectorType vertice = vertices.row(i);
        VectorType current = vertice - baryCenter;
        maxDist = maxDist.cwiseMax(current.cwiseAbs());
    }
    const Scalar maxDistNorm = maxDist.maxCoeff();

    // Rescale
    auto rescaledVertices = SampleMatrixType(vertices.rows(), 3);
    for (int i = 0; i < vertices.rows(); ++i) {
        VectorType current = vertices.row(i);
        rescaledVertices.row(i) = (current - baryCenter) / maxDistNorm;
    }
    return rescaledVertices;
}

inline void savePTSObject(PointCloudDiff<Scalar> &cloud, std::string filename) {

    // If the filename exist, add a number to the filename (without the extension)
    std::string filename_no_ext = filename.substr(0, filename.find_last_of("."));
    std::string ext = filename.substr(filename.find_last_of("."));
    int i = 0;
    while (std::ifstream(filename).good()) {
        filename = filename_no_ext + std::to_string(i) + ext;
        i++;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Could not open the file: " << filename << std::endl;
        return;
    }

    SampleMatrixType cloudV = cloud.points;
    SampleMatrixType cloudN = cloud.normals;

    file << "# x y z nx ny nz" << std::endl;
    for (int i = 0; i < cloudV.rows(); ++i) {
        file << cloudV.row(i)[0] << " " << cloudV.row(i)[1] << " " << cloudV.row(i)[2] << " " << cloudN.row(i)[0] << " " << cloudN.row(i)[1]
             << " " << cloudN.row(i)[2] << std::endl;
    }
    file.close();
}

inline void loadObject(PointCloudDiff<Scalar> &cloud, std::string filename, const Scalar sigma_pos, const Scalar sigma_normal) {
    IO::loadPointCloud<PointCloudDiff<Scalar>,Scalar>(cloud, filename);

    cloud.addNoisePosition(sigma_pos);
    cloud.addNoiseNormal(sigma_normal);
}

inline void create_tube(PointCloudDiff<Scalar> &cloud) {

    int size = 1000;

    SampleMatrixType cloudV(size, 3);
    SampleMatrixType cloudN(size, 3);
    for (int i = 0; i < size; ++i) {
        cloudN.row(i) = VectorType(0, 0, 0);
    }

    constexpr Scalar a = 4.0; // radius in the x-direction
    const Scalar b = 4.0; // radius in the y-direction
    const Scalar length = 20.0; // length of the tube

    for (int i = 0; i < size; ++i) {
        const Scalar u = (static_cast<Scalar>( rand() ) / (RAND_MAX)) * 2 * M_PI; // azimuthal angle
        const Scalar z = (static_cast<Scalar>( rand() ) / (RAND_MAX)) * length - length / 2; // z-coordinate

        // Vertex calculation
        cloudV.row(i) = VectorType(a * cos(u), b * sin(u), z);

        // Normal calculation
        cloudN.row(i) = VectorType(cos(u) / a, sin(u) / b, 0);

        // Normalize the normal vector
        cloudN.row(i)= cloudN.row(i) / cloudN.row(i).norm();
    }

    cloud = PointCloudDiff<Scalar>("MyTube", cloudV, cloudN);
}

inline void create_sphere(PointCloudDiff<Scalar> &cloud) {

    int size = 1000;

    SampleMatrixType cloudV(size, 3);
    SampleMatrixType cloudN(size, 3);
    for (int i = 0; i < size; ++i) {
        cloudN.row(i) = VectorType(0, 0, 0);
    }

    Scalar r = 5.0; // radius of the sphere

    for (int i = 0; i < size; ++i) {
        const Scalar theta = (static_cast<Scalar>(rand()) / (RAND_MAX)) * 2 * M_PI; // azimuthal angle
        const Scalar phi = (static_cast<Scalar>(rand()) / (RAND_MAX)) * M_PI; // polar angle

        // Vertex calculation
        cloudV.row(i) = VectorType(r * sin(phi) * cos(theta), // x
                                   r * sin(phi) * sin(theta), // y
                                   r * cos(phi) // z
        );

        // Normal calculation
        cloudN.row(i) = cloudV.row(i) /
                        r; // since the sphere is centered at origin, the normal is the position vector normalized
    }

    cloud = PointCloudDiff<Scalar>("MySphere", cloudV, cloudN);
}

void create_cube(PointCloudDiff<Scalar> &cloud, const VectorType &pos, const Scalar &dist = 0.1) {
    int size = 20 * 20 * 20;
    SampleMatrixType cloudV(size + 1, 3);
    SampleMatrixType cloudN(size + 1, 3);
    for (int i = 0; i < size + 1; ++i) {
        cloudN.row(i) = VectorType(0, 0, 0);
    }

    cloudV.row(0) = pos;
    cloudN.row(0) = VectorType(0, 0, 1);

    int index = 1;
    for (int i = -10; i < 10; i++) {
        for (int j = -10; j < 10; j++) {
            for (int k = -10; k < 10; k++) {
                cloudV.row(index) = pos + dist * VectorType(i, j, k);
                index++;
            }
        }
    }

    cloud = PointCloudDiff<Scalar>("MyCube", cloudV, cloudN);
}

template<typename Aabb>
void createVoxels(const std::vector<Aabb> &boundingBoxes, std::vector<VectorType> &vertices,
                  std::vector<std::array<int, 2>> &indices) {
    vertices = std::vector<VectorType>();
    indices = std::vector<std::array<int, 2>>();
    for (int i = 0; i < boundingBoxes.size(); i++) {
        Aabb bbox = boundingBoxes[i];
        VectorType min = bbox.min();
        VectorType max = bbox.max();
        vertices.push_back(VectorType(min[0], min[1], min[2]));
        vertices.push_back(VectorType(max[0], min[1], min[2]));
        vertices.push_back(VectorType(max[0], max[1], min[2]));
        vertices.push_back(VectorType(min[0], max[1], min[2]));
        vertices.push_back(VectorType(min[0], min[1], max[2]));
        vertices.push_back(VectorType(max[0], min[1], max[2]));
        vertices.push_back(VectorType(max[0], max[1], max[2]));
        vertices.push_back(VectorType(min[0], max[1], max[2]));
        indices.push_back({0 + 8 * i, 1 + 8 * i});
        indices.push_back({1 + 8 * i, 2 + 8 * i});
        indices.push_back({2 + 8 * i, 3 + 8 * i});
        indices.push_back({3 + 8 * i, 0 + 8 * i});
        indices.push_back({4 + 8 * i, 5 + 8 * i});
        indices.push_back({5 + 8 * i, 6 + 8 * i});
        indices.push_back({6 + 8 * i, 7 + 8 * i});
        indices.push_back({7 + 8 * i, 4 + 8 * i});
        indices.push_back({0 + 8 * i, 4 + 8 * i});
        indices.push_back({0 + 8 * i, 4 + 8 * i});
        indices.push_back({2 + 8 * i, 6 + 8 * i});
        indices.push_back({3 + 8 * i, 7 + 8 * i});
    }
}

inline std::pair<std::vector<VectorType>, std::vector<std::array<size_t, 4>>> create_frame(const VectorType &lowerBound,
                                                                             const VectorType &upperBound,
                                                                             const int nbSteps, const int axis,
                                                                             const Scalar slice = 0.0) {
    int sliceid = static_cast<int>(std::floor(slice * nbSteps));

    const auto dim1 = (axis + 1) % 3;
    const auto dim2 = (axis + 2) % 3;

    const Scalar du = (upperBound[dim1] - lowerBound[dim1]) / static_cast<Scalar>(nbSteps);
    const Scalar dv = (upperBound[dim2] - lowerBound[dim2]) / static_cast<Scalar>(nbSteps);
    const Scalar dw = (upperBound[axis] - lowerBound[axis]) / static_cast<Scalar>(nbSteps);

    const Scalar u = lowerBound[dim1];
    const Scalar v = lowerBound[dim2];
    const Scalar w = lowerBound[axis] + sliceid * dw;

    VectorType p;
    VectorType vu, vv;
    switch (axis) {
        case 0:
            p = VectorType(w, u, v);
            vu = VectorType(0, du, 0);
            vv = VectorType(0, 0, dv);
            break;
        case 1:
            p = VectorType(u, w, v);
            vu = VectorType(du, 0, 0);
            vv = VectorType(0, 0, dv);
            break;
        case 2:
            p = VectorType(u, v, w);
            vu = VectorType(du, 0, 0);
            vv = VectorType(0, dv, 0);
            break;
        default:
            break;
    }

    //   std::vector<VectorType> vertices(nbSteps*nbSteps);

    std::vector<VectorType> vertices(nbSteps * nbSteps);

    auto values = SampleVectorType(nbSteps * nbSteps);

    std::vector<std::array<size_t, 4>> faces;
    faces.reserve(nbSteps * nbSteps);
    std::array<size_t, 4> face{};

    // Regular grid construction
    for (size_t id = 0; id < nbSteps * nbSteps; ++id) {
        auto i = id % nbSteps;
        auto j = id / nbSteps;
        p = lowerBound + i * vu + j * vv;
        p[axis] += sliceid * dw;
        vertices[id] = p;
        face = {id, id + 1, id + 1 + nbSteps, id + nbSteps};
        if (((i + 1) < nbSteps) && ((j + 1) < nbSteps))
            faces.push_back(face);
    }
    return std::make_pair(vertices, faces);
}

class CylinderGenerator {

public:
    CylinderGenerator() = default;

    void generateCylinder(PointCloudDiff<Scalar> &cloud, const Scalar sigma_pos, const Scalar sigma_normal) const {

        auto parabolic_verts = SampleMatrixType(x_cylinder * z_cylinder, 3);
        auto parabolic_norms = SampleMatrixType(x_cylinder * z_cylinder, 3);

        const Scalar dx = (2.0f) / (x_cylinder - 1.0);
        const Scalar dz = (2.0f) / (z_cylinder - 1.0);

        for (int i = 0; i < x_cylinder; ++i) {
            for (int j = 0; j < z_cylinder; ++j) {
                Scalar x = -1 + i * dx;
                Scalar z = -1 + j * dz;
                Scalar y = a_cylinder + bx_cylinder * x + bz_cylinder * z +
                           c_a * (cx_cylinder * x + cz_cylinder * z) * (cx_cylinder * x + cz_cylinder * z);

                parabolic_verts.row(i * z_cylinder + j) = VectorType(x, y, z);
                parabolic_norms.row(i * z_cylinder + j) = VectorType(
                        bx_cylinder + 2 * c_a * (cx_cylinder * cx_cylinder * x + cx_cylinder * cz_cylinder * z),
                        -1,
                        bz_cylinder + 2 * c_a * (cz_cylinder * cz_cylinder * z + cx_cylinder * cz_cylinder * x));
            }
        }
        cloud = PointCloudDiff<Scalar>("MyParabolic", parabolic_verts, parabolic_norms);
        cloud.addNoisePosition(sigma_pos);
        cloud.addNoiseNormal(sigma_normal);
    }

public:
    // Parameters for the parabolic cylinder public for easy access and modification

    float a_cylinder = -0.3;

    float bx_cylinder = 0.2;
    float bz_cylinder = 0.1;

    float c_a = 0.6;
    float cx_cylinder = -0.8;
    float cz_cylinder = 0.2;

    int x_cylinder = 40;
    int z_cylinder = 40;


}; // class cylinderGenerator

class SinusGenerator {

    using MatrixType = Eigen::Matrix<Scalar, 3, 3>;
    using VectorType = Eigen::Matrix<Scalar, 3, 1>;
    // using SampleMatrixType = Eigen::Matrix<Scalar, Eigen::Dynamic, 3>;

public:
    SinusGenerator() = default;

    [[nodiscard]] std::pair<Scalar, VectorType> computeModulatingSinus(Scalar x) const {
        Scalar y_mod = h_sinus2 * sin(f_sinus2 * x + p_sinus2);
        VectorType norm_mod = VectorType(h_sinus2 * f_sinus2 * cos(f_sinus2 * x + p_sinus2), -1, 0);
        return std::make_pair(y_mod, norm_mod);
    }

    [[nodiscard]] std::pair<Scalar, VectorType> computeBaseSinus(Scalar x) const {
        Scalar y = h_sinus * sin(f_sinus * x + p_sinus);
        auto norm = VectorType(h_sinus * f_sinus * cos(f_sinus * x + p_sinus), -1, 0);
        return std::make_pair(y, norm);
    }

    void generateSinus(PointCloudDiff<Scalar> &cloud, Scalar sigma_pos, Scalar sigma_normal) const {
        auto sinus_verts = SampleMatrixType(x_sinus * z_sinus, 3);
        auto sinus_norms = SampleMatrixType(x_sinus * z_sinus, 3);

        const Scalar dx = (x_max - x_min) / (x_sinus - 1);
        const Scalar dz = (z_max - z_min) / (z_sinus - 1);

        for (int j = 0; j < z_sinus; ++j) {
            for (int i = 0; i < x_sinus; ++i) {
                Scalar x = x_min + i * dx;
                Scalar z = z_min + j * dz;

                auto [y_base, norm_base] = computeBaseSinus(x);
                auto [y_mod, norm_mod] = computeModulatingSinus(x);

                Scalar y = y_base + y_mod;
                const VectorType norm = (norm_base + norm_mod).normalized();

                sinus_verts.row(i * z_sinus + j) = VectorType(x, y, z);
                sinus_norms.row(i * z_sinus + j) = norm;
            }
        }

        cloud = PointCloudDiff<Scalar>("MySine", sinus_verts, sinus_norms);
        cloud.addNoisePosition(sigma_pos);
        cloud.addNoiseNormal(sigma_normal);
    }

    static void saveSinus(PointCloudDiff<Scalar> &cloud) { savePTSObject(cloud, "MySin.pts"); }

public:
    // Parameters for the base sinus
    float h_sinus = 0.3; // amplitude
    float f_sinus = 0.5; // frequency on x
    float p_sinus = 0.0; // phase shift

    // Parameters for the modulating sinus
    float h_sinus2 = 0.1; // amplitude of modulation
    float f_sinus2 = 0.3; // frequency of modulation on z
    float p_sinus2 = 0.0; // phase shift of modulation

    // Sampling parameters
    int x_sinus = 60;
    int z_sinus = 60;
    Scalar x_min = -2.0;
    Scalar x_max = 2.0;
    Scalar z_min = -2.0;
    Scalar z_max = 2.0;

    bool automatic_sinus = false;
};
