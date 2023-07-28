#include <Eigen/Dense>
#include <random>

template <typename VectorType>
Eigen::MatrixXd create_sphere (VectorType center, double radius, int nb_points) {
    Eigen::MatrixXd points(nb_points, 3);
    
    for (int i = 0; i < nb_points; ++i) {
        // Generate random spherical coordinates
        double theta = 2.0 * M_PI * rand() / RAND_MAX; // azimuthal angle [0, 2*pi]
        double phi = acos(1.0 - 2.0 * rand() / RAND_MAX); // polar angle [0, pi]

        // Convert spherical coordinates to Cartesian coordinates
        points(i, 0) = center(0) + radius * sin(phi) * cos(theta);
        points(i, 1) = center(1) + radius * sin(phi) * sin(theta);
        points(i, 2) = center(2) + radius * cos(phi);
    }
    
    return points;
}

Eigen::MatrixXd add_noise (Eigen::MatrixXd cloudVertices, double sigma){
    Eigen::MatrixXd cloudV_noisy = cloudVertices;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, sigma);

    for (int i = 0; i < cloudV_noisy.rows(); ++i) {
        for (int j = 0; j < cloudV_noisy.cols(); ++j) {
            cloudV_noisy(i, j) += dist(gen);
        }
    }
    return cloudV_noisy;
}

template<typename VectorType>
std::vector<VectorType> create_cube (const VectorType& pos) {
    std::vector<VectorType> out;
    double dist = 0.1;
    for (int i = -10 ; i < 10; i++ ){
        for (int j = -10; j < 10; j++){
            for (int k = -10; k < 10; k ++){
                out.push_back(pos + dist * VectorType(i, j, k));
            }
        }
    }
    return out;
}

std::array<double, 3> pointCloudDistance(const Eigen::MatrixXd& cloud1, const Eigen::MatrixXd& cloud2)
{
    assert(cloud1.rows() == cloud2.rows());
    assert(cloud1.cols() == cloud2.cols() && cloud1.cols() == 3);

    int num_points = cloud1.rows();
    std::array<double, 3> metrics;
    double distance_sum = 0.0;
    double distance_mean = 0.0;
    double distance_max = std::numeric_limits<double>::min();
    double distance_mean_err_quadra = 0.0;

    for (int i = 0; i < num_points; ++i)
    {
        // Compute the Euclidean distance between corresponding points
        double distance = (cloud1.row(i) - cloud2.row(i)).norm();
        distance_sum += distance;
        distance_mean_err_quadra += distance * distance;
        distance_max = std::max(distance_max, distance);
    }
    distance_mean = distance_sum / num_points;
    distance_mean_err_quadra = std::sqrt(distance_mean_err_quadra / num_points);

    metrics[0] = distance_sum;
    metrics[1] = distance_mean;
    metrics[2] = distance_mean_err_quadra;

    return metrics;
}