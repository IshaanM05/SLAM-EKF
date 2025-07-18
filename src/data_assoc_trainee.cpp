#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "dv_msgs/msg/cone.hpp"
#include "dv_msgs/msg/indexed_cone.hpp"
#include <numeric>
#include "motion_update_pkg/load_landmarks.hpp"
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

using namespace std;
struct JacobMatrices {
    MatrixXd zp, Hv, Hf, Sf;
};

double normalize_angle(double angle) {
    const double PI = std::acos(-1);  
    angle =
        std::fmod(angle, 2 * PI);  
  
    // Shift angle to be within the range [-PI, PI)
    if (angle >= PI) {
      angle -= 2 * PI;
    } else if (angle < -PI) {
      angle += 2 * PI;
    }
  
    return angle;
}
std::vector<Eigen::Vector3d> known_landmarks = {
    // Blue cones (color code = 0)
    Eigen::Vector3d(-3.7219, -0.7307, 0),
    Eigen::Vector3d(0.2447, 2.1900, 0),
    Eigen::Vector3d(6.1094, 2.7757, 0),
    Eigen::Vector3d(22.6296, 5.5379, 0),
    Eigen::Vector3d(25.4920, 6.7404, 0),
    Eigen::Vector3d(28.1389, 6.2025, 0),
    Eigen::Vector3d(32.5034, 3.3361, 0),
    Eigen::Vector3d(33.8922, 1.4012, 0),
    Eigen::Vector3d(30.5221, 5.0812, 0),
    Eigen::Vector3d(34.4761, -1.5043, 0),
    Eigen::Vector3d(33.9992, -5.0234, 0),
    Eigen::Vector3d(32.9925, -8.0595, 0),
    Eigen::Vector3d(32.0983, -10.3000, 0),
    Eigen::Vector3d(30.1824, -13.5399, 0),
    Eigen::Vector3d(24.1140, -17.0441, 0),
    Eigen::Vector3d(8.9345, 3.0637, 0),
    Eigen::Vector3d(27.2831, -15.5644, 0),
    Eigen::Vector3d(21.2536, -18.8389, 0),
    Eigen::Vector3d(18.0618, -20.4551, 0),
    Eigen::Vector3d(14.4209, -22.2634, 0),
    Eigen::Vector3d(10.5025, -24.3305, 0),
    Eigen::Vector3d(7.2514, -26.4217, 0),
    Eigen::Vector3d(3.6516, -27.4551, 0),
    Eigen::Vector3d(0.7886, -26.9459, 0),
    Eigen::Vector3d(-1.4625, -25.2249, 0),
    Eigen::Vector3d(-3.2427, -23.5760, 0),
    Eigen::Vector3d(12.8688, 3.0125, 0),
    Eigen::Vector3d(-5.1431, -21.3860, 0),
    Eigen::Vector3d(-5.6174, -17.8608, 0),
    Eigen::Vector3d(-5.9382, -15.4551, 0),
    Eigen::Vector3d(-5.5558, -12.8602, 0),
    Eigen::Vector3d(-5.1206, -10.3000, 0),
    Eigen::Vector3d(-4.7841, -7.2575, 0),
    Eigen::Vector3d(-4.8432, -4.0291, 0),
    Eigen::Vector3d(-2.1864, 1.4137, 0),
    Eigen::Vector3d(16.5042, 3.4245, 0),
    Eigen::Vector3d(20.1368, 4.4270, 0),

    // Yellow cones (color code = 1)
    Eigen::Vector3d(0.7816, -2.6920, 1),
    Eigen::Vector3d(20.1479, -0.3734, 1),
    Eigen::Vector3d(23.4312, 0.4990, 1),
    Eigen::Vector3d(25.9655, 1.5014, 1),
    Eigen::Vector3d(27.9652, 0.9833, 1),
    Eigen::Vector3d(29.6054, -1.1797, 1),
    Eigen::Vector3d(6.0900, -1.8555, 1),
    Eigen::Vector3d(29.7063, -4.1923, 1),
    Eigen::Vector3d(28.8760, -6.8209, 1),
    Eigen::Vector3d(28.1060, -8.7973, 1),
    Eigen::Vector3d(26.6765, -10.3000, 1),
    Eigen::Vector3d(22.4195, -13.1474, 1),
    Eigen::Vector3d(25.0732, -11.7312, 1),
    Eigen::Vector3d(20.2628, -14.2408, 1),
    Eigen::Vector3d(17.6516, -15.4551, 1),
    Eigen::Vector3d(14.7877, -16.9672, 1),
    Eigen::Vector3d(11.0203, -18.8633, 1),
    Eigen::Vector3d(8.9831, -1.7968, 1),
    Eigen::Vector3d(7.8188, -20.8555, 1),
    Eigen::Vector3d(5.4296, -22.4125, 1),
    Eigen::Vector3d(3.2361, -23.0081, 1),
    Eigen::Vector3d(0.9662, -21.9718, 1),
    Eigen::Vector3d(-0.9298, -19.2829, 1),
    Eigen::Vector3d(-1.1575, -16.0733, 1),
    Eigen::Vector3d(-1.0043, -12.9300, 1),
    Eigen::Vector3d(-0.6087, -10.3000, 1),
    Eigen::Vector3d(-0.3478, -7.2429, 1),
    Eigen::Vector3d(12.9402, -1.9241, 1),
    Eigen::Vector3d(-0.3455, -4.5119, 1),
    Eigen::Vector3d(16.5151, -1.6032, 1),

    // Big Orange (color code = 2)
    Eigen::Vector3d(3.3869, 2.7000, 2),
    Eigen::Vector3d(3.0007, 2.6890, 2),
    Eigen::Vector3d(3.3785, -1.9068, 2),
    Eigen::Vector3d(3.0133, -1.9065, 2),
};


JacobMatrices compute_jacobians(const std::vector<double>& mu_t, const Vector3d& xf, const Matrix2d& Pf, const Eigen::Matrix2d& Q_cov)
 {

    JacobMatrices ans;

    // Compute the relative position between the robot and the landmark
    double dx = xf(0) - mu_t[0];
    double dy = xf(1) - mu_t[1];
    double d2 = dx * dx + dy * dy;
    double d = std::sqrt(d2); // Euclidean distance (range)

    // Predicted measurement: range (d) and bearing (phi)
    ans.zp = MatrixXd(2, 1);
    ans.zp << d, normalize_angle(atan2(dy, dx) - mu_t[2]);

    // Jacobian of the measurement with respect to the robot state
    ans.Hv = MatrixXd(2, 3);
    ans.Hv << 
        -dx / d, -dy / d, 0.0,
        dy / d2, -dx / d2, -1.0;

    // Jacobian of the measurement with respect to the landmark state
    ans.Hf = MatrixXd(2, 2);
    ans.Hf << 
        dx / d, dy / d,
        -dy / d2, dx / d2;

    // Measurement covariance
    ans.Sf = ans.Hf * Pf * ans.Hf.transpose() + Q_cov;

    return ans;
}

bool performICTest(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov) {

    Eigen::Vector3d xf = known_landmarks[lm_id]; // Landmark position
    Eigen::Matrix2d Pf;
    Pf << 0.001, 0.0, 0.0, 0.001; // Landmark covariance

    JacobMatrices jm = compute_jacobians(mu_t, xf, Pf, Q_cov);

    Eigen::Vector2d dz;
    dz << z(0) - jm.zp(0), normalize_angle(z(1) - jm.zp(1));

    Eigen::Matrix2d Sf_inv = jm.Sf.inverse();
    double mahal_dist = dz.transpose() * Sf_inv * dz;

    return (mahal_dist <= 6.8);
}

double computeObservationLikelihood(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov) {

    Eigen::Vector3d xf=known_landmarks[lm_id]; // Landmark position
    Eigen::Matrix2d Pf;
    Pf <<0.001,0.0,0.0,0.001; // Landmark covariance

    JacobMatrices jm = compute_jacobians(mu_t, xf, Pf, Q_cov);

    Eigen::Vector2d dz;
    dz << z(0) - jm.zp(0), normalize_angle(z(1) - jm.zp(1));

    double den = 2.0 * M_PI * std::sqrt(jm.Sf.determinant());
    if (den <= 0.0 || std::isnan(den)) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Invalid denominator. Returning small value.");
        return 1e-12;
    }

    double exponent = -0.5 * dz.transpose() * jm.Sf.inverse() * dz;
    double likelihood = std::exp(exponent) / den;

    return likelihood;
}

std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov){ 

    std::vector<int> matchedConeIndices;

    std::vector<size_t> obs_indices(conesFromPerception.size());
    std::iota(obs_indices.begin(), obs_indices.end(), 0);
    // std::sort(obs_indices.begin(), obs_indices.end(), [&](size_t a, size_t b) {
    //     return conesFromPerception[a].range < conesFromPerception[b].range;
    // });
    for (size_t obs_idx : obs_indices) {
        const auto &obs = conesFromPerception[obs_idx];
        Eigen::Vector2d z(obs.location.x, obs.location.y);

        int best_landmark = -1;
        double best_likelihood = 0.0;

        // Filter landmarks using IC test and compute likelihood
        for (int lm_id = 0; lm_id < known_landmarks.size(); ++lm_id) {

            if (obs.color != known_landmarks[lm_id](2)) {
                continue;
            }

            if (performICTest(mu_t, lm_id, z, Q_cov)) {
                // IC test passed, now compute likelihood
                double likelihood = computeObservationLikelihood(mu_t, lm_id, z, Q_cov);

                if (likelihood > best_likelihood) {
                    best_likelihood = likelihood;
                    best_landmark = lm_id;
                }
            }   
        }
        // If no suitable landmark found or likelihood is below the threshold, mark as new landmark
        if (best_landmark == -1 || best_likelihood < 0.005) matchedConeIndices.push_back(-1); // New landmark
        else matchedConeIndices.push_back(best_landmark);
    }
    // std::ostringstream oss;
    // oss << "Matched Cone Indices: ";
    // for (const auto& index : matchedConeIndices) {
    //     oss << index << " ";
    // }

    return matchedConeIndices;
}