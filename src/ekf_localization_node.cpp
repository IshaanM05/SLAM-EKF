// ekf_localization_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "dv_msgs/msg/indexed_track.hpp"
#include "dv_msgs/msg/single_range_bearing_observation.hpp"
#include "dv_msgs/msg/vcu2_ai_speeds.hpp"

#include "motion_update_pkg/load_landmarks.hpp"

extern std::vector<Eigen::Vector3d> known_landmarks;
extern std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov);

class EKFLocalizationNode : public rclcpp::Node {
public:
    EKFLocalizationNode() : Node("ekf_localization_node") {
        this->declare_parameter("use_sim_time", true);
        initializeEKF();
        setupROS();
        RCLCPP_INFO(this->get_logger(), "✅ EKF Localization Node initialized");
    }

private:
    Eigen::VectorXd mu_;
    Eigen::MatrixXd Sigma_;
    double sigma_vx_ = 0.1, sigma_vy_ = 0.1, sigma_phi_ = 0.1;
    double sigma_r_ = 0.1, sigma_theta_ = 0.05;
    rclcpp::Time last_time_;
    bool first_measurement_ = true;

    double vx_ = 0.0, vy_ = 0.0, phi_dot_ = 0.0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<dv_msgs::msg::VCU2AISpeeds>::SharedPtr wheel_sub_;
    rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr perception_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void initializeEKF() {
        mu_ = Eigen::VectorXd::Zero(3);
        Sigma_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
        last_time_ = this->now();
    }

    void setupROS() {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf/pose", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&EKFLocalizationNode::imuCallback, this, std::placeholders::_1));

        wheel_sub_ = this->create_subscription<dv_msgs::msg::VCU2AISpeeds>(
            "/ros_can/wheel_speeds", 10, std::bind(&EKFLocalizationNode::wheelCallback, this, std::placeholders::_1));

        perception_sub_ = this->create_subscription<dv_msgs::msg::IndexedTrack>(
            "/perception/cones", 10, std::bind(&EKFLocalizationNode::perceptionCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        phi_dot_ = msg->angular_velocity.z;
        if (!first_measurement_) {
            motionUpdate(msg->header.stamp);
        }
    }

    void wheelCallback(const dv_msgs::msg::VCU2AISpeeds::SharedPtr msg) {
        vx_ = (msg->fl_wheel_speed + msg->fr_wheel_speed) / 2.0;
        vy_ = 0.0;
        auto now = this->get_clock()->now();
        if (!first_measurement_) {
            motionUpdate(now);
        }
    }

    void perceptionCallback(const dv_msgs::msg::IndexedTrack::SharedPtr msg) {
        if (msg->cones.empty()) return;

        Eigen::Matrix2d Q;
        Q << sigma_r_ * sigma_r_, 0,
             0, sigma_theta_ * sigma_theta_;

        std::vector<double> mu_vec = {mu_(0), mu_(1), mu_(2)};
        auto matches = performDataAssociation(mu_vec, msg->cones, Q);

        std::vector<dv_msgs::msg::SingleRangeBearingObservation> obs;
        for (const auto& cone : msg->cones) {
            dv_msgs::msg::SingleRangeBearingObservation o;
            o.range = cone.location.x;
            o.yaw = cone.location.y;
            o.color = cone.color;
            obs.push_back(o);
        }

        measurementUpdate(obs, matches, msg->header.stamp);
    }

    void motionUpdate(const rclcpp::Time& time) {
        double dt = (time - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            last_time_ = time;
            return;
        }

        double phi = mu_(2);

        Eigen::Matrix3d Gt;
        Gt << 1, 0, (vx_ * sin(phi) - vy_ * cos(phi)) * dt,
              0, 1, (vx_ * cos(phi) + vy_ * sin(phi)) * dt,
              0, 0, 1;

        Eigen::Matrix3d Vt;
        Vt << cos(phi) * dt, -sin(phi) * dt, 0,
              sin(phi) * dt,  cos(phi) * dt, 0,
              0, 0, dt;

        Eigen::Matrix3d M;
        M << sigma_vx_ * sigma_vx_, 0, 0,
             0, sigma_vy_ * sigma_vy_, 0,
             0, 0, sigma_phi_ * sigma_phi_;

        Sigma_ = Gt * Sigma_ * Gt.transpose() + Vt * M * Vt.transpose();

        mu_ += Eigen::Vector3d(
            cos(phi) * vx_ * dt - sin(phi) * vy_ * dt,
            sin(phi) * vx_ * dt + cos(phi) * vy_ * dt,
            phi_dot_ * dt
        );

        mu_(2) = normalizeAngle(mu_(2));
        last_time_ = time;
        first_measurement_ = false;
        publishState(time);
    }

    void measurementUpdate(
        const std::vector<dv_msgs::msg::SingleRangeBearingObservation>& obs,
        const std::vector<int>& matches,
        const rclcpp::Time& stamp) {

        for (size_t i = 0; i < obs.size(); ++i) {
            if (matches[i] < 0) continue;

            const auto& z = obs[i];
            const auto& l = known_landmarks[matches[i]];

            double dx = l(0) - mu_(0);
            double dy = l(1) - mu_(1);
            double q = dx * dx + dy * dy;
            double sqrt_q = std::sqrt(q);

            Eigen::Vector2d z_hat(sqrt_q, normalizeAngle(std::atan2(dy, dx) - mu_(2)));
            Eigen::Vector2d z_meas(z.range, z.yaw);
            Eigen::Vector2d innovation = z_meas - z_hat;
            innovation(1) = normalizeAngle(innovation(1));

            Eigen::Matrix<double, 2, 3> H;
            H << -dx / sqrt_q, -dy / sqrt_q, 0,
                  dy / q,     -dx / q,     -1;

            Eigen::Matrix2d R;
            R << sigma_r_ * sigma_r_, 0,
                 0, sigma_theta_ * sigma_theta_;

            Eigen::Matrix2d S = H * Sigma_ * H.transpose() + R;
            Eigen::Matrix<double, 3, 2> K = Sigma_ * H.transpose() * S.inverse();

            mu_ += K * innovation;
            mu_(2) = normalizeAngle(mu_(2));
            Sigma_ = (Eigen::Matrix3d::Identity() - K * H) * Sigma_;
        }

        publishState(stamp);
    }

    void publishState(const rclcpp::Time& stamp) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = mu_(0);
        odom.pose.pose.position.y = mu_(1);
        tf2::Quaternion q;
        q.setRPY(0, 0, mu_(2));
        odom.pose.pose.orientation = tf2::toMsg(q);
        odom_pub_->publish(odom);

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose;

        // Set non-zero covariance to avoid RViz ignoring it
        for (int i = 0; i < 36; ++i) pose.pose.covariance[i] = 0.0;
        pose.pose.covariance[0] = 0.1;   // x
        pose.pose.covariance[7] = 0.1;   // y
        pose.pose.covariance[35] = 0.05; // yaw

        pose_pub_->publish(pose);

        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = mu_(0);
        tf.transform.translation.y = mu_(1);
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);

        RCLCPP_INFO(this->get_logger(), "✅ Published /ekf/pose at (%.2f, %.2f, %.2f)", mu_(0), mu_(1), mu_(2));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::string csv_path = "/home/ishaan/ros2_ws/src/motion_update_pkg/data/small_track.csv";
    known_landmarks = loadLandmarksFromCSV(csv_path);
    auto node = std::make_shared<EKFLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

