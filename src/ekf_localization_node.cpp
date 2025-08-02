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
#include <iostream>

#include "dv_msgs/msg/indexed_track.hpp"
#include "dv_msgs/msg/indexed_cone.hpp"
#include "dv_msgs/msg/single_range_bearing_observation.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"

// External symbols - defined in data_assoc_trainee.cpp
extern std::vector<Eigen::Vector3d> known_landmarks;
extern std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov);

class EKFLocalizationNode : public rclcpp::Node {
public:
    EKFLocalizationNode() : Node("ekf_localization_node") {
        std::cout << "DEBUG: Constructor called with new code!" << std::endl;
        RCLCPP_ERROR(this->get_logger(), "CONSTRUCTOR DEBUG - NEW VERSION");
        
        // Only declare parameter if not already declared
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
        
        initialiseEKF();
        setupROS();
        RCLCPP_INFO(this->get_logger(), "EKF Localization Node initialized");
    }

private:
    // EKF state
    Eigen::VectorXd mu_;
    Eigen::MatrixXd Sigma_;
    double sigma_vx_ = 0.1, sigma_vy_ = 0.1, sigma_phi_ = 0.1;
    double sigma_r_ = 0.1, sigma_theta_ = 0.05;
    rclcpp::Time last_time_;
    bool first_measurement_ = true;

    double vx_ = 0.0, vy_ = 0.0, phi_dot_ = 0.0;
    bool has_wheel_data_ = false, has_imu_data_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_sub_;
    rclcpp::Subscription<dv_msgs::msg::IndexedTrack>::SharedPtr perception_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void initialiseEKF() {
        mu_.setZero(3);
        Sigma_ = Eigen::MatrixXd::Identity(3,3) * 0.1;
        last_time_ = this->now();
    }

    void setupROS() {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/ekf/odometry", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ekf/pose", 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&EKFLocalizationNode::imuCallback, this, std::placeholders::_1));

        wheel_sub_ = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
            "/ros_can/wheel_speeds", 10, std::bind(&EKFLocalizationNode::wheelCallback, this, std::placeholders::_1));

        perception_sub_ = this->create_subscription<dv_msgs::msg::IndexedTrack>(
            "/perception/cones", 10, std::bind(&EKFLocalizationNode::perceptionCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "IMU callback received");
        phi_dot_ = msg->angular_velocity.z;
        has_imu_data_ = true;

        if (has_wheel_data_ && !first_measurement_) {
            RCLCPP_INFO(this->get_logger(), "Triggering motion update from IMU");
            motionUpdate(msg->header.stamp);
        } else {
            RCLCPP_WARN(this->get_logger(), "Motion update skipped: wheel=%d, first=%d", 
                       has_wheel_data_, first_measurement_);
        }
    }

    void wheelCallback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg) {
        RCLCPP_ERROR(this->get_logger(), "WHEEL CALLBACK TRIGGERED - SUCCESS!");
        RCLCPP_INFO(this->get_logger(), "Wheel callback received");
        
        // Use correct field names from eufs_msgs
        vx_ = (msg->speeds.lf_speed + msg->speeds.rf_speed) / 2.0;
        vy_ = 0.0;
        has_wheel_data_ = true;

        // CRITICAL FIX: Use message timestamp instead of current time
        if (has_imu_data_ && !first_measurement_) {
            RCLCPP_INFO(this->get_logger(), "Triggering motion update from wheel");
            motionUpdate(msg->header.stamp);  // FIXED: Use msg->header.stamp
        } else {
            RCLCPP_WARN(this->get_logger(), "Motion update skipped from wheel: imu=%d, first=%d", 
                       has_imu_data_, first_measurement_);
        }
    }

    void perceptionCallback(const dv_msgs::msg::IndexedTrack::SharedPtr msg) {
        if (msg->track.empty()) return;

        // Force first measurement completion if needed
        if (first_measurement_) {
            first_measurement_ = false;
            RCLCPP_WARN(this->get_logger(), "Forced first measurement completion - starting EKF");
        }

        Eigen::Matrix2d Q;
        Q << sigma_r_*sigma_r_, 0,
             0, sigma_theta_*sigma_theta_;

        std::vector<double> mu_vec = {mu_(0), mu_(1), mu_(2)};
        auto matches = performDataAssociation(mu_vec, msg->track, Q);

        std::vector<dv_msgs::msg::SingleRangeBearingObservation> obs;
        obs.reserve(msg->track.size());

        for (const auto &cone : msg->track) {
            dv_msgs::msg::SingleRangeBearingObservation o;
            double dx = cone.location.x;
            double dy = cone.location.y;

            o.range = std::sqrt(dx*dx + dy*dy);
            o.yaw = std::atan2(dy, dx);
            o.pitch = 0.0;
            o.id = static_cast<int32_t>(cone.index);

            obs.push_back(o);
        }

        measurementUpdate(obs, matches, this->now());
    }

    void motionUpdate(const rclcpp::Time &time) {
        RCLCPP_INFO(this->get_logger(), "Motion update called");
        
        double dt = (time - last_time_).seconds();
        
        // FIXED: Use correct format specifiers for timestamp debugging
        RCLCPP_INFO(this->get_logger(), "Timestamps - Current: %.3f.%09ld, Last: %.3f.%09ld, dt: %.6f", 
                    time.seconds(), time.nanoseconds() % 1000000000,
                    last_time_.seconds(), last_time_.nanoseconds() % 1000000000, dt);
        
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Motion update rejected: dt=%.6f", dt);
            last_time_ = time;
            return;
        }

        double phi = mu_(2);

        Eigen::Matrix3d Gt;
        Gt << 1, 0, (vx_ * std::sin(phi) - vy_ * std::cos(phi)) * dt,
              0, 1, (vx_ * std::cos(phi) + vy_ * std::sin(phi)) * dt,
              0, 0, 1;

        Eigen::Matrix3d Vt;
        Vt << std::cos(phi) * dt, -std::sin(phi) * dt, 0,
              std::sin(phi) * dt,  std::cos(phi) * dt, 0,
              0, 0, dt;

        Eigen::Matrix3d M;
        M << sigma_vx_ * sigma_vx_, 0, 0,
             0, sigma_vy_ * sigma_vy_, 0,
             0, 0, sigma_phi_ * sigma_phi_;

        Sigma_ = Gt * Sigma_ * Gt.transpose() + Vt * M * Vt.transpose();

        mu_ += Eigen::Vector3d(
            std::cos(phi) * vx_ * dt - std::sin(phi) * vy_ * dt,
            std::sin(phi) * vx_ * dt + std::cos(phi) * vy_ * dt,
            phi_dot_ * dt
        );

        mu_(2) = normalizeAngle(mu_(2));
        last_time_ = time;
        first_measurement_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Motion update completed, publishing pose");
        publishState(time);
    }

    void measurementUpdate(const std::vector<dv_msgs::msg::SingleRangeBearingObservation>& obs,
                          const std::vector<int>& matches,
                          const rclcpp::Time& stamp) {
        RCLCPP_INFO(this->get_logger(), "Measurement update called with %zu observations", obs.size());

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

        RCLCPP_INFO(this->get_logger(), "Measurement update completed, publishing pose");
        publishState(stamp);
    }

    void publishState(const rclcpp::Time& stamp) {
        RCLCPP_INFO(this->get_logger(), "Publishing pose: (%.2f, %.2f, %.2f)", mu_(0), mu_(1), mu_(2));
        
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

        for (int i = 0; i < 36; ++i) pose.pose.covariance[i] = 0.0;
        pose.pose.covariance[0] = Sigma_(0, 0);
        pose.pose.covariance[7] = Sigma_(1, 1);
        pose.pose.covariance[35] = Sigma_(2, 2);

        pose_pub_->publish(pose);
        RCLCPP_INFO(this->get_logger(), "Pose published to /ekf/pose");

        geometry_msgs::msg::TransformStamped tf;
        tf.header = odom.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = mu_(0);
        tf.transform.translation.y = mu_(1);
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    std::cout << "Using hardcoded landmarks from data_assoc_trainee.cpp" << std::endl;
    
    auto node = std::make_shared<EKFLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




