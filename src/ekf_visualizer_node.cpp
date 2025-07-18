// File: ekf_visualizer_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class EKFVisualizer : public rclcpp::Node
{
public:
    EKFVisualizer() : Node("ekf_visualizer_node")
    {
        ekf_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/ekf/pose", 10,
            std::bind(&EKFVisualizer::ekf_callback, this, std::placeholders::_1));

        gt_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ground_truth/state", 10,
            std::bind(&EKFVisualizer::gt_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/ekf/pose_marker", 10);
    }

private:
    void ekf_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        publish_arrow_marker(msg->pose.pose, 0, 0.0, 1.0, 0.0);  // Green: EKF estimated
    }

    void gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        publish_arrow_marker(msg->pose.pose, 1, 1.0, 0.0, 0.0);  // Red: Ground truth
    }

    void publish_arrow_marker(const geometry_msgs::msg::Pose& pose, int id, float r, float g, float b)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "pose_arrow";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose;

        // Ensure orientation is valid
        if (marker.pose.orientation.w == 0.0 &&
            marker.pose.orientation.x == 0.0 &&
            marker.pose.orientation.y == 0.0 &&
            marker.pose.orientation.z == 0.0)
        {
            marker.pose.orientation.w = 1.0;
        }

        marker.scale.x = 1.0;  // Arrow length
        marker.scale.y = 0.2;  // Shaft diameter
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        marker.lifetime = rclcpp::Duration(0, 0); // 0 duration = infinite lifetime

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ekf_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFVisualizer>());
    rclcpp::shutdown();
    return 0;
}
