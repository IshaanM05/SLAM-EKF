#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "eufs_msgs/msg/car_state.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class PoseVisualizer : public rclcpp::Node {
public:
  PoseVisualizer() : Node("pose_visualizer") {
    prediction_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/prediction_pose", 10, std::bind(&PoseVisualizer::predictionCallback, this, std::placeholders::_1));

    ground_truth_sub_ = this->create_subscription<eufs_msgs::msg::CarState>(
      "/ground_truth/state", 10, std::bind(&PoseVisualizer::groundTruthCallback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr prediction_sub_;
  rclcpp::Subscription<eufs_msgs::msg::CarState>::SharedPtr ground_truth_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  void predictionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    publishMarker(*msg, "predicted", 0, 0.0f, 1.0f, 0.0f); // Green
  }

  void groundTruthCallback(const eufs_msgs::msg::CarState::SharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = msg->pose.pose;
    publishMarker(pose_msg, "ground_truth", 1, 1.0f, 0.0f, 0.0f); // Red
  }

  void publishMarker(const geometry_msgs::msg::PoseStamped& pose_msg,
                     const std::string& ns, int id,
                     float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header = pose_msg.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start, end;
    start.x = pose_msg.pose.position.x;
    start.y = pose_msg.pose.position.y;

    // Convert orientation to yaw
    tf2::Quaternion q(
      pose_msg.pose.orientation.x,
      pose_msg.pose.orientation.y,
      pose_msg.pose.orientation.z,
      pose_msg.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    end.x = start.x + 0.5 * std::cos(yaw);
    end.y = start.y + 0.5 * std::sin(yaw);

    marker.points.push_back(start);
    marker.points.push_back(end);
    marker.scale.x = 0.1;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    marker.lifetime = rclcpp::Duration::from_seconds(0);  // Persistent
    marker_pub_->publish(marker);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseVisualizer>());
  rclcpp::shutdown();
  return 0;
}
