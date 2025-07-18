#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "eufs_msgs/msg/wheel_speeds_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>




class PoseEstimator : public rclcpp::Node {
public:
  PoseEstimator() : Node("pose_estimator"){
      imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
          "/imu", 10, std::bind(&PoseEstimator::imu_callback, this, std::placeholders::_1));


      wheel_subscription = this->create_subscription<eufs_msgs::msg::WheelSpeedsStamped>(
          "/ros_can/wheel_speeds", 10, std::bind(&PoseEstimator::wheel_callback, this, std::placeholders::_1));


      pose_publish = this->create_publisher<geometry_msgs::msg::PoseStamped>("/prediction_pose", 10);
      prev_time_ = this->now();
  }




private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription;
  rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_subscription;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publish;




  double x = 0.0, y = 0.0, yaw = 0.0;
  double vx = 0.0, vy = 0.0, yaw_rate = 0.0;
  const double rwheel = 0.2;
  const double gear_ratio = 50.0;


  rclcpp::Time prev_time_;


  void wheel_callback(const eufs_msgs::msg::WheelSpeedsStamped::SharedPtr msg){
      double lb = msg->speeds.lb_speed;
      double rb = msg->speeds.rb_speed;
      double avg_speed = (lb + rb) / 2.0;
      vx = 2 * M_PI * rwheel * avg_speed / gear_ratio;
      updatePose(this->now());
  }


  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
      yaw_rate = msg->angular_velocity.z;
  }


  void updatePose(rclcpp::Time current_time){
      if (prev_time_.nanoseconds() == 0) {
      prev_time_ = current_time;
      return;
  }


  double dt = (current_time - prev_time_).seconds();
  if (dt <= 0.0){
      return;
  }


      double dx = std::cos(yaw) * vx * dt;
      double dy = std::sin(yaw) * vx * dt;
      double dyaw = yaw_rate * dt;


      x += dx;
      y += dy;
      yaw += dyaw;
      yaw = std::atan2(std::sin(yaw), std::cos(yaw));


      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header.stamp = current_time;
      pose_msg.header.frame_id = "map";

      pose_msg.pose.position.x = x;
      pose_msg.pose.position.y = y;
      pose_msg.pose.position.z = 0.0;


      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();


      RCLCPP_INFO(this->get_logger(), "current_time = %.6f, prev_time_ = %.6f",
  current_time.seconds(), prev_time_.seconds());


      pose_publish->publish(pose_msg);
      RCLCPP_INFO(this->get_logger(), "Publishing prediction pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
      prev_time_ = current_time;
  }
};




int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimator>());
  rclcpp::shutdown();
}
