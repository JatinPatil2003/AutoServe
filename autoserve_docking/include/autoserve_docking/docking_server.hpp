#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_eigen/tf2_eigen.hpp>

#include "autoserve_docking/controller.hpp"
#include "autoserve_docking/dock_detector.hpp"


namespace autoserve_docking
{

class DockingServer : public rclcpp::Node
{
public:
  DockingServer();
  void init();

private:
  void dockCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void controlLoop();
  Eigen::Matrix4f poseMsgToEigenMatrix(const geometry_msgs::msg::Pose& pose_msg);
  Eigen::Matrix4f transformStampedToEigenMatrix(const geometry_msgs::msg::TransformStamped &transform_msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dock_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;

  geometry_msgs::msg::Pose dock_pose_;

  std::shared_ptr<Controller> controller_;
  std::shared_ptr<DockDetector> dock_detector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool pose_received_ = false;
  bool goal_active_ = false;
  bool dock_found_ = false;

};

}  // namespace autoserve_docking
