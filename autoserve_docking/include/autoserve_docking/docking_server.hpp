#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "autoserve_docking/controller.hpp"

namespace autoserve_docking
{

class DockingServer : public rclcpp::Node
{
public:
  DockingServer();

private:
  void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void dockCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void controlLoop();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr dock_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Pose goal_pose_;
  bool pose_received_ = false;
  bool goal_active_ = false;

  std::shared_ptr<Controller> controller_;
};

}  // namespace autoserve_docking
