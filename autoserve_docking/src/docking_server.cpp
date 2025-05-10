#include "autoserve_docking/docking_server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace autoserve_docking
{

DockingServer::DockingServer()
: Node("docking_server"), controller_(std::make_shared<Controller>())
{
  amcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/amcl_pose", 10, std::bind(&DockingServer::amclCallback, this, _1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  dock_srv_ = create_service<std_srvs::srv::Trigger>(
    "/dock_to_pose", std::bind(&DockingServer::dockCallback, this, _1, _2));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&DockingServer::controlLoop, this));
}

void DockingServer::amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  current_pose_ = msg->pose.pose;
  pose_received_ = true;
}

void DockingServer::dockCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Set hardcoded target pose
  goal_pose_.position.x = 1.0;
  goal_pose_.position.y = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);  // theta = 0
  goal_pose_.orientation = tf2::toMsg(q);

  goal_active_ = true;

  response->success = true;
  response->message = "Docking initiated to hardcoded pose (x=1.0, y=0.0, theta=0.0)";
}

void DockingServer::controlLoop()
{
  RCLCPP_INFO(this->get_logger(), "In amcel: %d, service: %d", pose_received_, goal_active_);

  if (!pose_received_ || !goal_active_) return;

  geometry_msgs::msg::Twist cmd_vel;
  controller_->computeVelocityCommand(goal_pose_, current_pose_, cmd_vel, false);

    RCLCPP_INFO(this->get_logger(), "Velocity: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);

  // Check distance to goal
  double dx = goal_pose_.position.x - current_pose_.position.x;
  double dy = goal_pose_.position.y - current_pose_.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  // Check yaw difference
//   tf2::Quaternion q1, q2;
//   tf2::fromMsg(current_pose_.orientation, q1);
//   tf2::fromMsg(goal_pose_.orientation, q2);

//   double yaw_current = tf2::getYaw(q1);
//   double yaw_goal = tf2::getYaw(q2);
//   double dyaw = std::fabs(yaw_goal - yaw_current);

  if (dist < 0.05) {  // Thresholds: 5cm, 0.1 rad
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping...");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_active_ = false;
  }

  cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace autoserve_docking
