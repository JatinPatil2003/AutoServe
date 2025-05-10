#include "autoserve_docking/docking_server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace autoserve_docking
{

DockingServer::DockingServer()
: Node("docking_server"), controller_(std::make_shared<Controller>()),
tf_buffer_(get_clock()),  // Initialize TF buffer with clock
tf_listener_(tf_buffer_)   // Attach listener to buffer)
{
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  dock_srv_ = create_service<std_srvs::srv::Trigger>(
    "/dock_to_pose", std::bind(&DockingServer::dockCallback, this, _1, _2));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&DockingServer::controlLoop, this));
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
  if (!goal_active_) return;

  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);

    current_pose_.position.x = transform_stamped.transform.translation.x;
    current_pose_.position.y = transform_stamped.transform.translation.y;
    current_pose_.orientation = transform_stamped.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform base_footprint to map: %s", ex.what());
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  controller_->computeVelocityCommand(goal_pose_, current_pose_, cmd_vel, false);

    RCLCPP_INFO(this->get_logger(), "Velocity: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);

  // Check distance to goal
  double dx = goal_pose_.position.x - current_pose_.position.x;
  double dy = goal_pose_.position.y - current_pose_.position.y;
  double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < 0.02) {
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping...");
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    goal_active_ = false;
  }

  cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace autoserve_docking
