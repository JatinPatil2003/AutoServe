#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "autoserve_docking/controller.hpp"

namespace autoserve_docking
{

Controller::Controller()
{
  k_phi_ = 6.0;
  k_delta_ = 1.0;
  beta_ = 0.4;
  lambda_ = 2.0;
  v_linear_min_ = 0.05;
  v_linear_max_ = 0.25;
  v_angular_max_ = 0.4;
  slowdown_radius_ = 0.30;

  control_law_ = std::make_unique<autoserve_docking::SmoothControlLaw>(
    k_phi_, k_delta_, beta_, lambda_, slowdown_radius_, v_linear_min_, v_linear_max_,
    v_angular_max_);
}

void Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool backward)
{
  cmd = control_law_->calculateRegularVelocity(pose, backward);
}

void Controller::computeVelocityCommand(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & current, 
  geometry_msgs::msg::Twist & cmd, bool backward)
{
  cmd = control_law_->calculateRegularVelocity(target, current, backward);
}

}  // namespace autoserve_docking
