#ifndef AUTOSERVE_DOCKING__CONTROLLER_HPP_
#define AUTOSERVE_DOCKING__CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "autoserve_docking/smooth_control_law.hpp"

namespace autoserve_docking
{
class Controller
{
public:
  Controller();

  void computeVelocityCommand(
    const geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist & cmd, bool backward = false);
  void computeVelocityCommand(
    const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & current, 
    geometry_msgs::msg::Twist & cmd, bool backward =false);

protected:
  std::unique_ptr<autoserve_docking::SmoothControlLaw> control_law_;
  double k_phi_, k_delta_, beta_, lambda_;
  double slowdown_radius_, v_linear_min_, v_linear_max_, v_angular_max_;
};

}  // namespace autoserve_docking

#endif  // AUTOSERVE_DOCKING__CONTROLLER_HPP_
