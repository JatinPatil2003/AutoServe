#include <cmath>

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2/utils.hpp"

struct EgocentricPolarCoordinates
{
  float r;       // Radial distance between the robot pose and the target pose.
  float phi;     // Orientation of target with respect to the line of sight
                 // from the robot to the target.
  float delta;   // Steering angle of the robot with respect to the line of sight.

  EgocentricPolarCoordinates(
    const float & r_in = 0.0,
    const float & phi_in = 0.0,
    const float & delta_in = 0.0)
  : r(r_in), phi(phi_in), delta(delta_in) {}

  explicit EgocentricPolarCoordinates(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current = geometry_msgs::msg::Pose(), bool backward = false)
  {
    // Compute the difference between the target and the current pose
    float dX = target.position.x - current.position.x;
    float dY = target.position.y - current.position.y;
    // Compute the line of sight from the robot to the target
    // Flip it if the robot is moving backwards
    float line_of_sight = backward ? (std::atan2(-dY, dX) + M_PI) : std::atan2(-dY, dX);
    // Compute the ego polar coordinates
    r = sqrt(dX * dX + dY * dY);
    phi = angles::normalize_angle(tf2::getYaw(target.orientation) + line_of_sight);
    delta = angles::normalize_angle(tf2::getYaw(current.orientation) + line_of_sight);
  }
};

