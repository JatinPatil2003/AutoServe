#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "autoserve_docking/ego_polar_coords.hpp"
#include "autoserve_docking/smooth_control_law.hpp"


namespace autoserve_docking
{

SmoothControlLaw::SmoothControlLaw(
  double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
  double v_linear_min, double v_linear_max, double v_angular_max)
: k_phi_(k_phi), k_delta_(k_delta), beta_(beta), lambda_(lambda), slowdown_radius_(slowdown_radius),
  v_linear_min_(v_linear_min), v_linear_max_(v_linear_max), v_angular_max_(v_angular_max)
{
}

void SmoothControlLaw::setCurvatureConstants(
  double k_phi, double k_delta, double beta, double lambda)
{
  k_phi_ = k_phi;
  k_delta_ = k_delta;
  beta_ = beta;
  lambda_ = lambda;
}

void SmoothControlLaw::setSlowdownRadius(double slowdown_radius)
{
  slowdown_radius_ = slowdown_radius;
}

void SmoothControlLaw::setSpeedLimit(
  const double v_linear_min, const double v_linear_max, const double v_angular_max)
{
  v_linear_min_ = v_linear_min;
  v_linear_max_ = v_linear_max;
  v_angular_max_ = v_angular_max;
}

geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const geometry_msgs::msg::Pose & current,
  const bool & backward)
{
  // Convert the target to polar coordinates
  auto ego_coords = EgocentricPolarCoordinates(target, current, backward);
  // Calculate the curvature
  double curvature = calculateCurvature(ego_coords.r, ego_coords.phi, ego_coords.delta);
  
  if (fabs(curvature) > 10.0){
    curvature = 0.0;
  }
  
  // Invert the curvature if the robot is moving backwards
  curvature = backward ? -curvature : curvature;

  // Adjust the linear velocity as a function of the path curvature to
  // slowdown the controller as it approaches its target
  double v = v_linear_max_ / (1.0 + beta_ * std::pow(fabs(curvature), lambda_));

  // Slowdown when the robot is near the target to remove singularity
  v = std::min(v_linear_max_ * (ego_coords.r / slowdown_radius_), v);

  // Set some small v_min when far away from origin to promote faster
  // turning motion when the curvature is very high
  v = std::clamp(v, v_linear_min_, v_linear_max_);

  // Set the velocity to negative if the robot is moving backwards
  v = backward ? -v : v;

  // std::cout << "v: " << v << " curvature: " << curvature << std::endl;
  // Compute the angular velocity
  double w = curvature * v;
  // Bound angular velocity between [-max_angular_vel, max_angular_vel]
  double w_bound = std::clamp(w, -v_angular_max_, v_angular_max_);
  // And linear velocity to follow the curvature
  v = (curvature != 0.0) ? (w_bound / curvature) : v;

  // Return the velocity command
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w_bound;
  return cmd_vel;
}

geometry_msgs::msg::Twist SmoothControlLaw::calculateRegularVelocity(
  const geometry_msgs::msg::Pose & target, const bool & backward)
{
  return calculateRegularVelocity(target, geometry_msgs::msg::Pose(), backward);
}

double SmoothControlLaw::calculateCurvature(double r, double phi, double delta)
{
  // Calculate the proportional term of the control law as the product of the gain and the error:
  // difference between the actual steering angle and the virtual control for the slow subsystem
  double prop_term = k_delta_ * (delta - std::atan(-k_phi_ * phi));
  // Calculate the feedback control law for the steering
  double feedback_term = (1.0 + (k_phi_ / (1.0 + std::pow(k_phi_ * phi, 2)))) * sin(delta);
  // Calculate the path curvature
  return -1.0 / r * (prop_term + feedback_term);
}

}  // namespace autoserve_docking