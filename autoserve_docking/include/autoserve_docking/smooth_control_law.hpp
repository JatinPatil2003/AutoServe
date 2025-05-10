#include <algorithm>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace autoserve_docking 
{

class SmoothControlLaw
{
public:
  SmoothControlLaw(
    double k_phi, double k_delta, double beta, double lambda, double slowdown_radius,
    double v_linear_min, double v_linear_max, double v_angular_max);

  ~SmoothControlLaw() = default;

  void setCurvatureConstants(
    const double k_phi, const double k_delta, const double beta, const double lambda);

  void setSlowdownRadius(const double slowdown_radius);

  void setSpeedLimit(
    const double v_linear_min, const double v_linear_max, const double v_angular_max);

  geometry_msgs::msg::Twist calculateRegularVelocity(
    const geometry_msgs::msg::Pose & target,
    const geometry_msgs::msg::Pose & current,
    const bool & backward = false);

  geometry_msgs::msg::Twist calculateRegularVelocity(
    const geometry_msgs::msg::Pose & target, const bool & backward = false);

protected:
  double calculateCurvature(double r, double phi, double delta);

  double k_phi_;
  double k_delta_;
  double beta_;
  double lambda_;
  double slowdown_radius_;
  double v_linear_min_;
  double v_linear_max_;
  double v_angular_max_;
};

}  // namespace autoserve_docking

