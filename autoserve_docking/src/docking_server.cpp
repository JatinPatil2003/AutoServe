#include "autoserve_docking/docking_server.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace autoserve_docking
{

  DockingServer::DockingServer()
      : Node("docking_server"), controller_(std::make_shared<Controller>()),
        tf_buffer_(get_clock()), // Initialize TF buffer with clock
        tf_listener_(tf_buffer_) // Attach listener to buffer)
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    dock_srv_ = create_service<std_srvs::srv::Trigger>(
        "/dock_to_pose", std::bind(&DockingServer::dockCallback, this, _1, _2));

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&DockingServer::controlLoop, this));
  }

  void DockingServer::init()
  {
    dock_detector_ = std::make_shared<DockDetector>(shared_from_this());
  }

  void DockingServer::dockCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Set hardcoded target pose
    goal_pose_.position.x = 1.0;
    goal_pose_.position.y = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // theta = 0
    goal_pose_.orientation = tf2::toMsg(q);

    goal_active_ = true;
    dock_found_ = false;

    response->success = true;
    response->message = "Docking Started";
  }

  Eigen::Matrix4f DockingServer::poseMsgToEigenMatrix(const geometry_msgs::msg::Pose &pose_msg)
  {
    // Extract quaternion values from the pose message
    double qx = pose_msg.orientation.x;
    double qy = pose_msg.orientation.y;
    double qz = pose_msg.orientation.z;
    double qw = pose_msg.orientation.w;

    // Convert quaternion to Eigen rotation matrix
    Eigen::Quaternionf quat(qw, qx, qy, qz); // Eigen expects (w, x, y, z)
    Eigen::Matrix3f rotation = quat.toRotationMatrix();

    // Create Eigen::Matrix4f and set rotation and translation
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = pose_msg.position.x;
    transform(1, 3) = pose_msg.position.y;
    transform(2, 3) = pose_msg.position.z;

    return transform;
  }

  Eigen::Matrix4f DockingServer::transformStampedToEigenMatrix(const geometry_msgs::msg::TransformStamped &transform_msg)
  {
    // Extract quaternion values from the transform message
    double qx = transform_msg.transform.rotation.x;
    double qy = transform_msg.transform.rotation.y;
    double qz = transform_msg.transform.rotation.z;
    double qw = transform_msg.transform.rotation.w;

    // Convert quaternion to Eigen rotation matrix
    Eigen::Quaternionf quat(qw, qx, qy, qz); // Eigen expects (w, x, y, z)
    Eigen::Matrix3f rotation = quat.toRotationMatrix();

    // Extract translation values (x, y, z) from the transform message
    Eigen::Vector3f translation(transform_msg.transform.translation.x,
                                transform_msg.transform.translation.y,
                                transform_msg.transform.translation.z);

    // Create Eigen::Matrix4f and set rotation and translation
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;

    return transform;
  }

  void DockingServer::controlLoop()
  {
    if (!goal_active_)
      return;

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);

      current_pose_.position.x = transform_stamped.transform.translation.x;
      current_pose_.position.y = transform_stamped.transform.translation.y;
      current_pose_.orientation = transform_stamped.transform.rotation;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform base_footprint to map: %s", ex.what());
      goal_active_ = false;
      return;
    }

    int dock_detect_count = 0;

    while (!dock_found_ && dock_detect_count < 100)
    {
      if (dock_detector_->detectDockICP(dock_pose_))
      {
        try
        {
          geometry_msgs::msg::TransformStamped transform_stamped =
              tf_buffer_.lookupTransform("map", "ydlidar", tf2::TimePointZero);

          Eigen::Matrix4f map_to_ydlidar = transformStampedToEigenMatrix(transform_stamped);

          Eigen::Matrix4f ydlidar_to_dock = poseMsgToEigenMatrix(dock_pose_);

          Eigen::Matrix4f map_to_dock = map_to_ydlidar * ydlidar_to_dock;

          Eigen::Vector3f translation = map_to_dock.block<3, 1>(0, 3);
          Eigen::Matrix3f rotation = map_to_dock.block<3, 3>(0, 0);
          float theta = std::atan2(rotation(1, 0), rotation(0, 0));

          dock_pose_.position.x = translation.x();
          dock_pose_.position.y = translation.y();
          dock_pose_.orientation.z = std::sin(theta / 2.0);
          dock_pose_.orientation.w = std::cos(theta / 2.0);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_WARN(this->get_logger(), "Could not transform base_footprint to map: %s", ex.what());
          goal_active_ = false;
          return;
        }
        dock_found_ = true;
        break;
      }
      ++dock_detect_count;
    }

    if (!dock_found_)
    {
      RCLCPP_ERROR(this->get_logger(), "Dock detection failed after 100 attempts.");
      goal_active_ = false;
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    controller_->computeVelocityCommand(dock_pose_, current_pose_, cmd_vel, true);

    RCLCPP_INFO(this->get_logger(), "Velocity: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);

    // Check distance to goal
    double dx = dock_pose_.position.x - current_pose_.position.x;
    double dy = dock_pose_.position.y - current_pose_.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // Check angle to goal
    double angle_to_goal = std::atan2(dy, dx);  // Angle of the line to the goal from the current position
    // Extract the yaw (z-axis rotation) from the quaternion of the current orientation
    double quaternion_x = current_pose_.orientation.x;
    double quaternion_y = current_pose_.orientation.y;
    double quaternion_z = current_pose_.orientation.z;
    double quaternion_w = current_pose_.orientation.w;
    // Convert quaternion to yaw (z-axis rotation)
    double siny_cosp = 2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
    double cosy_cosp = 1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);  // Yaw angle (orientation)

    double angle_error = angle_to_goal - yaw;  // Difference between current orientation and desired angle

    // Normalize the angle error to be between -pi and pi
    if (angle_error > M_PI) {
        angle_error -= 2 * M_PI;
    } else if (angle_error < -M_PI) {
        angle_error += 2 * M_PI;
    }

    if (dist < 0.04)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached. Stopping...");
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      goal_active_ = false;
    }

    cmd_vel_pub_->publish(cmd_vel);
  }

} // namespace autoserve_docking
