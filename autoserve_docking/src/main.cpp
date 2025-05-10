// #include "rclcpp/rclcpp.hpp"
// #include "autoserve_docking/controller.hpp"

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);

//   auto node = rclcpp::Node::make_shared("controller_node");

//   auto controller = std::make_shared<autoserve_docking::Controller>();

//   rclcpp::spin(node);

//   rclcpp::shutdown();

//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "autoserve_docking/docking_server.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoserve_docking::DockingServer>());
  rclcpp::shutdown();
  return 0;
}