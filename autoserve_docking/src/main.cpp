#include "rclcpp/rclcpp.hpp"
#include "autoserve_docking/controller.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("controller_node");

  auto controller = std::make_shared<autoserve_docking::Controller>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
