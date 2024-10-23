#include "action_manager.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ActionManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
