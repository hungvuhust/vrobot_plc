#include "vrobot_plc/vrobot_plc.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vrobot_plc::VrobotPLC>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}