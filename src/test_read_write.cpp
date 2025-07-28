#include "vrobot_plc/vrobot_plc.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vrobot_plc::VrobotPLC>();

  std::map<std::string, uint16_t> kCoils = {
      { "M1", 0x0801},
      { "M2", 0x0802},
      {"X20", 0x0416},
      {"X21", 0x0417},
      {"X22", 0x0418},
  };

  std::map<std::string, uint16_t> kHoldingRegisters = {
      {"D0", 0x1000},
  };

  auto kLogger = node->get_logger();
  try {
    while (rclcpp::ok()) {
      for (auto &[name, address] : kCoils) {
        uint8_t data[3];
        node->read_coils(address, 1, data);
        RCLCPP_INFO(kLogger, "%s: %02X", name.c_str(), data[0] & 0x01);
      }

      for (auto &[name, address] : kHoldingRegisters) {
        uint16_t data[3];
        node->read_holding_registers(address, 1, data);
        RCLCPP_INFO(kLogger, "%s: %04X", name.c_str(), data[0]);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}