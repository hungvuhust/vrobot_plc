#include "vrobot_plc/vrobot_plc.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node    = std::make_shared<vrobot_plc::VrobotPLC>();
    auto kLogger = node->get_logger();

    std::map<std::string, uint16_t> kCoils = {
        { "M1", 2049},
        { "M2", 2050},
        {"X20", 1046},
        {"X21", 1047},
        {"X22", 1048},
    };

    std::map<std::string, uint16_t> kHoldingRegisters = {
        {"D4", 4100},
    };
    while (rclcpp::ok()) {
      for (auto &[name, address] : kCoils) {
        uint8_t data[3];
        if (!node->read_coils(address, 1, data)) {
          RCLCPP_ERROR(kLogger, "Failed to read coils");
          continue;
        }
        RCLCPP_INFO(kLogger, "%s: %02X", name.c_str(), data[0] & 0x01);
      }

      for (auto &[name, address] : kHoldingRegisters) {
        uint16_t data[3];
        if (!node->read_holding_registers(address, 1, data)) {
          RCLCPP_ERROR(kLogger, "Failed to read holding registers");
          continue;
        }
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