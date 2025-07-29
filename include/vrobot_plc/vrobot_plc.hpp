#pragma once

#include "modbus/modbus-tcp.h"
#include "modbus/modbus-version.h"
#include "modbus/modbus.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vrobot_plc/constance.hpp"
#include "vrobot_plc/msg/led_control.hpp"
#include "vrobot_plc/msg/plc_status.hpp"

using PlcStatus  = vrobot_plc::msg::PlcStatus;
using LedControl = vrobot_plc::msg::LedControl;

namespace vrobot_plc {
class VrobotPLC : public rclcpp::Node {
public:
  VrobotPLC();
  ~VrobotPLC();

private:
  bool init_modbus();
  bool init_node();
  bool init_params();

  // Modbus functions
  bool read_coils(uint16_t address, uint16_t count, uint8_t *data);
  bool write_coils(uint16_t address, uint16_t count, uint8_t *data);

  bool read_input_bits(uint16_t address, uint16_t count, uint8_t *data);
  bool write_input_bits(uint16_t address, uint16_t count, uint8_t *data);

  bool read_holding_registers(uint16_t address, uint16_t count, uint16_t *data);
  bool write_holding_registers(uint16_t address, uint16_t count,
                               uint16_t *data);

private:
  modbus_t   *mb;
  int         slave_id = 1;
  int         port     = 502;
  std::string ip       = "192.168.10.2";
  int         timeout  = 1000;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void                         timer_callback();

  // Publisher
  rclcpp::Publisher<PlcStatus>::SharedPtr plc_status_publisher_;

  // Subscriber
  rclcpp::Subscription<LedControl>::SharedPtr led_control_subscriber_;
  void led_control_callback(const LedControl::SharedPtr msg);
};
} // namespace vrobot_plc