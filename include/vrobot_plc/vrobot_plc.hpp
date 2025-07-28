#pragma once

#include "modbus/modbus-tcp.h"
#include "modbus/modbus-version.h"
#include "modbus/modbus.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace vrobot_plc {
class VrobotPLC : public rclcpp::Node {
public:
  VrobotPLC();
  ~VrobotPLC();

  bool read_coils(uint16_t address, uint16_t count, uint8_t *data);
  bool write_coils(uint16_t address, uint16_t count, uint8_t *data);

  bool read_input_bits(uint16_t address, uint16_t count, uint8_t *data);
  bool write_input_bits(uint16_t address, uint16_t count, uint8_t *data);

  bool read_holding_registers(uint16_t address, uint16_t count, uint16_t *data);
  bool write_holding_registers(uint16_t address, uint16_t count,
                               uint16_t *data);

private:
  bool init_modbus();

private:
  modbus_t   *mb;
  int         slave_id = 1;
  int         port     = 502;
  std::string ip       = "192.168.10.2";
  int         timeout  = 1000;
};
} // namespace vrobot_plc