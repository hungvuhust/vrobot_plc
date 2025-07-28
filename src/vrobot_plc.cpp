#include "vrobot_plc/vrobot_plc.hpp"
#include <iostream>
#include <modbus/modbus.h>

namespace vrobot_plc {
VrobotPLC::VrobotPLC() : Node("vrobot_plc") {
  RCLCPP_INFO(this->get_logger(), "VrobotPLC node initialized");
  if (!init_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize modbus");
    throw std::runtime_error("Failed to initialize modbus");
  }
}

VrobotPLC::~VrobotPLC() {
  if (mb) {
    modbus_close(mb);
    modbus_free(mb);
  }
  RCLCPP_INFO(this->get_logger(), "VrobotPLC node destroyed");
}

bool VrobotPLC::init_modbus() {
  mb = modbus_new_tcp(ip.c_str(), port);
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Unable to create the libmodbus context");
    return false;
  }
  modbus_set_response_timeout(mb, timeout, 0);
  modbus_set_slave(mb, slave_id);
  if (modbus_connect(mb) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Unable to connect to the modbus server");
    return false;
  }
  return true;
}

bool VrobotPLC::read_coils(uint16_t address, uint16_t count, uint8_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }
  try {
    int rc = modbus_read_bits(mb, address, count, data);
    if (rc == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read coils");
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to read coils: %s", e.what());
    return false;
  }
  return true;
}

bool VrobotPLC::read_input_bits(uint16_t address, uint16_t count,
                                uint8_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }
  try {
    int rc = modbus_read_input_bits(mb, address, count, data);
    if (rc == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read input bits");
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to read input bits: %s", e.what());
    return false;
  }
  return true;
}

bool VrobotPLC::write_coils(uint16_t address, uint16_t count, uint8_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }
  try {
    int rc = modbus_write_bits(mb, address, count, data);
    if (rc == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to write coils");
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to write coils: %s", e.what());
    return false;
  }
  return true;
}

bool VrobotPLC::read_holding_registers(uint16_t address, uint16_t count,
                                       uint16_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }
  try {
    int rc = modbus_read_registers(mb, address, count, data);
    if (rc == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read holding registers");
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to read holding registers: %s",
                 e.what());
    return false;
  }
  return true;
}

bool VrobotPLC::write_holding_registers(uint16_t address, uint16_t count,
                                        uint16_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }
  try {
    int rc = modbus_write_registers(mb, address, count, data);
    if (rc == -1) {
      RCLCPP_ERROR(this->get_logger(), "Unable to write holding registers");
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to write holding registers: %s",
                 e.what());
    return false;
  }
  return true;
}

} // namespace vrobot_plc