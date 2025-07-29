#include "vrobot_plc/vrobot_plc.hpp"
#include <iostream>
#include <modbus/modbus.h>

namespace vrobot_plc {
VrobotPLC::VrobotPLC() : Node("vrobot_plc") {
  RCLCPP_INFO(this->get_logger(), "VrobotPLC node initialized");

  if (!init_params()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize parameters");
    throw std::runtime_error("Failed to initialize parameters");
  }

  if (!init_modbus()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize modbus");
    throw std::runtime_error("Failed to initialize modbus");
  }

  if (!init_node()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize node");
    throw std::runtime_error("Failed to initialize node");
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

bool VrobotPLC::init_node() {
  // Publisher
  plc_status_publisher_ =
      this->create_publisher<PlcStatus>(kPlcStatusTopic, 10);

  // Subscriber
  led_control_subscriber_ = this->create_subscription<LedControl>(
      kLedControlTopic, 10,
      std::bind(&VrobotPLC::led_control_callback, this, std::placeholders::_1));

  // Timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&VrobotPLC::timer_callback, this));
  return true;
}

bool VrobotPLC::init_params() {
  this->declare_parameter("modbus_tcp_ip", "192.168.10.2");
  this->declare_parameter("modbus_tcp_port", 502);
  this->declare_parameter("modbus_tcp_slave_id", 1);
  this->declare_parameter("modbus_tcp_timeout", 1000);

  this->get_parameter("modbus_tcp_ip", ip);
  this->get_parameter("modbus_tcp_port", port);
  this->get_parameter("modbus_tcp_slave_id", slave_id);
  this->get_parameter("modbus_tcp_timeout", timeout);

  RCLCPP_INFO(this->get_logger(), "Modbus parameters initialized");
  RCLCPP_INFO(this->get_logger(), "Modbus IP: %s", ip.c_str());
  RCLCPP_INFO(this->get_logger(), "Modbus port: %d", port);
  RCLCPP_INFO(this->get_logger(), "Modbus slave ID: %d", slave_id);
  RCLCPP_INFO(this->get_logger(), "Modbus timeout: %d", timeout);

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

void VrobotPLC::timer_callback() {
  try {
    PlcStatus msg;
    msg.header.stamp = this->now();

    uint8_t data[1];
    if (!read_coils(kEmgButtonAddress, 1, data)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read emg button");
      return;
    }
    msg.is_emg = data[0] & 0x01;

    if (!read_input_bits(kFrontSafetyAddress, 1, data)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read front safety");
      return;
    }
    msg.is_front_safety = data[0] & 0x01;

    if (!read_input_bits(kBackSafetyAddress, 1, data)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read back safety");
      return;
    }
    msg.is_back_safety = data[0] & 0x01;

    plc_status_publisher_->publish(msg);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to read plc status: %s", e.what());
  }
}

void VrobotPLC::led_control_callback(const LedControl::SharedPtr msg) {
  try {
    uint8_t data[1];
    data[0] = msg->led_id;
    write_coils(kLedAddress, 1, data);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unable to write led control: %s",
                 e.what());
  }
}

} // namespace vrobot_plc