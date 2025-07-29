#include "vrobot_plc/vrobot_plc.hpp"
#include "vrobot_plc/constance.hpp"
#include <chrono>
#include <iostream>
#include <modbus/modbus.h>
#include <thread>

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

  // Try to connect with retry logic
  for (int retry = 0; retry < max_connection_retries; retry++) {
    if (modbus_connect(mb) != -1) {
      is_connected_              = true;
      consecutive_failure_count_ = 0;
      RCLCPP_INFO(this->get_logger(),
                  "Successfully connected to Modbus server");
      return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "Connection attempt %d/%d failed, retrying in %d ms...",
                retry + 1, max_connection_retries, retry_delay_ms);

    if (retry < max_connection_retries - 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "Unable to connect to the modbus server after %d retries",
               max_connection_retries);
  is_connected_ = false;
  return false;
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
  this->declare_parameter("max_connection_retries", 5);
  this->declare_parameter("retry_delay_ms", 1000);
  this->declare_parameter("max_consecutive_failures_", 10);

  this->get_parameter("modbus_tcp_ip", ip);
  this->get_parameter("modbus_tcp_port", port);
  this->get_parameter("modbus_tcp_slave_id", slave_id);
  this->get_parameter("modbus_tcp_timeout", timeout);
  this->get_parameter("max_connection_retries", max_connection_retries);
  this->get_parameter("retry_delay_ms", retry_delay_ms);
  this->get_parameter("max_consecutive_failures_", max_consecutive_failures_);

  RCLCPP_INFO(this->get_logger(), "Modbus parameters initialized");
  RCLCPP_INFO(this->get_logger(), "Modbus IP: %s", ip.c_str());
  RCLCPP_INFO(this->get_logger(), "Modbus port: %d", port);
  RCLCPP_INFO(this->get_logger(), "Modbus slave ID: %d", slave_id);
  RCLCPP_INFO(this->get_logger(), "Modbus timeout: %d", timeout);
  RCLCPP_INFO(this->get_logger(), "Max connection retries: %d",
              max_connection_retries);
  RCLCPP_INFO(this->get_logger(), "Retry delay: %d ms", retry_delay_ms);
  RCLCPP_INFO(this->get_logger(), "Max consecutive failures: %d",
              max_consecutive_failures_);

  return true;
}

bool VrobotPLC::read_coils(uint16_t address, uint16_t count, uint8_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }

  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Modbus not connected, operation skipped");
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
    is_connected_ = false;
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

  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Modbus not connected, operation skipped");
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
    is_connected_ = false;
    return false;
  }
  return true;
}

bool VrobotPLC::write_coils(uint16_t address, uint16_t count, uint8_t *data) {
  if (!mb) {
    RCLCPP_ERROR(this->get_logger(), "Modbus context not initialized");
    return false;
  }

  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Modbus not connected, operation skipped");
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
    is_connected_ = false;
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

  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Modbus not connected, operation skipped");
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
    is_connected_ = false;
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

  if (!is_connected_) {
    RCLCPP_WARN(this->get_logger(), "Modbus not connected, operation skipped");
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
    is_connected_ = false;
    return false;
  }
  return true;
}

void VrobotPLC::timer_callback() {
  try {
    // Check if we need to attempt reconnection
    if (!is_connected_ ||
        consecutive_failure_count_ >= max_consecutive_failures_) {
      if (!reconnect_modbus()) {
        consecutive_failure_count_++;
        RCLCPP_ERROR(this->get_logger(),
                     "Connection failed, consecutive failures: %d/%d",
                     consecutive_failure_count_, max_consecutive_failures_);
        return;
      }
    }

    PlcStatus msg;
    msg.header.stamp       = this->now();
    bool operation_success = true;

    uint16_t data_x[3];
    if (!read_holding_registers(kEmgButtonAddress, 3, data_x)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read emg button");
      operation_success = false;
    } else {
      msg.is_emg          = ((data_x[0]) & 0xFFFF) == 30;
      msg.is_front_safety = ((data_x[1]) & 0xFFFF) == 1001;
      msg.is_back_safety  = ((data_x[2]) & 0xFFFF) == 1001;
    }

    uint8_t data_m[2];
    if (!read_coils(kStartButtonAddress, 2, data_m)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to read start button");
      operation_success = false;
    } else {
      msg.start_button = data_m[0] & 0x01;
      msg.stop_button  = data_m[1] & 0x01;
    }

    if (operation_success) {
      plc_status_publisher_->publish(msg);
    }

    uint16_t data_led = led_control_data.load() & 0xFFFF;
    if (!write_holding_registers(kLedAddress, 1, &data_led)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to write led control");
      operation_success = false;
    }

    // Update failure counter based on operation success
    if (operation_success) {
      reset_failure_counter();
    } else {
      consecutive_failure_count_++;
      RCLCPP_WARN(this->get_logger(),
                  "Operation failed, consecutive failures: %d/%d",
                  consecutive_failure_count_, max_consecutive_failures_);
    }

  } catch (const std::exception &e) {
    consecutive_failure_count_++;
    if (consecutive_failure_count_ >= max_consecutive_failures_) {
      is_connected_ = false; // Mark as disconnected for retry logic
    }
    reset_failure_counter();
    RCLCPP_ERROR(this->get_logger(),
                 "Exception in timer callback: %s, consecutive failures: %d/%d",
                 e.what(), consecutive_failure_count_,
                 max_consecutive_failures_);
  }
}

void VrobotPLC::led_control_callback(const LedControl::SharedPtr msg) {
  led_control_data.store(msg->led_id);
}

bool VrobotPLC::reconnect_modbus() {
  RCLCPP_WARN(this->get_logger(),
              "Attempting to reconnect to Modbus server...");

  // Close existing connection
  if (mb) {
    modbus_close(mb);
  }

  // Try to reconnect with retry logic
  for (int retry = 0; retry < max_connection_retries; retry++) {
    if (modbus_connect(mb) != -1) {
      is_connected_              = true;
      consecutive_failure_count_ = 0;
      RCLCPP_INFO(this->get_logger(),
                  "Successfully reconnected to Modbus server");
      return true;
    }

    RCLCPP_WARN(this->get_logger(),
                "Reconnection attempt %d/%d failed, retrying in %d ms...",
                retry + 1, max_connection_retries, retry_delay_ms);

    if (retry < max_connection_retries - 1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "Failed to reconnect to Modbus server after %d retries",
               max_connection_retries);
  is_connected_ = false;
  return false;
}

bool VrobotPLC::is_connection_alive() {
  if (!mb || !is_connected_) {
    return false;
  }

  // Try a simple read operation to test connection
  uint16_t test_data;
  int      rc = modbus_read_registers(mb, 0, 1, &test_data);

  if (rc == -1) {
    is_connected_ = false;
    return false;
  }

  return true;
}

void VrobotPLC::reset_failure_counter() { consecutive_failure_count_ = 0; }

} // namespace vrobot_plc