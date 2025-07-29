#pragma once

#include <cstdint>

namespace vrobot_plc {

constexpr uint16_t kStartButtonAddress = 0x0800; // M0
constexpr uint16_t kStopButtonAddress  = 0x0801; // M1
constexpr uint16_t kFrontSafetyAddress = 0x0417; // X21
constexpr uint16_t kBackSafetyAddress  = 0x0418; // X22
constexpr uint16_t kEmgButtonAddress   = 0x0416; // X20

constexpr uint16_t kLedAddress = 0x1000; // D0

constexpr char kPlcStatusTopic[]  = "/vrobot/plc/status";
constexpr char kLedControlTopic[] = "/vrobot/plc/led_control";

} // namespace vrobot_plc