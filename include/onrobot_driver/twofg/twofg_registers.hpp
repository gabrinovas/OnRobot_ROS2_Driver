// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__TWOFG__TWOFG_REGISTERS_HPP_
#define ONROBOT_DRIVER__TWOFG__TWOFG_REGISTERS_HPP_

#include <cstdint>

/// Modbus map of the 2FG7 (OnRobot Connectivity Guide). These are the protocol of the device, not
/// settings.
namespace onrobot_driver::twofg
{

constexpr uint16_t REG_TARGET_WIDTH = 0;
constexpr uint16_t REG_TARGET_FORCE = 1;
constexpr uint16_t REG_TARGET_SPEED = 2;
constexpr uint16_t REG_COMMAND = 3;
constexpr uint16_t REG_STATUS = 256;
constexpr uint16_t REG_EXTERNAL_WIDTH = 257;
constexpr uint16_t REG_INTERNAL_WIDTH = 258;
constexpr uint16_t REG_MIN_EXTERNAL_WIDTH = 259;
constexpr uint16_t REG_MAX_EXTERNAL_WIDTH = 260;
constexpr uint16_t REG_MIN_INTERNAL_WIDTH = 261;
constexpr uint16_t REG_MAX_INTERNAL_WIDTH = 262;
constexpr uint16_t REG_FORCE = 263;

constexpr uint16_t CMD_GRIP_EXTERNAL = 1;
constexpr uint16_t CMD_GRIP_INTERNAL = 2;
constexpr uint16_t CMD_STOP = 3;

constexpr uint16_t STATUS_BUSY = 0x0001;
constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
constexpr uint16_t STATUS_ERROR_NOT_CALIBRATED = 0x0008;
constexpr uint16_t STATUS_ERROR_LINEAR_SENSOR = 0x0010;

/// Range of the speed register, in percent.
constexpr float SPEED_MIN_PERCENT = 10.0f;
constexpr float SPEED_MAX_PERCENT = 100.0f;

}  // namespace onrobot_driver::twofg

#endif  // ONROBOT_DRIVER__TWOFG__TWOFG_REGISTERS_HPP_
