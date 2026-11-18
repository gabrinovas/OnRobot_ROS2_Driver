// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__THREEFG__THREEFG_REGISTERS_HPP_
#define ONROBOT_DRIVER__THREEFG__THREEFG_REGISTERS_HPP_

#include <cstdint>

/// Modbus map of the 3FG15 (OnRobot Connectivity Guide). These are the protocol of the device, not
/// settings.
namespace onrobot_driver::threefg
{

constexpr uint16_t REG_TARGET_FORCE = 0;
constexpr uint16_t REG_TARGET_DIAMETER = 1;
constexpr uint16_t REG_TARGET_SPEED = 2;
constexpr uint16_t REG_GRIP_TYPE = 2;
constexpr uint16_t REG_CONTROL = 3;
constexpr uint16_t REG_STATUS = 256;
constexpr uint16_t REG_RAW_DIAMETER = 257;
constexpr uint16_t REG_DIAMETER_WITH_OFFSET = 258;
constexpr uint16_t REG_APPLIED_FORCE = 259;
constexpr uint16_t REG_FINGER_LENGTH = 270;
constexpr uint16_t REG_FINGER_POSITION = 272;
constexpr uint16_t REG_FINGERTIP_OFFSET = 273;
constexpr uint16_t REG_ACTUAL_WIDTH_WITH_OFFSET = 275;
constexpr uint16_t REG_MIN_DIAMETER = 513;
constexpr uint16_t REG_MAX_DIAMETER = 514;
constexpr uint16_t REG_CFG_FINGER_LENGTH = 1025;
constexpr uint16_t REG_CFG_FINGER_POSITION = 1027;
constexpr uint16_t REG_CFG_FINGERTIP_OFFSET = 1028;

constexpr uint16_t CMD_GRIP = 0x0001;
constexpr uint16_t CMD_MOVE = 0x0002;
constexpr uint16_t CMD_STOP = 0x0004;
constexpr uint16_t CMD_FLEXIBLE_GRIP = 0x0005;

constexpr uint16_t STATUS_BUSY = 0x0001;
constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
constexpr uint16_t STATUS_CALIBRATION_OK = 0x0008;

/// Range of the speed register, in percent.
constexpr float SPEED_MIN_PERCENT = 10.0f;
constexpr float SPEED_MAX_PERCENT = 100.0f;

/// Units of the registers.
constexpr float FORCE_UNITS_PER_PERCENT = 10.0f;     ///< force: 1/10 % of the maximum force
constexpr float LENGTH_UNITS_PER_MM = 10.0f;         ///< finger length: 1/10 mm
constexpr float OFFSET_UNITS_PER_MM = 100.0f;        ///< fingertip offset: 1/100 mm
constexpr float DIAMETER_UNITS_PER_METER = 10000.0f;  ///< diameters: 1/10 mm

}  // namespace onrobot_driver::threefg

#endif  // ONROBOT_DRIVER__THREEFG__THREEFG_REGISTERS_HPP_
