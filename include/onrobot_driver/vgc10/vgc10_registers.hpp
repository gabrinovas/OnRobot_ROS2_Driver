// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__VGC10__VGC10_REGISTERS_HPP_
#define ONROBOT_DRIVER__VGC10__VGC10_REGISTERS_HPP_

#include <cstdint>

/// Modbus map of the VG10 / VGC10 (OnRobot Connectivity Guide v1.22.0, page 47). These are the
/// protocol of the device, not settings.
namespace onrobot_driver::vgc10
{

constexpr uint16_t REG_CHANNEL_A_CTRL = 0x0000;
constexpr uint16_t REG_CHANNEL_B_CTRL = 0x0001;
constexpr uint16_t REG_CURRENT_LIMIT = 0x0002;
constexpr uint16_t REG_CHANNEL_A_VACUUM = 0x0102;
constexpr uint16_t REG_CHANNEL_B_VACUUM = 0x0103;

/// A channel control register holds the mode in its high byte and the target vacuum (%) in its low
/// byte.
constexpr uint8_t MODE_RELEASE = 0x00;
constexpr uint8_t MODE_GRIP = 0x01;
constexpr uint8_t MODE_IDLE = 0x02;

/// The vacuum registers hold 1/1000 of relative vacuum.
constexpr float VACUUM_FULL_SCALE = 1000.0f;

/// Bits of the status word this driver builds from the channel modes.
constexpr uint16_t STATUS_CHANNEL_A_GRIPPING = 0x01;
constexpr uint16_t STATUS_CHANNEL_B_GRIPPING = 0x02;

}  // namespace onrobot_driver::vgc10

#endif  // ONROBOT_DRIVER__VGC10__VGC10_REGISTERS_HPP_
