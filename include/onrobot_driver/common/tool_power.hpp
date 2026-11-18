// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__TOOL_POWER_HPP_
#define ONROBOT_DRIVER__COMMON__TOOL_POWER_HPP_

#include "onrobot_driver/common/IModbusConnection.hpp"

namespace onrobot_driver
{

/// Ask the Compute Box at the given Modbus address to power cycle the tool (OnRobot Connectivity
/// Guide: value 2 written to register 0). Used to recover the gripper after a safety stop.
void sendToolPowerReset(IModbusConnection & connection, int compute_box_address);

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__COMMON__TOOL_POWER_HPP_
