// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include "onrobot_driver/twofg/TwoFG.hpp"
#include "onrobot_driver/twofg/twofg_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace onrobot_driver
{

std::unique_ptr<OnRobotGripperBase> TwoFGHardwareInterface::createGripper(
  int device_address, const FingerSettings & settings)
{
  return std::make_unique<TwoFG>(device_address, settings);
}

}  // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(
  onrobot_driver::TwoFGHardwareInterface,
  hardware_interface::ActuatorInterface)
