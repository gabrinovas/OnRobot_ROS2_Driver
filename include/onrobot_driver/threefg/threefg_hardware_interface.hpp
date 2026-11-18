// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__THREEFG__THREEFG_HARDWARE_INTERFACE_HPP_
#define ONROBOT_DRIVER__THREEFG__THREEFG_HARDWARE_INTERFACE_HPP_

#include <memory>

#include "onrobot_driver/hardware/finger_hardware_interface.hpp"

namespace onrobot_driver
{

/// ros2_control plugin of the OnRobot 3FG15.
class ThreeFGHardwareInterface : public FingerHardwareInterface
{
protected:
  std::unique_ptr<OnRobotGripperBase> createGripper(
    int device_address, const FingerSettings & settings) override;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__THREEFG__THREEFG_HARDWARE_INTERFACE_HPP_
