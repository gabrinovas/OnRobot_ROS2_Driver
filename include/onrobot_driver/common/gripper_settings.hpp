// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__GRIPPER_SETTINGS_HPP_
#define ONROBOT_DRIVER__COMMON__GRIPPER_SETTINGS_HPP_

#include <cstdint>
#include <string>

namespace onrobot_driver
{

/// How to reach a gripper. Every field comes from configuration: none has a value in code.
struct ConnectionSettings
{
  std::string type;        ///< "tcp" (Compute Box) or "serial" (RS485 of the robot tool connector)
  std::string ip_address;  ///< tcp only
  int port;                ///< tcp only
  std::string device;      ///< serial only
  int timeout_ms;          ///< tcp: time to wait for a response
  int serial_timeout_ms;   ///< serial: time to wait for a response
  int connect_retry_ms;    ///< wait between two attempts to open the connection

  std::string endpoint() const
  {
    return type == "tcp" ? ip_address + ":" + std::to_string(port) : device;
  }
};

/// Working range and start-up values of a finger gripper (2FG7, 3FG15).
struct FingerSettings
{
  float min_width;      ///< m
  float max_width;      ///< m
  float max_force;      ///< N
  float default_force;  ///< N, written to the gripper when the connection opens
  float default_speed;  ///< %, written to the gripper when the connection opens
};

/// Working range and start-up values of a vacuum gripper (VGC10).
struct VacuumSettings
{
  uint8_t max_vacuum_percent;   ///< highest vacuum the gripper is asked for
  uint16_t max_current_ma;      ///< highest pump current limit accepted
  uint16_t initial_current_ma;  ///< pump current limit written when the connection opens
  /// normalised vacuum above which a target means "grip", not "release"
  float grip_threshold;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__COMMON__GRIPPER_SETTINGS_HPP_
