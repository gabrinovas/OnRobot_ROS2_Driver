// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <string>

#include "onrobot_driver/common/SerialConnectionWrapper.hpp"
#include "onrobot_driver/common/TCPConnectionWrapper.hpp"
#include "onrobot_driver/common/tool_power.hpp"
#include "onrobot_driver/monitor/gripper_status_monitor.hpp"

namespace onrobot_driver
{

void GripperStatusMonitor::onResetPower(
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  if (use_fake_hardware_) {
    RCLCPP_INFO(get_logger(), "Simulated tool power reset executed (fake hardware mode).");
    response->success = true;
    response->message = "Fake hardware: Simulated tool power reset command executed successfully.";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Executing tool power reset via Compute Box (address %d)...",
    compute_box_address_);
  try {
    // A connection of its own, open only for this command: the driver keeps the one to the gripper.
    std::unique_ptr<IModbusConnection> connection;
    if (connection_.type == "tcp") {
      connection = std::make_unique<TCPConnectionWrapper>(
        connection_.ip_address, connection_.port, connection_.timeout_ms);
    } else {
      connection = std::make_unique<SerialConnectionWrapper>(
        connection_.device, connection_.serial_timeout_ms);
    }
    sendToolPowerReset(*connection, compute_box_address_);
    connection->close();

    response->success = true;
    response->message = "Tool power reset command sent successfully to Compute Box.";
    RCLCPP_INFO(get_logger(), "Tool power reset command executed successfully.");
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("Failed to reset tool power: ") + error.what();
    RCLCPP_ERROR(get_logger(), "Error resetting tool power: %s", error.what());
  }
}

}  // namespace onrobot_driver
