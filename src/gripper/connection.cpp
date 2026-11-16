// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "MB/modbusUtils.hpp"
#include "onrobot_driver/common/OnRobotGripperBase.hpp"
#include "onrobot_driver/common/SerialConnectionWrapper.hpp"
#include "onrobot_driver/common/TCPConnectionWrapper.hpp"
#include "onrobot_driver/common/tool_power.hpp"

namespace onrobot_driver
{

namespace
{
// OnRobot Connectivity Guide: register and value that power cycle the tool from the Compute Box.
constexpr uint16_t kToolPowerRegister = 0;
constexpr uint16_t kToolPowerCycle = 2;
}  // namespace

void sendToolPowerReset(IModbusConnection & connection, int compute_box_address)
{
  std::vector<MB::ModbusCell> values = {MB::ModbusCell(kToolPowerCycle)};
  MB::ModbusRequest req(
    compute_box_address, MB::utils::WriteSingleAnalogOutputRegister, kToolPowerRegister, 1, values);
  connection.sendRequest(req);
}

OnRobotGripperBase::OnRobotGripperBase(int device_address)
: device_address_(device_address), connection_type_("unknown"), target_endpoint_("unknown")
{
}

OnRobotGripperBase::~OnRobotGripperBase()
{
  close();
}

std::unique_ptr<IModbusConnection> OnRobotGripperBase::open() const
{
  if (connection_type_ == "tcp") {
    return std::make_unique<TCPConnectionWrapper>(
      settings_.ip_address, settings_.port, settings_.timeout_ms);
  }
  return std::make_unique<SerialConnectionWrapper>(settings_.device, settings_.serial_timeout_ms);
}

bool OnRobotGripperBase::canReopen() const
{
  return (connection_type_ == "tcp" && !settings_.ip_address.empty()) ||
         (connection_type_ == "serial" && !settings_.device.empty());
}

void OnRobotGripperBase::connect(
  const ConnectionSettings & settings,
  std::function<bool()> keep_running)
{
  if (settings.type == "tcp") {
    if (settings.ip_address.empty()) {
      throw std::invalid_argument("Please provide an IP address for TCP connection.");
    }
    if (settings.timeout_ms <= 0) {
      throw std::invalid_argument("The TCP response timeout must be greater than zero.");
    }
  } else if (settings.type == "serial") {
    if (settings.device.empty()) {
      throw std::invalid_argument("Please provide a serial device for connection.");
    }
  } else {
    throw std::invalid_argument(
            "Unsupported connection type '" + settings.type + "' (tcp or serial).");
  }

  {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    settings_ = settings;
    connection_type_ = settings.type;
    target_endpoint_ = settings.endpoint();

    while (true) {
      if (keep_running && !keep_running()) {
        throw std::runtime_error("Connection aborted by shutdown signal");
      }
      try {
        connection_ = open();
        std::cout << "Connected to OnRobot gripper at " << target_endpoint_ << std::endl;
        break;
      } catch (const MB::ModbusException & ex) {
        std::cerr << "Waiting for OnRobot " << connection_type_ << " connection at " <<
          target_endpoint_ <<
          " (" << ex.what() << "). Retrying every " << settings_.connect_retry_ms << " ms..." <<
          std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(settings_.connect_retry_ms));
      }
    }
  }
  onAttached();
  applyDefaults();
}

void OnRobotGripperBase::attach(std::unique_ptr<IModbusConnection> connection)
{
  {
    std::lock_guard<std::mutex> lock(comm_mutex_);
    connection_ = std::move(connection);
    connection_type_ = "mock";
  }
  onAttached();
}

void OnRobotGripperBase::close()
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  if (connection_) {
    try {
      connection_->close();
    } catch (...) {
    }
    connection_.reset();
  }
}

bool OnRobotGripperBase::isConnected() const
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  return connection_ != nullptr;
}

bool OnRobotGripperBase::reconnect()
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  if (!canReopen()) {
    return false;
  }
  try {
    if (connection_) {
      try {
        connection_->close();
      } catch (...) {
      }
      connection_.reset();
    }
    connection_ = open();
    reconnect_count_++;
    std::cout << "[OnRobotGripperBase] Successfully reconnected to " << connection_type_ <<
      " endpoint: " << target_endpoint_ << std::endl;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

void OnRobotGripperBase::resetToolPower(int compute_box_address)
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  if (!connection_) {
    throw std::runtime_error("Cannot reset tool power: no active Modbus connection");
  }
  try {
    sendToolPowerReset(*connection_, compute_box_address);
    std::cout << "Sent Tool Power Reset command to Compute Box (address " << compute_box_address <<
      ")" <<
      std::endl;
  } catch (const MB::ModbusException & ex) {
    std::cerr << "Failed to reset tool power on Compute Box: " << ex.what() << std::endl;
    throw;
  }
}

}  // namespace onrobot_driver
