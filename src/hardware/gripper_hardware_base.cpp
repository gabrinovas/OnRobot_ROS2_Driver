// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>

#include "onrobot_driver/hardware/gripper_hardware_base.hpp"

namespace onrobot_driver
{

GripperHardwareBase::~GripperHardwareBase()
{
  stopWorker();
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    const HardwareParameters parameters(info);
    onrobot_type_ = parameters.text("onrobot_type");
    use_fake_hardware_ = parameters.flag("use_fake_hardware");
    device_address_ = parameters.integer("device_address");
    connection_.type = parameters.text("connection_type");
    // The address of the Compute Box belongs to the installation: the description carries it
    // only for a real gripper reached over tcp.
    if (connection_.type == "tcp" && !use_fake_hardware_) {
      connection_.ip_address = parameters.text("ip_address");
    }
    connection_.port = parameters.integer("port");
    connection_.device = parameters.text("device");
    connection_.timeout_ms = parameters.integer("timeout_ms");
    connection_.serial_timeout_ms = parameters.integer("serial_timeout_ms");
    connection_.connect_retry_ms = parameters.integer("connect_retry_ms");
    worker_period_ms_ = parameters.integer("worker_period_ms");
    reconnect_wait_ms_ = parameters.integer("reconnect_wait_ms");
    if (connection_.type != "tcp" && connection_.type != "serial") {
      throw ParameterError(
              "[Fail-Fast] Unsupported connection_type '" + connection_.type + "' (tcp or serial)");
    }
    loadParameters(parameters);
  } catch (const ParameterError & error) {
    RCLCPP_ERROR(logger(), "%s", error.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(
    logger(), "%s hardware interface initialized (conn=%s, fake=%s)", onrobot_type_.c_str(),
    connection_.type.c_str(), use_fake_hardware_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

void GripperHardwareBase::openConnection(OnRobotGripperBase & gripper)
{
  gripper.connect(connection_, []() {return rclcpp::ok();});
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!use_fake_hardware_) {
    startWorker();
  }
  RCLCPP_INFO(logger(), "%s activated (async worker running)", onrobot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  stopWorker();
  afterDeactivating();
  RCLCPP_INFO(logger(), "%s deactivated", onrobot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  stopWorker();
  destroyGripper();
  RCLCPP_INFO(logger(), "%s cleaned up", onrobot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stopWorker();
  destroyGripper();
  RCLCPP_INFO(logger(), "%s shutdown", onrobot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

GripperHardwareBase::CallbackReturn GripperHardwareBase::on_error(const rclcpp_lifecycle::State &)
{
  stopWorker();
  RCLCPP_ERROR(logger(), "%s entered error state", onrobot_type_.c_str());
  return CallbackReturn::SUCCESS;
}

void GripperHardwareBase::startWorker()
{
  if (!worker_running_) {
    worker_running_ = true;
    worker_thread_ = std::thread(&GripperHardwareBase::workerLoop, this);
  }
}

void GripperHardwareBase::stopWorker()
{
  if (worker_running_) {
    worker_running_ = false;
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }
}

void GripperHardwareBase::workerLoop()
{
  if (!gripper()) {
    return;
  }
  beforeCommunicating();
  const auto period = std::chrono::milliseconds(worker_period_ms_);

  while (worker_running_) {
    const auto loop_start = std::chrono::steady_clock::now();
    try {
      communicate();
      comm_healthy_ = true;
    } catch (const std::exception & error) {
      comm_healthy_ = false;
      onCommunicationError(error);
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_wait_ms_));
      try {
        if (gripper()->reconnect()) {
          comm_healthy_ = true;
          onCommunicationRestored();
          RCLCPP_INFO(logger(), "Modbus connection to %s restored.", onrobot_type_.c_str());
        }
      } catch (...) {
      }
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - loop_start);
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

}  // namespace onrobot_driver
