// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>
#include <string>
#include <vector>

#include "onrobot_driver/vgc10/vgc10_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace onrobot_driver
{

void VGC10HardwareInterface::loadParameters(const HardwareParameters & parameters)
{
  settings_.max_vacuum_percent = static_cast<uint8_t>(parameters.integer("max_vacuum_percent"));
  settings_.max_current_ma = static_cast<uint16_t>(parameters.integer("max_current_ma"));
  settings_.initial_current_ma = static_cast<uint16_t>(parameters.integer("initial_current_ma"));
  settings_.grip_threshold = static_cast<float>(parameters.number("grip_threshold"));
  tuning_.resend_threshold = parameters.number("resend_threshold");
  tuning_.grip_threshold = parameters.number("grip_threshold");
  tuning_.initial_effort = parameters.number("initial_effort");
  tuning_.nominal_read_period_s = parameters.number("nominal_read_period_s");
  tuning_.error_warning_period_ms = parameters.integer("error_warning_period_ms");
  tuning_.fake_smoothing = parameters.number("fake_smoothing");
  if (info_.joints.size() != channels_.size()) {
    throw ParameterError(
            "[Fail-Fast] The vacuum gripper has exactly 2 joints (channel A, channel B) in its "
            "<ros2_control> block, got " + std::to_string(info_.joints.size()));
  }
  for (size_t i = 0; i < channels_.size(); ++i) {
    channels_[i].joint_name = info_.joints[i].name;
    channels_[i].effort_command = tuning_.initial_effort;
  }
}

GripperHardwareBase::CallbackReturn VGC10HardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  state_buffer_.initRT(VacuumState());
  for (auto & channel : channels_) {
    channel.desired.store(0.0, std::memory_order_relaxed);
    channel.desired_effort.store(tuning_.initial_effort, std::memory_order_relaxed);
  }
  new_command_.store(false, std::memory_order_relaxed);

  if (use_fake_hardware_) {
    RCLCPP_INFO(logger(), "Configuring fake hardware for %s", onrobot_type_.c_str());
    for (auto & channel : channels_) {
      channel.state = 0.0;
      channel.command = 0.0;
    }
    return CallbackReturn::SUCCESS;
  }

  RCLCPP_INFO(
    logger(), "Connecting to physical %s via %s...",
    onrobot_type_.c_str(), connection_.type.c_str());
  try {
    gripper_ = std::make_unique<VGC10>(device_address_, settings_);
    openConnection(*gripper_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "Failed to connect to %s: %s", onrobot_type_.c_str(), error.what());
    destroyGripper();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void VGC10HardwareInterface::afterDeactivating()
{
  if (gripper_) {
    try {
      gripper_->releaseAll();
    } catch (...) {
    }
  }
}

std::vector<hardware_interface::StateInterface> VGC10HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & channel : channels_) {
    interfaces.emplace_back(channel.joint_name, "position", &channel.state);
    interfaces.emplace_back(channel.joint_name, "velocity", &channel.velocity);
    interfaces.emplace_back(channel.joint_name, "effort", &channel.effort);
    interfaces.emplace_back(channel.joint_name, "vacuum_level", &channel.level);
    interfaces.emplace_back(channel.joint_name, "pressure_kpa", &channel.pressure_kpa);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
VGC10HardwareInterface::export_command_interfaces()
{
  // max_effort and effort are two names of the same command: controllers differ in the one they
  // claim.
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & channel : channels_) {
    interfaces.emplace_back(channel.joint_name, "position", &channel.command);
    interfaces.emplace_back(channel.joint_name, "max_effort", &channel.effort_command);
    interfaces.emplace_back(channel.joint_name, "effort", &channel.effort_command);
  }
  return interfaces;
}

}  // namespace onrobot_driver

PLUGINLIB_EXPORT_CLASS(
  onrobot_driver::VGC10HardwareInterface,
  hardware_interface::ActuatorInterface)
