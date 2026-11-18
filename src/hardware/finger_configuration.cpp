// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "onrobot_driver/hardware/finger_hardware_interface.hpp"

namespace onrobot_driver
{

void FingerHardwareInterface::loadParameters(const HardwareParameters & parameters)
{
  min_width_ = parameters.number("min_width");
  max_width_ = parameters.number("max_width");
  max_force_ = parameters.number("max_force");
  settings_.min_width = static_cast<float>(min_width_);
  settings_.max_width = static_cast<float>(max_width_);
  settings_.max_force = static_cast<float>(max_force_);
  settings_.default_force = static_cast<float>(parameters.number("default_force"));
  settings_.default_speed = static_cast<float>(parameters.number("default_speed"));
  tuning_.configure_attempts = parameters.integer("configure_attempts");
  tuning_.configure_retry_wait_ms = parameters.integer("configure_retry_wait_ms");
  tuning_.force_resend_threshold = parameters.number("force_resend_threshold");
  tuning_.width_resend_threshold = parameters.number("width_resend_threshold");
  tuning_.nominal_read_period_s = parameters.number("nominal_read_period_s");
  tuning_.range_warning_period_ms = parameters.integer("range_warning_period_ms");
  tuning_.fake_smoothing = parameters.number("fake_smoothing");
  tuning_.fake_settled_threshold = parameters.number("fake_settled_threshold");
  tuning_.fake_busy_threshold = parameters.number("fake_busy_threshold");
  if (tuning_.configure_attempts < 1) {
    throw ParameterError("[Fail-Fast] Hardware parameter 'configure_attempts' must be at least 1");
  }
  if (info_.joints.size() != 1) {
    throw ParameterError(
            "[Fail-Fast] A finger gripper has exactly 1 joint in its <ros2_control> block, got " +
            std::to_string(info_.joints.size()));
  }
  joint_name_ = info_.joints[0].name;
  effort_command_ = settings_.default_force;
}

GripperHardwareBase::CallbackReturn FingerHardwareInterface::configureFake()
{
  RCLCPP_INFO(logger(), "Using fake hardware for %s", onrobot_type_.c_str());
  width_state_ = max_width_ / 2.0;
  width_command_ = width_state_;
  velocity_state_ = 0.0;
  effort_state_ = 0.0;
  return CallbackReturn::SUCCESS;
}

bool FingerHardwareInterface::connectAndReadWidth(float & initial_width)
{
  // The Compute Box can be slow to answer the first requests after being idle: several attempts.
  const std::string endpoint = connection_.endpoint();
  for (int attempt = 1; attempt <= tuning_.configure_attempts && rclcpp::ok(); ++attempt) {
    try {
      gripper_ = createGripper(device_address_, settings_);
      if (!gripper_) {
        throw std::runtime_error("gripper instance is null");
      }
      openConnection(*gripper_);
      initial_width = gripper_->getWidth();
      if (initial_width < 0) {
        throw std::runtime_error("could not read initial width");
      }
      return true;
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        logger(), "Attempt %d/%d to configure %s at %s (unit %d, timeout %d ms) failed: %s",
        attempt, tuning_.configure_attempts, onrobot_type_.c_str(), endpoint.c_str(),
        device_address_, connection_.timeout_ms, error.what());
      destroyGripper();
      if (attempt < tuning_.configure_attempts) {
        std::this_thread::sleep_for(std::chrono::milliseconds(tuning_.configure_retry_wait_ms));
      }
    }
  }
  RCLCPP_ERROR(
    logger(), "Failed to configure %s at %s (unit %d) after %d attempts", onrobot_type_.c_str(),
    endpoint.c_str(), device_address_, tuning_.configure_attempts);
  return false;
}

GripperHardwareBase::CallbackReturn FingerHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (use_fake_hardware_) {
    return configureFake();
  }

  float initial_width = -1.0f;
  if (!connectAndReadWidth(initial_width)) {
    return CallbackReturn::ERROR;
  }

  try {
    width_state_ = initial_width;
    width_command_ = initial_width;
    velocity_state_ = 0.0;
    effort_state_ = 0.0;

    FingerState initial_state;
    initial_state.position = initial_width;
    initial_state.status = gripper_->getStatusRaw();
    state_buffer_.initRT(initial_state);

    desired_width_.store(initial_width, std::memory_order_relaxed);
    desired_effort_.store(max_force_ / 2.0, std::memory_order_relaxed);
    new_command_.store(false, std::memory_order_relaxed);

    RCLCPP_INFO(
      logger(), "%s configured. Initial width: %.3f m. Limits: [%.3f, %.3f] m, Max force: %.1f N",
      onrobot_type_.c_str(), initial_width, min_width_, max_width_, max_force_);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger(), "Failed to configure %s: %s", onrobot_type_.c_str(), error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FingerHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.emplace_back(joint_name_, "position", &width_state_);
  interfaces.emplace_back(joint_name_, "velocity", &velocity_state_);
  interfaces.emplace_back(joint_name_, "effort", &effort_state_);
  interfaces.emplace_back(joint_name_, "grip_detected", &grip_detected_state_);
  interfaces.emplace_back(joint_name_, "busy", &busy_state_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
FingerHardwareInterface::export_command_interfaces()
{
  // max_effort and effort are two names of the same command: controllers differ in the one they
  // claim.
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.emplace_back(joint_name_, "position", &width_command_);
  interfaces.emplace_back(joint_name_, "max_effort", &effort_command_);
  interfaces.emplace_back(joint_name_, "effort", &effort_command_);
  return interfaces;
}

}  // namespace onrobot_driver
