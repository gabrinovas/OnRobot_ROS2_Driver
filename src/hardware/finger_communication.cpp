// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>

#include "onrobot_driver/hardware/finger_hardware_interface.hpp"

namespace onrobot_driver
{

namespace
{
// The 2FG7 and the 3FG15 report these two conditions in the same bits of their status register.
constexpr uint16_t kStatusBusy = 0x0001;
constexpr uint16_t kStatusGripDetected = 0x0002;
}  // namespace

void FingerHardwareInterface::simulate(double dt)
{
  const double previous = width_state_;
  width_state_ += static_cast<float>((width_command_ - width_state_) * tuning_.fake_smoothing);
  velocity_state_ = (width_state_ - previous) / dt;
  const double remaining = std::abs(width_command_ - width_state_);
  effort_state_ = remaining < tuning_.fake_settled_threshold ? effort_command_ : 0.0;
  busy_state_ = remaining > tuning_.fake_busy_threshold ? 1.0 : 0.0;
  const bool inside = width_command_<max_width_ && width_command_> min_width_;
  grip_detected_state_ = (busy_state_ == 0.0 && inside) ? 1.0 : 0.0;
}

hardware_interface::return_type FingerHardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (use_fake_hardware_) {
    const double dt = period.seconds();
    simulate(dt > 0.0 ? dt : tuning_.nominal_read_period_s);
    return hardware_interface::return_type::OK;
  }

  // The worker thread keeps this buffer up to date: reading it never waits for the bus.
  const FingerState * state = state_buffer_.readFromRT();
  if (state) {
    width_state_ = state->position;
    velocity_state_ = state->velocity;
    effort_state_ = state->effort;
    grip_detected_state_ = (state->status & kStatusGripDetected) ? 1.0 : 0.0;
    busy_state_ = (state->status & kStatusBusy) ? 1.0 : 0.0;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FingerHardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (use_fake_hardware_) {
    return hardware_interface::return_type::OK;
  }

  if (width_command_ < min_width_ || width_command_ > max_width_) {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, tuning_.range_warning_period_ms,
      "Command %.3f m out of range [%.3f, %.3f] for %s", width_command_, min_width_, max_width_,
      onrobot_type_.c_str());
    return hardware_interface::return_type::OK;
  }

  // Handed to the worker thread without locking.
  desired_width_.store(width_command_, std::memory_order_release);
  desired_effort_.store(effort_command_, std::memory_order_release);
  new_command_.store(true, std::memory_order_release);
  return hardware_interface::return_type::OK;
}

void FingerHardwareInterface::beforeCommunicating()
{
  last_width_ = width_state_;
  last_time_ = std::chrono::steady_clock::now();
  last_sent_width_ = -1.0;
  last_sent_effort_ = -1.0;
}

void FingerHardwareInterface::onCommunicationRestored()
{
  last_sent_width_ = -1.0;
  last_sent_effort_ = -1.0;
}

void FingerHardwareInterface::communicate()
{
  // 1. Send the pending command, skipping what the gripper already has.
  if (new_command_.exchange(false, std::memory_order_acq_rel)) {
    const double width = desired_width_.load(std::memory_order_acquire);
    const double effort = desired_effort_.load(std::memory_order_acquire);
    if (effort > 0.0 && std::abs(effort - last_sent_effort_) > tuning_.force_resend_threshold) {
      gripper_->setTargetForce(static_cast<float>(effort));
      last_sent_effort_ = effort;
    }
    if (std::abs(width - last_sent_width_) > tuning_.width_resend_threshold) {
      gripper_->moveGripper(static_cast<float>(width));
      last_sent_width_ = width;
    }
  }

  // 2. Read the state of the gripper.
  const float width = gripper_->getWidth();
  const float force = gripper_->getForce();
  const uint16_t status = gripper_->getStatusRaw();

  const auto now = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(now - last_time_).count();
  if (dt <= 0.0) {
    dt = worker_period_ms_ / 1000.0;
  }
  double velocity = 0.0;
  if (width >= 0.0f) {
    velocity = (static_cast<double>(width) - last_width_) / dt;
    last_width_ = static_cast<double>(width);
  }
  last_time_ = now;

  // 3. Publish it to the real-time loop. A failed read (negative value) keeps the last good width.
  FingerState state;
  state.position = (width >= 0.0f) ? static_cast<double>(width) : last_width_;
  state.velocity = velocity;
  state.effort = (force >= 0.0f) ? static_cast<double>(force) : 0.0;
  state.status = status;
  state_buffer_.writeFromNonRT(state);
}

}  // namespace onrobot_driver
