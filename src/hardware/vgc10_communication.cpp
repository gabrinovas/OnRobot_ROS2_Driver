// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <cmath>

#include "onrobot_driver/vgc10/vgc10_hardware_interface.hpp"

namespace onrobot_driver
{

namespace
{
constexpr double kPercentPerUnit = 100.0;      // vacuum 0..1 shown as a percentage
// full relative vacuum is 100 kPa under ambient pressure
constexpr double kKilopascalPerUnit = -100.0;
}  // namespace

hardware_interface::return_type VGC10HardwareInterface::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (use_fake_hardware_) {
    // Simulated pump: the vacuum ramps towards the command.
    double dt = period.seconds();
    if (dt <= 0.0) {
      dt = tuning_.nominal_read_period_s;
    }
    for (auto & channel : channels_) {
      const double previous = channel.state;
      channel.state += (channel.command - channel.state) * tuning_.fake_smoothing;
      channel.velocity = (channel.state - previous) / dt;
      channel.effort = channel.state * settings_.max_vacuum_percent;
      channel.level = channel.state;
      channel.pressure_kpa = kKilopascalPerUnit * channel.state;
    }
    return hardware_interface::return_type::OK;
  }

  // The worker thread keeps this buffer up to date: reading it never waits for the bus.
  const VacuumState * state = state_buffer_.readFromRT();
  if (state) {
    for (size_t i = 0; i < channels_.size(); ++i) {
      channels_[i].state = state->vacuum[i];
      channels_[i].effort = static_cast<double>(static_cast<float>(state->vacuum[i]) * 100.0f);
      channels_[i].level = state->vacuum[i];
      channels_[i].pressure_kpa = kKilopascalPerUnit * state->vacuum[i];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VGC10HardwareInterface::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (use_fake_hardware_) {
    return hardware_interface::return_type::OK;
  }
  // Handed to the worker thread without locking; the vacuum command is limited to 0..1.
  for (auto & channel : channels_) {
    channel.desired.store(std::max(0.0, std::min(channel.command, 1.0)), std::memory_order_release);
    channel.desired_effort.store(channel.effort_command, std::memory_order_release);
  }
  new_command_.store(true, std::memory_order_release);
  return hardware_interface::return_type::OK;
}

void VGC10HardwareInterface::beforeCommunicating()
{
  onCommunicationRestored();
}

void VGC10HardwareInterface::onCommunicationRestored()
{
  for (auto & channel : channels_) {
    channel.last_sent = -1.0;
  }
}

void VGC10HardwareInterface::onCommunicationError(const std::exception & error)
{
  RCLCPP_WARN_THROTTLE(
    logger(), clock_, tuning_.error_warning_period_ms,
    "Modbus communication error with %s: %s. Attempting background recovery...",
    onrobot_type_.c_str(),
    error.what());
}

void VGC10HardwareInterface::sendChannel(VGC10::Channel channel, VacuumChannelInterface & interface)
{
  const double vacuum = interface.desired.load(std::memory_order_acquire);
  const double effort = interface.desired_effort.load(std::memory_order_acquire);
  if (std::abs(vacuum - interface.last_sent) <= tuning_.resend_threshold) {
    return;
  }
  if (vacuum > tuning_.grip_threshold) {
    // An effort command is the vacuum wanted, in %; without one the vacuum command is scaled to the
    // range.
    const double highest = settings_.max_vacuum_percent;
    const double percent = std::min(effort > 0.0 ? effort : (vacuum * highest), highest);
    gripper_->setChannelControl(channel, VGC10::MODE_GRIP, static_cast<uint8_t>(percent));
  } else {
    gripper_->setChannelControl(channel, VGC10::MODE_RELEASE, 0);
  }
  interface.last_sent = vacuum;
}

void VGC10HardwareInterface::communicate()
{
  // 1. Send the pending command, skipping what the gripper already has.
  if (new_command_.exchange(false, std::memory_order_acq_rel)) {
    sendChannel(VGC10::Channel::A, channels_[0]);
    sendChannel(VGC10::Channel::B, channels_[1]);
  }

  // 2. Read both channels in one transaction and publish them to the real-time loop.
  float vacuum_a = 0.0f;
  float vacuum_b = 0.0f;
  if (gripper_->readBothVacuums(vacuum_a, vacuum_b)) {
    VacuumState state;
    state.vacuum = {static_cast<double>(vacuum_a), static_cast<double>(vacuum_b)};
    state_buffer_.writeFromNonRT(state);
  }
}

}  // namespace onrobot_driver
