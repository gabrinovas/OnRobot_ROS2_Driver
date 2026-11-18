// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#include "onrobot_driver/monitor/gripper_status_monitor.hpp"

namespace onrobot_driver
{

GripperStatusMonitor::GripperStatusMonitor()
: Node("gripper_status_monitor"), updater_(this)
{
  onrobot_type_ = required<std::string>("onrobot_type");
  gripper_kind_ = required<std::string>("gripper_kind");
  joints_ = required<std::vector<std::string>>("joints");
  connection_.type = required<std::string>("connection_type");
  connection_.ip_address = required<std::string>("ip_address");
  connection_.port = static_cast<int>(required<int64_t>("port"));
  connection_.device = required<std::string>("device");
  connection_.timeout_ms = static_cast<int>(required<int64_t>("timeout_ms"));
  connection_.serial_timeout_ms = static_cast<int>(required<int64_t>("serial_timeout_ms"));
  compute_box_address_ = static_cast<int>(required<int64_t>("compute_box_address"));
  use_fake_hardware_ = required<bool>("use_fake_hardware");
  thresholds_.telemetry_warning_age_s = required<double>("telemetry_warning_age_s");
  thresholds_.telemetry_error_age_s = required<double>("telemetry_error_age_s");
  thresholds_.gripping_effort_n = required<double>("finger.gripping_effort_n");
  thresholds_.moving_velocity_m_s = required<double>("finger.moving_velocity_m_s");
  thresholds_.vacuum_active = required<double>("vacuum.active_threshold");
  thresholds_.vacuum_drop_from = required<double>("vacuum.drop_from");
  thresholds_.vacuum_drop_to = required<double>("vacuum.drop_to");
  const auto queue_depth = static_cast<size_t>(required<int64_t>("queue_depth"));
  const auto period = std::chrono::duration<double>(required<double>("publish_period_s"));

  const size_t expected_joints = isVacuum() ? 2 : 1;
  if ((gripper_kind_ != "finger" && gripper_kind_ != "vacuum") ||
    joints_.size() != expected_joints)
  {
    const std::string joint_count = std::to_string(joints_.size());
    throw std::runtime_error(
            "[Fail-Fast] gripper_kind must be finger (1 joint) or vacuum (2 joints); got '" +
            gripper_kind_ + "' with " + joint_count + " joints");
  }
  position_.assign(joints_.size(), 0.0);
  velocity_.assign(joints_.size(), 0.0);
  effort_.assign(joints_.size(), 0.0);
  previous_position_.assign(joints_.size(), 0.0);

  updater_.setHardwareID("OnRobot " + onrobot_type_);
  updater_.add("Gripper Telemetry & Health", this, &GripperStatusMonitor::produceDiagnostics);

  status_publisher_ = create_publisher<std_msgs::msg::String>(
    required<std::string>("status_topic"), queue_depth);
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    required<std::string>("joint_states_topic"), queue_depth,
    std::bind(&GripperStatusMonitor::onJointState, this, std::placeholders::_1));
  reset_power_service_ = create_service<Trigger>(
    required<std::string>("reset_power_service"),
    std::bind(
      &GripperStatusMonitor::onResetPower, this, std::placeholders::_1,
      std::placeholders::_2));
  timer_ = create_wall_timer(period, std::bind(&GripperStatusMonitor::onTimer, this));

  RCLCPP_INFO(
    get_logger(), "Gripper Status Monitor started for '%s' (conn=%s, fake=%s)",
    onrobot_type_.c_str(), connection_.type.c_str(), use_fake_hardware_ ? "true" : "false");
}

void GripperStatusMonitor::onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  bool found = false;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t joint = 0; joint < joints_.size(); ++joint) {
      if (msg->name[i] != joints_[joint]) {
        continue;
      }
      if (i < msg->position.size()) {
        position_[joint] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        velocity_[joint] = msg->velocity[i];
      }
      if (i < msg->effort.size()) {
        effort_[joint] = msg->effort[i];
      }
      found = true;
    }
  }
  if (found) {
    last_joint_state_time_ = now();
    has_joint_state_ = true;
  }
}

void GripperStatusMonitor::onTimer()
{
  updater_.force_update();

  std_msgs::msg::String message;
  if (!has_joint_state_) {
    message.data = "OnRobot " + onrobot_type_ + ": Initializing...";
  } else {
    char text[256];
    if (isVacuum()) {
      snprintf(
        text, sizeof(text), "OnRobot %s: Channel A=%.1f%%, Channel B=%.1f%%", onrobot_type_.c_str(),
        position_[0] * 100.0, position_[1] * 100.0);
    } else {
      snprintf(
        text, sizeof(text), "OnRobot %s: width=%.1fmm, effort=%.1fN", onrobot_type_.c_str(),
        position_[0] * 1000.0, effort_[0]);
    }
    message.data = text;
  }
  status_publisher_->publish(message);
}

}  // namespace onrobot_driver
