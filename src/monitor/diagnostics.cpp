// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <cmath>
#include <cstdio>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "onrobot_driver/monitor/gripper_status_monitor.hpp"

namespace onrobot_driver
{

using diagnostic_msgs::msg::DiagnosticStatus;

void GripperStatusMonitor::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  status.add("Gripper Model", onrobot_type_);
  status.add("Connection Type", connection_.type);
  status.add("Endpoint", connection_.endpoint());
  status.add("Hardware Mode", use_fake_hardware_ ? "Simulation (Fake Hardware)" : "Physical Robot");

  if (!has_joint_state_) {
    status.add("Telemetry Age (s)", "no telemetry yet");
    status.summary(DiagnosticStatus::WARN, "Waiting for initial gripper telemetry");
    return;
  }
  const double age = (now() - last_joint_state_time_).seconds();
  status.addf("Telemetry Age (s)", "%.3f", age);
  if (age > thresholds_.telemetry_error_age_s) {
    status.summaryf(
      DiagnosticStatus::ERROR, "Communication timeout (> %.0fs without joint_states)",
      thresholds_.telemetry_error_age_s);
    return;
  }
  if (age > thresholds_.telemetry_warning_age_s) {
    status.summaryf(
      DiagnosticStatus::WARN, "Delayed telemetry update (> %.0fs)",
      thresholds_.telemetry_warning_age_s);
    return;
  }

  if (isVacuum()) {
    vacuumDiagnostics(status);
  } else {
    fingerDiagnostics(status);
  }
}

void GripperStatusMonitor::vacuumDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const double a = position_[0];
  const double b = position_[1];
  status.addf("Channel A Vacuum (%)", "%.1f %%", a * 100.0);
  status.addf("Channel B Vacuum (%)", "%.1f %%", b * 100.0);

  const auto dropped = [this](size_t channel) {
      return previous_position_[channel] > thresholds_.vacuum_drop_from &&
             position_[channel] < thresholds_.vacuum_drop_to;
    };
  const bool a_active = a > thresholds_.vacuum_active;
  const bool b_active = b > thresholds_.vacuum_active;

  if (dropped(0)) {
    status.summary(DiagnosticStatus::ERROR, "ALERT: Vacuum loss / object dropped on Channel A!");
  } else if (dropped(1)) {
    status.summary(DiagnosticStatus::ERROR, "ALERT: Vacuum loss / object dropped on Channel B!");
  } else if (a_active && b_active) {
    status.summaryf(
      DiagnosticStatus::OK, "Dual-Channel Active: A=%.0f%%, B=%.0f%%", a * 100.0,
      b * 100.0);
  } else if (a_active) {
    status.summaryf(DiagnosticStatus::OK, "Channel A Active: %.0f%%", a * 100.0);
  } else if (b_active) {
    status.summaryf(DiagnosticStatus::OK, "Channel B Active: %.0f%%", b * 100.0);
  } else {
    status.summary(DiagnosticStatus::OK, "Vacuum gripper Release / Standby");
  }
  previous_position_ = position_;
}

void GripperStatusMonitor::fingerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const double width = position_[0];
  const double velocity = velocity_[0];
  const double effort = effort_[0];
  status.addf("Width (mm)", "%.2f", width * 1000.0);
  status.addf("Velocity (mm/s)", "%.2f", velocity * 1000.0);
  status.addf("Effort (N)", "%.1f", effort);

  const bool moving = std::abs(velocity) > thresholds_.moving_velocity_m_s;
  const bool still = std::abs(velocity) < thresholds_.moving_velocity_m_s;
  if (std::abs(effort) > thresholds_.gripping_effort_n && still) {
    status.summaryf(
      DiagnosticStatus::OK, "Part Gripped: width=%.1fmm, effort=%.1fN", width * 1000.0, effort);
  } else if (moving) {
    status.summary(DiagnosticStatus::OK, "Gripper in motion");
  } else {
    status.summaryf(DiagnosticStatus::OK, "Gripper Standby at %.1fmm", width * 1000.0);
  }
}

}  // namespace onrobot_driver
