// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__MONITOR__GRIPPER_STATUS_MONITOR_HPP_
#define ONROBOT_DRIVER__MONITOR__GRIPPER_STATUS_MONITOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "onrobot_driver/common/gripper_settings.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace onrobot_driver
{

/// Thresholds of the diagnostics. All are parameters of the node
/// (config/gripper_status_monitor.yaml).
struct MonitorThresholds
{
  double telemetry_warning_age_s;  ///< joint states older than this: warning
  double telemetry_error_age_s;    ///< joint states older than this: error
  double gripping_effort_n;        ///< finger: effort above which a still gripper is holding a part
  double moving_velocity_m_s;      ///< finger: speed above which the gripper is moving
  double vacuum_active;            ///< vacuum: level above which a channel is working
  double vacuum_drop_from;         ///< vacuum: a fall from above this...
  double vacuum_drop_to;           ///< ...to below this between two reports is a lost part
};

/// Watches the joint states of one gripper, publishes its condition as diagnostics and as a text
/// line, and offers a service that power cycles the tool through the Compute Box.
class GripperStatusMonitor : public rclcpp::Node
{
public:
  GripperStatusMonitor();

private:
  using Trigger = std_srvs::srv::Trigger;

  /// Value of a parameter that must be set; throws a [Fail-Fast] error naming it when it is not.
  template<typename T>
  T required(const std::string & name);

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTimer();
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void fingerDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void vacuumDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  void onResetPower(
    const std::shared_ptr<Trigger::Request> request, std::shared_ptr<Trigger::Response> response);
  bool isVacuum() const {return gripper_kind_ == "vacuum";}

  std::string onrobot_type_;
  std::string gripper_kind_;         // finger or vacuum
  std::vector<std::string> joints_;  // finger: its joint. vacuum: channel A, channel B
  ConnectionSettings connection_;
  int compute_box_address_;
  bool use_fake_hardware_;
  MonitorThresholds thresholds_;

  // Last values of the joints, in the order of joints_.
  std::vector<double> position_;
  std::vector<double> velocity_;
  std::vector<double> effort_;
  std::vector<double> previous_position_;
  bool has_joint_state_{false};
  rclcpp::Time last_joint_state_time_;

  diagnostic_updater::Updater updater_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Service<Trigger>::SharedPtr reset_power_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

template<typename T>
T GripperStatusMonitor::required(const std::string & name)
{
  // A required parameter is one the launch file (or a parameter file) gives a value to.
  const auto & given = get_node_parameters_interface()->get_parameter_overrides();
  const auto found = given.find(name);
  if (found != given.end()) {
    if (!has_parameter(name)) {
      declare_parameter(name, found->second);
    }
    return get_parameter(name).get_value<T>();
  }
  throw std::runtime_error(
          "[Fail-Fast] Parameter '" + name + "' of node '" + get_fully_qualified_name() +
          "' is not set");
}

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__MONITOR__GRIPPER_STATUS_MONITOR_HPP_
