// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__HARDWARE__FINGER_HARDWARE_INTERFACE_HPP_
#define ONROBOT_DRIVER__HARDWARE__FINGER_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "onrobot_driver/hardware/gripper_hardware_base.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace onrobot_driver
{

/// State of a finger gripper as the worker thread hands it to the real-time loop.
struct FingerState
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  uint16_t status{0};
};

/// Values that tune the plugin of a finger gripper. All come from the <ros2_control> block of the
/// URDF.
struct FingerTuning
{
  int configure_attempts;             ///< attempts to connect and read the first width
  int configure_retry_wait_ms;        ///< wait between two of those attempts
  /// N: a force command this close to the last one sent is not sent
  double force_resend_threshold;
  double width_resend_threshold;      ///< m: same for the width
  /// period assumed by read() when the controller manager gives none
  double nominal_read_period_s;
  int range_warning_period_ms;        ///< at most one "out of range" warning in this time
  /// simulated gripper: share of the remaining travel covered per cycle
  double fake_smoothing;
  /// simulated gripper: distance to the target under which it has arrived
  double fake_settled_threshold;
  /// simulated gripper: distance to the target over which it is moving
  double fake_busy_threshold;
};

/// ros2_control plugin of a finger gripper: one joint, the distance between the fingers.
/// Concrete grippers only say which protocol class talks to the device.
class FingerHardwareInterface : public GripperHardwareBase
{
public:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

protected:
  /// Build the protocol object of the concrete gripper (not connected yet).
  virtual std::unique_ptr<OnRobotGripperBase> createGripper(
    int device_address, const FingerSettings & settings) = 0;

  void loadParameters(const HardwareParameters & parameters) override;
  OnRobotGripperBase * gripper() override {return gripper_.get();}
  void destroyGripper() override {gripper_.reset();}
  void beforeCommunicating() override;
  void communicate() override;
  void onCommunicationRestored() override;
  const char * loggerName() const override {return "OnRobotHardwareInterface";}

  std::unique_ptr<OnRobotGripperBase> gripper_;
  // what the protocol object receives (single precision, as the device)
  FingerSettings settings_{};
  FingerTuning tuning_{};
  // The range as configured, in double precision: commands are compared with these, so that a
  // command equal to a limit is inside the range.
  double min_width_{0.0};
  double max_width_{0.0};
  double max_force_{0.0};
  std::string joint_name_;

  // Read and written by the real-time loop only.
  double width_state_{0.0};
  double velocity_state_{0.0};
  double effort_state_{0.0};
  double grip_detected_state_{0.0};
  double busy_state_{0.0};
  double width_command_{0.0};
  double effort_command_{0.0};

private:
  CallbackReturn configureFake();
  bool connectAndReadWidth(float & initial_width);
  void simulate(double dt);

  realtime_tools::RealtimeBuffer<FingerState> state_buffer_;  // worker writes, real-time loop reads
  std::atomic<double> desired_width_{0.0};                    // real-time loop writes, worker reads
  std::atomic<double> desired_effort_{0.0};
  std::atomic<bool> new_command_{false};
  rclcpp::Clock clock_{RCL_STEADY_TIME};

  // Used by the worker thread only.
  double last_width_{0.0};
  double last_sent_width_{-1.0};
  double last_sent_effort_{-1.0};
  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__HARDWARE__FINGER_HARDWARE_INTERFACE_HPP_
