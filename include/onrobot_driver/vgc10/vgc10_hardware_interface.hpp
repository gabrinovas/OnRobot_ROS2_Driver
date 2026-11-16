// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__VGC10__VGC10_HARDWARE_INTERFACE_HPP_
#define ONROBOT_DRIVER__VGC10__VGC10_HARDWARE_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include "onrobot_driver/hardware/gripper_hardware_base.hpp"
#include "onrobot_driver/vgc10/VGC10.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace onrobot_driver
{

/// Vacuum of the two channels as the worker thread hands it to the real-time loop (0 released, 1
/// full).
struct VacuumState
{
  std::array<double, 2> vacuum{{0.0, 0.0}};
};

/// Values that tune the plugin of the vacuum gripper. All come from the <ros2_control> block of the
/// URDF.
struct VacuumTuning
{
  double resend_threshold;         ///< a vacuum command this close to the last one sent is not sent
  /// a command above this asks for vacuum; at or below it, for release
  double grip_threshold;
  double initial_effort;           ///< vacuum (%) asked for until a controller writes an effort
  /// period assumed by read() when the controller manager gives none
  double nominal_read_period_s;
  int error_warning_period_ms;     ///< at most one bus error warning in this time
  /// simulated gripper: share of the remaining vacuum reached per cycle
  double fake_smoothing;
};

/// One vacuum channel as seen by ros2_control: a virtual joint whose position is the vacuum.
struct VacuumChannelInterface
{
  std::string joint_name;
  double state{0.0};
  double velocity{0.0};
  double effort{0.0};        ///< vacuum, %
  double level{0.0};         ///< vacuum, 0 to 1 (same as state, under the name vacuum_level)
  double pressure_kpa{0.0};  ///< relative pressure
  double command{0.0};
  double effort_command{0.0};
  std::atomic<double> desired{0.0};         ///< real-time loop writes, worker reads
  std::atomic<double> desired_effort{0.0};
  double last_sent{-1.0};                   ///< worker thread only
};

/// ros2_control plugin of the OnRobot VGC10: two virtual joints, one per vacuum channel (A, then
/// B).
class VGC10HardwareInterface : public GripperHardwareBase
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
  void loadParameters(const HardwareParameters & parameters) override;
  OnRobotGripperBase * gripper() override {return gripper_.get();}
  void destroyGripper() override {gripper_.reset();}
  void beforeCommunicating() override;
  void communicate() override;
  void onCommunicationError(const std::exception & error) override;
  void onCommunicationRestored() override;
  void afterDeactivating() override;
  const char * loggerName() const override {return "VGC10HardwareInterface";}

  std::unique_ptr<VGC10> gripper_;
  VacuumSettings settings_{};
  VacuumTuning tuning_{};
  std::array<VacuumChannelInterface, 2> channels_;

private:
  void sendChannel(VGC10::Channel channel, VacuumChannelInterface & interface);

  realtime_tools::RealtimeBuffer<VacuumState> state_buffer_;  // worker writes, real-time loop reads
  std::atomic<bool> new_command_{false};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__VGC10__VGC10_HARDWARE_INTERFACE_HPP_
