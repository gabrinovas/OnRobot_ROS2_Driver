// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__HARDWARE__GRIPPER_HARDWARE_BASE_HPP_
#define ONROBOT_DRIVER__HARDWARE__GRIPPER_HARDWARE_BASE_HPP_

#include <atomic>
#include <exception>
#include <string>
#include <thread>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "onrobot_driver/common/OnRobotGripperBase.hpp"
#include "onrobot_driver/hardware/hardware_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace onrobot_driver
{

/// What the ros2_control plugins of every OnRobot gripper share: the parameters of the connection
/// and a worker thread that talks Modbus, so that read() and write(), called by the real-time loop,
/// never wait for the bus.
class GripperHardwareBase : public hardware_interface::ActuatorInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;

  ~GripperHardwareBase() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

protected:
  /// Read the parameters of the concrete gripper and check its joints; throw ParameterError when
  /// wrong.
  virtual void loadParameters(const HardwareParameters & parameters) = 0;
  virtual OnRobotGripperBase * gripper() = 0;
  virtual void destroyGripper() = 0;
  /// Give the gripper its connection. Tests replace it to attach a connection of their own.
  virtual void openConnection(OnRobotGripperBase & gripper);
  /// One cycle of the worker: send the pending command, read the state. May throw on a bus error.
  virtual void communicate() = 0;
  /// Called in the worker thread before its first cycle.
  virtual void beforeCommunicating() {}
  virtual void onCommunicationError(const std::exception & /*error*/) {}
  /// The connection is back: forget what was sent so that the next command is sent again.
  virtual void onCommunicationRestored() = 0;
  /// Called when the hardware is deactivated, after the worker stopped.
  virtual void afterDeactivating() {}
  virtual const char * loggerName() const = 0;

  rclcpp::Logger logger() const {return rclcpp::get_logger(loggerName());}
  void startWorker();
  void stopWorker();

  std::string onrobot_type_;
  ConnectionSettings connection_;
  int device_address_{0};
  bool use_fake_hardware_{false};
  int worker_period_ms_{0};
  int reconnect_wait_ms_{0};
  std::atomic<bool> comm_healthy_{true};

private:
  void workerLoop();

  std::thread worker_thread_;
  std::atomic<bool> worker_running_{false};
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__HARDWARE__GRIPPER_HARDWARE_BASE_HPP_
