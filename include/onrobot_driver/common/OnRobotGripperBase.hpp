// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__ONROBOTGRIPPERBASE_HPP_
#define ONROBOT_DRIVER__COMMON__ONROBOTGRIPPERBASE_HPP_

#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "onrobot_driver/common/IModbusConnection.hpp"
#include "onrobot_driver/common/gripper_settings.hpp"

namespace onrobot_driver
{

/// What every OnRobot gripper shares: the Modbus connection, with recovery, and the register
/// helpers.
class OnRobotGripperBase
{
public:
  explicit OnRobotGripperBase(int device_address);
  virtual ~OnRobotGripperBase();

  /// Open the connection, retrying until it succeeds or keep_running returns false, then write the
  /// start-up values of the gripper.
  void connect(const ConnectionSettings & settings, std::function<bool()> keep_running = nullptr);
  /// Use a connection built by the caller (tests, other transports). It is never reopened.
  void attach(std::unique_ptr<IModbusConnection> connection);
  bool reconnect();
  void close();
  bool isConnected() const;

  uint64_t getTotalRequests() const {return total_requests_.load();}
  uint64_t getFailedRequests() const {return failed_requests_.load();}
  uint64_t getReconnectCount() const {return reconnect_count_.load();}
  double getLastRoundtripMs() const {return last_roundtrip_ms_.load();}
  const std::string & getTargetEndpoint() const {return target_endpoint_;}
  int getDeviceAddress() const {return device_address_;}
  const std::string & getConnectionType() const {return connection_type_;}

  /// Send a request; on failure reopen the connection once and repeat it.
  MB::ModbusResponse sendRequest(const MB::ModbusRequest & req);

  /// Power cycle of the tool through the Compute Box at the given Modbus address.
  void resetToolPower(int compute_box_address);

  virtual float getWidth() = 0;
  virtual float getForce() = 0;
  virtual uint16_t getStatusRaw() = 0;
  virtual std::vector<int> getStatus();

  virtual void setTargetWidth(float width_val) = 0;
  virtual void setTargetForce(float force_val) = 0;
  virtual void setTargetSpeed(float speed_val) = 0;
  virtual void stop() = 0;
  virtual void moveGripper(float width_val) = 0;

  virtual float getMinWidth() const = 0;
  virtual float getMaxWidth() const = 0;
  virtual float getMaxForce() const = 0;

  /// Registers hold lengths in 1/10 mm; a length is rounded to the nearest one.
  static float fromTenthMM(uint16_t tenth_mm) {return static_cast<float>(tenth_mm) / 10000.0f;}
  static uint16_t toTenthMM(float meters)
  {
    return static_cast<uint16_t>(std::lround(meters * 10000.0f));
  }

protected:
  /// Called once a connection is in place (opened or attached).
  virtual void onAttached() {}
  /// Called after onAttached() when the connection was opened by connect().
  virtual void applyDefaults() {}

  /// Read one holding register. Throws MB::ModbusException.
  uint16_t readRegister(uint16_t address);
  /// Read one holding register; on a Modbus error report "Failed to read <what>." and return false.
  bool tryReadRegister(uint16_t address, const char * what, uint16_t & value);
  /// Write one holding register; on a Modbus error report "Failed to <what>." and throw it again.
  void writeRegister(uint16_t address, uint16_t value, const char * what);

  int device_address_;

private:
  std::unique_ptr<IModbusConnection> open() const;
  bool canReopen() const;

  std::unique_ptr<IModbusConnection> connection_;
  ConnectionSettings settings_;
  std::string connection_type_;
  std::string target_endpoint_;

  std::atomic<uint64_t> total_requests_{0};
  std::atomic<uint64_t> failed_requests_{0};
  std::atomic<uint64_t> reconnect_count_{0};
  std::atomic<double> last_roundtrip_ms_{0.0};

  mutable std::mutex comm_mutex_;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__COMMON__ONROBOTGRIPPERBASE_HPP_
