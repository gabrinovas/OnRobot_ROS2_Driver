// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__IMODBUSCONNECTION_HPP_
#define ONROBOT_DRIVER__COMMON__IMODBUSCONNECTION_HPP_

#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"

/// Transport of Modbus requests to a gripper: TCP (Compute Box), serial (tool RS485) or a test
/// double.
class IModbusConnection
{
public:
  virtual ~IModbusConnection() = default;

  /// Send a request and return the parsed response.
  virtual MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) = 0;

  virtual void close() = 0;
};

#endif  // ONROBOT_DRIVER__COMMON__IMODBUSCONNECTION_HPP_
