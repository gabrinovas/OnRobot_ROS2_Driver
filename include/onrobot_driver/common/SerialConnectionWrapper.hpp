// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__SERIALCONNECTIONWRAPPER_HPP_
#define ONROBOT_DRIVER__COMMON__SERIALCONNECTIONWRAPPER_HPP_

#include <string>
#include <tuple>

#include "MB/Serial/connection.hpp"
#include "onrobot_driver/common/IModbusConnection.hpp"

/// Modbus RTU connection over the RS485 line of the robot tool connector.
class SerialConnectionWrapper : public IModbusConnection
{
public:
  /// Line settings fixed by the OnRobot tool protocol: 1 Mbit/s, even parity, one stop bit.
  static constexpr int ToolBaudRate = 1000000;

  SerialConnectionWrapper(const std::string & device, int timeout_ms)
  : connection_(device)
  {
    connection_.connect();
    connection_.setTwoStopBits(false);
    connection_.setEvenParity();
    connection_.setBaudRate(ToolBaudRate);
    connection_.setTimeout(timeout_ms);
  }

  MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) override
  {
    connection_.send(req.toRaw());
    auto [response, raw_data] = connection_.awaitResponse();
    return response;
  }

  void close() override {}

private:
  MB::Serial::Connection connection_;
};

#endif  // ONROBOT_DRIVER__COMMON__SERIALCONNECTIONWRAPPER_HPP_
