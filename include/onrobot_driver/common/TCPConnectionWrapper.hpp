// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__COMMON__TCPCONNECTIONWRAPPER_HPP_
#define ONROBOT_DRIVER__COMMON__TCPCONNECTIONWRAPPER_HPP_

#include <poll.h>

#include <string>

#include "MB/TCP/connection.hpp"
#include "onrobot_driver/common/IModbusConnection.hpp"

/// Modbus TCP connection with a response timeout chosen by the caller. The bundled Modbus library
/// waits 500 ms for a response and offers no setter. OnRobot Compute Boxes may need more than 800
/// ms to answer the first requests after being idle, so the response is awaited here.
class TCPConnectionWrapper : public IModbusConnection
{
public:
  TCPConnectionWrapper(const std::string & ip, int port, int timeout_ms)
  : connection_(MB::TCP::Connection::with(ip, port)), timeout_ms_(timeout_ms) {}

  MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) override
  {
    connection_.sendRequest(req);

    // Once data is readable, the 500 ms poll inside awaitResponse() returns at once. On timeout the
    // caller must drop this connection: a late reply would otherwise be read as the answer to the
    // next request.
    pollfd descriptor{};
    descriptor.fd = connection_.getSockfd();
    descriptor.events = POLLIN;
    if (::poll(&descriptor, 1, timeout_ms_) <= 0) {
      throw MB::ModbusException(MB::utils::Timeout);
    }
    return connection_.awaitResponse();
  }

  void close() override {}

  int timeoutMs() const {return timeout_ms_;}

private:
  MB::TCP::Connection connection_;
  int timeout_ms_;
};

#endif  // ONROBOT_DRIVER__COMMON__TCPCONNECTIONWRAPPER_HPP_
