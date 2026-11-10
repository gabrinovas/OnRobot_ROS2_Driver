#pragma once
#include "IModbusConnection.hpp"
#include "MB/TCP/connection.hpp"
#include <poll.h>
#include <string>

class TCPConnectionWrapper : public IModbusConnection {
public:
    // The bundled Modbus library hard-codes a 500 ms response timeout (MB::TCP::Connection::DefaultTCPTimeout)
    // and exposes no setter. OnRobot Compute Boxes may need >800 ms to answer the first requests after
    // being idle, so we wait for the response ourselves with a configurable (longer) timeout.
    static constexpr int DefaultTimeoutMs = 2000;

    TCPConnectionWrapper(const std::string &ip, int port, int timeout_ms = DefaultTimeoutMs)
      : connection(MB::TCP::Connection::with(ip, port)),
        timeout_ms_(timeout_ms > 0 ? timeout_ms : DefaultTimeoutMs) {}

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        connection.sendRequest(req);

        // Wait for the response with our own timeout. Once data is readable, awaitResponse()'s
        // internal 500 ms poll returns immediately. On timeout the caller must drop this connection
        // (a late reply would otherwise be read as the answer to the next request).
        pollfd pfd{};
        pfd.fd = connection.getSockfd();
        pfd.events = POLLIN;
        if (::poll(&pfd, 1, timeout_ms_) <= 0) {
            throw MB::ModbusException(MB::utils::Timeout);
        }
        return connection.awaitResponse();
    }

    void close() override {
        // Connection cleanup if needed
    }

    int timeoutMs() const { return timeout_ms_; }

private:
    MB::TCP::Connection connection;
    int timeout_ms_;
};
