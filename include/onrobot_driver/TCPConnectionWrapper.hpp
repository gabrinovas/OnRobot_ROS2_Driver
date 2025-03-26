#pragma once
#include "IModbusConnection.hpp"
#include "MB/TCP/connection.hpp"
#include <string>

class TCPConnectionWrapper : public IModbusConnection {
public:
    // Create a TCP connection using the library’s static with() method.
    TCPConnectionWrapper(const std::string &ip, int port) 
      : connection(MB::TCP::Connection::with(ip, port)) {}

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            // sendRequest returns raw bytes;
            // we then parse them into a response.
            std::vector<uint8_t> rawResp = connection.sendRequest(req);
            return MB::ModbusResponse::fromRaw(rawResp);
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // TCP connection cleanup is handled in its destructor.
        // You could also explicitly close the socket if needed.
    }

private:
    MB::TCP::Connection connection;
};
