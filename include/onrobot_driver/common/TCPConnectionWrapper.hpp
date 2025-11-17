#pragma once
#include "IModbusConnection.hpp"
#include "MB/TCP/connection.hpp"
#include <string>

class TCPConnectionWrapper : public IModbusConnection {
public:
    TCPConnectionWrapper(const std::string &ip, int port) 
      : connection(MB::TCP::Connection::with(ip, port)) {}

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            connection.sendRequest(req);
            return connection.awaitResponse();
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // Connection cleanup if needed
    }

private:
    MB::TCP::Connection connection;
};