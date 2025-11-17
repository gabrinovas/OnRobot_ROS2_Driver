#pragma once
#include "IModbusConnection.hpp"
#include "MB/Serial/connection.hpp"
#include <string>
#include <tuple>

class SerialConnectionWrapper : public IModbusConnection {
public:
    SerialConnectionWrapper(const std::string &device)
    : connection(device) {
        connection.connect();
        connection.setTwoStopBits(false);
        connection.setEvenParity();
        connection.setBaudRate(1000000); // 1,000,000 bps per documentation
        connection.setTimeout(1000);
    }

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            connection.send(req.toRaw());
            auto [response, rawData] = connection.awaitResponse();
            return response;
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // Connection cleanup if needed
    }

private:
    MB::Serial::Connection connection;
};