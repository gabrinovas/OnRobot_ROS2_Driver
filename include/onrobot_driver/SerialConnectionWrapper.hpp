#pragma once
#include "IModbusConnection.hpp"
#include "MB/Serial/connection.hpp"
#include <string>
#include <tuple>

class SerialConnectionWrapper : public IModbusConnection {
public:
    // Create a Serial connection using the provided device path.
    SerialConnectionWrapper(const std::string &device)
      : connection(device) {
          connection.connect();
      }

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
        // The Serial connection destructor will close the file descriptor.
    }

private:
    MB::Serial::Connection connection;
};
