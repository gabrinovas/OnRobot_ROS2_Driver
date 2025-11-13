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
        // Set Modbus RTU settings according to OnRobot documentation
        connection.setOneStopBit();        // 1 stop bit (documentation specifies 1 stop bit)
        connection.setEvenParity();        // Even parity (documentation specifies even parity)
        connection.setBaudRate(1000000);   // 1,000,000 baud rate (from documentation page 7)
        connection.setDataBits(8);         // 8 data bits (documentation specifies 8 data bits)
        connection.setTimeout(1000);       // 1 second timeout
    }

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req) override {
        try {
            // Send the raw request bytes.
            connection.send(req.toRaw());
            // Wait for the complete response.
            auto [response, rawData] = connection.awaitResponse();
            return response;
        } catch (const MB::ModbusException &ex) {
            throw;
        }
    }

    void close() override {
        // Close the serial connection
        connection.close();
    }

private:
    MB::Serial::Connection connection;
};
