#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>

#include "onrobot_driver/IModbusConnection.hpp"
#include "onrobot_driver/TCPConnectionWrapper.hpp"
#include "onrobot_driver/SerialConnectionWrapper.hpp"

// Modbus request/response definitions and utilities.
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class RG {
public:
    // For TCP connection
    RG(const std::string &type, const std::string &ip, int port);
    // For Serial connection
    RG(const std::string &type, const std::string &device);
    ~RG();

    // Read commands
    float getFingertipOffset();
    float getWidth();
    std::vector<int> getStatus();
    float getWidthWithOffset();

    // Write commands
    void setControlMode(uint16_t command);
    void setTargetForce(uint16_t force_val);
    void setTargetWidth(uint16_t width_val);
    void closeGripper(uint16_t force_val = 400);
    void openGripper(uint16_t force_val = 400);
    void moveGripper(uint16_t width_val, uint16_t force_val = 400);

    // Explicit connection close if needed
    void closeConnection();

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    uint16_t max_width;
    uint16_t max_force;

    // Helper function to send a request and return a response.
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
};
