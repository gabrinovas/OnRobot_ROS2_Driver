#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>

#include "../common/IModbusConnection.hpp"
#include "../common/TCPConnectionWrapper.hpp"
#include "../common/SerialConnectionWrapper.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class TwoFG {
public:
    TwoFG(const std::string &type, const std::string &ip, int port, int device_address = 65);
    TwoFG(const std::string &type, const std::string &device, int device_address = 65);
    ~TwoFG();

    // Read commands
    float getWidth();
    std::vector<int> getStatus();
    uint16_t getStatusRaw();
    
    // Write commands
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val);
    void setTargetSpeed(float speed_val);
    void setCommand(uint16_t command);
    
    // Gripper control commands
    void gripExternal();
    void gripInternal();
    void stop();
    void moveGripper(float width_val, bool external_grip = true);

    // Utility functions
    float getMinWidth();
    float getMaxWidth();

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    int device_address_;
    
    // 2FG specifications
    static constexpr float MAX_WIDTH_2FG7 = 0.07f;
    static constexpr float MAX_WIDTH_2FG14 = 0.14f;
    static constexpr float MIN_WIDTH = 0.0f;
    static constexpr float MAX_FORCE_2FG7 = 70.0f;
    static constexpr float MAX_FORCE_2FG14 = 140.0f;

    // Default parameters
    float default_force_;
    float default_speed_;
    float max_width_;
    float max_force_;

    // Modbus registers
    static constexpr uint16_t REG_TARGET_WIDTH = 0;
    static constexpr uint16_t REG_TARGET_FORCE = 1;
    static constexpr uint16_t REG_TARGET_SPEED = 2;
    static constexpr uint16_t REG_COMMAND = 3;
    static constexpr uint16_t REG_STATUS = 256;
    static constexpr uint16_t REG_EXTERNAL_WIDTH = 257;
    static constexpr uint16_t REG_INTERNAL_WIDTH = 258;
    static constexpr uint16_t REG_MIN_EXTERNAL_WIDTH = 259;
    static constexpr uint16_t REG_MAX_EXTERNAL_WIDTH = 260;
    static constexpr uint16_t REG_MIN_INTERNAL_WIDTH = 261;
    static constexpr uint16_t REG_MAX_INTERNAL_WIDTH = 262;
    static constexpr uint16_t REG_FORCE = 263;

    // Control commands
    static constexpr uint16_t CMD_GRIP_EXTERNAL = 1;
    static constexpr uint16_t CMD_GRIP_INTERNAL = 2;
    static constexpr uint16_t CMD_STOP = 3;

    // Status bits
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_ERROR_NOT_CALIBRATED = 0x0008;
    static constexpr uint16_t STATUS_ERROR_LINEAR_SENSOR = 0x0010;

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);
    void initParams();
};