#pragma once
#include <memory>
#include <vector>
#include <string>
#include <stdexcept>
#include <iostream>
#include <thread>

#include "onrobot_driver/IModbusConnection.hpp"
#include "onrobot_driver/TCPConnectionWrapper.hpp"
#include "onrobot_driver/SerialConnectionWrapper.hpp"

// Modbus request/response definitions and utilities
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class ThreeFG {
public:
    // Constructors for TCP and Serial connections
    ThreeFG(const std::string &type, const std::string &ip, int port, int device_address);
    ThreeFG(const std::string &type, const std::string &device, int device_address = 65);
    ~ThreeFG();

    // Read commands
    float getWidth();
    std::vector<int> getStatus();
    uint16_t getStatusRaw();
    
    // Write commands
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val);
    void setGripType(uint16_t grip_type);
    void setControl(uint16_t command);
    
    // Gripper control commands
    void gripExternal();
    void gripInternal();
    void stop();
    void moveGripper(float width_val, bool external_grip = true);
    void flexibleGrip(float width_val);

    // Utility functions
    float getMinWidth();
    float getMaxWidth();

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    int device_address_;
    
    // 3FG specifications (in meters)
    static constexpr float MAX_WIDTH = 0.15f;  // 150mm for 3FG15
    static constexpr float MIN_WIDTH = 0.0f;
    static constexpr float MAX_FORCE = 140.0f;  // 140N for 3FG15

    // Default parameters
    float default_force_;

    // Constants for registers and commands based on documentation
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_DIAMETER = 1;
    static constexpr uint16_t REG_GRIP_TYPE = 2;
    static constexpr uint16_t REG_CONTROL = 3;
    static constexpr uint16_t REG_STATUS = 256;
    static constexpr uint16_t REG_RAW_DIAMETER = 257;
    static constexpr uint16_t REG_DIAMETER_WITH_OFFSET = 258;
    static constexpr uint16_t REG_FORCE_APPLIED = 259;
    static constexpr uint16_t REG_MIN_DIAMETER = 513;
    static constexpr uint16_t REG_MAX_DIAMETER = 514;

    // Control commands
    static constexpr uint16_t CMD_GRIP = 1;
    static constexpr uint16_t CMD_MOVE = 2;
    static constexpr uint16_t CMD_STOP = 4;
    static constexpr uint16_t CMD_FLEXIBLE_GRIP = 5;

    // Grip types
    static constexpr uint16_t GRIP_EXTERNAL = 0;
    static constexpr uint16_t GRIP_INTERNAL = 1;

    // Status bits
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
    static constexpr uint16_t STATUS_CALIBRATION = 0x0008;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    
    // Helper function to convert width from 1/10 mm to meters
    float fromTenthMM(uint16_t tenth_mm);
    
    // Helper function to convert width from meters to 1/10 mm
    uint16_t toTenthMM(float meters);
};