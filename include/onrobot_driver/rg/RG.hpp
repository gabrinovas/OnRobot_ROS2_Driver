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

// Modbus request/response definitions and utilities
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class RG {
public:
    // Constructors for TCP and Serial connections
    RG(const std::string &type, const std::string &ip, int port, int device_address = 65);  // Add device_address
    RG(const std::string &type, const std::string &device, int device_address = 65);        // Add device_address
    ~RG();

    // Read commands
    float getFingertipOffset();
    float getWidth();
    std::vector<int> getStatus();
    std::vector<int> getStatusAndPrint();
    float getWidthWithOffset();

    // Write commands
    void setFingertipOffset(float offset_val);
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val);
    void setControlMode(uint16_t command);
    
    // Gripper control commands
    void closeGripper();
    void openGripper();
    void moveGripper(float width_val);

    // Utility functions (add these for consistency)
    float getMinWidth();
    float getMaxWidth();

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    int device_address_;  // Add device address member
    
    // RG specifications
    static constexpr float MAX_WIDTH_RG2 = 0.11f;   // 110mm for RG2
    static constexpr float MAX_WIDTH_RG6 = 0.16f;   // 160mm for RG6
    static constexpr float MIN_WIDTH = 0.0f;
    static constexpr float MAX_FORCE_RG2 = 40.0f;   // 40N for RG2
    static constexpr float MAX_FORCE_RG6 = 120.0f;  // 120N for RG6

    // Default parameters
    float default_force_;
    float max_width_;
    float max_force_;
    float default_fingertip_offset;

    // Constants for registers and commands
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_WIDTH = 1;
    static constexpr uint16_t REG_CONTROL = 2;
    static constexpr uint16_t REG_FINGERTIP_OFFSET = 258;
    static constexpr uint16_t REG_ACTUAL_DEPTH = 263;
    static constexpr uint16_t REG_ACTUAL_RELATIVE_DEPTH = 264;
    static constexpr uint16_t REG_ACTUAL_WIDTH = 267;
    static constexpr uint16_t REG_STATUS = 268;
    static constexpr uint16_t REG_ACTUAL_WIDTH_WITH_OFFSET = 275;
    static constexpr uint16_t REG_SET_FINGERTIP_OFFSET = 1031;

    // Control commands
    static constexpr uint16_t CMD_GRIP = 0x0001;
    static constexpr uint16_t CMD_STOP = 0x0008;
    static constexpr uint16_t CMD_GRIP_WITH_OFFSET = 0x0010;

    // Status bits
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_S1_PUSHED = 0x0004;
    static constexpr uint16_t STATUS_S1_TRIGGERED = 0x0008;
    static constexpr uint16_t STATUS_S2_PUSHED = 0x0010;
    static constexpr uint16_t STATUS_S2_TRIGGERED = 0x0020;
    static constexpr uint16_t STATUS_SAFETY_ERROR = 0x0040;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    
    // Helper function to convert width
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);
    void initParams();
};
