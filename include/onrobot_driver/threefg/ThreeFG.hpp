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

class ThreeFG {
public:
    // Remove default parameters to avoid ambiguity
    ThreeFG(const std::string &ip, int port, int device_address);
    ThreeFG(const std::string &device, int device_address);
    ~ThreeFG();

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
    void gripInternal();
    void gripExternal();
    void stop();
    void moveGripper(float diameter_val);

    // Utility functions
    float getMinWidth();
    float getMaxWidth();

    // 3FG15 specific functions
    float getCurrentDiameter();
    float getDiameterWithOffset();
    float getAppliedForce();
    float getMinDiameter();
    float getMaxDiameter();
    float getFingerLength();
    float getFingerPosition();
    float getFingertipOffset();

    // Detailed status
    struct GripperStatus {
        bool busy;
        bool grip_detected;
        bool force_grip_detected;
        bool calibration_ok;
    };

    GripperStatus getDetailedStatus();

    // Advanced configuration
    void setGripType(bool internal);
    void setFingerLength(float mm);
    void setFingerPosition(float mm);
    void setFingertipOffset(float mm);

private:
    std::unique_ptr<IModbusConnection> connection;
    int device_address_;

    // 3FG15 specifications
    static constexpr float MAX_DIAMETER = 0.150f;
    static constexpr float MIN_DIAMETER = 0.0f;
    static constexpr float MAX_FORCE = 140.0f;

    // Default parameters
    float default_force_;
    float default_speed_;

    // Modbus registers (from documentation)
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_DIAMETER = 1;
    static constexpr uint16_t REG_TARGET_SPEED = 2;
    static constexpr uint16_t REG_GRIP_TYPE = 2;
    static constexpr uint16_t REG_CONTROL = 3;
    static constexpr uint16_t REG_STATUS = 256;
    static constexpr uint16_t REG_RAW_DIAMETER = 257;
    static constexpr uint16_t REG_DIAMETER_WITH_OFFSET = 258;
    static constexpr uint16_t REG_APPLIED_FORCE = 259;
    static constexpr uint16_t REG_FINGER_LENGTH = 270;
    static constexpr uint16_t REG_FINGER_POSITION = 272;
    static constexpr uint16_t REG_FINGERTIP_OFFSET = 273;
    static constexpr uint16_t REG_ACTUAL_WIDTH_WITH_OFFSET = 275;
    static constexpr uint16_t REG_MIN_DIAMETER = 513;
    static constexpr uint16_t REG_MAX_DIAMETER = 514;
    static constexpr uint16_t REG_CFG_FINGER_LENGTH = 1025;
    static constexpr uint16_t REG_CFG_FINGER_POSITION = 1027;
    static constexpr uint16_t REG_CFG_FINGERTIP_OFFSET = 1028;

    // Control commands
    static constexpr uint16_t CMD_GRIP = 0x0001;
    static constexpr uint16_t CMD_MOVE = 0x0002;
    static constexpr uint16_t CMD_STOP = 0x0004;
    static constexpr uint16_t CMD_FLEXIBLE_GRIP = 0x0005;

    // Status bits
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
    static constexpr uint16_t STATUS_CALIBRATION_OK = 0x0008;

    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);
    void initParams();
};