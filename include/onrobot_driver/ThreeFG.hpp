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
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"
#include "MB/modbusUtils.hpp"

class ThreeFG {
public:
    // Constructor for TCP connection
    ThreeFG(const std::string &ip, int port, int device_address = 65);
    ~ThreeFG();

    // Read commands
    float getWidth();  // Diameter in meters
    std::vector<int> getStatus();
    uint16_t getStatusRaw();
   
    // Write commands
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val); // Diameter in meters
    void setTargetSpeed(float speed_val);
    void setCommand(uint16_t command);
   
    // Gripper control commands
    void gripInternal();  // Centripetal grip for 3FG15
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

    // Advanced configuration (3FG15 specific)
    void setGripType(bool internal);
    void setFingerLength(float mm);
    void setFingerPosition(float mm);
    void setFingertipOffset(float mm);

private:
    std::unique_ptr<IModbusConnection> connection;
    int device_address_;

    // 3FG15 specifications
    static constexpr float MAX_DIAMETER = 0.150f;  // 150mm
    static constexpr float MIN_DIAMETER = 0.0f;
    static constexpr float MAX_FORCE = 140.0f;     // 140N

    // Default parameters
    float default_force_;
    float default_speed_;

    // 3FG15 Modbus registers
    static constexpr uint16_t REG_TARGET_WIDTH = 1; // diameter
    static constexpr uint16_t REG_TARGET_FORCE = 0;
    static constexpr uint16_t REG_TARGET_SPEED = 2;
    static constexpr uint16_t REG_COMMAND = 3;
    static constexpr uint16_t REG_STATUS = 256;
    static constexpr uint16_t REG_EXTERNAL_WIDTH = 257;  // current diameter
    static constexpr uint16_t REG_FORCE = 263;

    // Control commands
    static constexpr uint16_t CMD_GRIP_INTERNAL = 2;
    static constexpr uint16_t CMD_STOP = 3;

    // Status bits
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
    static constexpr uint16_t STATUS_CALIBRATION_OK = 0x0008;
    static constexpr uint16_t STATUS_ERROR_LINEAR_SENSOR = 0x0010;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
   
    // Conversion functions
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);
};