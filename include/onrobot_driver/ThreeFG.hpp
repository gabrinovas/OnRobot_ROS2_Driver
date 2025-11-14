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
    ThreeFG(const std::string &ip, int port);
    ThreeFG(const std::string &device, int baudrate = 115200);
    ~ThreeFG();

    // Read commands
    float getWidth();                    // Diameter in meters
    std::vector<int> getStatus();
    uint16_t getStatusRaw();
   
    // Write commands
    void setTargetForce(float force_val);
    void setTargetWidth(float width_val); // Diameter in meters
    void setTargetSpeed(float speed_val);
    void setCommand(uint16_t command);
   
    // Gripper control commands
    void gripInternal();  // 3FG15 only has internal grip (centripetal)
    void stop();
    void moveGripper(float width_val);

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

    // Detailed status for 3FG15
    struct GripperStatus {
        bool busy;
        bool grip_detected;
        bool force_grip_detected;
        bool calibration_ok;
    };

    GripperStatus getDetailedStatus();

    // Setters for advanced configuration (3FG15)
    void setGripType(bool internal);
    void setFingerLength(float mm);
    void setFingerPosition(float position);
    void setFingertipOffset(float mm);

private:
    std::unique_ptr<IModbusConnection> connection;
    int device_address_;

    // 3FG15 specifications
    static constexpr float MAX_DIAMETER = 0.150f;  // 150mm diameter
    static constexpr float MIN_DIAMETER = 0.0f;
    static constexpr float MAX_FORCE = 140.0f;     // 140N maximum force

    // Default parameters for 3FG15
    float default_force_;
    float default_speed_;

    // --- Corrected Registros Modbus para 3FG15 ---
    static constexpr uint16_t REG_TARGET_FORCE = 0;     // Force in 10% units (0-1000)
    static constexpr uint16_t REG_TARGET_WIDTH = 1;     // Diameter in 1/10 mm
    static constexpr uint16_t REG_GRIP_TYPE = 2;        // 0=external, 1=internal (3FG15 only uses internal)
    static constexpr uint16_t REG_COMMAND = 3;          // Control commands
    static constexpr uint16_t REG_STATUS = 256;         // Status register
    static constexpr uint16_t REG_CURRENT_DIAMETER = 257; // Current diameter in 1/10 mm
    static constexpr uint16_t REG_DIAMETER_WITH_OFFSET = 258; // Diameter with offset in 1/10 mm
    static constexpr uint16_t REG_APPLIED_FORCE = 259;  // Applied force in 1/10 %

    // --- Comandos ---
    static constexpr uint16_t CMD_GRIP_INTERNAL = 1; // 3FG15 uses grip command for internal grip
    static constexpr uint16_t CMD_STOP = 4;

    // --- Status bits ---
    static constexpr uint16_t STATUS_BUSY = 0x0001;
    static constexpr uint16_t STATUS_GRIP_DETECTED = 0x0002;
    static constexpr uint16_t STATUS_FORCE_GRIP_DETECTED = 0x0004;
    static constexpr uint16_t STATUS_CALIBRATION_OK = 0x0008;

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
   
    // Conversión de unidades (1/10 mm <-> metros)
    float fromTenthMM(uint16_t tenth_mm);
    uint16_t toTenthMM(float meters);
};
