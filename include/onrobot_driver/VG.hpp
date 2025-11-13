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

class VG {
public:
    // Constructors for TCP and Serial connections
    VG(const std::string &type, const std::string &ip, int port, int device_address);
    VG(const std::string &type, const std::string &device, int device_address = 65);
    ~VG();

    // Control commands
    void grip();
    void gripWithStatus();
    void release();
    void idle();
    
    // Channel control
    void setChannelA(uint8_t control_mode, uint8_t target_vacuum = 0);
    void setChannelB(uint8_t control_mode, uint8_t target_vacuum = 0);
    void gripChannelA(uint8_t target_vacuum);
    void gripChannelB(uint8_t target_vacuum);
    
    // Configuration
    void setCurrentLimit(uint16_t current_ma);
    
    // Status reading
    float getChannelAVacuum();
    float getChannelBVacuum();
    std::vector<int> getStatus();
    std::vector<std::string> getDetailedStatus();
    bool isGripping();
    bool waitForGrip(int timeout_ms = 3000);

private:
    std::unique_ptr<IModbusConnection> connection;
    std::string type;
    int device_address_;
    
    // Constants for registers and commands based on documentation
    static constexpr uint16_t REG_CHANNEL_A_CONTROL = 0;
    static constexpr uint16_t REG_CHANNEL_B_CONTROL = 1;
    static constexpr uint16_t REG_CURRENT_LIMIT = 2;
    static constexpr uint16_t REG_CHANNEL_A_VACUUM = 258;
    static constexpr uint16_t REG_CHANNEL_B_VACUUM = 259;

    // Control modes
    static constexpr uint8_t MODE_RELEASE = 0;
    static constexpr uint8_t MODE_GRIP = 1;
    static constexpr uint8_t MODE_IDLE = 2;

    // Default parameters
    static constexpr uint8_t DEFAULT_VACUUM = 40; // 40%
    static constexpr uint16_t DEFAULT_CURRENT_LIMIT = 500; // 500mA

    // Helper function to send a MODBUS request
    MB::ModbusResponse sendRequest(const MB::ModbusRequest &req);
    
    // Helper function to create control register value
    uint16_t createControlValue(uint8_t control_mode, uint8_t target_vacuum);
};
