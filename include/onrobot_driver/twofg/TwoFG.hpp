#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>

#include "../common/OnRobotGripperBase.hpp"

class TwoFG : public onrobot_driver::OnRobotGripperBase {
public:
    TwoFG(const std::string &type, const std::string &ip, int port, int device_address);
    TwoFG(const std::string &type, const std::string &ip, int port, int device_address,
          std::function<bool()> keep_running);
    TwoFG(const std::string &type, const std::string &device, int device_address);
    TwoFG(const std::string &type, const std::string &device, int device_address,
          std::function<bool()> keep_running);
    TwoFG(const std::string &type, int device_address, std::unique_ptr<IModbusConnection> connection);
    ~TwoFG() override;

    // Read commands
    float getWidth() override;
    float getForce() override;
    uint16_t getStatusRaw() override;
    std::vector<int> getStatus() override;
    
    // Write commands
    void setTargetForce(float force_val) override;
    void setTargetWidth(float width_val) override;
    void setTargetSpeed(float speed_val) override;
    void setCommand(uint16_t command);
    
    // Gripper control commands
    void gripExternal();
    void gripInternal();
    void stop() override;
    void moveGripper(float width_val) override { moveGripper(width_val, true); }
    void moveGripper(float width_val, bool external_grip);

    // Utility functions
    float getMinWidth() const override;
    float getMaxWidth() const override;
    float getMaxForce() const override;

private:
    std::string type;
    
    // 2FG7 specifications
    static constexpr float MAX_WIDTH_2FG7 = 0.07f;
    static constexpr float MIN_WIDTH = 0.0f;
    static constexpr float MAX_FORCE_2FG7 = 70.0f;

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

    void initParams();
};