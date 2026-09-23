#pragma once

#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <cstdint>

#include "../common/OnRobotGripperBase.hpp"

namespace onrobot_driver
{

class VGC10 : public OnRobotGripperBase
{
public:
    // Constants for Vacuum Modes
    static constexpr uint8_t MODE_RELEASE = 0x00;
    static constexpr uint8_t MODE_GRIP    = 0x01;
    static constexpr uint8_t MODE_IDLE    = 0x02;

    // Channel identifiers
    enum class Channel : uint8_t {
        A = 0,
        B = 1,
        BOTH = 2
    };

    // Constructors (TCP / Compute Box as default and primary interface)
    VGC10(const std::string &type, const std::string &ip, int port = 502, int device_address = 65);
    VGC10(const std::string &type, const std::string &ip, int port, int device_address,
          std::function<bool()> keep_running);
    VGC10(const std::string &type, const std::string &device, int device_address = 65);
    VGC10(const std::string &type, const std::string &device, int device_address,
          std::function<bool()> keep_running);
    ~VGC10() override;

    // Overridden base read commands (mapped to average/combined vacuum)
    float getWidth() override; // Returns normalized vacuum (0.0 to 1.0)
    float getForce() override; // Returns current limit (mA)
    uint16_t getStatusRaw() override;
    std::vector<int> getStatus() override;

    // Overridden base write commands
    void setTargetForce(float force_val) override; // Sets current limit (mA)
    void setTargetWidth(float width_val) override; // Sets target vacuum (0.0 to 1.0)
    void setTargetSpeed(float speed_val) override;
    void stop() override;                          // Releases all channels
    void moveGripper(float width_val) override;    // Sets target vacuum for both channels

    float getMinWidth() const override { return 0.0f; }
    float getMaxWidth() const override { return 1.0f; }
    float getMaxForce() const override { return 1000.0f; } // mA max

    // Channel-specific control methods
    void gripChannelA(uint8_t vacuum_pct = 60);
    void releaseChannelA();
    void idleChannelA();

    void gripChannelB(uint8_t vacuum_pct = 60);
    void releaseChannelB();
    void idleChannelB();

    void gripAll(uint8_t vacuum_pct = 60);
    void releaseAll();
    void idleAll();

    // Raw control register setting
    void setChannelControl(Channel ch, uint8_t mode, uint8_t target_vacuum_pct);
    void setCurrentLimit(uint16_t current_ma);

    // Vacuum telemetry (returns 0.0 to 1.0, where 1.0 = 100% vacuum)
    float getVacuumChannelA();
    float getVacuumChannelB();
    uint16_t getRawVacuumChannelA(); // in 1/1000 relative vacuum
    uint16_t getRawVacuumChannelB();
    bool readBothVacuums(float &vac_a, float &vac_b); // Single Modbus transaction for both channels

    uint8_t getModeChannelA() const { return current_mode_a_; }
    uint8_t getModeChannelB() const { return current_mode_b_; }
    uint8_t getTargetVacuumChannelA() const { return target_vacuum_a_; }
    uint8_t getTargetVacuumChannelB() const { return target_vacuum_b_; }

private:
    std::string type_;

    // Current setpoints
    uint8_t current_mode_a_{MODE_RELEASE};
    uint8_t target_vacuum_a_{0};
    uint8_t current_mode_b_{MODE_RELEASE};
    uint8_t target_vacuum_b_{0};
    uint16_t current_limit_ma_{500};

    // Modbus registers for VG10 / VGC10 (OnRobot Connectivity Guide v1.22.0 p.47)
    static constexpr uint16_t REG_CHANNEL_A_CTRL   = 0x0000;
    static constexpr uint16_t REG_CHANNEL_B_CTRL   = 0x0001;
    static constexpr uint16_t REG_CURRENT_LIMIT    = 0x0002;
    static constexpr uint16_t REG_CHANNEL_A_VACUUM = 0x0102;
    static constexpr uint16_t REG_CHANNEL_B_VACUUM = 0x0103;

    void initParams();
    void writeChannelRegister(uint16_t reg_addr, uint8_t mode, uint8_t target_pct);
};

} // namespace onrobot_driver
