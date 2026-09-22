#include "onrobot_driver/vgc10/VGC10.hpp"
#include <algorithm>
#include <iostream>

namespace onrobot_driver
{

VGC10::VGC10(const std::string &type, const std::string &ip, int port, int device_address)
    : VGC10(type, ip, port, device_address, nullptr)
{
}

VGC10::VGC10(const std::string &type, const std::string &ip, int port, int device_address,
             std::function<bool()> keep_running)
    : OnRobotGripperBase(device_address), type_(type)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for Compute Box TCP connection.");

    connectTCP(ip, port, keep_running);
    initParams();
}

VGC10::VGC10(const std::string &type, const std::string &device, int device_address)
    : VGC10(type, device, device_address, nullptr)
{
}

VGC10::VGC10(const std::string &type, const std::string &device, int device_address,
             std::function<bool()> keep_running)
    : OnRobotGripperBase(device_address), type_(type)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");

    connectSerial(device, keep_running);
    initParams();
}

VGC10::~VGC10()
{
    try
    {
        releaseAll();
    }
    catch (...)
    {
    }
    close();
}

void VGC10::initParams()
{
    current_limit_ma_ = 500;
    current_mode_a_ = MODE_RELEASE;
    target_vacuum_a_ = 0;
    current_mode_b_ = MODE_RELEASE;
    target_vacuum_b_ = 0;

    try
    {
        setCurrentLimit(current_limit_ma_);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[VGC10] Warning: could not set initial current limit: " << e.what() << std::endl;
    }
}

void VGC10::writeChannelRegister(uint16_t reg_addr, uint8_t mode, uint8_t target_pct)
{
    // Clamp target vacuum to valid 0-80% per OnRobot spec
    uint8_t clamped_pct = std::min<uint8_t>(target_pct, 80);
    uint16_t reg_value = (static_cast<uint16_t>(mode) << 8) | static_cast<uint16_t>(clamped_pct);

    std::vector<MB::ModbusCell> values = {MB::ModbusCell(reg_value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, reg_addr, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &e)
    {
        std::cerr << "[VGC10] Failed to write channel register " << reg_addr << ": " << e.what() << std::endl;
        throw;
    }
}

void VGC10::setCurrentLimit(uint16_t current_ma)
{
    // Clamp to max 1000 mA per OnRobot manual
    uint16_t clamped_limit = std::min<uint16_t>(current_ma, 1000);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(clamped_limit)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CURRENT_LIMIT, 1, values);
    try
    {
        sendRequest(req);
        current_limit_ma_ = clamped_limit;
    }
    catch (const MB::ModbusException &e)
    {
        std::cerr << "[VGC10] Failed to set current limit: " << e.what() << std::endl;
        throw;
    }
}

void VGC10::setChannelControl(Channel ch, uint8_t mode, uint8_t target_vacuum_pct)
{
    if (ch == Channel::A || ch == Channel::BOTH)
    {
        writeChannelRegister(REG_CHANNEL_A_CTRL, mode, target_vacuum_pct);
        current_mode_a_ = mode;
        target_vacuum_a_ = target_vacuum_pct;
    }
    if (ch == Channel::B || ch == Channel::BOTH)
    {
        writeChannelRegister(REG_CHANNEL_B_CTRL, mode, target_vacuum_pct);
        current_mode_b_ = mode;
        target_vacuum_b_ = target_vacuum_pct;
    }
}

void VGC10::gripChannelA(uint8_t vacuum_pct)
{
    setChannelControl(Channel::A, MODE_GRIP, vacuum_pct);
}

void VGC10::releaseChannelA()
{
    setChannelControl(Channel::A, MODE_RELEASE, 0);
}

void VGC10::idleChannelA()
{
    setChannelControl(Channel::A, MODE_IDLE, 0);
}

void VGC10::gripChannelB(uint8_t vacuum_pct)
{
    setChannelControl(Channel::B, MODE_GRIP, vacuum_pct);
}

void VGC10::releaseChannelB()
{
    setChannelControl(Channel::B, MODE_RELEASE, 0);
}

void VGC10::idleChannelB()
{
    setChannelControl(Channel::B, MODE_IDLE, 0);
}

void VGC10::gripAll(uint8_t vacuum_pct)
{
    setChannelControl(Channel::BOTH, MODE_GRIP, vacuum_pct);
}

void VGC10::releaseAll()
{
    setChannelControl(Channel::BOTH, MODE_RELEASE, 0);
}

void VGC10::idleAll()
{
    setChannelControl(Channel::BOTH, MODE_IDLE, 0);
}

uint16_t VGC10::getRawVacuumChannelA()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CHANNEL_A_VACUUM, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        if (!resp.registerValues().empty() && resp.registerValues().front().isReg())
        {
            return resp.registerValues().front().reg();
        }
    }
    catch (const MB::ModbusException &e)
    {
        std::cerr << "[VGC10] Failed to read Channel A vacuum: " << e.what() << std::endl;
    }
    return 0;
}

uint16_t VGC10::getRawVacuumChannelB()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CHANNEL_B_VACUUM, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        if (!resp.registerValues().empty() && resp.registerValues().front().isReg())
        {
            return resp.registerValues().front().reg();
        }
    }
    catch (const MB::ModbusException &e)
    {
        std::cerr << "[VGC10] Failed to read Channel B vacuum: " << e.what() << std::endl;
    }
    return 0;
}

float VGC10::getVacuumChannelA()
{
    // 1/1000 of relative vacuum -> 0.0 to 1.0 (clamped)
    uint16_t raw = getRawVacuumChannelA();
    float normalized = static_cast<float>(raw) / 1000.0f;
    return std::max(0.0f, std::min(normalized, 1.0f));
}

float VGC10::getVacuumChannelB()
{
    uint16_t raw = getRawVacuumChannelB();
    float normalized = static_cast<float>(raw) / 1000.0f;
    return std::max(0.0f, std::min(normalized, 1.0f));
}

// ================= Base Interface Implementations =================

float VGC10::getWidth()
{
    // Combined / Average vacuum level normalized to [0.0, 1.0]
    float vac_a = getVacuumChannelA();
    float vac_b = getVacuumChannelB();
    return (vac_a + vac_b) / 2.0f;
}

float VGC10::getForce()
{
    return static_cast<float>(current_limit_ma_);
}

uint16_t VGC10::getStatusRaw()
{
    // Status synthesis: bit 0 = A busy/gripping, bit 1 = B busy/gripping
    uint16_t status = 0;
    if (current_mode_a_ == MODE_GRIP) status |= 0x01;
    if (current_mode_b_ == MODE_GRIP) status |= 0x02;
    return status;
}

std::vector<int> VGC10::getStatus()
{
    return {current_mode_a_, target_vacuum_a_, current_mode_b_, target_vacuum_b_};
}

void VGC10::setTargetForce(float force_val)
{
    setCurrentLimit(static_cast<uint16_t>(force_val));
}

void VGC10::setTargetWidth(float width_val)
{
    // 0.0 to 1.0 -> map to 0 to 80% vacuum for both channels
    if (width_val > 0.05f)
    {
        uint8_t target_pct = static_cast<uint8_t>(std::min(width_val * 80.0f, 80.0f));
        gripAll(target_pct);
    }
    else
    {
        releaseAll();
    }
}

void VGC10::setTargetSpeed(float /*speed_val*/)
{
    // No-op for vacuum pump
}

void VGC10::stop()
{
    releaseAll();
}

void VGC10::moveGripper(float width_val)
{
    setTargetWidth(width_val);
}

} // namespace onrobot_driver
