#include "onrobot_driver/threefg/ThreeFG.hpp"

ThreeFG::ThreeFG(const std::string &ip, int port, int device_address)
    : ThreeFG(ip, port, device_address, nullptr)
{
}

ThreeFG::ThreeFG(const std::string &ip, int port, int device_address,
                 std::function<bool()> keep_running)
    : onrobot_driver::OnRobotGripperBase(device_address)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");

    connectTCP(ip, port, keep_running);
    initParams();
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

ThreeFG::ThreeFG(const std::string &device, int device_address)
    : ThreeFG(device, device_address, nullptr)
{
}

ThreeFG::ThreeFG(const std::string &device, int device_address,
                 std::function<bool()> keep_running)
    : onrobot_driver::OnRobotGripperBase(device_address)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");

    connectSerial(device, keep_running);
    initParams();
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

ThreeFG::~ThreeFG()
{
    close();
}

void ThreeFG::initParams()
{
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f; // 50% speed
}

void ThreeFG::moveGripper(float diameter_val)
{
    // Clamp diameter to valid range
    float clamped_diameter = std::max(getMinDiameter(), std::min(diameter_val, getMaxDiameter()));
    
    // Set target diameter
    setTargetWidth(clamped_diameter);
    
    // Use move command (without force) for simple positioning
    setCommand(CMD_MOVE);
}

void ThreeFG::gripInternal()
{
    setGripType(true);  // true = internal grip
    setCommand(CMD_GRIP);
}

void ThreeFG::gripExternal()
{
    setGripType(false); // false = external grip
    setCommand(CMD_GRIP);
}

void ThreeFG::stop()
{
    setCommand(CMD_STOP);
}

void ThreeFG::setCommand(uint16_t command)
{
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(command)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CONTROL, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set command." << std::endl;
        throw;
    }
}

void ThreeFG::setTargetForce(float force_val)
{
    // Force is in 10% units (0-1000 for 0-100%)
    float clamped_force = std::max(0.0f, std::min(force_val, MAX_FORCE));
    float force_percent = (clamped_force / MAX_FORCE) * 100.0f;
    uint16_t force_reg_value = static_cast<uint16_t>(force_percent * 10.0f); // Convert to 1/10 %
    
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(force_reg_value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_FORCE, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target force." << std::endl;
        throw;
    }
}

void ThreeFG::setTargetWidth(float diameter_val)
{
    // Clamp diameter to valid range
    float clamped_diameter = std::max(getMinDiameter(), std::min(diameter_val, getMaxDiameter()));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(toTenthMM(clamped_diameter))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_DIAMETER, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target diameter." << std::endl;
        throw;
    }
}

void ThreeFG::setTargetSpeed(float speed_val)
{
    // Clamp speed to valid range (10-100%)
    float clamped_speed = std::max(10.0f, std::min(speed_val, 100.0f));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(static_cast<uint16_t>(clamped_speed))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_SPEED, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target speed." << std::endl;
        throw;
    }
}

float ThreeFG::getWidth()
{
    return getCurrentDiameter(); // For compatibility, width = diameter for 3FG15
}

float ThreeFG::getCurrentDiameter()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_RAW_DIAMETER, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read diameter." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getDiameterWithOffset()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_DIAMETER_WITH_OFFSET, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        int16_t regValue = resp.registerValues().front().isReg() ? static_cast<int16_t>(resp.registerValues().front().reg()) : 0;
        return static_cast<float>(regValue) / 10000.0f; // Convert 1/10 mm to meters (signed)
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read diameter with offset." << std::endl;
        return -1.0f;
    }
}

uint16_t ThreeFG::getStatusRaw()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_STATUS, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        return resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read status." << std::endl;
        return 0;
    }
}

float ThreeFG::getAppliedForce()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_APPLIED_FORCE, 1);
    try {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t reg_value = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        // Convert from 1/10 % back to Newtons
        return (static_cast<float>(reg_value) / 10.0f / 100.0f) * MAX_FORCE;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read applied force." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getMinDiameter()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_MIN_DIAMETER, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read min diameter." << std::endl;
        return MIN_DIAMETER;
    }
}

float ThreeFG::getMaxDiameter()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_MAX_DIAMETER, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read max diameter." << std::endl;
        return MAX_DIAMETER;
    }
}

float ThreeFG::getMinWidth() const
{
    return MIN_DIAMETER;
}

float ThreeFG::getMaxWidth() const
{
    return MAX_DIAMETER;
}

float ThreeFG::getMaxForce() const
{
    return MAX_FORCE;
}

float ThreeFG::getFingerLength()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FINGER_LENGTH, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return static_cast<float>(regValue) / 10.0f; // Convert from 1/10 mm to mm
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read finger length." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getFingerPosition()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FINGER_POSITION, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read finger position." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getFingertipOffset()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FINGERTIP_OFFSET, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return static_cast<float>(regValue) / 100.0f; // Convert from 1/100 mm to mm
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read fingertip offset." << std::endl;
        return -1.0f;
    }
}

ThreeFG::GripperStatus ThreeFG::getDetailedStatus()
{
    GripperStatus status{};
    uint16_t raw = getStatusRaw();
    
    status.busy = raw & STATUS_BUSY;
    status.grip_detected = raw & STATUS_GRIP_DETECTED;
    status.force_grip_detected = raw & STATUS_FORCE_GRIP_DETECTED;
    status.calibration_ok = raw & STATUS_CALIBRATION_OK;
    
    return status;
}

void ThreeFG::setGripType(bool internal)
{
    uint16_t value = internal ? 1 : 0;
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_GRIP_TYPE, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set grip type." << std::endl;
        throw;
    }
}

void ThreeFG::setFingerLength(float mm)
{
    uint16_t value = static_cast<uint16_t>(mm * 10.0f); // Convert mm to 1/10 mm
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CFG_FINGER_LENGTH, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set finger length." << std::endl;
        throw;
    }
}

void ThreeFG::setFingerPosition(float pos)
{
    uint16_t value = static_cast<uint16_t>(pos);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CFG_FINGER_POSITION, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set finger position." << std::endl;
        throw;
    }
}

void ThreeFG::setFingertipOffset(float mm)
{
    uint16_t value = static_cast<uint16_t>(mm * 100.0f); // Convert mm to 1/100 mm
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CFG_FINGERTIP_OFFSET, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set fingertip offset." << std::endl;
        throw;
    }
}