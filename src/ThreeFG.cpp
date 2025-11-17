#include "onrobot_driver/threefg/ThreeFG.hpp"
#include "onrobot_driver/common/TCPConnectionWrapper.hpp"
#include "onrobot_driver/common/SerialConnectionWrapper.hpp"
#include <chrono>
#include <thread>

ThreeFG::ThreeFG(const std::string &ip, int port, int device_address)
    : device_address_(device_address)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");

    // Attempt to establish TCP connection, retrying until successful
    while (true) {
        try {
            connection = std::make_unique<TCPConnectionWrapper>(ip, port);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to establish TCP connection: " << ex.what()
                      << ". Retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    initParams();
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

ThreeFG::ThreeFG(const std::string &device, int device_address)
    : device_address_(device_address)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");

    // Attempt to establish Serial connection, retrying until successful
    while (true) {
        try {
            connection = std::make_unique<SerialConnectionWrapper>(device);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Failed to establish Serial connection: " << ex.what()
                      << ". Retrying in 1 second..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    initParams();
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

ThreeFG::~ThreeFG()
{
    if (connection)
        connection->close();
}

void ThreeFG::initParams()
{
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f; // 50% speed
}

MB::ModbusResponse ThreeFG::sendRequest(const MB::ModbusRequest &req)
{
    try
    {
        return connection->sendRequest(req);
    }
    catch (const MB::ModbusException &ex)
    {
        std::cerr << "Modbus exception: " << ex.what() << std::endl;
        throw;
    }
}

float ThreeFG::fromTenthMM(uint16_t tenth_mm)
{
    return static_cast<float>(tenth_mm) / 10000.0f; // Convert 1/10 mm to meters
}

uint16_t ThreeFG::toTenthMM(float meters)
{
    return static_cast<uint16_t>(meters * 10000.0f); // Convert meters to 1/10 mm
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
    setGripType(false);  // false = external grip  
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
    float force_percent = (force_val / MAX_FORCE) * 100.0f;
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

std::vector<int> ThreeFG::getStatus()
{
    std::vector<int> status_list(4, 0);
    uint16_t status_raw = getStatusRaw();
    
    status_list[0] = (status_raw & STATUS_BUSY) ? 1 : 0;
    status_list[1] = (status_raw & STATUS_GRIP_DETECTED) ? 1 : 0;
    status_list[2] = (status_raw & STATUS_FORCE_GRIP_DETECTED) ? 1 : 0;
    status_list[3] = (status_raw & STATUS_CALIBRATION_OK) ? 1 : 0;
    
    return status_list;
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

float ThreeFG::getMinWidth() { return getMinDiameter(); }
float ThreeFG::getMaxWidth() { return getMaxDiameter(); }

float ThreeFG::getDiameterWithOffset()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_DIAMETER_WITH_OFFSET, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read diameter with offset." << std::endl;
        return -1.0f;
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
    uint16_t value = static_cast<uint16_t>(mm * 10.0f); // Convert to 1/10 mm
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

void ThreeFG::setFingerPosition(float position)
{
    uint16_t value = static_cast<uint16_t>(position);
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
    uint16_t value = static_cast<uint16_t>(mm * 100.0f); // Convert to 1/100 mm
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