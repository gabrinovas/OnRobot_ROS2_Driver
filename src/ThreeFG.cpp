#include "ThreeFG.hpp"

ThreeFG::ThreeFG(const std::string &type, const std::string &ip, int port, int device_address)
    : type(type), device_address_(device_address)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "3fg15" && type != "3fg25")
        throw std::invalid_argument("Please specify either '3fg15' or '3fg25'.");

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

    default_force_ = MAX_FORCE / 2;
    setTargetForce(default_force_);
}

ThreeFG::ThreeFG(const std::string &type, const std::string &device, int device_address)
    : type(type), device_address_(device_address)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "3fg15" && type != "3fg25")
        throw std::invalid_argument("Please specify either '3fg15' or '3fg25'.");

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

    default_force_ = MAX_FORCE / 2;
    setTargetForce(default_force_);
}

ThreeFG::~ThreeFG()
{
    if (connection)
        connection->close();
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
    return static_cast<float>(tenth_mm) / 10000.0f;
}

uint16_t ThreeFG::toTenthMM(float meters)
{
    return static_cast<uint16_t>(meters * 10000.0f);
}

void ThreeFG::moveGripper(float width_val, bool external_grip)
{
    float clamped_width = std::max(MIN_WIDTH, std::min(width_val, MAX_WIDTH));
    
    setTargetWidth(clamped_width);
    setGripType(external_grip ? GRIP_EXTERNAL : GRIP_INTERNAL);
    setControl(CMD_GRIP);
}

void ThreeFG::flexibleGrip(float width_val)
{
    float clamped_width = std::max(MIN_WIDTH, std::min(width_val, MAX_WIDTH));
    
    setTargetWidth(clamped_width);
    setControl(CMD_FLEXIBLE_GRIP);
}

void ThreeFG::gripExternal()
{
    setGripType(GRIP_EXTERNAL);
    setControl(CMD_GRIP);
}

void ThreeFG::gripInternal()
{
    setGripType(GRIP_INTERNAL);
    setControl(CMD_GRIP);
}

void ThreeFG::stop()
{
    setControl(CMD_STOP);
}

void ThreeFG::setControl(uint16_t command)
{
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(command)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CONTROL, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set control command." << std::endl;
        throw;
    }
}

void ThreeFG::setGripType(uint16_t grip_type)
{
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(grip_type)};
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

void ThreeFG::setTargetForce(float force_val)
{
    float clamped_force = std::max(0.0f, std::min(force_val, MAX_FORCE));
    // Force is provided in 10% increments (0-1000 for 0-100%)
    uint16_t force_reg = static_cast<uint16_t>(clamped_force / MAX_FORCE * 1000.0f);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(force_reg)};
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

void ThreeFG::setTargetWidth(float width_val)
{
    float clamped_width = std::max(MIN_WIDTH, std::min(width_val, MAX_WIDTH));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(toTenthMM(clamped_width))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_DIAMETER, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target width." << std::endl;
        throw;
    }
}

float ThreeFG::getWidth()
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
        std::cerr << "Failed to read width." << std::endl;
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
    status_list[3] = (status_raw & STATUS_CALIBRATION) ? 1 : 0;
    
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

float ThreeFG::getMinWidth()
{
    return MIN_WIDTH;
}

float ThreeFG::getMaxWidth()
{
    return MAX_WIDTH;
}