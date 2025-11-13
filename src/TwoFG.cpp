#include "TwoFG.hpp"

TwoFG::TwoFG(const std::string &type, const std::string &ip, int port, int device_address)
    : type(type), device_address_(device_address)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "2fg7" && type != "2fg14")
        throw std::invalid_argument("Please specify either '2fg7' or '2fg14'.");

    // Set specifications based on type
    if (type == "2fg7") {
        MAX_WIDTH = 0.07f;   // 70mm for 2FG7
        MAX_FORCE = 70.0f;   // 70N for 2FG7
    } else { // 2fg14
        MAX_WIDTH = 0.14f;   // 140mm for 2FG14  
        MAX_FORCE = 70.0f;   // 70N for 2FG14
    }
    MIN_WIDTH = 0.0f;

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

    // Set default parameters
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f; // 50% speed

    // Set defaults on the gripper
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

TwoFG::TwoFG(const std::string &type, const std::string &device, int device_address)
    : type(type), device_address_(device_address)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "2fg7" && type != "2fg14")
        throw std::invalid_argument("Please specify either '2fg7' or '2fg14'.");

    // Set specifications based on type
    if (type == "2fg7") {
        MAX_WIDTH = 0.07f;   // 70mm for 2FG7
        MAX_FORCE = 70.0f;   // 70N for 2FG7
    } else { // 2fg14
        MAX_WIDTH = 0.14f;   // 140mm for 2FG14  
        MAX_FORCE = 70.0f;   // 70N for 2FG14
    }
    MIN_WIDTH = 0.0f;

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

    // Set default parameters
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f; // 50% speed

    // Set defaults on the gripper
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
}

TwoFG::~TwoFG()
{
    if (connection)
        connection->close();
}

MB::ModbusResponse TwoFG::sendRequest(const MB::ModbusRequest &req)
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

float TwoFG::fromTenthMM(uint16_t tenth_mm)
{
    return static_cast<float>(tenth_mm) / 10000.0f; // Convert 1/10 mm to meters
}

uint16_t TwoFG::toTenthMM(float meters)
{
    return static_cast<uint16_t>(meters * 10000.0f); // Convert meters to 1/10 mm
}

void TwoFG::moveGripper(float width_val, bool external_grip)
{
    // Clamp width to valid range
    float clamped_width = std::max(MIN_WIDTH, std::min(width_val, MAX_WIDTH));
    
    // Check if gripper is busy before sending command
    uint16_t status = getStatusRaw();
    if (status & STATUS_BUSY) {
        std::cerr << "Gripper is busy, cannot accept new command" << std::endl;
        return;
    }
    
    // Set target width
    setTargetWidth(clamped_width);
    
    // Execute appropriate grip command
    if (external_grip) {
        gripExternal();
    } else {
        gripInternal();
    }
}

void TwoFG::gripExternal()
{
    setCommand(CMD_GRIP_EXTERNAL);
}

void TwoFG::gripInternal()
{
    setCommand(CMD_GRIP_INTERNAL);
}

void TwoFG::stop()
{
    setCommand(CMD_STOP);
}

void TwoFG::setCommand(uint16_t command)
{
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(command)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_COMMAND, 1, values);
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

void TwoFG::setTargetForce(float force_val)
{
    // Clamp force to valid range (0-MAX_FORCE)
    float clamped_force = std::max(0.0f, std::min(force_val, MAX_FORCE));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(static_cast<uint16_t>(clamped_force))};
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

void TwoFG::setTargetWidth(float width_val)
{
    // Clamp width to valid range
    float clamped_width = std::max(MIN_WIDTH, std::min(width_val, MAX_WIDTH));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(toTenthMM(clamped_width))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_WIDTH, 1, values);
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

void TwoFG::setTargetSpeed(float speed_val)
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

float TwoFG::getWidth()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_EXTERNAL_WIDTH, 1);
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

std::vector<int> TwoFG::getStatus()
{
    std::vector<int> status_list(4, 0);
    uint16_t status_raw = getStatusRaw();
    
    status_list[0] = (status_raw & STATUS_BUSY) ? 1 : 0;
    status_list[1] = (status_raw & STATUS_GRIP_DETECTED) ? 1 : 0;
    status_list[2] = (status_raw & STATUS_ERROR_NOT_CALIBRATED) ? 1 : 0;
    status_list[3] = (status_raw & STATUS_ERROR_LINEAR_SENSOR) ? 1 : 0;
    
    return status_list;
}

uint16_t TwoFG::getStatusRaw()
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

float TwoFG::getMinWidth()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_MIN_EXTERNAL_WIDTH, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read minimum width." << std::endl;
        return MIN_WIDTH;
    }
}

float TwoFG::getMaxWidth()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_MAX_EXTERNAL_WIDTH, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read maximum width." << std::endl;
        return MAX_WIDTH;
    }
}

float TwoFG::getCurrentForce()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FORCE, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return static_cast<float>(regValue); // Force in N
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read current force." << std::endl;
        return -1.0f;
    }
}