#include "RG.hpp"

RG::RG(const std::string &type, const std::string &ip, int port)
    : type(type)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "rg2" && type != "rg6")
        throw std::invalid_argument("Please specify either 'rg2' or 'rg6'.");

    // Set specifications based on type
    if (type == "rg2")
    {
        max_width = 0.11f;
        max_force = 40.0f;
        min_width = 0.0f;
        default_fingertip_offset = 0.0049f;
    }
    else if (type == "rg6")
    {
        max_width = 0.16f;
        max_force = 120.0f;
        min_width = 0.0f;
        default_fingertip_offset = 0.0f;
    }

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

    default_force = max_force / 2;

    // Set these defaults on the gripper.
    setTargetForce(default_force);
    setFingertipOffset(default_fingertip_offset);
}

RG::RG(const std::string &type, const std::string &device)
    : type(type)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "rg2" && type != "rg6")
        throw std::invalid_argument("Please specify either 'rg2' or 'rg6'.");

    // Set specifications based on type
    if (type == "rg2")
    {
        max_width = 0.11f;
        max_force = 40.0f;
        min_width = 0.0f;
        default_fingertip_offset = 0.0049f;
    }
    else if (type == "rg6")
    {
        max_width = 0.16f;
        max_force = 120.0f;
        min_width = 0.0f;
        default_fingertip_offset = 0.0f;
    }

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

    default_force = max_force / 2;

    // Set these defaults on the gripper.
    setTargetForce(default_force);
    setFingertipOffset(default_fingertip_offset);
}

RG::~RG()
{
    if (connection)
        connection->close();
}

MB::ModbusResponse RG::sendRequest(const MB::ModbusRequest &req)
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

float RG::fromTenthMM(uint16_t tenth_mm)
{
    return static_cast<float>(tenth_mm) / 10000.0f;
}

uint16_t RG::toTenthMM(float meters)
{
    return static_cast<uint16_t>(meters * 10000.0f);
}

void RG::moveGripper(float width_val)
{
    // Clamp width to valid range
    float clamped_width = std::max(min_width, std::min(width_val, max_width));
    
    // Check if gripper is busy before sending command
    uint16_t status = getStatus()[0];
    if (status & STATUS_BUSY) {
        std::cerr << "Gripper is busy, cannot accept new command" << std::endl;
        return;
    }
    
    // First stop any ongoing motion.
    setControlMode(CMD_STOP);
    // Then set the target width
    setTargetWidth(clamped_width);
    // Finally, execute the motion command.
    setControlMode(CMD_GRIP_WITH_OFFSET);
}

void RG::openGripper()
{
    // Open gripper: target width is max_width.
    std::cout << "Opening gripper to max width: " << max_width << std::endl;
    moveGripper(max_width);
}

void RG::closeGripper()
{
    // Close gripper: target width is 0.
    std::cout << "Closing gripper to min width: 0" << std::endl;
    moveGripper(min_width);
}

void RG::setControlMode(uint16_t command)
{
    // Write the control command to the REG_CONTROL register.
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(command)};
    MB::ModbusRequest req(DEVICE_ID, MB::utils::WriteSingleAnalogOutputRegister, REG_CONTROL, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set control mode." << std::endl;
    }
}

void RG::setTargetForce(float force_val)
{
    // Clamp force to valid range
    float clamped_force = std::max(0.0f, std::min(force_val, max_force));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell((uint16_t)(clamped_force*10.0f))};
    MB::ModbusRequest req(DEVICE_ID, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_FORCE, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target force." << std::endl;
    }
}

void RG::setTargetWidth(float width_val)
{
    // Clamp width to valid range
    float clamped_width = std::max(min_width, std::min(width_val, max_width));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell((uint16_t)(clamped_width*10000.0f))};
    MB::ModbusRequest req(DEVICE_ID, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_WIDTH, 1, values);
    try
    {
        sendRequest(req);
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set target width." << std::endl;
    }
}

void RG::setFingertipOffset(float offset_val)
{
    std::vector<MB::ModbusCell> values = {MB::ModbusCell((uint16_t)(offset_val*10000.0f))};
    MB::ModbusRequest req(DEVICE_ID, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, REG_SET_FINGERTIP_OFFSET, 1, values);
    try
    {
        sendRequest(req);
        std::cout << "Fingertip offset set to " << offset_val << std::endl;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set fingertip offset." << std::endl;
    }
}

float RG::getFingertipOffset()
{
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FINGERTIP_OFFSET, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return (float)regValue / 10000.0f;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read fingertip offset." << std::endl;
        return -1.0f;
    }
}

float RG::getWidth()
{
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_ACTUAL_WIDTH, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return (float)regValue / 10000.0f;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read width." << std::endl;
        return -1.0f;
    }
}

std::vector<int> RG::getStatus()
{
    std::vector<int> status_list(7, 0);
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_STATUS, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t reg = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        status_list[0] = (reg & STATUS_BUSY) ? 1 : 0;
        status_list[1] = (reg & STATUS_GRIP_DETECTED) ? 1 : 0;
        status_list[2] = (reg & STATUS_S1_PUSHED) ? 1 : 0;
        status_list[3] = (reg & STATUS_S1_TRIGGERED) ? 1 : 0;
        status_list[4] = (reg & STATUS_S2_PUSHED) ? 1 : 0;
        status_list[5] = (reg & STATUS_S2_TRIGGERED) ? 1 : 0;
        status_list[6] = (reg & STATUS_SAFETY_ERROR) ? 1 : 0;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read status." << std::endl;
        status_list.assign(7, -1);
    }
    return status_list;
}

std::vector<int> RG::getStatusAndPrint()
{
    std::vector<int> status_list(7, 0);
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_STATUS, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t reg = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        // Interpret individual bits.
        if (reg & STATUS_BUSY)
        {
            std::cout << "A motion is ongoing so new commands are not accepted." << std::endl;
            status_list[0] = 1;
        }
        if (reg & STATUS_GRIP_DETECTED)
        {
            std::cout << "An internal- or external grip is detected." << std::endl;
            status_list[1] = 1;
        }
        if (reg & STATUS_S1_PUSHED)
        {
            std::cout << "Safety switch 1 is pushed." << std::endl;
            status_list[2] = 1;
        }
        if (reg & STATUS_S1_TRIGGERED)
        {
            std::cout << "Safety circuit 1 is activated so it will not move." << std::endl;
            status_list[3] = 1;
        }
        if (reg & STATUS_S2_PUSHED)
        {
            std::cout << "Safety switch 2 is pushed." << std::endl;
            status_list[4] = 1;
        }
        if (reg & STATUS_S2_TRIGGERED)
        {
            std::cout << "Safety circuit 2 is activated so it will not move." << std::endl;
            status_list[5] = 1;
        }
        if (reg & STATUS_SAFETY_ERROR)
        {
            std::cout << "Any of the safety switches is pushed." << std::endl;
            status_list[6] = 1;
        }
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read status." << std::endl;
        status_list.assign(7, -1);
    }
    return status_list;
}

float RG::getWidthWithOffset()
{
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_ACTUAL_WIDTH_WITH_OFFSET, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return (float)regValue / 10000.0f;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read width with offset." << std::endl;
        return -1.0f;
    }
}

float RG::getCurrentForce()
{
    MB::ModbusRequest req(DEVICE_ID, MB::utils::ReadAnalogOutputHoldingRegisters, REG_FORCE, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return static_cast<float>(regValue) / 10.0f; // Force in N (divided by 10 based on documentation)
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read current force." << std::endl;
        return -1.0f;
    }
}

float RG::getMinWidth()
{
    // For RG grippers, minimum width is typically 0
    return min_width;
}

float RG::getMaxWidth()
{
    return max_width;
}