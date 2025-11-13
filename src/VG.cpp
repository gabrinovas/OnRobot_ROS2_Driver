#include "VG.hpp"

VG::VG(const std::string &type, const std::string &ip, int port, int device_address)
    : type(type), device_address_(device_address)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "vg10" && type != "vgc10")
        throw std::invalid_argument("Please specify either 'vg10' or 'vgc10'.");

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

    // Set default current limit
    setCurrentLimit(DEFAULT_CURRENT_LIMIT);
}

VG::VG(const std::string &type, const std::string &device, int device_address)
    : type(type), device_address_(device_address)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "vg10" && type != "vgc10")
        throw std::invalid_argument("Please specify either 'vg10' or 'vgc10'.");

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

    // Set default current limit
    setCurrentLimit(DEFAULT_CURRENT_LIMIT);
}

VG::~VG()
{
    if (connection)
        connection->close();
}

MB::ModbusResponse VG::sendRequest(const MB::ModbusRequest &req)
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

uint16_t VG::createControlValue(uint8_t control_mode, uint8_t target_vacuum)
{
    // Control register format:
    // Bits 15-8: Control mode
    // Bits 7-0: Target vacuum (only used in grip mode)
    return (static_cast<uint16_t>(control_mode) << 8) | target_vacuum;
}

void VG::grip()
{
    // Grip with default vacuum level on both channels
    setChannelA(MODE_GRIP, DEFAULT_VACUUM);
    setChannelB(MODE_GRIP, DEFAULT_VACUUM);
}

void VG::gripWithStatus()
{
    setChannelA(MODE_GRIP, DEFAULT_VACUUM);
    setChannelB(MODE_GRIP, DEFAULT_VACUUM);
    
    // Wait for vacuum to build up (simple implementation)
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Check if grip was successful
    float vacuum_a = getChannelAVacuum();
    float vacuum_b = getChannelBVacuum();
    
    if (vacuum_a < 10.0f && vacuum_b < 10.0f) {
        std::cerr << "Grip may not have been successful. Vacuum levels too low." << std::endl;
    }
}

void VG::release()
{
    // Release both channels
    setChannelA(MODE_RELEASE, 0);
    setChannelB(MODE_RELEASE, 0);
}

void VG::idle()
{
    // Set both channels to idle
    setChannelA(MODE_IDLE, 0);
    setChannelB(MODE_IDLE, 0);
}

void VG::gripChannelA(uint8_t target_vacuum)
{
    setChannelA(MODE_GRIP, target_vacuum);
}

void VG::gripChannelB(uint8_t target_vacuum)
{
    setChannelB(MODE_GRIP, target_vacuum);
}

void VG::setChannelA(uint8_t control_mode, uint8_t target_vacuum)
{
    if (target_vacuum > 80) {
        std::cerr << "Warning: Target vacuum should not exceed 80%" << std::endl;
        target_vacuum = 80;
    }

    uint16_t control_value = createControlValue(control_mode, target_vacuum);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(control_value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CHANNEL_A_CONTROL, 1, values);
    
    try
    {
        sendRequest(req);
        std::cout << "Channel A set to mode: " << static_cast<int>(control_mode) 
                  << ", vacuum: " << static_cast<int>(target_vacuum) << "%" << std::endl;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set Channel A control." << std::endl;
        throw;
    }
}

void VG::setChannelB(uint8_t control_mode, uint8_t target_vacuum)
{
    if (target_vacuum > 80) {
        std::cerr << "Warning: Target vacuum should not exceed 80%" << std::endl;
        target_vacuum = 80;
    }

    uint16_t control_value = createControlValue(control_mode, target_vacuum);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(control_value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CHANNEL_B_CONTROL, 1, values);
    
    try
    {
        sendRequest(req);
        std::cout << "Channel B set to mode: " << static_cast<int>(control_mode) 
                  << ", vacuum: " << static_cast<int>(target_vacuum) << "%" << std::endl;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set Channel B control." << std::endl;
        throw;
    }
}

void VG::setCurrentLimit(uint16_t current_ma)
{
    if (current_ma > 1000) {
        std::cerr << "Warning: Current limit should not exceed 1000mA" << std::endl;
        current_ma = 1000;
    }

    std::vector<MB::ModbusCell> values = {MB::ModbusCell(current_ma)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CURRENT_LIMIT, 1, values);
    
    try
    {
        sendRequest(req);
        std::cout << "Current limit set to: " << current_ma << "mA" << std::endl;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to set current limit." << std::endl;
        throw;
    }
}

float VG::getChannelAVacuum()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CHANNEL_A_VACUUM, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        
        // Convert from 1/1000 of relative vacuum to percentage
        // Documentation says: "provided in 1/1000 of relative vacuum"
        // Assuming full scale is 100% = 1000 units
        float vacuum_percent = static_cast<float>(regValue) / 10.0f;
        
        return vacuum_percent;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read Channel A vacuum." << std::endl;
        return -1.0f;
    }
}

float VG::getChannelBVacuum()
{
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CHANNEL_B_VACUUM, 1);
    try
    {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        
        // Convert from 1/1000 of relative vacuum to percentage
        float vacuum_percent = static_cast<float>(regValue) / 10.0f;
        
        return vacuum_percent;
    }
    catch (const MB::ModbusException &)
    {
        std::cerr << "Failed to read Channel B vacuum." << std::endl;
        return -1.0f;
    }
}

std::vector<int> VG::getStatus()
{
    std::vector<int> status_list(2, 0);
    
    try
    {
        // Read vacuum levels to determine status
        float vacuum_a = getChannelAVacuum();
        float vacuum_b = getChannelBVacuum();
        
        // Simple status: 1 if vacuum detected, 0 otherwise
        // Using 5% threshold to account for noise
        status_list[0] = (vacuum_a > 5.0f) ? 1 : 0;
        status_list[1] = (vacuum_b > 5.0f) ? 1 : 0;
    }
    catch (const std::exception &)
    {
        std::cerr << "Failed to read VG status." << std::endl;
        status_list.assign(2, -1);
    }
    
    return status_list;
}

bool VG::isGripping()
{
    float vacuum_a = getChannelAVacuum();
    float vacuum_b = getChannelBVacuum();
    
    // Consider gripping if either channel has significant vacuum
    float vacuum_threshold = 10.0f; // 10% vacuum threshold
    return (vacuum_a > vacuum_threshold) || (vacuum_b > vacuum_threshold);
}