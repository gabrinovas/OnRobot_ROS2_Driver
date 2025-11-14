#include "ThreeFG.hpp"

ThreeFG::ThreeFG(const std::string &ip, int port)
    : device_address_(65)
{
    if (ip.empty()) throw std::invalid_argument("IP required for TCP.");

    while (true) {
        try {
            connection = std::make_unique<TCPConnectionWrapper>(ip, port);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "TCP connection failed: " << ex.what() << ". Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Set default parameters for 3FG15
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f;

    // Set initial configuration for 3FG15
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
    setGripType(true); // Default to internal grip for 3FG15
}

ThreeFG::ThreeFG(const std::string &device, int baudrate)
    : device_address_(65)
{
    if (device.empty()) throw std::invalid_argument("Device path required for serial.");

    while (true) {
        try {
            connection = std::make_unique<SerialConnectionWrapper>(device);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Serial connection failed: " << ex.what() << ". Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    // Set default parameters for 3FG15
    default_force_ = MAX_FORCE / 2;
    default_speed_ = 50.0f;

    // Set initial configuration for 3FG15
    setTargetForce(default_force_);
    setTargetSpeed(default_speed_);
    setGripType(true); // Default to internal grip for 3FG15
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

void ThreeFG::moveGripper(float width_val)
{
    float clamped_width = std::max(MIN_DIAMETER, std::min(width_val, MAX_DIAMETER));
    setTargetWidth(clamped_width);
    gripInternal(); // 3FG15 only has internal grip (centripetal)
}

void ThreeFG::gripInternal()
{
    setCommand(CMD_GRIP_INTERNAL);
}

void ThreeFG::stop()
{
    setCommand(CMD_STOP);
}

void ThreeFG::setCommand(uint16_t command)
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

void ThreeFG::setTargetForce(float force_val)
{
    float clamped_force = std::max(0.0f, std::min(force_val, MAX_FORCE));
    // 3FG15 force is in 10% units (0-1000 for 0-100%)
    uint16_t force_percent = static_cast<uint16_t>((clamped_force / MAX_FORCE) * 1000.0f);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(force_percent)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_FORCE, 1, values);
    try {
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target force." << std::endl;
        throw;
    }
}

void ThreeFG::setTargetWidth(float width_val)
{
    float clamped_width = std::max(MIN_DIAMETER, std::min(width_val, MAX_DIAMETER));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(toTenthMM(clamped_width))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_WIDTH, 1, values);
    try {
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target width." << std::endl;
        throw;
    }
}

void ThreeFG::setTargetSpeed(float speed_val)
{
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
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CURRENT_DIAMETER, 1);
    try {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    } catch (const MB::ModbusException &) {
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

float ThreeFG::getMinWidth() { return MIN_DIAMETER; }
float ThreeFG::getMaxWidth() { return MAX_DIAMETER; }

float ThreeFG::getCurrentDiameter() {
    return getWidth(); // Reuse getWidth for current diameter
}

float ThreeFG::getDiameterWithOffset() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_DIAMETER_WITH_OFFSET, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        std::cerr << "Failed to read diameter with offset." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getAppliedForce() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_APPLIED_FORCE, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        // Convert from 1/10 % back to Newtons
        return (static_cast<float>(val) / 1000.0f) * MAX_FORCE;
    } catch (...) {
        std::cerr << "Failed to read applied force." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getMinDiameter() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 513, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        std::cerr << "Failed to read min diameter." << std::endl;
        return 0.0f;
    }
}

float ThreeFG::getMaxDiameter() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 514, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        std::cerr << "Failed to read max diameter." << std::endl;
        return MAX_DIAMETER;
    }
}

float ThreeFG::getFingerLength() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 270, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg()) / 10.0f; // 1/10 mm to mm
    } catch (...) {
        std::cerr << "Failed to read finger length." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getFingerPosition() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 272, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg());
    } catch (...) {
        std::cerr << "Failed to read finger position." << std::endl;
        return -1.0f;
    }
}

float ThreeFG::getFingertipOffset() {
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 273, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg()) / 100.0f; // 1/100 mm to mm
    } catch (...) {
        std::cerr << "Failed to read fingertip offset." << std::endl;
        return -1.0f;
    }
}

ThreeFG::GripperStatus ThreeFG::getDetailedStatus() {
    GripperStatus status{};
    uint16_t raw = getStatusRaw();
    status.busy = raw & STATUS_BUSY;
    status.grip_detected = raw & STATUS_GRIP_DETECTED;
    status.force_grip_detected = raw & STATUS_FORCE_GRIP_DETECTED;
    status.calibration_ok = raw & STATUS_CALIBRATION_OK;
    return status;
}

void ThreeFG::setGripType(bool internal) {
    uint16_t value = internal ? 1 : 0;
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_GRIP_TYPE, 1, values);
    try {
        sendRequest(req);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set grip type: " << e.what() << std::endl;
        throw;
    }
}

void ThreeFG::setFingerLength(float mm) {
    uint16_t value = static_cast<uint16_t>(mm * 10.0f); // mm to 1/10 mm
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1025, 1, values);
    try {
        sendRequest(req);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set finger length: " << e.what() << std::endl;
        throw;
    }
}

void ThreeFG::setFingerPosition(float position) {
    uint16_t value = static_cast<uint16_t>(position); // Position 1, 2, or 3
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1027, 1, values);
    try {
        sendRequest(req);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set finger position: " << e.what() << std::endl;
        throw;
    }
}

void ThreeFG::setFingertipOffset(float mm) {
    uint16_t value = static_cast<uint16_t>(mm * 100.0f); // mm to 1/100 mm
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1028, 1, values);
    try {
        sendRequest(req);
    } catch (const std::exception& e) {
        std::cerr << "Failed to set fingertip offset: " << e.what() << std::endl;
        throw;
    }
}
