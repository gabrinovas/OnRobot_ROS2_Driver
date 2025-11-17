#include "FG.hpp"


void FG::initParams() {
    if (type == "2fg7") {
        params = {0.070f, 0.0f, 70.0f, 35.0f, 50.0f};  // 70mm, 70N
    } else if (type == "2fg14") {
        params = {0.140f, 0.0f, 140.0f, 70.0f, 50.0f}; // 140mm, 140N
    } else if (type == "3fg15") {
        params = {0.150f, 0.0f, 140.0f, 2.0f, 20.0f}; // 150mm diámetro, 240N máx
    } else {
        throw std::invalid_argument("Unsupported gripper type: " + type);
    }
}

FG::FG(const std::string &type, const std::string &ip, int port)
    : type(type), device_address_(65)
{
    if (ip.empty()) throw std::invalid_argument("IP required for TCP.");
    if (type != "2fg7" && type != "2fg14" && type != "3fg15")
        throw std::invalid_argument("Invalid gripper type: " + type);

    while (true) {
        try {
            connection = std::make_unique<TCPConnectionWrapper>(ip, port);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "TCP connection failed: " << ex.what() << ". Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    initParams();
    setTargetForce(params.default_force);
    //setTargetSpeed(params.default_speed);
}

/*
FG::FG(const std::string &type, const std::string &device, int baudrate)
    : type(type), device_address_(65)
{
    if (device.empty()) throw std::invalid_argument("Device required for serial.");
    if (type != "2fg7" && type != "2fg14" && type != "3fg15")
        throw std::invalid_argument("Invalid gripper type: " + type);

    while (true) {
        try {
            connection = std::make_unique<SerialConnectionWrapper>(device, baudrate);
            break;
        } catch (const std::exception &ex) {
            std::cerr << "Serial connection failed: " << ex.what() << ". Retrying..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    initParams();
    setTargetForce(params.default_force);
    //setTargetSpeed(params.default_speed);
}
*/
FG::~FG()
{
    if (connection)
        connection->close();
}

MB::ModbusResponse FG::sendRequest(const MB::ModbusRequest &req)
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

float FG::fromTenthMM(uint16_t tenth_mm)
{
    return static_cast<float>(tenth_mm) / 10000.0f; // Convert 1/10 mm to meters
}

uint16_t FG::toTenthMM(float meters)
{
    return static_cast<uint16_t>(meters * 10000.0f); // Convert meters to 1/10 mm
}

void FG::moveGripper(float width_val, bool external_grip)
{
    float clamped_width = std::max(params.min_width, std::min(width_val, params.max_width));
    setTargetWidth(clamped_width);

    if (type == "3fg15") {
        gripInternal(); // 3FG15 solo tiene agarre interno (centrípeto)
    } else {
        if (external_grip) {
            gripExternal();
        } else {
            gripInternal();
        }
    }
}

void FG::gripExternal()
{
    if (type == "3fg15") {
        // 3FG15 no tiene agarre externo → mapear a interno o ignorar
        std::cerr << "3FG15 does not support external grip. Using internal grip." << std::endl;
        gripInternal();
    } else {
        setCommand(CMD_GRIP_EXTERNAL);
    }
}


void FG::gripInternal()
{
    setCommand(CMD_GRIP_INTERNAL); // Válido para todos
}

void FG::stop()
{
    setCommand(CMD_STOP);
}

void FG::setCommand(uint16_t command)
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

void FG::setTargetForce(float force_val)
{
    float clamped_force = std::max(0.0f, std::min(force_val, params.max_force));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(static_cast<uint16_t>(clamped_force))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_FORCE, 1, values);
    try {
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target force." << std::endl;
        throw;
    }
}


void FG::setTargetWidth(float width_val)
{
    float clamped_width = std::max(params.min_width, std::min(width_val, params.max_width));
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(toTenthMM(clamped_width))};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_TARGET_WIDTH, 1, values);
    try {
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target width." << std::endl;
        throw;
    }
}

void FG::setTargetSpeed(float speed_val)
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

float FG::getWidth()
{
    uint16_t reg = (type == "3fg15") ? REG_EXTERNAL_WIDTH : REG_EXTERNAL_WIDTH;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, reg, 1);
    try {
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return fromTenthMM(regValue);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read width." << std::endl;
        return -1.0f;
    }
}
std::vector<int> FG::getStatus()
{
    std::vector<int> status_list(4, 0);
    uint16_t status_raw = getStatusRaw();
    
    status_list[0] = (status_raw & STATUS_BUSY) ? 1 : 0;
    status_list[1] = (status_raw & STATUS_GRIP_DETECTED) ? 1 : 0;
    status_list[2] = (status_raw & STATUS_ERROR_NOT_CALIBRATED) ? 1 : 0;
    status_list[3] = (status_raw & STATUS_ERROR_LINEAR_SENSOR) ? 1 : 0;
    
    return status_list;
}

uint16_t FG::getStatusRaw()
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

float FG::getMinWidth() { return params.min_width; }
float FG::getMaxWidth() { return params.max_width; }

float FG::getCurrentDiameter() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 257, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        std::cerr << "Failed to read current diameter." << std::endl;
        return -1.0f;
    }
}

float FG::getDiameterWithOffset() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 258, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        return -1.0f;
    }
}

float FG::getAppliedForce() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 259, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg());
    } catch (...) {
        return -1.0f;
    }
}

float FG::getMinDiameter() {
    if (type != "3fg15") return params.min_width;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 513, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        return 0.0f;
    }
}

float FG::getMaxDiameter() {
    if (type != "3fg15") return params.max_width;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 514, 1);
    try {
        auto resp = sendRequest(req);
        uint16_t val = resp.registerValues().front().reg();
        return fromTenthMM(val);
    } catch (...) {
        return 0.15f;
    }
}

float FG::getFingerLength() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 270, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg());
    } catch (...) {
        return -1.0f;
    }
}

float FG::getFingerPosition() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 272, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg());
    } catch (...) {
        return -1.0f;
    }
}

float FG::getFingertipOffset() {
    if (type != "3fg15") return -1.0f;
    MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, 273, 1);
    try {
        auto resp = sendRequest(req);
        return static_cast<float>(resp.registerValues().front().reg());
    } catch (...) {
        return -1.0f;
    }
}

FG::GripperStatus FG::getDetailedStatus() {
    GripperStatus status{};
    if (type != "3fg15") return status;

    uint16_t raw = getStatusRaw();  // Reusa el registro 256
    status.busy = raw & 0x0001;
    status.grip_detected = raw & 0x0002;
    status.force_grip_detected = raw & 0x0004;
    status.calibration_ok = raw & 0x0008;
    return status;
}

// === ESCRITURA DE CONFIGURACIÓN ===
void FG::setGripType(bool internal) {
    if (type != "3fg15") return;
    uint16_t value = internal ? 1 : 0;
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 2, 1, values);
    sendRequest(req);
}

void FG::setFingerLength(float mm) {
    if (type != "3fg15") return;
    uint16_t value = static_cast<uint16_t>(mm);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1025, 1, values);
    sendRequest(req);
}

void FG::setFingerPosition(float mm) {
    if (type != "3fg15") return;
    uint16_t value = static_cast<uint16_t>(mm);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1027, 1, values);
    sendRequest(req);
}

void FG::setFingertipOffset(float mm) {
    if (type != "3fg15") return;
    uint16_t value = static_cast<uint16_t>(mm);
    std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
    MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, 1028, 1, values);
    sendRequest(req);
}
