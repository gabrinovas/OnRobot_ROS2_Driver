#include "RG.hpp"

RG::RG(const std::string &type, const std::string &ip, int port)
    : type(type)
{
    if (ip.empty())
        throw std::invalid_argument("Please provide an IP address for TCP connection.");
    if (type != "rg2" && type != "rg6")
        throw std::invalid_argument("Please specify either 'rg2' or 'rg6'.");

    if (type == "rg2") {
        max_width = 1100;
        max_force = 400;
    } else if (type == "rg6") {
        max_width = 1600;
        max_force = 1200;
    }
    // Instantiate the TCP connection wrapper.
    connection = std::make_unique<TCPConnectionWrapper>(ip, port);
}

RG::RG(const std::string &type, const std::string &device)
    : type(type)
{
    if (device.empty())
        throw std::invalid_argument("Please provide a serial device for connection.");
    if (type != "rg2" && type != "rg6")
        throw std::invalid_argument("Please specify either 'rg2' or 'rg6'.");

    if (type == "rg2") {
        max_width = 1100;
        max_force = 400;
    } else if (type == "rg6") {
        max_width = 1600;
        max_force = 1200;
    }
    // Instantiate the Serial connection wrapper.
    connection = std::make_unique<SerialConnectionWrapper>(device);
}

RG::~RG() {
    closeConnection();
}

void RG::closeConnection() {
    if (connection)
        connection->close();
}

MB::ModbusResponse RG::sendRequest(const MB::ModbusRequest &req) {
    try {
        return connection->sendRequest(req);
    } catch (const MB::ModbusException &ex) {
        std::cerr << "Modbus exception: " << ex.what() << std::endl;
        throw;
    }
}

float RG::getFingertipOffset() {
    try {
        // Read one holding register at address 258 (function code 0x03).
        MB::ModbusRequest req(65, MB::utils::ReadAnalogOutputHoldingRegisters, 258, 1);
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return regValue / 10.0f;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read fingertip offset." << std::endl;
        return -1.0f;
    }
}

float RG::getWidth() {
    try {
        MB::ModbusRequest req(65, MB::utils::ReadAnalogOutputHoldingRegisters, 267, 1);
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return regValue / 10.0f;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read width." << std::endl;
        return -1.0f;
    }
}

std::vector<int> RG::getStatus() {
    std::vector<int> status_list(7, 0);
    try {
        MB::ModbusRequest req(65, MB::utils::ReadAnalogOutputHoldingRegisters, 268, 1);
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t reg = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        // Interpret bits (assuming LSB is bit 0)
        if (reg & (1 << 0)) {
            std::cout << "A motion is ongoing so new commands are not accepted." << std::endl;
            status_list[0] = 1;
        }
        if (reg & (1 << 1)) {
            std::cout << "An internal- or external grip is detected." << std::endl;
            status_list[1] = 1;
        }
        if (reg & (1 << 2)) {
            std::cout << "Safety switch 1 is pushed." << std::endl;
            status_list[2] = 1;
        }
        if (reg & (1 << 3)) {
            std::cout << "Safety circuit 1 is activated so it will not move." << std::endl;
            status_list[3] = 1;
        }
        if (reg & (1 << 4)) {
            std::cout << "Safety switch 2 is pushed." << std::endl;
            status_list[4] = 1;
        }
        if (reg & (1 << 5)) {
            std::cout << "Safety circuit 2 is activated so it will not move." << std::endl;
            status_list[5] = 1;
        }
        if (reg & (1 << 6)) {
            std::cout << "Any of the safety switches is pushed." << std::endl;
            status_list[6] = 1;
        }
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read status." << std::endl;
        status_list.assign(7, -1);
    }
    return status_list;
}

float RG::getWidthWithOffset() {
    try {
        MB::ModbusRequest req(65, MB::utils::ReadAnalogOutputHoldingRegisters, 275, 1);
        MB::ModbusResponse resp = sendRequest(req);
        uint16_t regValue = resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
        return regValue / 10.0f;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to read width with offset." << std::endl;
        return -1.0f;
    }
}

void RG::setControlMode(uint16_t command) {
    try {
        // Write single register at address 2 (function code 0x06).
        MB::ModbusRequest req(65, MB::utils::WriteSingleAnalogOutputRegister, 2, 1, { MB::ModbusCell(command) });
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set control mode." << std::endl;
    }
}

void RG::setTargetForce(uint16_t force_val) {
    try {
        MB::ModbusRequest req(65, MB::utils::WriteSingleAnalogOutputRegister, 0, 1, { MB::ModbusCell(force_val) });
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target force." << std::endl;
    }
}

void RG::setTargetWidth(uint16_t width_val) {
    try {
        MB::ModbusRequest req(65, MB::utils::WriteSingleAnalogOutputRegister, 1, 1, { MB::ModbusCell(width_val) });
        sendRequest(req);
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to set target width." << std::endl;
    }
}

void RG::closeGripper(uint16_t force_val) {
    try {
        // Prepare parameters: [force, 0, 16]
        std::vector<MB::ModbusCell> values = { MB::ModbusCell(static_cast<uint16_t>(force_val)),
            MB::ModbusCell(static_cast<uint16_t>(0)),
            MB::ModbusCell(static_cast<uint16_t>(16)) };

        MB::ModbusRequest req(65, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, 0, 3, values);
        sendRequest(req);
        std::cout << "Start closing gripper." << std::endl;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to close gripper." << std::endl;
    }
}

void RG::openGripper(uint16_t force_val) {
    try {
        // Parameters: [force, max_width, 16]
        std::vector<MB::ModbusCell> values = { MB::ModbusCell(static_cast<uint16_t>(force_val)),
            MB::ModbusCell(static_cast<uint16_t>(max_width)),
            MB::ModbusCell(static_cast<uint16_t>(16)) };

        MB::ModbusRequest req(65, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, 0, 3, values);
        sendRequest(req);
        std::cout << "Start opening gripper." << std::endl;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to open gripper." << std::endl;
    }
}

void RG::moveGripper(uint16_t width_val, uint16_t force_val) {
    try {
        // Parameters: [force, width, 16]
        std::vector<MB::ModbusCell> values = { MB::ModbusCell(static_cast<uint16_t>(force_val)),
            MB::ModbusCell(static_cast<uint16_t>(width_val)),
            MB::ModbusCell(static_cast<uint16_t>(16)) };

        MB::ModbusRequest req(65, MB::utils::WriteMultipleAnalogOutputHoldingRegisters, 0, 3, values);
        sendRequest(req);
        std::cout << "Start moving gripper." << std::endl;
    } catch (const MB::ModbusException &) {
        std::cerr << "Failed to move gripper." << std::endl;
    }
}
