#include <gtest/gtest.h>
#include <vector>
#include <cstdint>

// Nota: modbusRequest.hpp incluye modbusUtils.hpp que ya incluye MB/crc.hpp con guardas
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"
#include "MB/modbusException.hpp"

// =============================================================================
// Suite 1: Algoritmo de Verificación de Redundancia Cíclica (CRC-16 Modbus)
// =============================================================================
class ModbusCRCTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Tramas de referencia estándar Modbus RTU (Slave=0x11, varios códigos de función)
        req_fn1_payload = {0x11, 0x01, 0x00, 0x13, 0x00, 0x25}; // CRC esperado: 0x840E (0x0E, 0x84)
        req_fn3_payload = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03}; // CRC esperado: 0x8776 (0x76, 0x87)
        req_fn6_payload = {0x11, 0x06, 0x00, 0x01, 0x00, 0x03}; // CRC esperado: 0x9B9A (0x9A, 0x9B)
        req_fn16_payload = {0x11, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02}; // CRC esperado: 0xF0C6
    }

    std::vector<uint8_t> req_fn1_payload;
    std::vector<uint8_t> req_fn3_payload;
    std::vector<uint8_t> req_fn6_payload;
    std::vector<uint8_t> req_fn16_payload;
};

TEST_F(ModbusCRCTest, StandardVectorsMatchExpectedCRC) {
    uint16_t crc1 = MB::CRC::calculateCRC(req_fn1_payload.data(), req_fn1_payload.size());
    EXPECT_EQ(crc1, 0x840E);

    uint16_t crc3 = MB::CRC::calculateCRC(req_fn3_payload.data(), req_fn3_payload.size());
    EXPECT_EQ(crc3, 0x8776);

    uint16_t crc6 = MB::CRC::calculateCRC(req_fn6_payload.data(), req_fn6_payload.size());
    EXPECT_EQ(crc6, 0x9B9A);

    uint16_t crc16 = MB::CRC::calculateCRC(req_fn16_payload.data(), req_fn16_payload.size());
    EXPECT_EQ(crc16, 0xF0C6);
}

TEST_F(ModbusCRCTest, ZeroLengthAndSingleByteSafety) {
    // Un buffer vacío debe devolver el valor inicial del registro CRC (0xFFFF)
    uint16_t crc_empty = MB::CRC::calculateCRC(nullptr, 0);
    EXPECT_EQ(crc_empty, 0xFFFF);

    uint8_t single_byte = 0x41;
    uint16_t crc_single = MB::CRC::calculateCRC(&single_byte, 1);
    EXPECT_NE(crc_single, 0xFFFF);
}

TEST_F(ModbusCRCTest, BitCorruptionChangesCRC) {
    std::vector<uint8_t> corrupted = req_fn3_payload;
    corrupted[3] ^= 0x01; // Invertir 1 bit

    uint16_t crc_original = MB::CRC::calculateCRC(req_fn3_payload.data(), req_fn3_payload.size());
    uint16_t crc_corrupted = MB::CRC::calculateCRC(corrupted.data(), corrupted.size());

    EXPECT_NE(crc_original, crc_corrupted);
}

// =============================================================================
// Suite 2: Serialización y Deserialización de Tramas ModbusRequest
// =============================================================================
class ModbusRequestFramesTest : public ::testing::Test {};

TEST_F(ModbusRequestFramesTest, Function03ReadHoldingRegisters) {
    // Slave 0x41 (65 OnRobot default), FC 0x03, Reg 256, 1 register
    std::vector<uint8_t> raw = {0x41, 0x03, 0x01, 0x00, 0x00, 0x01};
    uint16_t crc = MB::CRC::calculateCRC(raw.data(), raw.size());
    raw.push_back(crc & 0xFF);
    raw.push_back((crc >> 8) & 0xFF);

    MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
    EXPECT_EQ(req.slaveID(), 0x41);
    EXPECT_EQ(req.functionCode(), 0x03);
    EXPECT_EQ(req.registerAddress(), 256);
    EXPECT_EQ(req.numberOfRegisters(), 1);

    // Re-serializar y verificar coincidencia exacta
    std::vector<uint8_t> output = req.toRaw();
    uint16_t out_crc = MB::CRC::calculateCRC(output.data(), output.size());
    output.push_back(out_crc & 0xFF);
    output.push_back((out_crc >> 8) & 0xFF);
    EXPECT_EQ(output, raw);
}

TEST_F(ModbusRequestFramesTest, Function06WriteSingleRegister) {
    // Slave 0x41, FC 0x06, Reg 3 (Command), Value 1 (Grip External)
    std::vector<uint8_t> raw = {0x41, 0x06, 0x00, 0x03, 0x00, 0x01};
    uint16_t crc = MB::CRC::calculateCRC(raw.data(), raw.size());
    raw.push_back(crc & 0xFF);
    raw.push_back((crc >> 8) & 0xFF);

    MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
    EXPECT_EQ(req.slaveID(), 0x41);
    EXPECT_EQ(req.functionCode(), 0x06);
    EXPECT_EQ(req.registerAddress(), 3);
    ASSERT_GE(req.registerValues().size(), 1u);
    EXPECT_EQ(req.registerValues()[0].reg(), 1);
}

TEST_F(ModbusRequestFramesTest, Function16WriteMultipleRegisters) {
    // Slave 65, FC 0x10, Reg 0, 2 registers: Width=350 (0x015E), Force=140 (0x008C)
    std::vector<uint8_t> raw = {
        0x41, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
        0x01, 0x5E, 0x00, 0x8C
    };
    uint16_t crc = MB::CRC::calculateCRC(raw.data(), raw.size());
    raw.push_back(crc & 0xFF);
    raw.push_back((crc >> 8) & 0xFF);

    MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
    EXPECT_EQ(req.slaveID(), 0x41);
    EXPECT_EQ(req.functionCode(), 0x10);
    EXPECT_EQ(req.registerAddress(), 0);
    EXPECT_EQ(req.numberOfRegisters(), 2);
    ASSERT_EQ(req.registerValues().size(), 2u);
    EXPECT_EQ(req.registerValues()[0].reg(), 0x015E);
    EXPECT_EQ(req.registerValues()[1].reg(), 0x008C);
}

TEST_F(ModbusRequestFramesTest, CorruptedCRCThrowsException) {
    std::vector<uint8_t> raw = {0x41, 0x03, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00}; // CRC inválido
    EXPECT_THROW(MB::ModbusRequest::fromRawCRC(raw), MB::ModbusException);
}

// =============================================================================
// Suite 3: Serialización y Deserialización de Tramas ModbusResponse
// =============================================================================
class ModbusResponseFramesTest : public ::testing::Test {};

TEST_F(ModbusResponseFramesTest, ValidResponseWithRegisters) {
    // Respuesta a lectura de 2 registros (Status=0x0001, Width=350 tenth mm):
    // Slave 65, FC 3, ByteCount 4, Reg1=0x0001, Reg2=0x015E
    std::vector<uint8_t> raw = {0x41, 0x03, 0x04, 0x00, 0x01, 0x01, 0x5E};
    uint16_t crc = MB::CRC::calculateCRC(raw.data(), raw.size());
    raw.push_back(crc & 0xFF);
    raw.push_back((crc >> 8) & 0xFF);

    MB::ModbusResponse resp = MB::ModbusResponse::fromRawCRC(raw);
    EXPECT_EQ(resp.slaveID(), 0x41);
    EXPECT_EQ(resp.functionCode(), 0x03);
    ASSERT_EQ(resp.registerValues().size(), 2u);
    EXPECT_EQ(resp.registerValues()[0].reg(), 0x0001);
    EXPECT_EQ(resp.registerValues()[1].reg(), 350);
}
