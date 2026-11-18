// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "MB/modbusException.hpp"
#include "MB/modbusRequest.hpp"
#include "MB/modbusResponse.hpp"

namespace
{

/// Append the Modbus CRC-16 of a frame to it, low byte first.
std::vector<uint8_t> with_crc(std::vector<uint8_t> frame)
{
  const uint16_t crc = MB::CRC::calculateCRC(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);
  return frame;
}

uint16_t crc_of(const std::vector<uint8_t> & frame)
{
  return MB::CRC::calculateCRC(frame.data(), frame.size());
}

// Reference Modbus RTU frames (slave 0x11) and their known CRC.
const std::vector<uint8_t> kReadCoils = {0x11, 0x01, 0x00, 0x13, 0x00, 0x25};
const std::vector<uint8_t> kReadHolding = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
const std::vector<uint8_t> kWriteSingle = {0x11, 0x06, 0x00, 0x01, 0x00, 0x03};
const std::vector<uint8_t> kWriteMultiple = {
  0x11, 0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02};

TEST(ModbusCrc, StandardFramesHaveTheirKnownCrc)
{
  EXPECT_EQ(crc_of(kReadCoils), 0x840E);
  EXPECT_EQ(crc_of(kReadHolding), 0x8776);
  EXPECT_EQ(crc_of(kWriteSingle), 0x9B9A);
  EXPECT_EQ(crc_of(kWriteMultiple), 0xF0C6);
}

TEST(ModbusCrc, AnEmptyFrameGivesTheInitialValue)
{
  EXPECT_EQ(MB::CRC::calculateCRC(nullptr, 0), 0xFFFF);
  const uint8_t single_byte = 0x41;
  EXPECT_NE(MB::CRC::calculateCRC(&single_byte, 1), 0xFFFF);
}

TEST(ModbusCrc, OneFlippedBitChangesTheCrc)
{
  std::vector<uint8_t> corrupted = kReadHolding;
  corrupted[3] ^= 0x01;
  EXPECT_NE(crc_of(kReadHolding), crc_of(corrupted));
}

TEST(ModbusRequestFrames, ReadHoldingRegistersRoundTrip)
{
  // Slave 0x41 (65, the address of a single OnRobot gripper), function 3, register 256, 1 register.
  const std::vector<uint8_t> raw = with_crc({0x41, 0x03, 0x01, 0x00, 0x00, 0x01});

  const MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
  EXPECT_EQ(req.slaveID(), 0x41);
  EXPECT_EQ(req.functionCode(), 0x03);
  EXPECT_EQ(req.registerAddress(), 256);
  EXPECT_EQ(req.numberOfRegisters(), 1);
  EXPECT_EQ(with_crc(req.toRaw()), raw);
}

TEST(ModbusRequestFrames, WriteSingleRegister)
{
  // Slave 0x41, function 6, register 3 (command), value 1 (grip external).
  const std::vector<uint8_t> raw = with_crc({0x41, 0x06, 0x00, 0x03, 0x00, 0x01});

  const MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
  EXPECT_EQ(req.slaveID(), 0x41);
  EXPECT_EQ(req.functionCode(), 0x06);
  EXPECT_EQ(req.registerAddress(), 3);
  ASSERT_GE(req.registerValues().size(), 1u);
  EXPECT_EQ(req.registerValues()[0].reg(), 1);
}

TEST(ModbusRequestFrames, WriteMultipleRegisters)
{
  // Slave 0x41, function 16, register 0, 2 registers: width 350 (0x015E), force 140 (0x008C).
  const std::vector<uint8_t> raw = with_crc(
    {0x41, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x01, 0x5E, 0x00, 0x8C});

  const MB::ModbusRequest req = MB::ModbusRequest::fromRawCRC(raw);
  EXPECT_EQ(req.slaveID(), 0x41);
  EXPECT_EQ(req.functionCode(), 0x10);
  EXPECT_EQ(req.registerAddress(), 0);
  EXPECT_EQ(req.numberOfRegisters(), 2);
  ASSERT_EQ(req.registerValues().size(), 2u);
  EXPECT_EQ(req.registerValues()[0].reg(), 0x015E);
  EXPECT_EQ(req.registerValues()[1].reg(), 0x008C);
}

TEST(ModbusRequestFrames, AWrongCrcIsRejected)
{
  const std::vector<uint8_t> raw = {0x41, 0x03, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00};
  EXPECT_THROW(MB::ModbusRequest::fromRawCRC(raw), MB::ModbusException);
}

TEST(ModbusResponseFrames, ResponseWithTwoRegisters)
{
  // Answer to a read of 2 registers: slave 0x41, function 3, 4 bytes, status 0x0001, width 350.
  const std::vector<uint8_t> raw = with_crc({0x41, 0x03, 0x04, 0x00, 0x01, 0x01, 0x5E});

  const MB::ModbusResponse resp = MB::ModbusResponse::fromRawCRC(raw);
  EXPECT_EQ(resp.slaveID(), 0x41);
  EXPECT_EQ(resp.functionCode(), 0x03);
  ASSERT_EQ(resp.registerValues().size(), 2u);
  EXPECT_EQ(resp.registerValues()[0].reg(), 0x0001);
  EXPECT_EQ(resp.registerValues()[1].reg(), 350);
}

}  // namespace
