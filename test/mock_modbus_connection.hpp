// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef MOCK_MODBUS_CONNECTION_HPP_
#define MOCK_MODBUS_CONNECTION_HPP_

#include <map>
#include <vector>

#include "MB/modbusUtils.hpp"
#include "onrobot_driver/common/IModbusConnection.hpp"

/// Modbus connection for tests: a table of holding registers that answers reads and stores writes,
/// and keeps every write in order.
class MockModbusConnection : public IModbusConnection
{
public:
  struct Write
  {
    uint16_t address;
    uint16_t value;
    bool operator==(const Write & other) const
    {
      return address == other.address && value == other.value;
    }
  };

  MB::ModbusResponse sendRequest(const MB::ModbusRequest & req) override
  {
    const auto function = static_cast<MB::utils::MBFunctionCode>(req.functionCode());
    const uint16_t address = req.registerAddress();
    const uint16_t count = req.numberOfRegisters();

    if (function == MB::utils::ReadAnalogInputRegisters ||
      function == MB::utils::ReadAnalogOutputHoldingRegisters)
    {
      std::vector<MB::ModbusCell> cells;
      for (uint16_t i = 0; i < count; ++i) {
        cells.emplace_back(registers[address + i]);
      }
      return MB::ModbusResponse(req.slaveID(), function, address, count, cells);
    }
    if (function == MB::utils::WriteSingleAnalogOutputRegister) {
      if (!req.registerValues().empty()) {
        registers[address] = req.registerValues()[0].reg();
        writes.push_back({address, registers[address]});
      }
      return MB::ModbusResponse(req.slaveID(), function, address, 1, req.registerValues());
    }
    return MB::ModbusResponse(req.slaveID(), function);
  }

  void close() override {closed = true;}

  std::map<uint16_t, uint16_t> registers;
  std::vector<Write> writes;
  bool closed{false};
};

#endif  // MOCK_MODBUS_CONNECTION_HPP_
