// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "MB/modbusUtils.hpp"
#include "onrobot_driver/common/OnRobotGripperBase.hpp"

namespace onrobot_driver
{

MB::ModbusResponse OnRobotGripperBase::sendRequest(const MB::ModbusRequest & req)
{
  std::lock_guard<std::mutex> lock(comm_mutex_);
  total_requests_++;
  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&started]() {
      return std::chrono::duration<double,
               std::milli>(std::chrono::steady_clock::now() - started).count();
    };

  if (!connection_) {
    // The connection was dropped earlier: open it again when the way to do it is known.
    try {
      if (canReopen()) {
        connection_ = open();
        reconnect_count_++;
        std::cout << "[OnRobotGripperBase] Restored connection to " << target_endpoint_ <<
          std::endl;
      }
    } catch (...) {
      failed_requests_++;
      throw std::runtime_error("Modbus connection is not open and reconnection failed");
    }
    if (!connection_) {
      failed_requests_++;
      throw std::runtime_error("Modbus connection is not open");
    }
  }

  try {
    MB::ModbusResponse resp = connection_->sendRequest(req);
    last_roundtrip_ms_ = elapsed_ms();
    return resp;
  } catch (const std::exception & ex) {
    failed_requests_++;
    // One automatic reconnection and one retry.
    try {
      if (connection_) {
        try {
          connection_->close();
        } catch (...) {
        }
        connection_.reset();
      }
      if (canReopen()) {
        connection_ = open();
      }
      if (connection_) {
        reconnect_count_++;
        MB::ModbusResponse resp = connection_->sendRequest(req);
        last_roundtrip_ms_ = elapsed_ms();
        std::cout << "[OnRobotGripperBase] Auto-reconnect succeeded; request recovered." <<
          std::endl;
        return resp;
      }
    } catch (const std::exception & reconnect_ex) {
      throw std::runtime_error(
              "Modbus request to " + target_endpoint_ + " (unit " + std::to_string(
                device_address_) +
              ", timeout " + std::to_string(settings_.timeout_ms) + " ms) failed (" + ex.what() +
              ") and the retry after reconnecting also failed (" + reconnect_ex.what() + ")");
    }
    throw;
  }
}

uint16_t OnRobotGripperBase::readRegister(uint16_t address)
{
  MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, address, 1);
  MB::ModbusResponse resp = sendRequest(req);
  return resp.registerValues().front().isReg() ? resp.registerValues().front().reg() : 0;
}

bool OnRobotGripperBase::tryReadRegister(uint16_t address, const char * what, uint16_t & value)
{
  try {
    value = readRegister(address);
    return true;
  } catch (const MB::ModbusException &) {
    std::cerr << "Failed to read " << what << "." << std::endl;
    return false;
  }
}

void OnRobotGripperBase::writeRegister(uint16_t address, uint16_t value, const char * what)
{
  std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
  MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, address, 1,
    values);
  try {
    sendRequest(req);
  } catch (const MB::ModbusException &) {
    std::cerr << "Failed to " << what << "." << std::endl;
    throw;
  }
}

std::vector<int> OnRobotGripperBase::getStatus()
{
  const uint16_t raw = getStatusRaw();
  std::vector<int> bits(16, 0);
  for (int i = 0; i < 16; ++i) {
    bits[i] = (raw >> i) & 1;
  }
  return bits;
}

}  // namespace onrobot_driver
