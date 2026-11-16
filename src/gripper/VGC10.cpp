// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <iostream>
#include <vector>

#include "MB/modbusUtils.hpp"
#include "onrobot_driver/vgc10/VGC10.hpp"

namespace onrobot_driver
{

using namespace vgc10;  // NOLINT(build/namespaces): the register map of this gripper

namespace
{
float normalised(uint16_t raw)
{
  return std::max(0.0f, std::min(static_cast<float>(raw) / VACUUM_FULL_SCALE, 1.0f));
}
}  // namespace

VGC10::VGC10(int device_address, const VacuumSettings & settings)
: OnRobotGripperBase(device_address), settings_(settings), current_limit_ma_(
    settings.initial_current_ma)
{
}

VGC10::~VGC10()
{
  try {
    releaseAll();
  } catch (...) {
  }
  close();
}

void VGC10::onAttached()
{
  current_limit_ma_ = settings_.initial_current_ma;
  current_mode_a_ = MODE_RELEASE;
  target_vacuum_a_ = 0;
  current_mode_b_ = MODE_RELEASE;
  target_vacuum_b_ = 0;
  try {
    setCurrentLimit(current_limit_ma_);
  } catch (const std::exception & e) {
    std::cerr << "[VGC10] Warning: could not set initial current limit: " << e.what() << std::endl;
  }
}

void VGC10::writeChannelRegister(uint16_t address, uint8_t mode, uint8_t target_pct)
{
  const uint8_t clamped = std::min<uint8_t>(target_pct, settings_.max_vacuum_percent);
  const uint16_t value = (static_cast<uint16_t>(mode) << 8) | static_cast<uint16_t>(clamped);
  std::vector<MB::ModbusCell> values = {MB::ModbusCell(value)};
  MB::ModbusRequest req(device_address_, MB::utils::WriteSingleAnalogOutputRegister, address, 1,
    values);
  try {
    sendRequest(req);
  } catch (const MB::ModbusException & e) {
    std::cerr << "[VGC10] Failed to write channel register " << address << ": " << e.what() <<
      std::endl;
    throw;
  }
}

void VGC10::setCurrentLimit(uint16_t current_ma)
{
  const uint16_t clamped = std::min<uint16_t>(current_ma, settings_.max_current_ma);
  std::vector<MB::ModbusCell> values = {MB::ModbusCell(clamped)};
  MB::ModbusRequest req(
    device_address_, MB::utils::WriteSingleAnalogOutputRegister, REG_CURRENT_LIMIT, 1, values);
  try {
    sendRequest(req);
    current_limit_ma_ = clamped;
  } catch (const MB::ModbusException & e) {
    std::cerr << "[VGC10] Failed to set current limit: " << e.what() << std::endl;
    throw;
  }
}

void VGC10::setChannelControl(Channel channel, uint8_t mode, uint8_t target_vacuum_pct)
{
  if (channel == Channel::A || channel == Channel::BOTH) {
    writeChannelRegister(REG_CHANNEL_A_CTRL, mode, target_vacuum_pct);
    current_mode_a_ = mode;
    target_vacuum_a_ = target_vacuum_pct;
  }
  if (channel == Channel::B || channel == Channel::BOTH) {
    writeChannelRegister(REG_CHANNEL_B_CTRL, mode, target_vacuum_pct);
    current_mode_b_ = mode;
    target_vacuum_b_ = target_vacuum_pct;
  }
}

void VGC10::gripChannelA(uint8_t vacuum_pct) {setChannelControl(Channel::A, MODE_GRIP, vacuum_pct);}
void VGC10::releaseChannelA() {setChannelControl(Channel::A, MODE_RELEASE, 0);}
void VGC10::idleChannelA() {setChannelControl(Channel::A, MODE_IDLE, 0);}
void VGC10::gripChannelB(uint8_t vacuum_pct) {setChannelControl(Channel::B, MODE_GRIP, vacuum_pct);}
void VGC10::releaseChannelB() {setChannelControl(Channel::B, MODE_RELEASE, 0);}
void VGC10::idleChannelB() {setChannelControl(Channel::B, MODE_IDLE, 0);}
void VGC10::gripAll(uint8_t vacuum_pct) {setChannelControl(Channel::BOTH, MODE_GRIP, vacuum_pct);}
void VGC10::releaseAll() {setChannelControl(Channel::BOTH, MODE_RELEASE, 0);}
void VGC10::idleAll() {setChannelControl(Channel::BOTH, MODE_IDLE, 0);}

uint16_t VGC10::rawVacuum(uint16_t address, const char * channel)
{
  MB::ModbusRequest req(device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, address, 1);
  try {
    MB::ModbusResponse resp = sendRequest(req);
    if (!resp.registerValues().empty() && resp.registerValues().front().isReg()) {
      return resp.registerValues().front().reg();
    }
  } catch (const MB::ModbusException & e) {
    std::cerr << "[VGC10] Failed to read Channel " << channel << " vacuum: " << e.what() <<
      std::endl;
  }
  return 0;
}

uint16_t VGC10::getRawVacuumChannelA() {return rawVacuum(REG_CHANNEL_A_VACUUM, "A");}
uint16_t VGC10::getRawVacuumChannelB() {return rawVacuum(REG_CHANNEL_B_VACUUM, "B");}
float VGC10::getVacuumChannelA() {return normalised(getRawVacuumChannelA());}
float VGC10::getVacuumChannelB() {return normalised(getRawVacuumChannelB());}

bool VGC10::readBothVacuums(float & vac_a, float & vac_b)
{
  // The two vacuum registers are consecutive: A first.
  MB::ModbusRequest req(
    device_address_, MB::utils::ReadAnalogOutputHoldingRegisters, REG_CHANNEL_A_VACUUM, 2);
  MB::ModbusResponse resp = sendRequest(req);
  const auto & regs = resp.registerValues();
  if (regs.size() >= 2 && regs[0].isReg() && regs[1].isReg()) {
    vac_a = normalised(regs[0].reg());
    vac_b = normalised(regs[1].reg());
    return true;
  }
  return false;
}

float VGC10::getWidth()
{
  float vac_a = 0.0f;
  float vac_b = 0.0f;
  try {
    if (readBothVacuums(vac_a, vac_b)) {
      return (vac_a + vac_b) / 2.0f;
    }
  } catch (...) {
  }
  return (getVacuumChannelA() + getVacuumChannelB()) / 2.0f;
}

float VGC10::getForce() {return static_cast<float>(current_limit_ma_);}

uint16_t VGC10::getStatusRaw()
{
  uint16_t status = 0;
  if (current_mode_a_ == MODE_GRIP) {
    status |= STATUS_CHANNEL_A_GRIPPING;
  }
  if (current_mode_b_ == MODE_GRIP) {
    status |= STATUS_CHANNEL_B_GRIPPING;
  }
  return status;
}

std::vector<int> VGC10::getStatus()
{
  return {current_mode_a_, target_vacuum_a_, current_mode_b_, target_vacuum_b_};
}

void VGC10::setTargetForce(float force_val) {setCurrentLimit(static_cast<uint16_t>(force_val));}

void VGC10::setTargetWidth(float width_val)
{
  if (width_val > settings_.grip_threshold) {
    const float highest = static_cast<float>(settings_.max_vacuum_percent);
    gripAll(static_cast<uint8_t>(std::min(width_val * highest, highest)));
  } else {
    releaseAll();
  }
}

void VGC10::setTargetSpeed(float /*speed_val*/) {}
void VGC10::stop() {releaseAll();}
void VGC10::moveGripper(float width_val) {setTargetWidth(width_val);}

}  // namespace onrobot_driver
