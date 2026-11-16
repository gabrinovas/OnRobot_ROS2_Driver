// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>
#include <vector>

#include "onrobot_driver/twofg/TwoFG.hpp"
#include "onrobot_driver/twofg/twofg_registers.hpp"

// The register map of this gripper.
using namespace onrobot_driver::twofg;  // NOLINT(build/namespaces)

TwoFG::TwoFG(int device_address, const onrobot_driver::FingerSettings & settings)
: onrobot_driver::OnRobotGripperBase(device_address), settings_(settings)
{
}

TwoFG::~TwoFG()
{
  close();
}

void TwoFG::applyDefaults()
{
  setTargetForce(settings_.default_force);
  setTargetSpeed(settings_.default_speed);
}

float TwoFG::clampedWidth(float width_val) const
{
  return std::max(settings_.min_width, std::min(width_val, settings_.max_width));
}

void TwoFG::moveGripper(float width_val, bool external_grip)
{
  setTargetWidth(clampedWidth(width_val));
  if (external_grip) {
    gripExternal();
  } else {
    gripInternal();
  }
}

void TwoFG::gripExternal()
{
  setCommand(CMD_GRIP_EXTERNAL);
}

void TwoFG::gripInternal()
{
  setCommand(CMD_GRIP_INTERNAL);
}

void TwoFG::stop()
{
  setCommand(CMD_STOP);
}

void TwoFG::setCommand(uint16_t command)
{
  writeRegister(REG_COMMAND, command, "set command");
}

void TwoFG::setTargetForce(float force_val)
{
  const float clamped = std::max(0.0f, std::min(force_val, settings_.max_force));
  writeRegister(REG_TARGET_FORCE, static_cast<uint16_t>(clamped), "set target force");
}

void TwoFG::setTargetWidth(float width_val)
{
  writeRegister(REG_TARGET_WIDTH, toTenthMM(clampedWidth(width_val)), "set target width");
}

void TwoFG::setTargetSpeed(float speed_val)
{
  const float clamped = std::max(SPEED_MIN_PERCENT, std::min(speed_val, SPEED_MAX_PERCENT));
  writeRegister(REG_TARGET_SPEED, static_cast<uint16_t>(clamped), "set target speed");
}

float TwoFG::getWidth()
{
  uint16_t value = 0;
  return tryReadRegister(REG_EXTERNAL_WIDTH, "width", value) ? fromTenthMM(value) : -1.0f;
}

float TwoFG::getForce()
{
  // The register holds newtons.
  uint16_t value = 0;
  return tryReadRegister(REG_FORCE, "force", value) ? static_cast<float>(value) : -1.0f;
}

std::vector<int> TwoFG::getStatus()
{
  const uint16_t raw = getStatusRaw();
  return {
    (raw & STATUS_BUSY) ? 1 : 0,
    (raw & STATUS_GRIP_DETECTED) ? 1 : 0,
    (raw & STATUS_ERROR_NOT_CALIBRATED) ? 1 : 0,
    (raw & STATUS_ERROR_LINEAR_SENSOR) ? 1 : 0};
}

uint16_t TwoFG::getStatusRaw()
{
  uint16_t value = 0;
  return tryReadRegister(REG_STATUS, "status", value) ? value : 0;
}
