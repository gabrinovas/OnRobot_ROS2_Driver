// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <algorithm>

#include "onrobot_driver/threefg/ThreeFG.hpp"
#include "onrobot_driver/threefg/threefg_registers.hpp"

// The register map of this gripper.
using namespace onrobot_driver::threefg;  // NOLINT(build/namespaces)

ThreeFG::ThreeFG(int device_address, const onrobot_driver::FingerSettings & settings)
: onrobot_driver::OnRobotGripperBase(device_address), settings_(settings)
{
}

ThreeFG::~ThreeFG()
{
  close();
}

void ThreeFG::applyDefaults()
{
  setTargetForce(settings_.default_force);
  setTargetSpeed(settings_.default_speed);
}

float ThreeFG::clampedDiameter(float diameter_val)
{
  return std::max(getMinDiameter(), std::min(diameter_val, getMaxDiameter()));
}

void ThreeFG::moveGripper(float diameter_val)
{
  // Plain positioning: the move command, which applies no grip force.
  setTargetWidth(clampedDiameter(diameter_val));
  setCommand(CMD_MOVE);
}

void ThreeFG::gripInternal()
{
  setGripType(true);
  setCommand(CMD_GRIP);
}

void ThreeFG::gripExternal()
{
  setGripType(false);
  setCommand(CMD_GRIP);
}

void ThreeFG::stop()
{
  setCommand(CMD_STOP);
}

void ThreeFG::setCommand(uint16_t command)
{
  writeRegister(REG_CONTROL, command, "set command");
}

void ThreeFG::setTargetForce(float force_val)
{
  const float clamped = std::max(0.0f, std::min(force_val, settings_.max_force));
  const float percent = (clamped / settings_.max_force) * 100.0f;
  writeRegister(
    REG_TARGET_FORCE, static_cast<uint16_t>(percent * FORCE_UNITS_PER_PERCENT), "set target force");
}

void ThreeFG::setTargetWidth(float diameter_val)
{
  writeRegister(
    REG_TARGET_DIAMETER, toTenthMM(clampedDiameter(diameter_val)),
    "set target diameter");
}

void ThreeFG::setTargetSpeed(float speed_val)
{
  const float clamped = std::max(SPEED_MIN_PERCENT, std::min(speed_val, SPEED_MAX_PERCENT));
  writeRegister(REG_TARGET_SPEED, static_cast<uint16_t>(clamped), "set target speed");
}

float ThreeFG::getCurrentDiameter()
{
  uint16_t value = 0;
  return tryReadRegister(REG_RAW_DIAMETER, "diameter", value) ? fromTenthMM(value) : -1.0f;
}

float ThreeFG::getDiameterWithOffset()
{
  // The register is signed.
  uint16_t value = 0;
  if (!tryReadRegister(REG_DIAMETER_WITH_OFFSET, "diameter with offset", value)) {
    return -1.0f;
  }
  return static_cast<float>(static_cast<int16_t>(value)) / DIAMETER_UNITS_PER_METER;
}

uint16_t ThreeFG::getStatusRaw()
{
  uint16_t value = 0;
  return tryReadRegister(REG_STATUS, "status", value) ? value : 0;
}

float ThreeFG::getAppliedForce()
{
  uint16_t value = 0;
  if (!tryReadRegister(REG_APPLIED_FORCE, "applied force", value)) {
    return -1.0f;
  }
  return (static_cast<float>(value) / FORCE_UNITS_PER_PERCENT / 100.0f) * settings_.max_force;
}

float ThreeFG::getMinDiameter()
{
  uint16_t value = 0;
  return tryReadRegister(
    REG_MIN_DIAMETER, "min diameter",
    value) ? fromTenthMM(value) : settings_.min_width;
}

float ThreeFG::getMaxDiameter()
{
  uint16_t value = 0;
  return tryReadRegister(
    REG_MAX_DIAMETER, "max diameter",
    value) ? fromTenthMM(value) : settings_.max_width;
}

float ThreeFG::getFingerLength()
{
  uint16_t value = 0;
  if (!tryReadRegister(REG_FINGER_LENGTH, "finger length", value)) {
    return -1.0f;
  }
  return static_cast<float>(value) / LENGTH_UNITS_PER_MM;
}

float ThreeFG::getFingerPosition()
{
  uint16_t value = 0;
  return tryReadRegister(
    REG_FINGER_POSITION, "finger position",
    value) ? static_cast<float>(value) : -1.0f;
}

float ThreeFG::getFingertipOffset()
{
  uint16_t value = 0;
  if (!tryReadRegister(REG_FINGERTIP_OFFSET, "fingertip offset", value)) {
    return -1.0f;
  }
  return static_cast<float>(value) / OFFSET_UNITS_PER_MM;
}

ThreeFG::GripperStatus ThreeFG::getDetailedStatus()
{
  const uint16_t raw = getStatusRaw();
  GripperStatus status{};
  status.busy = raw & STATUS_BUSY;
  status.grip_detected = raw & STATUS_GRIP_DETECTED;
  status.force_grip_detected = raw & STATUS_FORCE_GRIP_DETECTED;
  status.calibration_ok = raw & STATUS_CALIBRATION_OK;
  return status;
}

void ThreeFG::setGripType(bool internal)
{
  writeRegister(REG_GRIP_TYPE, internal ? 1 : 0, "set grip type");
}

void ThreeFG::setFingerLength(float mm)
{
  writeRegister(
    REG_CFG_FINGER_LENGTH, static_cast<uint16_t>(mm * LENGTH_UNITS_PER_MM), "set finger length");
}

void ThreeFG::setFingerPosition(float position)
{
  writeRegister(REG_CFG_FINGER_POSITION, static_cast<uint16_t>(position), "set finger position");
}

void ThreeFG::setFingertipOffset(float mm)
{
  writeRegister(
    REG_CFG_FINGERTIP_OFFSET, static_cast<uint16_t>(mm * OFFSET_UNITS_PER_MM),
    "set fingertip offset");
}
