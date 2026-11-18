// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__THREEFG__THREEFG_HPP_
#define ONROBOT_DRIVER__THREEFG__THREEFG_HPP_

#include <cstdint>

#include "onrobot_driver/common/OnRobotGripperBase.hpp"

/// Protocol of the OnRobot 3FG15 three-finger gripper. Its "width" is the grip diameter.
class ThreeFG : public onrobot_driver::OnRobotGripperBase
{
public:
  struct GripperStatus
  {
    bool busy;
    bool grip_detected;
    bool force_grip_detected;
    bool calibration_ok;
  };

  ThreeFG(int device_address, const onrobot_driver::FingerSettings & settings);
  ~ThreeFG() override;

  float getWidth() override {return getCurrentDiameter();}
  float getForce() override {return getAppliedForce();}
  uint16_t getStatusRaw() override;
  GripperStatus getDetailedStatus();

  void setTargetForce(float force_val) override;
  void setTargetWidth(float diameter_val) override;
  void setTargetSpeed(float speed_val) override;
  void setCommand(uint16_t command);

  void gripInternal();
  void gripExternal();
  void stop() override;
  void moveGripper(float diameter_val) override;

  float getMinWidth() const override {return settings_.min_width;}
  float getMaxWidth() const override {return settings_.max_width;}
  float getMaxForce() const override {return settings_.max_force;}

  float getCurrentDiameter();
  float getDiameterWithOffset();
  float getAppliedForce();
  /// Diameter range reported by the gripper for the mounted fingers (the configured range on a read
  /// error).
  float getMinDiameter();
  float getMaxDiameter();
  float getFingerLength();    ///< mm
  float getFingerPosition();
  float getFingertipOffset();  ///< mm

  void setGripType(bool internal);
  void setFingerLength(float mm);
  void setFingerPosition(float position);
  void setFingertipOffset(float mm);

protected:
  void applyDefaults() override;

private:
  float clampedDiameter(float diameter_val);

  onrobot_driver::FingerSettings settings_;
};

#endif  // ONROBOT_DRIVER__THREEFG__THREEFG_HPP_
