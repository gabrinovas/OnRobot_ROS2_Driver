// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__TWOFG__TWOFG_HPP_
#define ONROBOT_DRIVER__TWOFG__TWOFG_HPP_

#include <cstdint>
#include <vector>

#include "onrobot_driver/common/OnRobotGripperBase.hpp"

/// Protocol of the OnRobot 2FG7 parallel gripper.
class TwoFG : public onrobot_driver::OnRobotGripperBase
{
public:
  TwoFG(int device_address, const onrobot_driver::FingerSettings & settings);
  ~TwoFG() override;

  float getWidth() override;
  float getForce() override;
  uint16_t getStatusRaw() override;
  /// busy, grip detected, not calibrated, linear sensor error
  std::vector<int> getStatus() override;

  void setTargetForce(float force_val) override;
  void setTargetWidth(float width_val) override;
  void setTargetSpeed(float speed_val) override;
  void setCommand(uint16_t command);

  void gripExternal();
  void gripInternal();
  void stop() override;
  void moveGripper(float width_val) override {moveGripper(width_val, true);}
  void moveGripper(float width_val, bool external_grip);

  float getMinWidth() const override {return settings_.min_width;}
  float getMaxWidth() const override {return settings_.max_width;}
  float getMaxForce() const override {return settings_.max_force;}

protected:
  void applyDefaults() override;

private:
  float clampedWidth(float width_val) const;

  onrobot_driver::FingerSettings settings_;
};

#endif  // ONROBOT_DRIVER__TWOFG__TWOFG_HPP_
