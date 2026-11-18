// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ONROBOT_DRIVER__VGC10__VGC10_HPP_
#define ONROBOT_DRIVER__VGC10__VGC10_HPP_

#include <cstdint>
#include <vector>

#include "onrobot_driver/common/OnRobotGripperBase.hpp"
#include "onrobot_driver/vgc10/vgc10_registers.hpp"

namespace onrobot_driver
{

/// Protocol of the OnRobot VGC10 vacuum gripper: two independent channels, A and B. As a generic
/// gripper its "width" is the normalised vacuum (0 released, 1 full) and its "force" the pump
/// current limit in mA.
class VGC10 : public OnRobotGripperBase
{
public:
  static constexpr uint8_t MODE_RELEASE = vgc10::MODE_RELEASE;
  static constexpr uint8_t MODE_GRIP = vgc10::MODE_GRIP;
  static constexpr uint8_t MODE_IDLE = vgc10::MODE_IDLE;

  enum class Channel : uint8_t {A = 0, B = 1, BOTH = 2};

  VGC10(int device_address, const VacuumSettings & settings);
  ~VGC10() override;

  float getWidth() override;   ///< mean vacuum of both channels
  float getForce() override;   ///< current limit, mA
  uint16_t getStatusRaw() override;
  /// mode of A, target vacuum of A, mode of B, target vacuum of B
  std::vector<int> getStatus() override;

  void setTargetForce(float force_val) override;  ///< current limit, mA
  void setTargetWidth(float width_val) override;  ///< vacuum of both channels, 0 to 1
  void setTargetSpeed(float speed_val) override;  ///< a pump has no speed: does nothing
  void stop() override;                           ///< releases both channels
  void moveGripper(float width_val) override;

  float getMinWidth() const override {return 0.0f;}
  float getMaxWidth() const override {return 1.0f;}
  float getMaxForce() const override {return static_cast<float>(settings_.max_current_ma);}

  void gripChannelA(uint8_t vacuum_pct);
  void releaseChannelA();
  void idleChannelA();
  void gripChannelB(uint8_t vacuum_pct);
  void releaseChannelB();
  void idleChannelB();
  void gripAll(uint8_t vacuum_pct);
  void releaseAll();
  void idleAll();

  void setChannelControl(Channel channel, uint8_t mode, uint8_t target_vacuum_pct);
  void setCurrentLimit(uint16_t current_ma);

  float getVacuumChannelA();          ///< 0 to 1
  float getVacuumChannelB();
  uint16_t getRawVacuumChannelA();    ///< 1/1000 of relative vacuum
  uint16_t getRawVacuumChannelB();
  /// Both channels in one Modbus transaction.
  bool readBothVacuums(float & vac_a, float & vac_b);

  uint8_t getModeChannelA() const {return current_mode_a_;}
  uint8_t getModeChannelB() const {return current_mode_b_;}
  uint8_t getTargetVacuumChannelA() const {return target_vacuum_a_;}
  uint8_t getTargetVacuumChannelB() const {return target_vacuum_b_;}

protected:
  void onAttached() override;

private:
  void writeChannelRegister(uint16_t address, uint8_t mode, uint8_t target_pct);
  uint16_t rawVacuum(uint16_t address, const char * channel);

  VacuumSettings settings_;
  uint8_t current_mode_a_{MODE_RELEASE};
  uint8_t target_vacuum_a_{0};
  uint8_t current_mode_b_{MODE_RELEASE};
  uint8_t target_vacuum_b_{0};
  uint16_t current_limit_ma_;
};

}  // namespace onrobot_driver

#endif  // ONROBOT_DRIVER__VGC10__VGC10_HPP_
