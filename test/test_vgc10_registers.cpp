// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

#include "mock_modbus_connection.hpp"
#include "onrobot_driver/vgc10/VGC10.hpp"

namespace
{
using onrobot_driver::VacuumSettings;
using onrobot_driver::VGC10;
using Write = MockModbusConnection::Write;
namespace reg = onrobot_driver::vgc10;

constexpr uint16_t control(uint8_t mode, uint8_t percent)
{
  return static_cast<uint16_t>((mode << 8) | percent);
}

/// VGC10 with the values of its configuration file, attached to a mock connection.
class VGC10Test : public ::testing::Test
{
protected:
  void SetUp() override
  {
    auto connection = std::make_unique<MockModbusConnection>();
    mock = connection.get();
    gripper = std::make_unique<VGC10>(65, VacuumSettings{80, 1000, 500, 0.05f});
    gripper->attach(std::move(connection));
    mock->writes.clear();
  }

  MockModbusConnection * mock{nullptr};
  std::unique_ptr<VGC10> gripper;
};

TEST_F(VGC10Test, AttachingAConnectionWritesTheInitialCurrentLimit)
{
  auto connection = std::make_unique<MockModbusConnection>();
  MockModbusConnection * fresh = connection.get();
  VGC10 other(65, VacuumSettings{80, 1000, 500, 0.05f});
  other.attach(std::move(connection));
  EXPECT_EQ(fresh->writes, (std::vector<Write>{{reg::REG_CURRENT_LIMIT, 500}}));
}

TEST_F(VGC10Test, ChannelsAreControlledIndependentlyAndClampedToTheHighestVacuum)
{
  gripper->gripChannelA(60);
  gripper->gripChannelB(200);
  gripper->releaseChannelA();
  gripper->idleChannelB();
  EXPECT_EQ(
    mock->writes, (std::vector<Write>{
      {reg::REG_CHANNEL_A_CTRL, control(reg::MODE_GRIP, 60)},
      {reg::REG_CHANNEL_B_CTRL, control(reg::MODE_GRIP, 80)},
      {reg::REG_CHANNEL_A_CTRL, control(reg::MODE_RELEASE, 0)},
      {reg::REG_CHANNEL_B_CTRL, control(reg::MODE_IDLE, 0)}}));
}

TEST_F(VGC10Test, ATargetAboveTheThresholdGripsBothChannelsAndBelowItReleasesThem)
{
  gripper->setTargetWidth(0.5f);   // half of the highest vacuum
  gripper->setTargetWidth(0.04f);
  EXPECT_EQ(
    mock->writes, (std::vector<Write>{
      {reg::REG_CHANNEL_A_CTRL, control(reg::MODE_GRIP, 40)},
      {reg::REG_CHANNEL_B_CTRL, control(reg::MODE_GRIP, 40)},
      {reg::REG_CHANNEL_A_CTRL, control(reg::MODE_RELEASE, 0)},
      {reg::REG_CHANNEL_B_CTRL, control(reg::MODE_RELEASE, 0)}}));
}

TEST_F(VGC10Test, TheCurrentLimitIsClamped)
{
  gripper->setCurrentLimit(5000);
  EXPECT_EQ(mock->writes, (std::vector<Write>{{reg::REG_CURRENT_LIMIT, 1000}}));
  EXPECT_FLOAT_EQ(gripper->getForce(), 1000.0f);
}

TEST_F(VGC10Test, ReadsTheVacuumOfBothChannels)
{
  mock->registers[reg::REG_CHANNEL_A_VACUUM] = 750;  // 75 %
  mock->registers[reg::REG_CHANNEL_B_VACUUM] = 250;  // 25 %

  EXPECT_NEAR(gripper->getVacuumChannelA(), 0.75f, 1e-3f);
  EXPECT_NEAR(gripper->getVacuumChannelB(), 0.25f, 1e-3f);
  EXPECT_NEAR(gripper->getWidth(), 0.50f, 1e-3f);
  float a = 0.0f;
  float b = 0.0f;
  EXPECT_TRUE(gripper->readBothVacuums(a, b));
  EXPECT_NEAR(a, 0.75f, 1e-3f);
  EXPECT_NEAR(b, 0.25f, 1e-3f);
}
}  // namespace
