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
#include "onrobot_driver/twofg/TwoFG.hpp"
#include "onrobot_driver/twofg/twofg_registers.hpp"

namespace
{
using onrobot_driver::FingerSettings;
using Write = MockModbusConnection::Write;
namespace reg = onrobot_driver::twofg;

/// 2FG7 with the range of its configuration file, attached to a mock connection.
class TwoFGTest : public ::testing::Test
{
protected:
  struct Gripper : TwoFG
  {
    using TwoFG::TwoFG;
    using TwoFG::applyDefaults;
  };

  void SetUp() override
  {
    auto connection = std::make_unique<MockModbusConnection>();
    mock = connection.get();
    gripper = std::make_unique<Gripper>(65, FingerSettings{0.035f, 0.073f, 70.0f, 35.0f, 50.0f});
    gripper->attach(std::move(connection));
  }

  MockModbusConnection * mock{nullptr};
  std::unique_ptr<Gripper> gripper;
};

TEST_F(TwoFGTest, RangeComesFromTheSettings)
{
  EXPECT_FLOAT_EQ(gripper->getMinWidth(), 0.035f);
  EXPECT_FLOAT_EQ(gripper->getMaxWidth(), 0.073f);
  EXPECT_FLOAT_EQ(gripper->getMaxForce(), 70.0f);
  EXPECT_FLOAT_EQ(onrobot_driver::OnRobotGripperBase::fromTenthMM(350), 0.035f);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.035f), 350);
}

TEST_F(TwoFGTest, LengthsAreRoundedToTheNearestTenthOfMillimetre)
{
  // 0.036 m is 359.99997 tenths of a millimetre as a float: cutting the decimals gave 359.
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.036f), 360);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.0503f), 503);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.055f), 550);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.05494f), 549);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.05496f), 550);
  EXPECT_EQ(onrobot_driver::OnRobotGripperBase::toTenthMM(0.0f), 0);
}

TEST_F(TwoFGTest, AttachingAConnectionWritesNothing)
{
  EXPECT_TRUE(mock->writes.empty());
}

TEST_F(TwoFGTest, OpeningAConnectionWritesTheDefaultForceAndSpeed)
{
  gripper->applyDefaults();
  EXPECT_EQ(
    mock->writes,
    (std::vector<Write>{{reg::REG_TARGET_FORCE, 35}, {reg::REG_TARGET_SPEED, 50}}));
}

TEST_F(TwoFGTest, ReadsWidthForceAndStatus)
{
  mock->registers[reg::REG_EXTERNAL_WIDTH] = 450;  // 45.0 mm
  mock->registers[reg::REG_FORCE] = 35;
  mock->registers[reg::REG_STATUS] = reg::STATUS_BUSY | reg::STATUS_GRIP_DETECTED;

  EXPECT_NEAR(gripper->getWidth(), 0.045f, 1e-4f);
  EXPECT_FLOAT_EQ(gripper->getForce(), 35.0f);
  EXPECT_EQ(gripper->getStatusRaw(), 0x0003);
  EXPECT_EQ(gripper->getStatus(), (std::vector<int>{1, 1, 0, 0}));
}

TEST_F(TwoFGTest, WritesTargetsClampedToTheRange)
{
  gripper->setTargetWidth(0.040f);
  gripper->setTargetWidth(0.500f);
  gripper->setTargetForce(60.0f);
  gripper->setTargetForce(500.0f);
  gripper->setTargetSpeed(80.0f);
  gripper->setTargetSpeed(1.0f);
  EXPECT_EQ(
    mock->writes, (std::vector<Write>{
      {reg::REG_TARGET_WIDTH, 400}, {reg::REG_TARGET_WIDTH, 730}, {reg::REG_TARGET_FORCE, 60},
      {reg::REG_TARGET_FORCE, 70}, {reg::REG_TARGET_SPEED, 80}, {reg::REG_TARGET_SPEED, 10}}));
}

TEST_F(TwoFGTest, CommandsAndMoves)
{
  gripper->gripExternal();
  gripper->gripInternal();
  gripper->stop();
  gripper->moveGripper(0.050f);
  gripper->moveGripper(0.060f, false);
  EXPECT_EQ(
    mock->writes, (std::vector<Write>{
      {reg::REG_COMMAND, reg::CMD_GRIP_EXTERNAL}, {reg::REG_COMMAND, reg::CMD_GRIP_INTERNAL},
      {reg::REG_COMMAND, reg::CMD_STOP}, {reg::REG_TARGET_WIDTH, 500},
      {reg::REG_COMMAND, reg::CMD_GRIP_EXTERNAL}, {reg::REG_TARGET_WIDTH, 600},
      {reg::REG_COMMAND, reg::CMD_GRIP_INTERNAL}}));
}

TEST_F(TwoFGTest, ClosingTheGripperDropsTheConnection)
{
  EXPECT_TRUE(gripper->isConnected());
  gripper->close();
  EXPECT_FALSE(gripper->isConnected());
}
}  // namespace
