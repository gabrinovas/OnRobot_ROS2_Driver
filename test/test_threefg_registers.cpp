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
#include "onrobot_driver/threefg/ThreeFG.hpp"
#include "onrobot_driver/threefg/threefg_registers.hpp"

namespace
{
using onrobot_driver::FingerSettings;
using Write = MockModbusConnection::Write;
namespace reg = onrobot_driver::threefg;

/// 3FG15 with the range of its configuration file, attached to a mock connection.
class ThreeFGTest : public ::testing::Test
{
protected:
  struct Gripper : ThreeFG
  {
    using ThreeFG::ThreeFG;
    using ThreeFG::applyDefaults;
  };

  void SetUp() override
  {
    auto connection = std::make_unique<MockModbusConnection>();
    mock = connection.get();
    mock->registers[reg::REG_MIN_DIAMETER] = 0;
    mock->registers[reg::REG_MAX_DIAMETER] = 1500;  // 150.0 mm
    gripper = std::make_unique<Gripper>(65, FingerSettings{0.0f, 0.150f, 140.0f, 70.0f, 50.0f});
    gripper->attach(std::move(connection));
  }

  MockModbusConnection * mock{nullptr};
  std::unique_ptr<Gripper> gripper;
};

TEST_F(ThreeFGTest, RangeComesFromTheSettings)
{
  EXPECT_FLOAT_EQ(gripper->getMinWidth(), 0.0f);
  EXPECT_FLOAT_EQ(gripper->getMaxWidth(), 0.150f);
  EXPECT_FLOAT_EQ(gripper->getMaxForce(), 140.0f);
}

TEST_F(ThreeFGTest, OpeningAConnectionWritesTheDefaultForceAndSpeed)
{
  gripper->applyDefaults();
  // 70 N of 140 N is 50.0 %, written in 1/10 %.
  EXPECT_EQ(
    mock->writes,
    (std::vector<Write>{{reg::REG_TARGET_FORCE, 500}, {reg::REG_TARGET_SPEED, 50}}));
}

TEST_F(ThreeFGTest, WritesTargetsInTheUnitsOfTheRegisters)
{
  gripper->setTargetWidth(0.090f);
  gripper->setTargetForce(120.0f);  // 85.71 % of 140 N
  gripper->stop();
  EXPECT_EQ(
    mock->writes, (std::vector<Write>{
      {reg::REG_TARGET_DIAMETER, 900}, {reg::REG_TARGET_FORCE, 857},
      {reg::REG_CONTROL, reg::CMD_STOP}}));
}

TEST_F(ThreeFGTest, ATargetOutsideTheRangeReportedByTheGripperIsClamped)
{
  mock->registers[reg::REG_MIN_DIAMETER] = 100;   // 10.0 mm
  mock->registers[reg::REG_MAX_DIAMETER] = 1400;  // 140.0 mm
  gripper->setTargetWidth(0.300f);
  gripper->setTargetWidth(0.001f);
  EXPECT_EQ(
    mock->writes,
    (std::vector<Write>{{reg::REG_TARGET_DIAMETER, 1400}, {reg::REG_TARGET_DIAMETER, 100}}));
}

TEST_F(ThreeFGTest, MovingPositionsWithoutGripForce)
{
  gripper->moveGripper(0.050f);
  EXPECT_EQ(
    mock->writes,
    (std::vector<Write>{{reg::REG_TARGET_DIAMETER, 500}, {reg::REG_CONTROL, reg::CMD_MOVE}}));
}

TEST_F(ThreeFGTest, ReadsDiameterForceAndStatus)
{
  mock->registers[reg::REG_RAW_DIAMETER] = 900;
  mock->registers[reg::REG_APPLIED_FORCE] = 500;  // 50.0 % of 140 N
  mock->registers[reg::REG_STATUS] = reg::STATUS_BUSY | reg::STATUS_CALIBRATION_OK;

  EXPECT_NEAR(gripper->getWidth(), 0.090f, 1e-4f);
  EXPECT_NEAR(gripper->getForce(), 70.0f, 1e-3f);
  const auto status = gripper->getDetailedStatus();
  EXPECT_TRUE(status.busy);
  EXPECT_FALSE(status.grip_detected);
  EXPECT_TRUE(status.calibration_ok);
}
}  // namespace
