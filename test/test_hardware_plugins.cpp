// Copyright 2026 Tony Le, Gabriel Novas
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/component_parser.hpp"
#include "onrobot_driver/threefg/threefg_hardware_interface.hpp"
#include "onrobot_driver/twofg/twofg_hardware_interface.hpp"
#include "onrobot_driver/vgc10/vgc10_hardware_interface.hpp"

namespace
{
using hardware_interface::CallbackReturn;
using hardware_interface::HardwareInfo;

/// The <ros2_control> block that the hardware description of this package generates for one
/// gripper: simulated, or real over tcp when the address of a Compute Box is given.
HardwareInfo generated_hardware(const std::string & gripper, const std::string & compute_box = "")
{
  const std::string description =
    ament_index_cpp::get_package_share_directory("onrobot_driver") +
    "/urdf/onrobot_driver.urdf.xacro";
  const std::string mode = compute_box.empty() ?
    " use_fake_hardware:=true" : " use_fake_hardware:=false ip_address:=" + compute_box;
  const std::string command = "xacro " + description + " onrobot_type:=" + gripper + mode +
    " use_mock_hardware:=false connection_type:=tcp device:=/tmp/ttyUR port:=502"
    " device_address:=65";
  std::string urdf;
  std::array<char, 4096> buffer;
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(command.c_str(), "r"), pclose);
  while (pipe && fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    urdf += buffer.data();
  }
  const auto blocks = hardware_interface::parse_control_resources_from_urdf(urdf);
  EXPECT_EQ(blocks.size(), 1u) << "xacro output of " << gripper << ":\n" << urdf;
  return blocks.empty() ? HardwareInfo() : blocks.front();
}

std::unique_ptr<onrobot_driver::GripperHardwareBase> make_plugin(const std::string & gripper)
{
  if (gripper == "2fg7") {
    return std::make_unique<onrobot_driver::TwoFGHardwareInterface>();
  }
  if (gripper == "3fg15") {
    return std::make_unique<onrobot_driver::ThreeFGHardwareInterface>();
  }
  return std::make_unique<onrobot_driver::VGC10HardwareInterface>();
}

class PluginTest : public ::testing::TestWithParam<std::string> {};

TEST_P(PluginTest, TheGeneratedDescriptionHasEverythingThePluginRequires)
{
  const HardwareInfo info = generated_hardware(GetParam());
  auto plugin = make_plugin(GetParam());
  ASSERT_EQ(plugin->on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

  // Five state interfaces and three command interfaces per joint, under the joint names of the
  // description.
  const auto states = plugin->export_state_interfaces();
  const auto commands = plugin->export_command_interfaces();
  EXPECT_EQ(states.size(), 5 * info.joints.size());
  EXPECT_EQ(commands.size(), 3 * info.joints.size());
  EXPECT_EQ(states.front().get_prefix_name(), info.joints.front().name);
}

TEST_P(PluginTest, EveryParameterOfTheDescriptionIsRequired)
{
  const HardwareInfo complete = generated_hardware(GetParam());
  ASSERT_FALSE(complete.hardware_parameters.empty());
  for (const auto & parameter : complete.hardware_parameters) {
    HardwareInfo incomplete = complete;
    incomplete.hardware_parameters.erase(parameter.first);
    EXPECT_EQ(make_plugin(GetParam())->on_init(incomplete), CallbackReturn::ERROR)
      << "the plugin started without '" << parameter.first << "'";
  }
}

TEST_P(PluginTest, ARealGripperOverTcpNeedsTheAddressOfItsComputeBox)
{
  const HardwareInfo simulated = generated_hardware(GetParam());
  EXPECT_EQ(simulated.hardware_parameters.count("ip_address"), 0u);

  const HardwareInfo real = generated_hardware(GetParam(), "192.0.2.1");
  EXPECT_EQ(real.hardware_parameters.at("ip_address"), "192.0.2.1");
  EXPECT_EQ(make_plugin(GetParam())->on_init(real), CallbackReturn::SUCCESS);

  HardwareInfo without_address = real;
  without_address.hardware_parameters.erase("ip_address");
  EXPECT_EQ(make_plugin(GetParam())->on_init(without_address), CallbackReturn::ERROR);
}

TEST_P(PluginTest, AWrongNumberOfJointsIsAnError)
{
  HardwareInfo info = generated_hardware(GetParam());
  info.joints.push_back(info.joints.front());
  EXPECT_EQ(make_plugin(GetParam())->on_init(info), CallbackReturn::ERROR);
}

TEST_P(PluginTest, TheSimulatedGripperFollowsItsCommand)
{
  const HardwareInfo info = generated_hardware(GetParam());
  auto plugin = make_plugin(GetParam());
  ASSERT_EQ(plugin->on_init(info), CallbackReturn::SUCCESS);
  ASSERT_EQ(plugin->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  auto states = plugin->export_state_interfaces();
  auto commands = plugin->export_command_interfaces();

  const double start = states.front().get_value();
  const double target = start + 0.01;
  commands.front().set_value(target);
  const rclcpp::Time now(0, 0, RCL_STEADY_TIME);
  const rclcpp::Duration period(0, 20000000);
  for (int cycle = 0; cycle < 200; ++cycle) {
    plugin->read(now, period);
    plugin->write(now, period);
  }
  EXPECT_NEAR(states.front().get_value(), target, 1e-4);
}

INSTANTIATE_TEST_SUITE_P(Grippers, PluginTest, ::testing::Values("2fg7", "3fg15", "vgc10"));

TEST(HardwareParameters, AMissingOrMalformedValueIsAFailFastError)
{
  HardwareInfo info;
  info.name = "OnRobotGripper";
  info.hardware_parameters = {{"port", "502"}, {"ratio", "0.5"}, {"word", "abc"}, {"fake", "True"}};
  const onrobot_driver::HardwareParameters parameters(info);

  EXPECT_EQ(parameters.integer("port"), 502);
  EXPECT_DOUBLE_EQ(parameters.number("ratio"), 0.5);
  EXPECT_TRUE(parameters.flag("fake"));
  EXPECT_THROW(parameters.text("absent"), onrobot_driver::ParameterError);
  EXPECT_THROW(parameters.number("word"), onrobot_driver::ParameterError);
  EXPECT_THROW(parameters.integer("ratio"), onrobot_driver::ParameterError);
  EXPECT_THROW(parameters.flag("word"), onrobot_driver::ParameterError);
  try {
    parameters.text("absent");
  } catch (const onrobot_driver::ParameterError & error) {
    EXPECT_NE(std::string(error.what()).find("[Fail-Fast]"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("'absent'"), std::string::npos);
  }
}

}  // namespace
