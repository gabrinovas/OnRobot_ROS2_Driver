# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check the configuration files of the driver and the Python helpers that read them."""

import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.substitutions import Command
from onrobot_description.config_loader import ConfigError, package_file, require
from onrobot_description.grippers import gripper_names, load_gripper
import onrobot_driver
from onrobot_driver.factory import _constructor_arguments
import pytest


@pytest.mark.parametrize('gripper', gripper_names())
def test_every_gripper_of_the_description_has_its_driver_file(gripper):
    driver = onrobot_driver.load_driver_gripper(gripper)
    assert require(driver, 'hardware', 'plugin').startswith('onrobot_driver::')
    package_file('onrobot_driver', require(driver, 'controllers', 'file'))
    assert require(driver, 'controllers', 'action')
    assert require(driver, 'controllers', 'position')
    module = __import__(f"onrobot_driver.{require(driver, 'python', 'module')}", fromlist=['x'])
    assert hasattr(module, require(driver, 'python', 'class'))


@pytest.mark.parametrize('gripper', gripper_names())
def test_the_python_class_is_built_from_the_configuration(gripper):
    driver = onrobot_driver.load_driver_gripper(gripper)
    parameters = dict(require(driver, 'hardware', 'parameters'), __file__=driver['__file__'])
    arguments = _constructor_arguments(load_gripper(gripper), parameters)
    module = __import__(f"onrobot_driver.{require(driver, 'python', 'module')}", fromlist=['x'])
    instance = getattr(module, require(driver, 'python', 'class'))(device_address=65, **arguments)
    assert not instance.is_connected()


@pytest.mark.parametrize('gripper', gripper_names())
def test_the_hardware_description_names_the_joints_of_the_gripper(gripper):
    description = package_file('onrobot_driver', 'urdf', 'onrobot_driver.urdf.xacro')
    urdf = Command([
        'xacro ', description, f' onrobot_type:={gripper} prefix:=cell_ use_fake_hardware:=true',
        ' use_mock_hardware:=false connection_type:=tcp device:=/tmp/ttyUR',
        ' port:=502 device_address:=65']).perform(LaunchContext())
    block = ET.fromstring(urdf).find('ros2_control')
    joints = [joint.attrib['name'] for joint in block.findall('joint')]
    assert joints == onrobot_driver.gripper_joints(gripper, 'cell_')
    names = {parameter.attrib['name'] for parameter in block.find('hardware').findall('param')}
    driver = onrobot_driver.load_driver_gripper(gripper)
    assert set(require(driver, 'hardware', 'parameters')) <= names


def test_an_unknown_gripper_or_connection_value_is_a_fail_fast_error():
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*2fg7, 3fg15, vgc10'):
        onrobot_driver.load_driver_gripper('no_such_gripper')
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\] Unknown connection values'):
        onrobot_driver.create_gripper('2fg7', no_such_value=1)


def test_the_compute_box_address_has_no_default_and_is_asked_for():
    arguments = require(onrobot_driver.driver_config(), 'launch_arguments')
    assert require(arguments, 'ip_address', 'default') == ''
    with pytest.raises(ConfigError, match=r'\[Fail-Fast\].*ip_address'):
        onrobot_driver.create_gripper('2fg7')


@pytest.mark.parametrize('address, expected', [('', None), ('192.0.2.1', '192.0.2.1')])
def test_the_description_carries_the_compute_box_address_only_when_given(address, expected):
    description = package_file('onrobot_driver', 'urdf', 'onrobot_driver.urdf.xacro')
    given = f' ip_address:={address}' if address else ''
    urdf = Command([
        'xacro ', description, ' onrobot_type:=2fg7 use_fake_hardware:=true',
        ' use_mock_hardware:=false connection_type:=tcp device:=/tmp/ttyUR',
        given, ' port:=502 device_address:=65']).perform(LaunchContext())
    hardware = ET.fromstring(urdf).find('ros2_control').find('hardware')
    found = {parameter.attrib['name']: parameter.text for parameter in hardware.findall('param')}
    assert found.get('ip_address') == expected


def test_the_launch_arguments_have_their_defaults_in_the_configuration():
    arguments = require(onrobot_driver.driver_config(), 'launch_arguments')
    for name in ('onrobot_type', 'connection_type', 'ip_address', 'port', 'device',
                 'device_address', 'prefix', 'ns', 'use_fake_hardware'):
        require(arguments, name, 'default')
        require(arguments, name, 'description')
