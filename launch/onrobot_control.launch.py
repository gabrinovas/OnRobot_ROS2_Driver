#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run one OnRobot gripper on its own: controller manager, controllers, monitor and RViz."""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from onrobot_description.config_loader import ConfigError, package_file, require
from onrobot_description.grippers import gripper_names, load_gripper
from onrobot_description.launch_arguments import as_launch_text, declare_arguments
from onrobot_driver import driver_config, gripper_joints, load_driver_gripper

PACKAGE = 'onrobot_driver'
# Launch arguments handed to the hardware description as they are.
DESCRIPTION_ARGUMENTS = ('onrobot_type', 'connection_type', 'device', 'ip_address', 'port',
                         'device_address', 'prefix', 'use_fake_hardware')


def launch_setup(context, *args, **kwargs):
    config = driver_config()
    values = {name: LaunchConfiguration(name).perform(context)
              for name in require(config, 'launch_arguments')}
    gripper = load_driver_gripper(values['onrobot_type'])
    namespace = values['ns']
    if not values['port'].isdigit():
        port = values['port']
        raise ConfigError(f"[Fail-Fast] Launch argument 'port' is not a number: '{port}'")
    simulated = values['use_fake_hardware'].lower() == 'true'
    if values['connection_type'] == 'tcp' and not simulated and not values['ip_address']:
        raise ConfigError(
            "[Fail-Fast] Launch argument 'ip_address' (the address of the Compute Box) is "
            'required with connection_type:=tcp and the real gripper')

    description_command = [FindExecutable(name='xacro'), ' ',
                           package_file(PACKAGE, require(config, 'launch', 'description_file'))]
    for name in DESCRIPTION_ARGUMENTS:
        if values[name] or name == 'prefix':  # xacro takes an empty value as a missing argument
            description_command += [f' {name}:=', values[name]]
    mock = as_launch_text(require(config, 'launch', 'use_mock_hardware'))
    description_command += [' use_mock_hardware:=', mock]
    robot_description = {
        'robot_description': ParameterValue(Command(description_command), value_type=str)}

    controllers = ParameterFile(
        package_file(PACKAGE, require(gripper, 'controllers', 'file')), allow_substs=True)
    action_controllers = values['use_gripper_action_controller'].lower() == 'true'
    spawned = [require(config, 'launch', 'joint_state_broadcaster')]
    spawned += require(gripper, 'controllers', 'action' if action_controllers else 'position')

    monitor_parameters = {
        'onrobot_type': values['onrobot_type'],
        'gripper_kind': require(load_gripper(values['onrobot_type']), 'kind'),
        'joints': gripper_joints(values['onrobot_type'], values['prefix']),
        'connection_type': values['connection_type'],
        'ip_address': values['ip_address'],
        'port': int(values['port']),
        'device': values['device'],
        'timeout_ms': require(config, 'connection', 'timeout_ms'),
        'serial_timeout_ms': require(config, 'connection', 'serial_timeout_ms'),
        'compute_box_address': require(config, 'compute_box_address'),
        'use_fake_hardware': values['use_fake_hardware'].lower() == 'true',
    }
    rviz_package, rviz_file = require(config, 'launch', 'rviz_config')

    def node(package, executable, **options):
        return Node(namespace=namespace, package=package, executable=executable, **options)

    return [
        node('controller_manager', 'ros2_control_node', output='screen',
             parameters=[robot_description, controllers]),
        node('robot_state_publisher', 'robot_state_publisher', output='both',
             parameters=[robot_description],
             condition=IfCondition(LaunchConfiguration('launch_rsp'))),
        *[node('controller_manager', 'spawner', arguments=[controller], output='screen')
          for controller in spawned],
        node(PACKAGE, 'gripper_status_monitor', name='gripper_status_monitor', output='screen',
             parameters=[package_file(PACKAGE, require(config, 'launch', 'monitor_config')),
                         monitor_parameters]),
        node('rviz2', 'rviz2', name='rviz2', output='screen',
             arguments=['-d', package_file(rviz_package, rviz_file)],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    arguments = declare_arguments(
        driver_config(), 'launch_arguments', choices={'onrobot_type': gripper_names()})
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])
