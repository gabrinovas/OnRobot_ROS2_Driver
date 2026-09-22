#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Retrieve configuration values resolved dynamically from context
    onrobot_type_val = context.perform_substitution(LaunchConfiguration('onrobot_type'))
    connection_type_val = context.perform_substitution(LaunchConfiguration('connection_type'))
    device_val = context.perform_substitution(LaunchConfiguration('device'))
    ip_address_val = context.perform_substitution(LaunchConfiguration('ip_address'))
    port_val = context.perform_substitution(LaunchConfiguration('port'))
    device_address_val = context.perform_substitution(LaunchConfiguration('device_address'))
    prefix_val = context.perform_substitution(LaunchConfiguration('prefix'))
    ns_val = context.perform_substitution(LaunchConfiguration('ns'))
    use_fake_hardware_val = context.perform_substitution(LaunchConfiguration('use_fake_hardware'))
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rsp = LaunchConfiguration('launch_rsp')

    # Path to the xacro file in the onrobot_description package
    xacro_file = PathJoinSubstitution([
        FindPackageShare('onrobot_description'),
        'urdf',
        'onrobot.urdf.xacro'
    ])

    # Process the xacro to generate the robot description (URDF)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'onrobot_type:=', onrobot_type_val,
        ' ',
        'connection_type:=', connection_type_val,
        ' ',
        'device:=', device_val,
        ' ',
        'ip_address:=', ip_address_val,
        ' ',
        'port:=', port_val,
        ' ',
        'device_address:=', device_address_val,
        ' ',
        'prefix:=', prefix_val,
        ' ',
        'use_fake_hardware:=', use_fake_hardware_val,
        ' ',
        'name:=onrobot'
    ])

    # Use ParameterValue to properly handle the robot description string
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Determine which controller config and hardware interface to use based on resolved gripper type
    if onrobot_type_val == '2fg7':
        controller_config_filename = 'twofg_controllers.yaml'
        hw_interface_plugin = 'onrobot_driver::TwoFGHardwareInterface'
    elif onrobot_type_val == '3fg15':
        controller_config_filename = 'threefg_controllers.yaml'
        hw_interface_plugin = 'onrobot_driver::ThreeFGHardwareInterface'
    else:
        raise RuntimeError(f"Unsupported onrobot_type: '{onrobot_type_val}'. Supported types are: '2fg7', '3fg15'")

    controller_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_driver'),
        'config',
        controller_config_filename
    ])
    controller_config = ParameterFile(controller_config_file, allow_substs=True)

    # Add hardware interface parameter to robot description
    robot_description_with_hw = robot_description.copy()
    robot_description_with_hw['hardware_interface_plugin'] = hw_interface_plugin

    # Launch the ros2_control node
    ros2_control_node = Node(
        namespace=ns_val,
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_with_hw, controller_config],
        output='screen'
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        namespace=ns_val,
        package='robot_state_publisher',
        condition=IfCondition(launch_rsp),
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )

    # Spawn the joint state broadcaster
    joint_state_spawner = Node(
        namespace=ns_val,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the gripper controller
    gripper_controller_spawner = Node(
        namespace=ns_val,
        package='controller_manager',
        executable='spawner',
        arguments=['finger_width_controller'],
        output='screen'
    )

    # Launch RViz for visualization using the config from onrobot_description
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_description'),
        'rviz',
        'view_onrobot.rviz'
    ])
    rviz_node = Node(
        namespace=ns_val,
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Status monitor node
    gripper_status_node = Node(
        namespace=ns_val,
        package='onrobot_driver',
        executable='gripper_status_monitor',
        name='gripper_status_monitor',
        output='screen',
        parameters=[{
            'onrobot_type': onrobot_type_val,
        }]
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        gripper_controller_spawner,
        gripper_status_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'onrobot_type',
            default_value='2fg7',
            description='Type of OnRobot gripper.',
            choices=['2fg7', '3fg15'],
        ),
        DeclareLaunchArgument(
            'connection_type',
            default_value='tcp',
            description='Connection type for the OnRobot gripper. TCP for Compute Box. Serial for UR Tool I/O (RS485).',
            choices=['serial', 'tcp'],
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/tmp/ttyUR',
            description='Device name for the serial connection (e.g. /tmp/ttyUR or /dev/ttyUSB0). Only used when connection_type is serial.',
        ),
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.1',
            description='IP address for the TCP connection. Only used when connection_type is tcp.',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='502',
            description='Port for the TCP connection. Only used when connection_type is tcp.',
        ),
        DeclareLaunchArgument(
            'device_address',
            default_value='65',
            description='Modbus device address for the gripper. Default is 65 (0x41) for single gripper setups.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for joint names (useful for multi-robot setups).',
        ),
        DeclareLaunchArgument(
            'ns',
            default_value='onrobot',
            description='Namespace for the nodes. Useful for separate gripper and robot control setups.',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization.',
        ),
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='true',
            description='Launch robot state publisher.',
        ),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface for testing.',
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


if __name__ == '__main__':
    generate_launch_description()
