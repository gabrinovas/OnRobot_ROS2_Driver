#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    # Launch configuration variables
    onrobot_type = LaunchConfiguration('onrobot_type')
    connection_type = LaunchConfiguration('connection_type')
    device = LaunchConfiguration('device')
    ip_address = LaunchConfiguration('ip_address')
    port = LaunchConfiguration('port')
    device_address = LaunchConfiguration('device_address')
    prefix = LaunchConfiguration('prefix')
    ns = LaunchConfiguration('ns')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_rsp = LaunchConfiguration('launch_rsp')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'onrobot_type',
            description='Type of OnRobot gripper.',
            choices=['rg2', 'rg6', '2fg7', '2fg14', '3fg15'],
            default_value='rg2',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'connection_type',
            description='Connection type for the OnRobot gripper. TCP for the Control Box. Serial for the UR Tool I/O (RS485).',
            choices=['serial', 'tcp'],
            default_value='tcp',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'device',
            default_value='/tmp/ttyUR',
            description='Device name for the serial connection. Only used when connection_type is serial.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ip_address',
            default_value='192.168.1.1',
            description='IP address for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value='502',
            description='Port for the TCP connection. Only used when connection_type is tcp.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'device_address',
            default_value='65',
            description='Modbus device address for the gripper. Default is 65 for single gripper setups.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for joint names (useful for multi-robot setups).',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'ns',
            default_value='onrobot',
            description='Namespace for the nodes. Useful for separate gripper and robot control setups.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz for visualization.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'launch_rsp',
            default_value='true',
            description='Launch robot state publisher.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Use fake hardware interface for testing.',
        )
    )

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
        'onrobot_type:=', onrobot_type,
        ' ',
        'connection_type:=', connection_type,
        ' ',
        'device:=', device,
        ' ',
        'ip_address:=', ip_address,
        ' ',
        'port:=', port,
        ' ',
        'device_address:=', device_address,
        ' ',
        'prefix:=', prefix,
        ' ',
        'use_fake_hardware:=', use_fake_hardware,
        ' ',
        'name:=onrobot'
    ])
    
    # Use ParameterValue to properly handle the robot description string
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Determine which controller config to use based on gripper type
    def get_controller_config():
        onrobot_type_str = str(onrobot_type)
        if onrobot_type_str.startswith('2fg'):
            return PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'config',
                'twofg_controllers.yaml'
            ])
        elif onrobot_type_str == '3fg15':
            return PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'config',
                'threefg_controllers.yaml'
            ])
        else:  # rg2, rg6
            return PathJoinSubstitution([
                FindPackageShare('onrobot_driver'),
                'config',
                'rg_controllers.yaml'
            ])

    # Determine which hardware interface to use based on gripper type
    def get_hardware_interface():
        onrobot_type_str = str(onrobot_type)
        if onrobot_type_str.startswith('2fg'):
            return 'onrobot_driver::TwoFGHardwareInterface'
        elif onrobot_type_str == '3fg15':
            return 'onrobot_driver::ThreeFGHardwareInterface'
        else:  # rg2, rg6
            return 'onrobot_driver::RGHardwareInterface'

    # Path to the appropriate controller configuration file
    controller_config_file = get_controller_config()
    controller_config = ParameterFile(controller_config_file, allow_substs=True)

    # Add hardware interface parameter to robot description
    hardware_interface_plugin = get_hardware_interface()
    robot_description_with_hw = robot_description.copy()
    robot_description_with_hw['hardware_interface_plugin'] = hardware_interface_plugin

    # Launch the ros2_control node
    ros2_control_node = Node(
        namespace=ns,
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_with_hw, controller_config],
        output='screen'
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        namespace=ns,
        package='robot_state_publisher',
        condition=IfCondition(launch_rsp),
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )

    # Spawn the joint state and gripper controllers
    joint_state_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Determine which controller to spawn based on gripper type
    def get_controller_name():
        onrobot_type_str = str(onrobot_type)
        if onrobot_type_str.startswith('2fg') or onrobot_type_str == '3fg15':
            return 'finger_width_controller'
        else:  # rg2, rg6
            return 'finger_width_controller'  # Use same name as your functional config

    controller_name = get_controller_name()
    gripper_controller_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=[controller_name],
        output='screen'
    )

    # Launch RViz for visualization using the config from onrobot_description
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_description'),
        'rviz',
        'view_onrobot.rviz'
    ])
    rviz_node = Node(
        namespace=ns,
        package='rviz2',
        condition=IfCondition(launch_rviz),
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # Optional: Add a node to monitor gripper status (for all gripper types)
    gripper_status_node = Node(
        namespace=ns,
        package='onrobot_driver',
        executable='gripper_status_monitor',
        name='gripper_status_monitor',
        output='screen',
        parameters=[{
            'onrobot_type': onrobot_type,
        }]
    )

    nodes_to_start = [
        # Declare launch arguments
        *declared_arguments,

        # Launch nodes
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        gripper_controller_spawner,
        rviz_node,
        gripper_status_node,
    ]

    return LaunchDescription(nodes_to_start)

if __name__ == '__main__':
    generate_launch_description()
