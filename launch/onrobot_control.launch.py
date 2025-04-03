#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
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
    description_package = LaunchConfiguration('description_package')
    prefix = LaunchConfiguration('prefix')
    ns = LaunchConfiguration('ns')
    launch_rviz = LaunchConfiguration('launch_rviz')

    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'onrobot_type',
            description='Type of OnRobot gripper.',
            choices=['rg2', 'rg6'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'connection_type',
            description='Connection type for the OnRobot gripper. TCP for the Control Box. Serial for the UR Tool I/O (RS485).',
            choices=['serial', 'tcp'],
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
            'description_package',
            default_value='onrobot_description',
            description='Package with the OnRobot URDF/XACRO files.',
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

    # Path to the xacro file in the onrobot_description package
    xacro_file = PathJoinSubstitution([
        FindPackageShare(description_package),
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
        'prefix:=', prefix,
        ' ',
        'name:=onrobot'
    ])
    robot_description = {'robot_description': robot_description_content}

    # Path to the controller configuration file (using ParameterFile to load YAML)
    controller_config_file = PathJoinSubstitution([
        FindPackageShare('onrobot_driver'),
        'config',
        'rg_controllers.yaml'
    ])
    controller_config = ParameterFile(controller_config_file, allow_substs=True)

    # Launch the ros2_control node
    ros2_control_node = Node(
        namespace=ns,
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        namespace=ns,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )

    # Spawn the joint state and finger width controllers
    joint_state_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    finger_width_spawner = Node(
        namespace=ns,
        package='controller_manager',
        executable='spawner',
        arguments=['finger_width_controller'],
        output='screen'
    )

    # Launch RViz for visualization using the config from onrobot_description
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(description_package),
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

    return LaunchDescription([
        # Declare launch arguments
        *declared_arguments,

        # Launch nodes
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        finger_width_spawner,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
