#!/usr/bin/env python3
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

def generate_launch_description():
    # Declare launch arguments
    onrobot_type_arg = DeclareLaunchArgument(
        'onrobot_type',
        default_value='rg2',
        description='OnRobot type to load.',
    )
    description_package_arg = DeclareLaunchArgument(
        'description_package',
        default_value='onrobot_description',
        description='Package with the OnRobot URDF/XACRO files.',
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Prefix for joint names (useful for multi-robot setups).',
    )

    # Launch configuration variables
    onrobot_type = LaunchConfiguration('onrobot_type')
    description_package = LaunchConfiguration('description_package')
    prefix = LaunchConfiguration('prefix')

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
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen'
    )

    # Launch the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both'
    )

    # Spawn the joint state and finger width controllers
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    finger_width_spawner = Node(
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
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        onrobot_type_arg,
        description_package_arg,
        prefix_arg,
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_spawner,
        finger_width_spawner,
        rviz_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
