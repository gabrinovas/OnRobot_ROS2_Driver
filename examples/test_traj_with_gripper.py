#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Example: the moves of test_traj.py with a finger gripper commanded at the start of some of them.

The gripper is driven through its position controller (launch the driver with
use_gripper_action_controller:=false).
"""

import time

from arm_trajectory import ArmTrajectoryNode, examples_config
from onrobot_description.config_loader import require
from std_msgs.msg import Float64MultiArray

import rclpy


def main():
    rclpy.init()
    config = examples_config()
    gripper = require(config, 'finger_gripper')
    gripper['__file__'] = config['__file__']
    node = ArmTrajectoryNode('ur_gripper_trajectory_example', config)
    commands = node.create_publisher(
        Float64MultiArray, require(gripper, 'command_topic'),
        require(config, 'arm', 'queue_depth'))

    def set_width(width):
        commands.publish(Float64MultiArray(data=[float(width)]))
        node.get_logger().info(f'Gripper width commanded: {width * 1000.0:.0f} mm')

    try:
        position = node.read_current_joints()
        if position is None:
            node.get_logger().error('No joint states received: the arm is not moved')
            return
        widths = require(gripper, 'width_after_move')
        for pose_name, duration in require(config, 'sequence'):
            if pose_name in widths:
                set_width(widths[pose_name])
            position = node.move(position, pose_name, duration)
        set_width(require(gripper, 'release_width'))
        time.sleep(require(gripper, 'release_wait_s'))
        node.get_logger().info('Sequence completed')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
