#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Example: move a UR arm through the poses of examples.yaml (home, up, down, home)."""

from arm_trajectory import ArmTrajectoryNode, examples_config
from onrobot_description.config_loader import require

import rclpy


def main():
    rclpy.init()
    config = examples_config()
    node = ArmTrajectoryNode('ur_trajectory_example', config)
    try:
        position = node.read_current_joints()
        if position is None:
            node.get_logger().error('No joint states received: the arm is not moved')
            return
        for pose_name, duration in require(config, 'sequence'):
            position = node.move(position, pose_name, duration)
        node.get_logger().info('Sequence completed')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
