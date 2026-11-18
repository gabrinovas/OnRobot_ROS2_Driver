#!/usr/bin/env python3
# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Example: the two vacuum channels of the VGC10, one after the other.

  python3 test_vgc10.py --mode python [--ip-address A] [--port P] [--device-address D]
      talks to the gripper directly (Modbus), without ROS.
  python3 test_vgc10.py --mode ros2
      sends GripperCommand goals to the running driver
      (onrobot_control.launch.py onrobot_type:=vgc10).
"""

import argparse
import time

from arm_trajectory import examples_config
from onrobot_description.config_loader import require


def run_direct(values, overrides):
    """Drive the gripper through its Python class."""
    import onrobot_driver

    gripper = onrobot_driver.create_gripper(require(values, 'onrobot_type'), **overrides)
    wait = require(values, 'step_wait_s')

    def report():
        _, vacuum_a, vacuum_b = gripper.read_both_vacuums()
        print(f'Vacuum: channel A {vacuum_a * 100:.1f} %, channel B {vacuum_b * 100:.1f} % '
              f'(last request {gripper.get_last_roundtrip_ms():.1f} ms)')

    try:
        report()
        print('Channel A grips')
        gripper.grip_channel_a(require(values, 'vacuum_a_percent'))
        time.sleep(wait)
        report()
        print('Channel B grips, channel A keeps its vacuum')
        gripper.grip_channel_b(require(values, 'vacuum_b_percent'))
        time.sleep(wait)
        report()
        print('Channel A releases, channel B keeps its vacuum')
        gripper.release_channel_a()
        time.sleep(wait)
        report()
    finally:
        print('Both channels release')
        gripper.release_all()


def run_actions(values):
    """Drive the gripper through the action controllers of the running driver."""
    from control_msgs.action import GripperCommand
    import rclpy
    from rclpy.action import ActionClient

    rclpy.init()
    node = rclpy.create_node('vgc10_example')
    clients = {channel: ActionClient(node, GripperCommand, require(values, f'action_{channel}'))
               for channel in ('a', 'b')}
    try:
        for client in clients.values():
            if not client.wait_for_server(timeout_sec=require(values, 'action_wait_s')):
                raise RuntimeError('The action servers of the vacuum channels are not available')

        def send(channel, position, effort):
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(effort)
            sent = clients[channel].send_goal_async(goal)
            rclpy.spin_until_future_complete(node, sent)
            if not sent.result().accepted:
                raise RuntimeError(f'Channel {channel}: goal rejected')
            rclpy.spin_until_future_complete(node, sent.result().get_result_async())
            time.sleep(require(values, 'step_wait_s'))

        print('Channel A grips')
        send('a', 1.0, require(values, 'vacuum_a_percent'))
        print('Channel B grips')
        send('b', 1.0, require(values, 'vacuum_b_percent'))
        print('Channel A releases')
        send('a', 0.0, 0.0)
        print('Channel B releases')
        send('b', 0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='OnRobot VGC10 example')
    parser.add_argument('--mode', choices=['python', 'ros2'], required=True)
    for name in ('ip-address', 'port', 'device-address'):
        parser.add_argument(f'--{name}', help='override of config/driver.yaml (mode python)')
    arguments = parser.parse_args()

    config = examples_config()
    values = require(config, 'vacuum_gripper')
    values['__file__'] = config['__file__']
    if arguments.mode == 'python':
        given = {'ip_address': arguments.ip_address, 'port': arguments.port,
                 'device_address': arguments.device_address}
        run_direct(values, {name: value for name, value in given.items() if value is not None})
    else:
        run_actions(values)


if __name__ == '__main__':
    main()
