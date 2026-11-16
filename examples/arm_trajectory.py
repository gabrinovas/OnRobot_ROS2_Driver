# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Smooth joint moves of a UR arm, shared by the example scripts of this folder."""

import os
import time

import numpy as np
from onrobot_description.config_loader import load_config, require
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import rclpy
from rclpy.node import Node


def examples_config():
    """Return the values of examples.yaml, the file next to this module."""
    return load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'examples.yaml'))


class ArmTrajectoryNode(Node):
    """Publishes cubic spline trajectories between the joint poses of the configuration."""

    def __init__(self, name, config):
        super().__init__(name)
        self.config = config
        self.arm = require(config, 'arm')
        self.arm['__file__'] = config['__file__']
        self.joints = require(self.arm, 'joints')
        self.publisher = self.create_publisher(
            JointTrajectory, require(self.arm, 'trajectory_topic'),
            require(self.arm, 'queue_depth'))

    def pose(self, name):
        """Return the joint positions of a named pose."""
        return np.array(require(self.arm, 'poses', name), dtype=float)

    def read_current_joints(self):
        """Return the current joint positions, or None when no joint state arrives in time."""
        received = []
        subscription = self.create_subscription(
            JointState, require(self.arm, 'joint_states_topic'), received.append, 1)
        deadline = time.monotonic() + require(self.arm, 'joint_states_wait_s')
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(subscription)
        if not received:
            return None
        positions = dict(zip(received[0].name, received[0].position))
        missing = [joint for joint in self.joints if joint not in positions]
        if missing:
            raise RuntimeError(f'Joint states without the joints {missing}')
        return np.array([positions[joint] for joint in self.joints])

    def spline_points(self, start, end, duration):
        """Return the points of a move that starts and ends at rest, slowed down if too fast."""
        period = 1.0 / require(self.arm, 'sample_rate_hz')
        limit = require(self.arm, 'max_velocity_rad_s') * require(self.arm, 'velocity_margin')

        def splines(seconds):
            at_rest = ((1, 0.0), (1, 0.0))
            return [CubicSpline([0.0, seconds], [start[i], end[i]], bc_type=at_rest)
                    for i in range(len(self.joints))]

        times = np.arange(0, duration + period, period)
        fastest = max(np.max(np.abs(spline(times, 1))) for spline in splines(duration))
        if fastest > limit:
            duration *= fastest / limit
            self.get_logger().info(f'Move slowed down to {duration:.2f} s')
            times = np.arange(0, duration + period, period)

        points = []
        curves = splines(duration)
        for instant in times:
            point = JointTrajectoryPoint()
            point.positions = [float(curve(instant)) for curve in curves]
            point.velocities = [float(curve(instant, 1)) for curve in curves]
            point.accelerations = [float(curve(instant, 2)) for curve in curves]
            point.time_from_start.sec = int(instant)
            point.time_from_start.nanosec = int((instant - int(instant)) * 1e9)
            points.append(point)
        return points, duration

    def move(self, start, pose_name, duration):
        """Publish the move from `start` to a named pose, wait for it and return the pose."""
        target = self.pose(pose_name)
        points, duration = self.spline_points(start, target, duration)
        message = JointTrajectory()
        message.joint_names = list(self.joints)
        message.points = points
        self.publisher.publish(message)
        self.get_logger().info(f"Moving to '{pose_name}': {len(points)} points, {duration:.2f} s")
        settle = 'settle_s' if pose_name == 'home' else 'step_settle_s'
        time.sleep(duration + require(self.arm, settle))
        return target
