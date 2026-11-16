# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Python side of the OnRobot driver.

create_gripper() builds the protocol object of a gripper from the configuration files and opens
its connection; the classes themselves are the modules TwoFG, ThreeFG and VGC10 of this package.
"""

from onrobot_driver.factory import create_gripper, driver_config, gripper_joints
from onrobot_driver.factory import load_driver_gripper

__all__ = ['create_gripper', 'driver_config', 'gripper_joints', 'load_driver_gripper']
