# Copyright 2026 Tony Le, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Build and connect the protocol object of a gripper from the configuration files."""

import importlib
import os

from onrobot_description.config_loader import ConfigError, load_config, package_file, require
from onrobot_description.grippers import gripper_names, load_gripper

PACKAGE = 'onrobot_driver'


def driver_config():
    """Return the content of config/driver.yaml of this package."""
    return load_config(package_file(PACKAGE, 'config', 'driver.yaml'))


def load_driver_gripper(name):
    """Return how the driver runs one gripper (config/grippers/<name>.yaml of this package)."""
    if name not in gripper_names():
        available = ', '.join(gripper_names())
        raise ConfigError(f"[Fail-Fast] Unknown gripper '{name}'. Available grippers: {available}")
    return load_config(package_file(PACKAGE, os.path.join('config', 'grippers', f'{name}.yaml')))


def gripper_joints(name, prefix):
    """Return the names of the joints of a gripper as they are in the URDF, with their prefix."""
    return [prefix + joint for joint in require(load_gripper(name), 'joints')]


def _constructor_arguments(description, parameters):
    """Return the keyword arguments of the protocol class, by kind of gripper."""
    if require(description, 'kind') == 'finger':
        joint = next(iter(require(description, 'joints')))
        return {
            'min_width': require(description, 'joints', joint, 'lower'),
            'max_width': require(description, 'joints', joint, 'upper'),
            'max_force': require(parameters, 'max_force'),
            'default_force': require(parameters, 'default_force'),
            'default_speed': require(parameters, 'default_speed'),
        }
    names = ('max_vacuum_percent', 'max_current_ma', 'initial_current_ma', 'grip_threshold')
    return {name: require(parameters, name) for name in names}


def create_gripper(name, **overrides):
    """
    Build the protocol object of the gripper `name` and open its connection.

    Every value comes from config/driver.yaml and from the two files of the gripper. `overrides`
    replaces connection values: connection_type, ip_address, port, device, device_address.
    """
    config = driver_config()
    gripper = load_driver_gripper(name)
    arguments = require(config, 'launch_arguments')
    known = ('connection_type', 'ip_address', 'port', 'device', 'device_address')
    unknown = set(overrides) - set(known)
    if unknown:
        raise ConfigError(
            f'[Fail-Fast] Unknown connection values {sorted(unknown)}; valid: {known}')
    connection = {key: overrides.get(key, require(arguments, key, 'default')) for key in known}
    if connection['connection_type'] == 'tcp' and not connection['ip_address']:
        raise ConfigError(
            "[Fail-Fast] create_gripper needs ip_address=<address of the Compute Box> with the "
            'tcp connection: it belongs to your installation and has no default')

    module = importlib.import_module(f"{PACKAGE}.{require(gripper, 'python', 'module')}")
    parameters = dict(require(gripper, 'hardware', 'parameters'), __file__=gripper['__file__'])
    instance = getattr(module, require(gripper, 'python', 'class'))(
        device_address=int(connection['device_address']),
        **_constructor_arguments(load_gripper(name), parameters))

    retry = require(config, 'connection', 'connect_retry_ms')
    if connection['connection_type'] == 'tcp':
        instance.connect_tcp(
            ip_address=str(connection['ip_address']), port=int(connection['port']),
            timeout_ms=require(config, 'connection', 'timeout_ms'), connect_retry_ms=retry)
    elif connection['connection_type'] == 'serial':
        instance.connect_serial(
            device=str(connection['device']),
            serial_timeout_ms=require(config, 'connection', 'serial_timeout_ms'),
            connect_retry_ms=retry)
    else:
        raise ConfigError(
            f"[Fail-Fast] Unsupported connection_type '{connection['connection_type']}' "
            '(tcp or serial)')
    return instance
