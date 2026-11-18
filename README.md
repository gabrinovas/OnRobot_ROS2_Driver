# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 (Humble) driver of the OnRobot grippers **2FG7**, **3FG15** and **VGC10**: `ros2_control` hardware
plugins, a status monitor, C++ protocol classes with Python modules, and examples. The grippers are reached by
Modbus TCP (Compute Box) or Modbus RTU (RS485 of the robot tool connector), following the OnRobot Connectivity
Guide v1.22.0.

The geometry of the grippers is in
[OnRobot_ROS2_Description](https://github.com/gabrinovas/OnRobot_ROS2_Description).

## Build

```sh
git clone --recurse-submodules https://github.com/gabrinovas/OnRobot_ROS2_Driver.git src/OnRobot_ROS2_Driver
vcs import src --input src/OnRobot_ROS2_Driver/required.repos
sudo apt install libnet1-dev
colcon build
source install/setup.bash
```

## Run a gripper

```sh
ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=2fg7 ip_address:=<address of your Compute Box>
```

starts, in the namespace `onrobot`, a controller manager with the plugin of the gripper, its controllers, the
status monitor and RViz.

| Argument | Meaning |
|---|---|
| `onrobot_type` | gripper: `2fg7`, `3fg15`, `vgc10` (the files in `config/grippers`) |
| `connection_type` | `tcp` (Compute Box) or `serial` (tool RS485) |
| `ip_address`, `port` | Compute Box (tcp). `ip_address` has no default: it is required with the real gripper over tcp |
| `device` | serial device (serial) |
| `device_address` | Modbus address of the gripper |
| `use_fake_hardware` | simulate the gripper inside the driver |
| `use_gripper_action_controller` | `true`: GripperCommand action controllers (MoveIt 2); `false`: position controllers |
| `prefix`, `ns` | joint name prefix, namespace |
| `launch_rsp`, `launch_rviz` | start a robot_state_publisher, start RViz |

The value an argument takes when it is not given is in [config/driver.yaml](config/driver.yaml).

### Interfaces

| What | Name |
|---|---|
| joint states | `/onrobot/joint_states` |
| finger grippers, action | `/onrobot/gripper_action_controller/gripper_cmd` (`control_msgs/action/GripperCommand`: width in m, effort in N) |
| finger grippers, topic | `/onrobot/finger_width_controller/commands` (`std_msgs/msg/Float64MultiArray`) |
| VGC10, actions | `/onrobot/gripper_channel_a_controller/gripper_cmd`, `.../gripper_channel_b_controller/gripper_cmd` (position 0 to 1 = vacuum, effort = vacuum in %) |
| VGC10, topic | `/onrobot/vacuum_group_controller/commands` |
| state as text, diagnostics | `/onrobot/gripper_status`, `/diagnostics` |
| tool power reset | `/onrobot/reset_power` (`std_srvs/srv/Trigger`) |

```sh
ros2 action send_goal /onrobot/gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 40.0}}"
```

## Configuration

No value is written in the code. Everything is in `config/`:

| File | Content |
|---|---|
| `driver.yaml` | launch arguments and their defaults, Modbus timeouts, Modbus address of the Compute Box |
| `grippers/<gripper>.yaml` | plugin of the gripper, the values the plugin needs, its controllers, its Python class |
| `controllers/*.yaml` | parameters of the controllers |
| `gripper_status_monitor.yaml` | topics and thresholds of the monitor |

The launch file and the hardware description ([urdf/onrobot_ros2_control.xacro](urdf/onrobot_ros2_control.xacro))
read these files; the plugins receive every value as a `<param>` of the `<ros2_control>` block and do not start
when one is missing (the error names it). Errors of this kind start with `[Fail-Fast]`.

### Grip force of the 2FG7

The 2FG7 can grip with up to 140 N, and this driver limits it to 70 N. Two values hold that limit and a third
one decides what is actually asked for; to use the whole range, change the three together:

| Where | Value | What it is |
|---|---|---|
| `config/grippers/2fg7.yaml` | `hardware.parameters.max_force: 70.0` | highest force the plugin writes to the gripper; a higher command is cut to it |
| `config/controllers/twofg_controllers.yaml` | `gripper_action_controller.max_effort: 70.0` | highest effort the action controller accepts |
| your application | `max_effort` of its `GripperCommand` goals | the force of each grip |

### In another description

```xml
<xacro:include filename="$(find onrobot_driver)/urdf/onrobot_ros2_control.xacro"/>
<xacro:onrobot_ros2_control onrobot_type="2fg7" prefix="" use_fake_hardware="false" use_mock_hardware="false"
  connection_type="tcp" device="/tmp/ttyUR" ip_address="192.0.2.1" port="502" device_address="65"/>
```

`use_fake_hardware` keeps the plugin and makes it simulate the gripper; `use_mock_hardware` replaces the plugin
by the generic mock of ros2_control.

### Adding a gripper

1. Its description (see OnRobot_ROS2_Description).
2. Its protocol class (`include/onrobot_driver/<gripper>/`, `src/gripper/`), from `OnRobotGripperBase`.
3. Its plugin: a finger gripper only says which protocol class it uses (see
   `src/hardware/twofg_hardware_interface.cpp`); add it to `cmake/onrobot_plugins.cmake` and its
   `<gripper>_hardware_interface.xml`.
4. `config/grippers/<gripper>.yaml` and its controllers file.

## Python

```python
import onrobot_driver

gripper = onrobot_driver.create_gripper('2fg7', ip_address='192.0.2.1')  # your Compute Box
print('Width:', gripper.getWidth())
gripper.moveGripper(0.05, True)
```

`create_gripper` builds the protocol object from the configuration files and opens the connection; the
connection values can be overridden (`connection_type`, `ip_address`, `port`, `device`, `device_address`). The
classes are `onrobot_driver.TwoFG.TwoFG`, `onrobot_driver.ThreeFG.ThreeFG` and `onrobot_driver.VGC10.VGC10`.

## Code

| Folder | Content |
|---|---|
| `include/onrobot_driver/common`, `src/gripper` | Modbus connections and the base of the protocol classes |
| `include/onrobot_driver/<gripper>` | protocol class, register map and plugin of each gripper |
| `include/onrobot_driver/hardware`, `src/hardware` | what the plugins share; the finger and vacuum plugins |
| `include/onrobot_driver/monitor`, `src/monitor` | status monitor |
| `src/bindings`, `onrobot_driver/` | Python modules and helpers |
| `cmake/` | build, one file per kind of target |
| `include/onrobot_driver/Modbus` | bundled Modbus library (git submodule) |

Tests: `colcon test --packages-select onrobot_driver` (protocol classes against a mock connection, the plugins
against the generated description, configuration, linters).

## License

MIT, see [LICENSE](LICENSE). Tony Le (original author), Gabriel Novas.
