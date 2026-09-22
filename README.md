# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 driver and ros2_control plugins for OnRobot grippers (**2FG7** and **3FG15**).  
Provides C++ APIs, Python bindings and example test nodes to integrate grippers with robot controllers.

## Highlights
- Modbus TCP / Serial support for OnRobot grippers based on OnRobot Connectivity Guide v1.22.0.
- Hardware interface plugins for ROS 2 Control (Humble): TwoFG (2FG7) and ThreeFG (3FG15).
- Python bindings (pybind11) for direct scripting and tests.
- Example Python nodes to run motion sequences (in [examples/](examples)):
  - [`UR5eFullZTrajectory`](examples/test_traj.py)
  - [`UR5eGripperTrajectory`](examples/test_traj_with_gripper.py)

## Supported grippers
- 2FG7 — see [`TwoFG`](include/onrobot_driver/twofg/TwoFG.hpp)
- 3FG15 — see [`ThreeFG`](include/onrobot_driver/threefg/ThreeFG.hpp)

## Quick start — build
1. Clone repo into ROS2 workspace:
   ```bash
   git clone --recurse-submodules https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
   ```
2. Import additional repos:
   ```bash
   vcs import src --input src/onrobot_driver/required.repos
   ```
3. Install system dependency:
   ```bash
   sudo apt install libnet1-dev
   ```
4. Build:
   ```bash
   colcon build --symlink-install
   ```
5. Source:
   ```bash
   source install/setup.bash
   ```

## Python Modules (pybind11)
The driver automatically installs Python bindings via `ament_cmake_python`. Once `source install/setup.bash` is run, both import styles work natively without any manual file copying:

```python
# Direct module import:
import TwoFG
import ThreeFG

# Or namespaced package import:
from onrobot_driver import TwoFG, ThreeFG

gripper = TwoFG.TwoFG("2fg7", "192.168.1.1", 502, 65)
print("Current width:", gripper.getWidth())
```

## Running the driver (example)
- Launch the ros2_control node with the standard MoveIt 2 action controller:
   ```bash
   ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=2fg7 connection_type:=serial
   ```
- Key launch arguments:
  - `onrobot_type`: `2fg7` | `3fg15`
  - `connection_type`: `serial` | `tcp`
  - `use_gripper_action_controller`: `true` (default, standard GripperCommand action server) | `false` (topic-based position controller)
  - `device` / `ip_address` / `port` / `device_address`
  - `use_fake_hardware`: `true` | `false`
  - `prefix`: joint name prefix for multi-robot setups

## Control Interfaces
- Gripper joint state (position, velocity, effort in N): `/onrobot/joint_states`
- **MoveIt 2 Action Server** (default):
  ```bash
  ros2 action send_goal /onrobot/gripper_action_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.04, max_effort: 40.0}}"
  ```
- **Topic-based command** (when `use_gripper_action_controller:=false`):
  ```bash
  ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.035]}"
  ```
- Example motion sequences:
  - [`examples/test_traj.py`](examples/test_traj.py) — robot trajectory test.
  - [`examples/test_traj_with_gripper.py`](examples/test_traj_with_gripper.py) — coordinated robot and gripper sequence.

## Plugin and configuration files
- Hardware plugin XMLs:
  - [twofg_hardware_interface.xml](twofg_hardware_interface.xml)
  - [threefg_hardware_interface.xml](threefg_hardware_interface.xml)
- Controller configs in [config/](config):
  - [twofg_controllers.yaml](config/twofg_controllers.yaml)
  - [threefg_controllers.yaml](config/threefg_controllers.yaml)

## License
- MIT — see [LICENSE](LICENSE)

## Authors
- Tony Le — original repository author.
- Gabriel Novas - forked repository updater and maintainer