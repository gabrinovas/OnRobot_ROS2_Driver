# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 driver and ros2_control plugins for OnRobot grippers (**2FG7** and **3FG15**).  
Provides C++ APIs, Python bindings and example test nodes to integrate grippers with robot controllers.

## Highlights
- Modbus TCP / Serial support for OnRobot grippers based on OnRobot Connectivity Guide v1.22.0.
- Hardware interface plugins for ROS 2 Control (Humble): TwoFG (2FG7) and ThreeFG (3FG15).
- Python bindings (pybind11) for direct scripting and tests.
- Example Python nodes to run motion sequences:
  - [`UR5eFullZTrajectory`](src/test_traj.py)
  - [`UR5eGripperTrajectory`](src/test_traj_with_gripper.py)

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

## Notes about Python modules
- The CMake configuration builds Python modules via pybind11:
  - TwoFG, ThreeFG (see [CMakeLists.txt](CMakeLists.txt)).
- The helper script [install_fg.sh](install_fg.sh) demonstrates how to copy the generated ThreeFG module into a target Python location for legacy setups.

## Running the driver (example)
- Launch the ros2_control node with the appropriate hardware plugin and controller configuration using the provided launch file:
  ```bash
  ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=2fg7 connection_type:=serial
  ```
  - The launch implementation exposes [`generate_launch_description`](launch/onrobot_control.launch.py).
- Key launch arguments:
  - `onrobot_type`: `2fg7` | `3fg15`
  - `connection_type`: `serial` | `tcp`
  - `device` / `ip_address` / `port` / `device_address`
  - `use_fake_hardware`: `true` | `false`
  - `prefix`: joint name prefix for multi-robot setups

## Topics and control
- Gripper joint state (meters): `/onrobot/joint_states`
- Command the gripper with the finger_width controller:
  ```bash
  ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.035]}"
  ```
- Example test nodes:
  - [`src/test_traj.py`](src/test_traj.py) — motion sequence without gripper control.
  - [`src/test_traj_with_gripper.py`](src/test_traj_with_gripper.py) — motion + gripper control sequence.

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