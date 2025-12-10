# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS 2 driver and ros2_control plugins for OnRobot grippers (RG, 2FG, 3FG).  
Provides C++ APIs, Python bindings and example test nodes to integrate grippers with robot controllers.

Highlights
- Modbus TCP / Serial support for OnRobot grippers.
- Hardware interface plugins for ROS 2 Control: RG, TwoFG and ThreeFG.
- Python bindings (pybind11) for direct scripting and tests.
- Example Python nodes to run motion sequences:
  - [`UR5eFullZTrajectory`](src/test_traj.py)
  - [`UR5eGripperTrajectory`](src/test_traj_with_gripper.py)

Supported grippers
- RG2 / RG6 — see [`RG`](include/onrobot_driver/rg/RG.hpp)
- 2FG7 / 2FG14 — see [`TwoFG`](include/onrobot_driver/twofg/TwoFG.hpp)
- 3FG15 — see [`ThreeFG`](include/onrobot_driver/threefg/ThreeFG.hpp)

Quick start — build
1. Clone repo into ROS2 workspace:
   git clone --recurse-submodules https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
2. Import additional repos:
   vcs import src --input src/onrobot_driver/required.repos
3. Install system dependency:
   sudo apt install libnet1-dev
4. Build:
   colcon build --symlink-install
5. Source:
   source install/setup.bash

Notes about Python modules
- The CMake configuration builds Python modules via pybind11:
  - TwoFG, ThreeFG, RG (see [CMakeLists.txt](CMakeLists.txt)).
- The helper script [install_fg.sh](install_fg.sh) demonstrates how to copy the generated ThreeFG module into a target Python location for legacy setups.

Running the driver (example)
- Launch the ros2_control node with the appropriate hardware plugin and controller configuration using the provided launch file:
  ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=serial
  - The launch implementation exposes [`generate_launch_description`](launch/onrobot_control.launch.py).
- Key launch arguments:
  - onrobot_type: rg2 | rg6 | 2fg7 | 2fg14 | 3fg15
  - connection_type: serial | tcp
  - device / ip_address / port / device_address
  - use_fake_hardware: true | false
  - prefix: joint name prefix for multi-robot setups

Topics and control
- Gripper joint state (meters): /onrobot/joint_states
- Command the gripper with the finger_width controller:
  ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
- Example test nodes:
  - [`src/test_traj.py`](src/test_traj.py) — motion sequence without gripper control.
  - [`src/test_traj_with_gripper.py`](src/test_traj_with_gripper.py) — motion + gripper control sequence.

Plugin and configuration files
- Hardware plugin XMLs:
  - [rg_hardware_interface.xml](rg_hardware_interface.xml)
  - [twofg_hardware_interface.xml](twofg_hardware_interface.xml)
  - [threefg_hardware_interface.xml](threefg_hardware_interface.xml)
- Controller configs in [config/](config):
  - [rg_controllers.yaml](config/rg_controllers.yaml)
  - [twofg_controllers.yaml](config/twofg_controllers.yaml)
  - [threefg_controllers.yaml](config/threefg_controllers.yaml)

Python bindings & examples
- Python modules are built as shared objects and installed with the package (see `pybind11_add_module` entries in [CMakeLists.txt](CMakeLists.txt)).
- Use the provided example nodes to exercise the driver:
  - [`UR5eFullZTrajectory`](src/test_traj.py)
  - [`UR5eGripperTrajectory`](src/test_traj_with_gripper.py)

Development notes & TODO
- Target force is currently set to half the max force (see [`RG::initParams`](src/RG.cpp)). Consider exposing as parameter or service.
- Implement a Gripper Action Controller to support `gripper_action_interface` (see TODO in README).
- Add support for additional OnRobot grippers and features.

License
- MIT — see [LICENSE](LICENSE)

Authors
- Tony Le — original repository author.
- Gabriel Novas - forked repository updater and maintainer