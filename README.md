# OnRobot_ROS2_Driver

<img src=doc/images/ur_onrobot.gif width=30%>

ROS2 driver for OnRobot Grippers.
When mounted on an Universal Robot e-Series, the gripper can be controlled through Tool I/O via RS-485.

## Features
- ROS2 driver for OnRobot grippers controlled via Modbus TCP or Serial
- Currently supported grippers:
    - [RG2](https://onrobot.com/en/products/rg2-gripper)
    - [RG6](https://onrobot.com/en/products/rg6-gripper) 
- ROS2 Control hardware interface plugin

## Dependencies (included in the installation steps below)

- [onrobot_description](https://github.com/tonydle/OnRobot_ROS2_Description)
- libnet1-dev (for Modbus TCP/Serial)
- [Modbus](https://github.com/Mazurel/Modbus) C++ library (included as a submodule)

## Installation

1. Navigate to your ROS2 workspace and **clone the repository** into the `src` directory:
   ```sh
   git clone https://github.com/tonydle/OnRobot_ROS2_Driver.git src/onrobot_driver
   ```
2. Install git dependencies using `vcs`:
   ```sh
   vcs import src --input src/onrobot_driver/required.repos
   ```
3. Install libnet:
   ```sh
   sudo apt install libnet1-dev
   ```
4. Build using colcon with symlink install:
   ```sh
   colcon build --symlink-install
   ```
5. Source the workspace:
   ```sh
   source install/setup.bash
   ```

## Hardware Setup
### Option 1: Using the Tool I/O of a Universal Robot e-Series robot (Modbus Serial)
1. Use a short cable to connect the OnRobot Quick Changer to the Tool I/O of the UR robot.
2. On the UR Teach Pendant, install the [RS485 Daemon URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap) following the steps below:
   - Download the URCap from [Releases](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/releases)
   - Follow the [URCap Installation Guide](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/blob/master/doc/install_urcap.md)
   - Note: Currently there is a bug where if you have the robotiq_grippers URCap installed, the RS485 URCap cannot run.
    Follow the issue [here](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/issues/9) for updates.
   - Restart robot
3. Setup Tool I/O parameters (Installation -> General -> Tool I/O)

   <img src=doc/images/installation_tool_io.png width=60%>

      - Controlled by: User
      - Communication Interface:
         - Baud Rate: 1M
         - Parity: Even
         - Stop Bits: One
         - RX Idle Chars: 1.5
         - TX Idle Chars: 3.5
      - Tool Output Voltage: 24V
      - Standard Output:
         - Digital Output 0: Sinking (NPN)
         - Digital Output 1: Sinking (NPN)
4. When launching ur_robot_driver, enable `use_tool_communication`. An example launch command is given below:
   ```sh
   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.0.194 use_tool_communication:=true
   ```

### Option 2: Using the OnRobot Compute Box (Modbus TCP)
1. Connect the cable between Compute Box and Tool Changer
2. Connect an Ethernet cable between Compute Box and your computer

## Usage
### Launch the driver
Launch the driver with `onrobot_type` [`rg2`,`rg6`] and `connection_type` [`serial` (UR Tool I/O) or `tcp` (Control Box)] arguments.
   ```sh
   ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=rg2 connection_type:=serial launch_rviz:=true
   ```
### Get the `finger_width` joint state (metres)
   ```sh
   ros2 topic echo /onrobot/joint_states
   ```
### Control the gripper with `finger_width_controller`(JointGroupPositionController)
   ```sh
   ros2 topic pub --once /onrobot/finger_width_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
   ```

## Author
[Tony Le](https://github.com/tonydle)

## License
This software is released under the MIT License, see [LICENSE](./LICENSE).