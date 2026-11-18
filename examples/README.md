# Examples

Scripts that show how to drive a UR arm and an OnRobot gripper. Their values (topics, poses, times, vacuum
levels) are in [examples.yaml](examples.yaml): the poses are those of one UR5e cell, so adapt them before
running the scripts on a real robot.

| Script | What it does |
|---|---|
| `test_traj.py` | Moves the arm home, up, down and home again with smooth trajectories sent to the scaled joint trajectory controller. |
| `test_traj_with_gripper.py` | The same moves, commanding a finger gripper (2FG7, 3FG15) at the start of some of them through `/onrobot/finger_width_controller/commands`. |
| `test_vgc10.py` | Uses the two vacuum channels of the VGC10 one after the other, either directly from Python (`--mode python`) or through the action controllers of the running driver (`--mode ros2`). |

`arm_trajectory.py` holds what the first two share.

## Requirements

- A running arm driver (real or simulated) with `scaled_joint_trajectory_controller` active.
- For the gripper scripts, the gripper driver:
  `ros2 launch onrobot_driver onrobot_control.launch.py onrobot_type:=3fg15 use_gripper_action_controller:=false`
  (`test_traj_with_gripper.py`), or `onrobot_type:=vgc10` (`test_vgc10.py --mode ros2`).
- `numpy` and `scipy` for the trajectory scripts.

## Running

With the workspace sourced, from this folder:

```sh
python3 test_traj.py
python3 test_traj_with_gripper.py
python3 test_vgc10.py --mode python --ip-address <address of your Compute Box>
python3 test_vgc10.py --mode ros2
```
