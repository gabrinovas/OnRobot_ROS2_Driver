#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool, String
import numpy as np
from scipy.interpolate import CubicSpline
import time
import threading
#from TwoFG import TwoFG
from ThreeFG import ThreeFG

class UR5eGripperTrajectory(Node):
    def __init__(self):
        super().__init__('UR5eGripperTrajectory')

        # Gripper type parameter (can be set via command line)
        self.declare_parameter('gripper_type', '3fg15')
        self.gripper_type = self.get_parameter('gripper_type').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # === Gripper state publishers ===
        self.pub_diam = self.create_publisher(Float32, '/gripper/diameter', 10)
        self.pub_force = self.create_publisher(Float32, '/gripper/force', 10)
        self.pub_busy = self.create_publisher(Bool, '/gripper/busy', 10)
        self.pub_status = self.create_publisher(String, '/gripper/status', 10)

        self.max_vel = 1.5
        self.freq = 125

        # YOUR REAL HOME (from /joint_states)
        self.q_home = np.array([
            1.582956,   # shoulder_pan_joint
            -1.850573,  # shoulder_lift_joint
            1.796592,   # elbow_joint
            -1.442179,  # wrist_1_joint
            -1.519554,  # wrist_2_joint
            0.154681    # wrist_3_joint
        ])

        # Z Waypoints (only shoulder_lift and elbow change)
        self.q_high = np.array([1.582956, -2.2,  1.6, -1.442179, -1.519554, 0.154681])  # Z ≈ 0.70 m
        self.q_mid  = np.array([1.582956, -2,  1.7, -1.442179, -1.519554, 0.154681])  # Z ≈ 0.60 m

        self.t_to_home = 3.0
        self.t_up      = 2.5
        self.t_down    = 2.5
        self.t_back    = 3.0

        # === INITIAL PRINT ===
        print("="*70)
        print(f"STARTING SEQUENCE: HOME → Z → GRIP → HOME")
        print(f"GRIPPER TYPE: {self.gripper_type.upper()}")
        print("="*70)

        # === Gripper - Modular approach ===
        self.gripper = None
        try:
            if self.gripper_type == "3fg15":
                self.gripper = ThreeFG("192.168.1.1", 502,65)
                print("3FG15 GRIPPER CONNECTED")
                self.get_logger().info("3FG15 Gripper connected")
            elif self.gripper_type.startswith("2fg"):
                self.gripper = TwoFG(self.gripper_type, "192.168.1.1", 502)
                print(f"{self.gripper_type.upper()} GRIPPER CONNECTED")
                self.get_logger().info(f"{self.gripper_type.upper()} Gripper connected")
            else:
                print(f"Unsupported gripper type: {self.gripper_type}")
                self.get_logger().warn(f"Unsupported gripper type: {self.gripper_type}")
        except ImportError as e:
            print(f"Error importing gripper module: {e}")
            self.get_logger().error(f"Error importing gripper module: {e}")
        except Exception as e:
            print(f"Error connecting with gripper: {e}")
            self.get_logger().error(f"Error connecting with gripper: {e}")

        # === Monitoring thread ===
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info(f'Starting sequence for {self.gripper_type}: HOME → Z → GRIP → HOME')

    # =============================================
    # TRAJECTORY METHODS
    # =============================================
    def read_current_joints(self):
        """Reads /joint_states once"""
        joint_state = [None]

        def callback(msg):
            if joint_state[0] is None:
                joint_state[0] = msg

        sub = self.create_subscription(JointState, '/joint_states', callback, 1)
        start = self.get_clock().now()
        while joint_state[0] is None and (self.get_clock().now() - start).nanoseconds * 1e-9 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        if joint_state[0] is None:
            print("Could not read /joint_states → using q_home")
            self.get_logger().error('Could not read /joint_states')
            return self.q_home.copy()

        name_to_idx = {n: i for i, n in enumerate(joint_state[0].name)}
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        q = np.zeros(6)
        for i, name in enumerate(order):
            q[i] = joint_state[0].position[name_to_idx.get(name, i)]
        print(f"Current position: {q.round(4)}")
        return q

    def generate_spline_trajectory(self, q_start, q_end, duration):
        """Cubic spline with vel=0 at the ends"""
        times_wp = [0.0, duration]
        waypoints = [q_start, q_end]

        splines = []
        for i in range(6):
            sp = CubicSpline(times_wp, [waypoints[0][i], waypoints[1][i]],
                             bc_type=((1, 0.0), (1, 0.0)))
            splines.append(sp)

        dt = 1.0 / self.freq
        t_vec = np.arange(0, duration + dt, dt)
        max_vels = [np.max(np.abs(sp(t_vec, 1))) for sp in splines]
        scale = max(max_vels) / (self.max_vel * 0.9)
        if scale > 1.0:
            duration *= scale
            print(f"Scaling trajectory → {duration:.2f}s (max vel: {max(max_vels):.2f} rad/s)")
            return self.generate_spline_trajectory(q_start, q_end, duration)

        points = []
        for t in t_vec:
            pt = JointTrajectoryPoint()
            pt.positions = [float(sp(t)) for sp in splines]
            pt.velocities = [float(sp(t, 1)) for sp in splines]
            pt.accelerations = [float(sp(t, 2)) for sp in splines]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            points.append(pt)
        return points, duration

    def publish_trajectory(self, points):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        msg.points = points
        self.publisher_.publish(msg)
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
        print(f"Published: {len(points)} points | Duration: {duration:.2f}s")
        self.get_logger().info(f'Published: {len(points)} points')

    # =============================================
    # MONITOR LOOP (PRINTS TO TERMINAL)
    # =============================================
    def monitor_loop(self):
        rate = self.create_rate(2)  # 2 Hz
        while rclpy.ok():
            if not self.gripper:
                rate.sleep()
                continue
            try:
                # Common methods for both gripper types
                width = self.gripper.getWidth()
                status_list = self.gripper.getStatus()
                
                # Publish common data
                self.pub_diam.publish(Float32(data=width))
                self.pub_busy.publish(Bool(data=bool(status_list.get('busy', False))))

                # Gripper-specific monitoring
                if self.gripper_type == "3fg15":
                    # 3FG15 specific monitoring
                    force = self.gripper.getAppliedForce()
                    status_dict = self.gripper.getDetailedStatus()
                    
                    self.pub_force.publish(Float32(data=force))
                    
                    state = "BUSY" if status_dict.get('busy', False) else "IDLE"
                    grip = "GRIP" if status_dict.get('grip_detected', False) else "OPEN"
                    print(f"3FG15 → {state} | {grip} | Diameter: {width*1000:6.1f}mm | Force: {force:5.1f}N", end='\r')
                    
                    msg = f"3FG15 | {state} | {grip} | {width*1000:.1f}mm | {force:.1f}N"
                else:
                    # 2FG series monitoring
                    state = "BUSY" if status_list[0] else "IDLE"
                    grip = "GRIP" if status_list[1] else "OPEN"
                    print(f"{self.gripper_type.upper()} → {state} | {grip} | Width: {width*1000:6.1f}mm", end='\r')
                    
                    msg = f"{self.gripper_type.upper()} | {state} | {grip} | {width*1000:.1f}mm"

                self.pub_status.publish(String(data=msg))

            except Exception as e:
                print(f"Error reading gripper {self.gripper_type}: {e}", end='\r')
                self.get_logger().warn(f"Error reading gripper {self.gripper_type}: {e}")
            rate.sleep()

    # =============================================
    # GRIPPER CONTROL METHODS
    # =============================================
    def control_gripper(self, width, action_name=""):
        """Control the gripper based on its type"""
        if not self.gripper:
            print(f"GRIPPER UNAVAILABLE for {action_name}")
            return
            
        try:
            if self.gripper_type == "3fg15":
                # 3FG15 only supports internal grip
                self.gripper.setTargetWidth(width)
                self.gripper.gripInternal()
                print(f"3FG15 → {action_name}: Diameter {width*1000:.1f}mm")
            else:
                # 2FG series - use external grip by default
                self.gripper.moveGripper(width, external_grip=True)
                print(f"{self.gripper_type.upper()} → {action_name}: Width {width*1000:.1f}mm")
                
        except Exception as e:
            print(f"Error controlling gripper {self.gripper_type}: {e}")
            self.get_logger().error(f"Error controlling gripper: {e}")

    # =============================================
    # SEQUENCE (PRINTS EACH STEP)
    # =============================================
    def run_sequence(self):
        q_current = self.read_current_joints()

        # Define gripper widths based on type
        if self.gripper_type == "3fg15":
            grip_narrow = 0.9   # 40mm diameter
            grip_wide = 2.0     # 100mm diameter  
            grip_final = 1.0    # 40mm diameter
        else:  # 2FG series
            grip_narrow = 0.035  # 35mm width
            grip_wide = 0.07     # 70mm width
            grip_final = 0.05    # 50mm width

        # 1. Go to HOME
        """
        print("\n1. Moving to HOME...")
        self.get_logger().info('1. Moving to HOME...')
        points, _ = self.generate_spline_trajectory(q_current, self.q_home, self.t_to_home)
        self.publish_trajectory(points)
        time.sleep(self.t_to_home + 1.0)
       
        # 2. Move up in Z + narrow GRIP
        print("2. Moving up in Z + narrow GRIP...")
        self.get_logger().info('2. Moving up in Z...')
        points, _ = self.generate_spline_trajectory(self.q_home, self.q_high, self.t_up)
        self.publish_trajectory(points)
        """
        self.control_gripper(grip_narrow, "NARROW GRIP")
        time.sleep(self.t_up + 0.5)
        """
        # 3. Move down + wide GRIP
        print("3. Moving down + wide GRIP...")
        self.get_logger().info('3. Moving down + GRAB...')
        points, _ = self.generate_spline_trajectory(self.q_high, self.q_mid, self.t_down)
        self.publish_trajectory(points)
        """
        self.control_gripper(grip_wide, "WIDE GRIP")
        time.sleep(self.t_down + 0.5)
        """
        # 4. Return to HOME + narrow GRIP
        print("4. Returning to HOME + narrow GRIP...")
        self.get_logger().info('4. Returning to HOME...')
        points, _ = self.generate_spline_trajectory(self.q_mid, self.q_home, self.t_back)
        self.publish_trajectory(points)
        """
        self.control_gripper(grip_final, "FINAL GRIP")
        time.sleep(self.t_back + 1.0)

        print("\n" + "="*70)
        print("SEQUENCE COMPLETED SUCCESSFULLY")
        print("="*70)
        self.get_logger().info('Full sequence completed.')

# =============================================
# MAIN
# =============================================
def main():
    rclpy.init()
    node = UR5eGripperTrajectory()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"\nError during sequence: {e}")
    finally:
        time.sleep(2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()