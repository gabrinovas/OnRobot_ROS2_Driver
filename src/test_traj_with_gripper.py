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
from FG import FG  # ← Funciona con el .so copiado

class UR5eGripperTrajectory(Node):
    def __init__(self):
        super().__init__('UR5eGripperTrajectory')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # === Publicadores de estado del gripper ===
        self.pub_diam = self.create_publisher(Float32, '/gripper/diameter', 10)
        self.pub_force = self.create_publisher(Float32, '/gripper/force', 10)
        self.pub_busy = self.create_publisher(Bool, '/gripper/busy', 10)
        self.pub_status = self.create_publisher(String, '/gripper/status', 10)

        self.max_vel = 1.5
        self.freq = 125

        # TU HOME REAL (de /joint_states)
        self.q_home = np.array([
            1.582956,   # shoulder_pan_joint
            -1.850573,  # shoulder_lift_joint
            1.796592,   # elbow_joint
            -1.442179,  # wrist_1_joint
            -1.519554,  # wrist_2_joint
            0.154681    # wrist_3_joint
        ])

        # Waypoints Z (solo shoulder_lift y elbow cambian)
        self.q_high = np.array([1.582956, -2.2,  1.6, -1.442179, -1.519554, 0.154681])  # Z ≈ 0.70 m
        self.q_mid  = np.array([1.582956, -2,  1.7, -1.442179, -1.519554, 0.154681])  # Z ≈ 0.60 m

        self.t_to_home = 3.0
        self.t_up      = 2.5
        self.t_down    = 2.5
        self.t_back    = 3.0

        # === IMPRESIÓN INICIAL ===
        print("="*70)
        print("INICIANDO SECUENCIA: HOME → Z → GRIP → HOME")
        print("="*70)

        # === Gripper ===
        try:
            self.gripper = FG("3fg15", "192.168.1.1", 502)
            print("GRIPPER 3FG15 CONECTADO")
            self.get_logger().info("Gripper 3FG15 conectado")
        except Exception as e:
            print(f"GRIPPER NO CONECTADO: {e}")
            self.get_logger().warn(f"Gripper no conectado: {e}")
            self.gripper = None

        # === Hilo de monitoreo ===
        self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info('Iniciando secuencia: HOME → Z → GRIP → HOME')

    # =============================================
    # MÉTODOS DE TRAYECTORIA
    # =============================================
    def read_current_joints(self):
        """Lee /joint_states una vez"""
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
            print("No se pudo leer /joint_states → usando q_home")
            self.get_logger().error('No se pudo leer /joint_states')
            return self.q_home.copy()

        name_to_idx = {n: i for i, n in enumerate(joint_state[0].name)}
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        q = np.zeros(6)
        for i, name in enumerate(order):
            q[i] = joint_state[0].position[name_to_idx.get(name, i)]
        print(f"Posición actual: {q.round(4)}")
        return q

    def generate_spline_trajectory(self, q_start, q_end, duration):
        """Spline cúbico con vel=0 en extremos"""
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
            print(f"Escalando trayectoria → {duration:.2f}s (vel máx: {max(max_vels):.2f} rad/s)")
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
        print(f"Publicado: {len(points)} puntos | Duración: {duration:.2f}s")
        self.get_logger().info(f'Publicado: {len(points)} puntos')

    # =============================================
    # MONITOR LOOP (IMPRIME EN TERMINAL)
    # =============================================
    def monitor_loop(self):
        rate = self.create_rate(2)  # 2 Hz
        while rclpy.ok():
            if not self.gripper:
                rate.sleep()
                continue
            try:
                diam = self.gripper.getCurrentDiameter()
                force = self.gripper.getAppliedForce()
                status_dict = self.gripper.getDetailedStatus()

                # Publicar
                self.pub_diam.publish(Float32(data=diam))
                self.pub_force.publish(Float32(data=force))
                self.pub_busy.publish(Bool(data=status_dict.get('busy', False)))

                # IMPRIMIR EN TERMINAL
                state = "BUSY" if status_dict.get('busy') else "IDLE"
                grip = "GRIP" if status_dict.get('grip_detected') else "OPEN"
                print(f"GRIPPER → {state} | {grip} | Diámetro: {diam*1000:6.1f}mm | Fuerza: {force:5.1f}N", end='\r')

                # Publicar estado
                msg = f"{state} | {grip} | {diam*1000:.1f}mm | {force:.1f}N"
                self.pub_status.publish(String(data=msg))
            except Exception as e:
                print(f"Error lectura gripper: {e}", end='\r')
                self.get_logger().warn(f"Error lectura gripper: {e}")
            rate.sleep()

    # =============================================
    # SECUENCIA (IMPRIME CADA PASO)
    # =============================================
    def run_sequence(self):
        q_current = self.read_current_joints()

        # 1. Ir a HOME
        print("\n1. Moviéndose a HOME...")
        self.get_logger().info('1. Moviéndose a HOME...')
        points, _ = self.generate_spline_trajectory(q_current, self.q_home, self.t_to_home)
        self.publish_trajectory(points)
        time.sleep(self.t_to_home + 1.0)

        # 2. Subir en Z + GRIP 0.04m
        print("2. Subiendo en Z + GRIP (0.04m)...")
        self.get_logger().info('2. Subiendo en Z...')
        points, _ = self.generate_spline_trajectory(self.q_home, self.q_high, self.t_up)
        self.publish_trajectory(points)

        if self.gripper:
            self.gripper.setTargetWidth(0.04)
            self.gripper.gripInternal()
            print("GRIP ACTIVADO → 0.04m")
            self.get_logger().info("→ GRIP ACTIVADO")
    
        time.sleep(self.t_up + 0.5)

        # 3. Bajar + GRIP 0.1m
        print("3. Bajando + GRIP (0.1m)...")
        self.get_logger().info('3. Bajando + AGARRAR...')
        points, _ = self.generate_spline_trajectory(self.q_high, self.q_mid, self.t_down)
        self.publish_trajectory(points)

        if self.gripper:
            self.gripper.setTargetWidth(0.1)
            self.gripper.gripInternal()
            print("GRIP ACTIVADO → 0.1m")
            self.get_logger().info("→ GRIP ACTIVADO")

        time.sleep(self.t_down + 0.5)

        # 4. Volver a HOME + GRIP 0.04m
        print("4. Volviendo a HOME + GRIP (0.04m)...")
        self.get_logger().info('4. Volviendo a HOME...')
        points, _ = self.generate_spline_trajectory(self.q_mid, self.q_home, self.t_back)
        self.publish_trajectory(points)

        if self.gripper:
            self.gripper.setTargetWidth(0.04)
            self.gripper.gripInternal()
            print("GRIP ACTIVADO → 0.04m")
            self.get_logger().info("→ GRIP ACTIVADO")

        time.sleep(self.t_back + 1.0)

        print("\n" + "="*70)
        print("SECUENCIA COMPLETADA CON ÉXITO")
        print("="*70)
        self.get_logger().info('Secuencia completa finalizada.')

# =============================================
# MAIN
# =============================================
def main():
    rclpy.init()
    node = UR5eGripperTrajectory()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    finally:
        time.sleep(2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()