#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from scipy.interpolate import CubicSpline
import time
from ThreeFG import ThreeFG  # ← Funciona con el .so copiado


class UR5eGripperTrajectory(Node):

    def __init__(self):
        super().__init__('UR5eGripperTrajectory')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.max_vel = 1.5
        self.freq = 125

        # Posiciones del robot
        self.q_home = np.array([
            1.582956, -1.850573, 1.796592,
            -1.442179, -1.519554, 0.154681
        ])
        self.q_high = np.array([1.582956, -2.2, 1.6, -1.442179, -1.519554, 0.154681])
        self.q_mid  = np.array([1.582956, -2.0, 1.7, -1.442179, -1.519554, 0.154681])

        self.t_to_home = 3.0
        self.t_up      = 2.5
        self.t_down    = 2.5
        self.t_back    = 3.0

        print("="*70)
        print("INICIANDO SECUENCIA: HOME → Z → GRIP → HOME")
        print("="*70)

        # ============ CONEXIÓN GRIPPER ============
        try:
            self.gripper = ThreeFG("192.168.1.1", 502,65)
     
            print("GRIPPER 3FG15 CONECTADO")
            self.get_logger().info("Gripper 3FG15 conectado")
        except Exception as e:
            print(f"GRIPPER NO CONECTADO: {e}")
            self.get_logger().warn(f"Gripper no conectado: {e}")
            self.gripper = None

        self.get_logger().info('Iniciando secuencia: HOME → Z → GRIP → HOME')

    # ------------------------------------------------------------------
    # Lectura única de joint_states
    # ------------------------------------------------------------------
    def read_current_joints(self):
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
            return self.q_home.copy()

        name_to_idx = {n: i for i, n in enumerate(joint_state[0].name)}
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        q = np.zeros(6)
        for i, name in enumerate(order):
            q[i] = joint_state[0].position[name_to_idx.get(name, i)]

        print(f"Posición actual leída: {q.round(4)}")
        return q

    # ------------------------------------------------------------------
    # Generación de trayectoria spline (vel=0 en extremos)
    # ------------------------------------------------------------------
    def generate_spline_trajectory(self, q_start, q_end, duration):
        times_wp = [0.0, duration]
        splines = []
        for i in range(6):
            sp = CubicSpline(times_wp,
                             [q_start[i], q_end[i]],
                             bc_type=((1, 0.0), (1, 0.0)))
            splines.append(sp)

        dt = 1.0 / self.freq
        t_vec = np.arange(0, duration + dt, dt)

        # Re-escalado para respetar velocidad máxima
        max_vels = [np.max(np.abs(sp(t_vec, 1))) for sp in splines]
        scale = max(max_vels) / (self.max_vel * 0.9)
        if scale > 1.0:
            duration *= scale
            print(f"Escalando trayectoria → {duration:.2f}s (vel máx: {max(max_vels):.2f} rad/s)")

            # Regeneramos con la nueva duración
            times_wp = [0.0, duration]
            splines = []
            for i in range(6):
                sp = CubicSpline(times_wp,
                                 [q_start[i], q_end[i]],
                                 bc_type=((1, 0.0), (1, 0.0)))
                splines.append(sp)
            t_vec = np.arange(0, duration + dt, dt)

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

    # ------------------------------------------------------------------
    # Publicación de la trayectoria
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Impresión bonita del estado del gripper (sin hilo)
    # ------------------------------------------------------------------
    def print_gripper_status(self):
        if not self.gripper:
            print("GRIPPER → NO CONECTADO                              ", end='\r')
            return

        try:
            diam = self.gripper.getCurrentDiameter()
            force = self.gripper.getAppliedForce()
            status_dict = self.gripper.getDetailedStatus()

            state = "BUSY" if status_dict.get('busy') else "IDLE"
            grip  = "GRIP" if status_dict.get('grip_detected') else "OPEN"

            print(f"GRIPPER → {state:4} | {grip:4} | Diámetro: {diam*1000:6.1f} mm | Fuerza: {force:5.1f} N",
                  end='\r')
        except Exception as e:
            print(f"GRIPPER → ERROR lectura: {e}                              ", end='\r')

    # ------------------------------------------------------------------
    # SECUENCIA PRINCIPAL (con monitoreo del gripper integrado)
    # ------------------------------------------------------------------
    def run_sequence(self):
        q_current = self.read_current_joints()

        # 1. Ir a HOME
        print("\n1. Moviéndose a HOME...")
        points, _ = self.generate_spline_trajectory(q_current, self.q_home, self.t_to_home)
        #self.publish_trajectory(points)

        start_time = time.time()
        while time.time() - start_time < self.t_to_home + 1.0:
            self.print_gripper_status()
            time.sleep(0.5)

        # 2. Subir + GRIP 0.04 m
        print("\n2. Subiendo en Z + GRIP (0.04 m)...")
        points, _ = self.generate_spline_trajectory(self.q_home, self.q_high, self.t_up)
        #self.publish_trajectory(points)

        if self.gripper:
        
            self.gripper.gripInternal()

            self.gripper.moveGripper(0.03)

            print("   → GRIP ACTIVADO (0.04 m)")

        start_time = time.time()
        while time.time() - start_time < self.t_up + 0.5:
            self.print_gripper_status()
            time.sleep(0.5)

        # 3. Bajar + GRIP 0.1 m
        print("3. Bajando + GRIP (0.1 m)...")
        points, _ = self.generate_spline_trajectory(self.q_high, self.q_mid, self.t_down)
        #self.publish_trajectory(points)

        if self.gripper:
        
            self.gripper.gripInternal()
            self.gripper.moveGripper(0.1)
            print("   → GRIP ACTIVADO (0.1 m)")

        start_time = time.time()
        while time.time() - start_time < self.t_down + 0.5:
            self.print_gripper_status()
            time.sleep(0.5)

        # 4. Volver a HOME + GRIP 0.04 m
        print("4. Volviendo a HOME + GRIP (0.04 m)...")
        points, _ = self.generate_spline_trajectory(self.q_mid, self.q_home, self.t_back)
        #self.publish_trajectory(points)

        if self.gripper:
          
            self.gripper.gripInternal()
            self.gripper.moveGripper(0.07)
            print("   → GRIP ACTIVADO (0.04 m)")

        start_time = time.time()
        while time.time() - start_time < self.t_back + 1.0:
            self.print_gripper_status()
            time.sleep(0.5)

        print("\n" + "="*70)
        print("SECUENCIA COMPLETADA CON ÉXITO")
        print("="*70)


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    rclpy.init()
    node = UR5eGripperTrajectory()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    finally:
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()