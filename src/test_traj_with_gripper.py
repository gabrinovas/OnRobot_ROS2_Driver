#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from scipy.interpolate import CubicSpline
import time
from ThreeFG import ThreeFG  # ← Asegúrate de tener el .so en la misma carpeta o en PYTHONPATH


class UR5eGripperTrajectory(Node):
    def __init__(self):
        super().__init__('ur5e_gripper_trajectory')

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',  # o /joint_trajectory_controller si usas otro
            10
        )

        self.max_vel = 1.5      # rad/s máximo permitido por joint
        self.freq = 125         # Hz de la trayectoria

        # ==============================================================================
        # POSICIONES 100% ADAPTADAS AL ROBOT QUE TIENES AHORA (mirando hacia delante)
        # shoulder_pan ≈ 0.05 rad → robot girado ~180° respecto al script original
        # ==============================================================================
        self.q_home = np.array([ 0.0503, -1.8506,  1.7966, -1.4422, -1.5196,  0.1547])
        self.q_high = np.array([ 0.0503, -2.2000,  1.6000, -1.4422, -1.5196,  0.1547])  # +10 cm aprox en Z
        self.q_mid  = np.array([ 0.0503, -2.0000,  1.7000, -1.4422, -1.5196,  0.1547])  # posición de agarre

        # Tiempos de cada tramo (ajústalos si quieres más rápido o más lento)
        self.t_to_home = 3.0
        self.t_up      = 2.5
        self.t_down    = 2.5
        self.t_back    = 3.0

        print("="*80)
        print("  SECUENCIA ADAPTADA AL ROBOT GIRADO 180° (shoulder_pan ≈ 0 rad)")
        print("  HOME → SUBIR → AGARRE → BAJAR → SOLTAR → HOME")
        print("="*80)

        # ============ CONEXIÓN GRIPPER 3FG15 ============
        try:
            self.gripper = ThreeFG("192.168.1.1", 502, 65)
            print("GRIPPER 3FG15 CONECTADO CORRECTAMENTE")
            self.get_logger().info("Gripper 3FG15 conectado")
        except Exception as e:
            print(f"GRIPPER NO CONECTADO: {e}")
            self.get_logger().warn(f"Gripper no conectado: {e}")
            self.gripper = None

    # ------------------------------------------------------------------
    # Lectura única de joint_states
    # ------------------------------------------------------------------
    def read_current_joints(self):
        joint_state = [None]

        def callback(msg):
            if joint_state[0] is None:
                joint_state[0] = msg

        sub = self.create_subscription(JointState, '/joint_states', callback, 10)
        start = self.get_clock().now()
        while joint_state[0] is None and (self.get_clock().now() - start).nanoseconds * 1e-9 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)

        if joint_state[0] is None:
            print("No se pudo leer /joint_states → usando q_home como fallback")
            return self.q_home.copy()

        name_to_idx = {n: i for i, n in enumerate(joint_state[0].name)}
        order = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        q = np.zeros(6)
        for i, name in enumerate(order):
            idx = name_to_idx.get(name, i)
            if idx < len(joint_state[0].position):
                q[i] = joint_state[0].position[idx]

        print(f"Posición actual leída: {q.round(4)}")
        return q

    # ------------------------------------------------------------------
    # Generación de trayectoria cúbica con velocidad 0 en extremos
    # ------------------------------------------------------------------
    def generate_spline_trajectory(self, q_start,Q_end, duration):
        times_wp = [0.0, duration]
        splines = []
        for i in range(6):
            sp = CubicSpline(times_wp,
                             [q_start[i], q_end[i]],
                             bc_type=((1, 0.0), (1, 0.0)))  # vel=0 en inicio y fin
            splines.append(sp)

        dt = 1.0 / self.freq
        t_vec = np.arange(0, duration + dt, dt)

        # Re-escalado automático para no superar max_vel
        max_vels = [np.max(np.abs(sp(t_vec, 1))) for sp in splines]
        scale = max(max_vels) / (self.max_vel * 0.95)
        if scale > 1.0:
            duration *= scale
            print(f"  → Trayectoria escalada a {duration:.2f}s para respetar vel ≤ {self.max_vel} rad/s")

            # Regenerar con nueva duración
            times_wp = [0.0, duration]
            splines = []
            for i in range(6):
                sp = CubicSpline(times_wp, [q_start[i], q_end[i]], bc_type=((1, 0.0), (1, 0.0)))
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
    # Publicación
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
        print(f"  Publicado: {len(points)} puntos | Duración: {duration:.2f}s")

    # ------------------------------------------------------------------
    # Estado del gripper en tiempo real
    # ------------------------------------------------------------------
    def print_gripper_status(self):
        if not self.gripper:
            print("GRIPPER → NO CONECTADO          ", end='\r')
            return
        try:
            diam = self.gripper.getCurrentDiameter() * 1000   # mm
            force = self.gripper.getAppliedForce()
            status = self.gripper.getDetailedStatus()
            state = "BUSY" if status.get('busy') else "IDLE"
            grip = "GRIP" if status.get('grip_detected') else "OPEN"
            print(f"GRIPPER → {state} | {grip} | Ø {diam:6.1f} mm | F {force:4.1f} N          ", end='\r')
        except:
            print("GRIPPER → ERROR lectura          ", end='\r')

    # ------------------------------------------------------------------
    # SECUENCIA PRINCIPAL
    # ------------------------------------------------------------------
    def run_sequence(self):
        q_current = self.read_current_joints()

        print("\n1. Moviendo a posición HOME segura...")
        points, _ = self.generate_spline_trajectory(q_current, self.q_home, self.t_to_home)
        self.publish_trajectory(points)
        start_time = time.time()
        while time.time() - start_time < self.t_to_home + 1.5:
            self.print_gripper_status()
            time.sleep(0.2)

        print("\n2. Subiendo en Z y preparando agarre...")
        points, _ = self.generate_spline_trajectory(self.q_home, self.q_high, self.t_up)
        self.publish_trajectory(points)
        if self.gripper:
            self.gripper.gripInternal()      # agarre interno
            self.gripper.moveGripper(0.040)  # abrir un poco antes de bajar
        start_time = time.time()
        while time.time() - start_time < self.t_up + 0.5:
            self.print_gripper_status()
            time.sleep(0.2)

        print("\n3. Bajando y agarrando el objeto...")
        points, _ = self.generate_spline_trajectory(self.q_high, self.q_mid, self.t_down)
        self.publish_trajectory(points)
        if self.gripper:
            self.gripper.gripInternal()
            self.gripper.moveGripper(0.100)  # agarre fuerte
        start_time = time.time()
        while time.time() - start_time < self.t_down + 1.0:
            self.print_gripper_status()
            time.sleep(0.2)

        time.sleep(1.0)  # esperar a que el grip se estabilice

        print("\n4. Volviendo a HOME con el objeto...")
        points, _ = self.generate_spline_trajectory(self.q_mid, self.q_home, self.t_back)
        self.publish_trajectory(points)
        if self.gripper:
            self.gripper.moveGripper(0.070)  # mantener agarre medio
        start_time = time.time()
        while time.time() - start_time < self.t_back + 1.0:
            self.print_gripper_status()
            time.sleep(0.2)

        print("\n" + "="*80)
        print("      SECUENCIA COMPLETADA CON ÉXITO")
        print("="*80)


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    rclpy.init()
    node = UR5eGripperTrajectory()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        print("\n\nInterrumpido por el usuario")
    finally:
        time.sleep(1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()