#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.interpolate import CubicSpline
import time

class UR5eGripperTrajectory(Node):
    def __init__(self):
        super().__init__('ur5e_gripper_trajectory')

        # Publisher para el robot (sin cambios)
        self.robot_pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        # Publisher para el gripper OnRobot 3FG15
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/onrobot/finger_width_controller/commands',
            10
        )

        self.max_vel = 1.5
        self.freq = 125

        # Posiciones del robot (sin cambios)
        self.q_home = np.array([1.582956, -1.850573, 1.796592, -1.442179, -1.519554, 0.154681])
        self.q_high = np.array([1.582956, -2.2,     1.6,      -1.442179, -1.519554, 0.154681])
        self.q_mid  = np.array([1.582956, -2.0,     1.7,      -1.442179, -1.519554, 0.154681])

        self.t_to_home = 3.0
        self.t_up      = 2.5
        self.t_down    = 2.5
        self.t_back    = 3.0

        print("="*70)
        print("INICIANDO SECUENCIA: HOME → Z → GRIP → HOME")
        print("Control del gripper 3FG15 vía topic ROS2")
        print("="*70)

        self.get_logger().info('Nodo iniciado - control por topic /onrobot/finger_width_controller/commands')

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
            idx = name_to_idx.get(name, i)
            if idx < len(joint_state[0].position):
                q[i] = joint_state[0].position[idx]
        print(f"Posición actual leída: {q.round(4)}")
        return q

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

        # Re-escalado por velocidad máxima
        max_vels = [np.max(np.abs(sp(t_vec, 1))) for sp in splines]
        scale = max(max_vels) / (self.max_vel * 0.9)
        if scale > 1.0:
            duration *= scale
            print(f"Escalando trayectoria → {duration:.2f}s (vel máx: {max(max_vels):.2f} rad/s)")
            times_wp = [0.0, duration]
            splines = [CubicSpline(times_wp, [q_start[i], q_end[i]], bc_type=((1, 0.0), (1, 0.0))) for i in range(6)]
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
    def publish_trajectory(self, points):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        msg.points = points
        self.robot_pub.publish(msg)
        duration = points[-1].time_from_start.sec + points[-1].time_from_start.nanosec / 1e9
        print(f"Publicado: {len(points)} puntos | Duración: {duration:.2f}s")

    # ------------------------------------------------------------------
    # NUEVA FUNCIÓN: Control del gripper por topic
    # ------------------------------------------------------------------
    def set_gripper_width(self, width_meters: float, grip: bool = False):
        """
        Envía comando al gripper 3FG15
        width_meters: diámetro externo deseado en metros (ej: 0.05 = 50 mm)
        grip: si True, hace "grip detect" (cierra hasta detectar pieza)
        """
        msg = Float64MultiArray()
        msg.data = [width_meters]
        
        self.gripper_pub.publish(msg)

    # ------------------------------------------------------------------
    def print_gripper_status(self):
        # Simplemente mostramos el último comando enviado (feedback opcional)
        print("GRIPPER → controlado por topic ROS2                      ", end='\r')

    # ------------------------------------------------------------------
    def run_sequence(self):
        q_current = self.read_current_joints()

        # 1. Ir a HOME
        print("\n1. Moviéndose a HOME...")
        points, _ = self.generate_spline_trajectory(q_current, self.q_home, self.t_to_home)
        self.publish_trajectory(points)
        time.sleep(self.t_to_home + 1.0)

        # 2. Subir + GRIP a ~30 mm (con detección)
        print("\n2. Subiendo en Z + GRIP (30 mm con detección)...")
        points, _ = self.generate_spline_trajectory(self.q_home, self.q_high, self.t_up)
        self.publish_trajectory(points)
        self.set_gripper_width(0.03, grip=True)   # grip detect hasta 30 mm máx
        time.sleep(self.t_up + 0.5)

        # 3. Bajar + asegurar agarre (opcional: cerrar más)
        print("\n3. Bajando + reforzando agarre...")
        points, _ = self.generate_spline_trajectory(self.q_high, self.q_mid, self.t_down)
        self.publish_trajectory(points)
        self.set_gripper_width(0.07, grip=True)   # grip detect hasta 70 mm (más margen)
        time.sleep(self.t_down + 0.5)

        # 4. Volver a HOME + mantener agarre
        print("\n4. Volviendo a HOME (manteniendo pieza)...")
        points, _ = self.generate_spline_trajectory(self.q_mid, self.q_home, self.t_back)
        self.publish_trajectory(points)
        time.sleep(self.t_back + 1.0)

        # Opcional: soltar al final
        print("\nSoltando pieza...")
        self.set_gripper_width(0.150, grip=False)  # abrir completamente (~150 mm)
        time.sleep(2.0)

        print("\n" + "="*70)
        print("SECUENCIA COMPLETADA CON ÉXITO")
        print("="*70)

# ==============================================================================
def main():
    rclpy.init()
    node = UR5eGripperTrajectory()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        print("\nInterrumpido por usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()