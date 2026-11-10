#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_vgc10.py - Script de prueba interactivo / demostración para OnRobot VGC10

Prueba el control de doble canal independiente (Canal A y Canal B) vía Python (Modbus TCP)
o mediante el envío de acciones de ROS 2.

Ejecutar dentro del contenedor Docker generado por Dockerfile.aimen:
    python3 examples/test_vgc10.py --ip 192.168.1.1 --mode python
o
    python3 examples/test_vgc10.py --mode ros2
"""

import argparse
import time
import sys

def test_direct_python(ip: str, port: int, device_address: int):
    print(f"\n[VGC10 Python Test] Conectando a Compute Box en {ip}:{port} (Addr: {device_address})...")
    try:
        from onrobot_driver import VGC10
    except ImportError:
        try:
            import VGC10
        except ImportError:
            print("❌ Error: No se pudo importar el módulo VGC10. Asegúrate de compilar y hacer source install/setup.bash")
            sys.exit(1)

    try:
        gripper = VGC10("vgc10", ip, port, device_address)
        print("✅ Conexión establecida con el VGC10.")
    except Exception as e:
        print(f"❌ Error al conectar con el VGC10: {e}")
        return

    try:
        print("\n--- 1. Lectura inicial de telemetría (Single Modbus Transaction) ---")
        ok, vac_a, vac_b = gripper.read_both_vacuums()
        print(f"Vacío Canal A: {vac_a * 100:.1f} % | Vacío Canal B: {vac_b * 100:.1f} %")
        print(f"Estado de conexión: {'Conectado' if gripper.is_connected() else 'Desconectado'} | Latencia: {gripper.get_last_roundtrip_ms():.2f} ms | Peticiones: {gripper.get_total_requests()}")

        print("\n--- 2. Succión en Canal A (60% vacío) manteniendo Canal B liberado ---")
        gripper.grip_channel_a(60)
        time.sleep(2.0)
        print(f"Vacío Canal A: {gripper.get_vacuum_channel_a() * 100:.1f} %")
        print(f"Vacío Canal B: {gripper.get_vacuum_channel_b() * 100:.1f} %")

        print("\n--- 3. Succión en Canal B (70% vacío) manteniendo Canal A aspirando ---")
        gripper.grip_channel_b(70)
        time.sleep(2.0)
        print(f"Vacío Canal A: {gripper.get_vacuum_channel_a() * 100:.1f} %")
        print(f"Vacío Canal B: {gripper.get_vacuum_channel_b() * 100:.1f} %")

        print("\n--- 4. Liberar Canal A (manteniendo Canal B con vacío) ---")
        gripper.release_channel_a()
        time.sleep(2.0)
        print(f"Vacío Canal A: {gripper.get_vacuum_channel_a() * 100:.1f} %")
        print(f"Vacío Canal B: {gripper.get_vacuum_channel_b() * 100:.1f} %")

        print("\n--- 5. Liberar ambos canales ---")
        gripper.release_all()
        time.sleep(1.0)
        print("✅ Test completado con éxito.")

    except KeyboardInterrupt:
        print("\nInterrupción detectada. Liberando canales de vacío...")
        gripper.release_all()
    except Exception as e:
        print(f"❌ Error durante la ejecución del test: {e}")
        gripper.release_all()

def test_ros2_actions():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from control_msgs.action import GripperCommand

    print("\n[VGC10 ROS 2 Test] Enviando comandos a través de GripperActionController...")
    rclpy.init()
    node = Node("test_vgc10_ros2_client")

    client_a = ActionClient(node, GripperCommand, "/onrobot/gripper_channel_a_controller/gripper_cmd")
    client_b = ActionClient(node, GripperCommand, "/onrobot/gripper_channel_b_controller/gripper_cmd")

    print("Esperando servidores de acción...")
    if not client_a.wait_for_server(timeout_sec=5.0) or not client_b.wait_for_server(timeout_sec=5.0):
        print("❌ No se encontraron los servidores de acción. Asegúrate de haber lanzado onrobot_control.launch.py con onrobot_type:=vgc10")
        rclpy.shutdown()
        return

    def send_cmd(client, position: float, max_effort: float):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        handle = future.result()
        if not handle.accepted:
            print("Comando rechazado")
            return
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)

    print("\n-> Aspirando con Canal A (vacío al 60%)...")
    send_cmd(client_a, position=1.0, max_effort=60.0)
    time.sleep(2.0)

    print("-> Aspirando con Canal B (vacío al 70%)...")
    send_cmd(client_b, position=1.0, max_effort=70.0)
    time.sleep(2.0)

    print("-> Soltando Canal A...")
    send_cmd(client_a, position=0.0, max_effort=0.0)
    time.sleep(2.0)

    print("-> Soltando Canal B...")
    send_cmd(client_b, position=0.0, max_effort=0.0)
    print("✅ Prueba ROS 2 finalizada.")

    node.destroy_node()
    rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Prueba del gripper OnRobot VGC10")
    parser.add_argument("--mode", choices=["python", "ros2"], default="python",
                        help="Modo de prueba: python (conexión directa Modbus TCP) o ros2 (Action Servers)")
    parser.add_argument("--ip", default="192.168.1.1", help="IP del Compute Box OnRobot")
    parser.add_argument("--port", type=int, default=502, help="Puerto Modbus TCP (por defecto 502)")
    parser.add_argument("--device-address", type=int, default=65, help="Dirección Modbus de la herramienta (por defecto 65)")

    args = parser.parse_args()
    if args.mode == "python":
        test_direct_python(args.ip, args.port, args.device_address)
    else:
        test_ros2_actions()

if __name__ == "__main__":
    main()
