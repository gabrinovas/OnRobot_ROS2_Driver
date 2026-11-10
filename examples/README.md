# Ejemplos de Trayectorias y Control (OnRobot Gripper + UR5e)

Este directorio contiene scripts de ejemplo para probar secuencias de trayectoria y sincronización entre manipuladores (ej. Universal Robots UR5e) y pinzas OnRobot (**2FG7** y **3FG15**).

## Archivos de Ejemplo

- **`test_traj.py`**:
  Ejecuta una trayectoria suave en el eje Z (HOME → Subida → Bajada → HOME) para un robot UR5e a través del tópico `/scaled_joint_trajectory_controller/joint_trajectory` utilizando interpolación por splines cúbicos a 125 Hz. No interactúa directamente con la pinza.

- **`test_traj_with_gripper.py`**:
  Ejecuta una secuencia combinada de manipulador y pinza OnRobot (HOME → Z → GRIP → HOME):
  1. Mueve el robot UR5e hacia abajo.
  2. Cierra la pinza enviando un comando al controlador de la pinza (`/onrobot/finger_width_controller/commands`).
  3. Levanta el robot con el objeto agarrado.
  4. Vuelve a abrir la pinza y retorna a HOME.

## Prerrequisitos

- Driver o simulación de Universal Robots activa (`ur_robot_driver` o simulación con `ros2_control` / URSim).
- Controlador `scaled_joint_trajectory_controller` activo para el brazo UR.
- Driver de la pinza activo (`ros2 launch onrobot_driver onrobot_control.launch.py ...`).
- Dependencias de Python adicionales:
  ```bash
  pip install numpy scipy
  ```

## Modo de Ejecución

Una vez que el entorno de ROS 2 esté configurado y los nodos levantados:

```bash
# Sincronización de trayectoria brazo + pinza
python3 examples/test_traj_with_gripper.py

# Trayectoria de prueba solo de brazo
python3 examples/test_traj.py
```
