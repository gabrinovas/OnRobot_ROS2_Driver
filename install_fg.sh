#!/bin/bash
SO_FILE=$(find ~/mujoco_ros_ws/build/onrobot_driver -name "FG*.so" | head -1)
if [ -z "$SO_FILE" ]; then
  echo "ERROR: FG.so no encontrado. Compila onrobot_driver primero."
  exit 1
fi

TARGET=~/mujoco_ros_ws/install/ur_onrobot_moveit_config/local/lib/python3.10/dist-packages/FG.so
mkdir -p "$(dirname "$TARGET")"
cp "$SO_FILE" "$TARGET"
echo "FG.so copiado a $TARGET"

source ~/mujoco_ros_ws/install/setup.bash
python3 -c "from FG import FG; print('FG LISTO!')"
