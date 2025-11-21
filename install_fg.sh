#!/bin/bash

# Busca el nuevo módulo ThreeFG.so (ya no se llama FG.so)
SO_FILE=$(find ~/static/drims2_ws/install -name "ThreeFG*.so" | head -1)

if [ -z "$SO_FILE" ]; then
  echo "ERROR: ThreeFG.so no encontrado. Asegúrate de haber hecho colcon build --symlink-install"
  exit 1
fi

# Cambia el destino: ahora el módulo se llama ThreeFG, no FG
TARGET=~/static/drims2_ws/install/ur_onrobot_moveit_config/local/lib/python3.10/dist-packages/ThreeFG.so

mkdir -p "$(dirname "$TARGET")"
cp "$SO_FILE" "$TARGET"
echo "ThreeFG.so copiado a $TARGET"

# Opcional: crear un __init__.py para que el paquete sea importable
echo "from .ThreeFG import ThreeFG" > ~/static/drims2_ws/install/ur_onrobot_moveit_config/local/lib/python3.10/dist-packages/__init__.py

source ~/static/drims2_ws/install/setup.bash
python3 -c "from ThreeFG import ThreeFG; print('ThreeFG LISTO!')"