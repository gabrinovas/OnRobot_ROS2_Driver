#!/bin/bash

# Find the new ThreeFG.so module (no longer called FG.so)
SO_FILE=$(find ~/static/drims2_ws/install -name "ThreeFG*.so" | head -1)

if [ -z "$SO_FILE" ]; then
  echo "ERROR: ThreeFG.so not found. Make sure you have run colcon build --symlink-install"
  exit 1
fi

# Change the destination: now the module is called ThreeFG, not FG
TARGET=~/static/drims2_ws/install/ur_onrobot_moveit_config/local/lib/python3.10/dist-packages/ThreeFG.so

mkdir -p "$(dirname "$TARGET")"
cp "$SO_FILE" "$TARGET"
echo "ThreeFG.so copied to $TARGET"

# Optional: create an __init__.py so the package is importable
echo "from .ThreeFG import ThreeFG" > ~/static/drims2_ws/install/ur_onrobot_moveit_config/local/lib/python3.10/dist-packages/__init__.py

source ~/static/drims2_ws/install/setup.bash
python3 -c "from ThreeFG import ThreeFG; print('ThreeFG READY!')"