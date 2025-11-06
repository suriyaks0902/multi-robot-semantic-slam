#!/usr/bin/env bash

set -euo pipefail

WS_DIR="/home/suriya/Desktop/fleet_ws"
SETUP_WS="${WS_DIR}/setup_workspace.sh"

# Ensure TURTLEBOT3 model
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}

open_term() {
  local title="$1"; shift
  local cmd="$*"
  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="$title" -- bash -lc "cd ${WS_DIR}; source ${SETUP_WS}; ${cmd}; exec bash"
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "$title" -e bash -lc "cd ${WS_DIR}; source ${SETUP_WS}; ${cmd}; exec bash" &
  else
    echo "No supported terminal emulator found (gnome-terminal or xterm)." >&2
    exit 1
  fi
}

echo "Launching Week 1 stack with TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"

# 1) Gazebo + Robot + SLAM + Semantic nodes
open_term "Gazebo + Semantic Stack" "ros2 launch semantic_fleet warehouse_semantic_slam.launch.py"

sleep 3

# 2) Teleop in dedicated interactive terminal
open_term "Teleop" "ros2 run turtlebot3_teleop teleop_keyboard"

sleep 2

# 3) RViz with custom config
open_term "RViz Semantic" "ros2 launch semantic_fleet rviz_semantic.launch.py"

echo "All terminals launched."


