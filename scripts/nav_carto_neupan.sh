#!/bin/bash
# Cartographer 纯定位 + Hybrid A* + NeuPAN 导航（含 Gazebo）
# 用法:
#   终端1: bash scripts/nav_carto_neupan.sh
#   终端2: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$PROJECT_DIR/install/setup.bash"

echo "=== Gazebo + 机器人 ==="
ros2 launch ackermann_robot map.launch.py &
sleep 5

echo "=== Cartographer 纯定位 + Hybrid A* ==="
ros2 launch robot_slam navigation_carto.launch.py \
    map:="$PROJECT_DIR/src/maps/my_new_map.yaml" \
    map_pgm:="$PROJECT_DIR/src/maps/my_new_map.pgm" \
    pbstream:="$PROJECT_DIR/src/maps/my_new_map.pbstream"
