#!/bin/bash
# Cartographer 纯定位 + DWB 导航（含 Gazebo）
# 用法: bash scripts/nav_carto_dwb.sh
# 需要先有 my_map.pbstream (见 README 建图部分)

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$PROJECT_DIR/install/setup.bash"

echo "=== Gazebo + 机器人 ==="
ros2 launch ackermann_robot map.launch.py &
sleep 5

echo "=== Cartographer 纯定位 + DWB ==="
ros2 launch robot_slam navigation_cartographer.launch.py
