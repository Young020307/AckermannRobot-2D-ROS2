#!/bin/bash
# AMCL + DWB 导航（含 Gazebo，无 NeuPAN）
# 用法: bash scripts/nav_amcl_dwb.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$PROJECT_DIR/install/setup.bash"

echo "=== Gazebo + 机器人 ==="
ros2 launch ackermann_robot map.launch.py &
sleep 5

echo "=== AMCL + DWB ==="
ros2 launch robot_slam navigation_dwb.launch.py
