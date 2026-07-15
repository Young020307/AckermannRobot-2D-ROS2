#!/bin/bash
# AMCL + NeuPAN 导航（含 Gazebo，无 DWB）
# 用法: bash scripts/nav_amcl_neupan.sh  然后另开终端 bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source "$PROJECT_DIR/install/setup.bash"

echo "=== Gazebo + 机器人 ==="
ros2 launch ackermann_robot map.launch.py &
sleep 5

echo "=== AMCL + Nav2 (planner/bt/smoother, 不含 DWB) ==="
ros2 launch robot_slam navigation_neupan.launch.py
