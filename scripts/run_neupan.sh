#!/bin/bash
# 手动启动 NeuPAN 节点（需先开 Gazebo + 导航）
# 用法: bash scripts/run_neupan.sh

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

export PYTHONPATH="/home/young/miniconda3/envs/neupan/lib/python3.10/site-packages:$PYTHONPATH"
export LD_LIBRARY_PATH="/home/young/miniconda3/envs/neupan/lib:$LD_LIBRARY_PATH"

ros2 run neupan_ros2 neupan_node --ros-args \
  --params-file "$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot/robot.yaml" \
  -p robot_config_dir:="$PROJECT_DIR/src/neupan_ros2/config/robots/ackermann_robot"
