#!/bin/bash
# 建图完成后同步保存 pbstream + PGM/YAML
#   - pbstream: 通过 cartographer 服务保存
#   - PGM+YAML: 通过 map_saver_cli 直接从 /map topic 保存
#
# 用法:
#   bash scripts/save_map.sh [output_stem]
#
# 默认输出:
#   src/maps/my_map.pbstream
#   src/maps/my_map.pgm
#   src/maps/my_map.yaml

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
MAP_STEM="${1:-$PROJECT_DIR/src/maps/my_map}"

echo "=== 保存 Cartographer 地图 ==="
echo ""

# 1. 结束当前轨迹
echo "[1/3] 结束 trajectory..."
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

sleep 1

# 2. 保存 pbstream
echo "[2/3] 保存 pbstream → ${MAP_STEM}.pbstream ..."
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '${MAP_STEM}.pbstream'}"

sleep 1

# 3. 直接从 /map topic 保存 PGM + YAML
echo "[3/3] 保存 PGM + YAML → ${MAP_STEM}.pgm + ${MAP_STEM}.yaml ..."
ros2 run nav2_map_server map_saver_cli \
    -t /map \
    -f "$MAP_STEM" \
    --occ 0.65 \
    --free 0.25

echo ""
echo "=== 完成 ==="
echo "  pbstream: ${MAP_STEM}.pbstream"
echo "  pgm:      ${MAP_STEM}.pgm"
echo "  yaml:     ${MAP_STEM}.yaml"
grep "origin:" "${MAP_STEM}.yaml"
