# AckermannRobot-2D-ROS2

ROS2 阿克曼底盘 2D 激光雷达导航与建图仿真项目。

## 项目结构

```
src/
├── ackermann_robot/        # 机器人模型 (XACRO/URDF)、Gazebo 仿真、控制器配置、键盘控制、cmd_vel_mux
├── hybrid_astar_planner/   # Standalone Hybrid A* 全局规划器 (直读 PGM，无 Nav2 依赖)
├── robot_slam/             # Cartographer SLAM/纯定位配置、导航 launch 文件
├── neupan_ros2/            # NeuPAN 神经网络局部规划器 + DUNE 模型训练
├── scan_filter/            # Bayesian 动态障碍物扫描滤波器 (逐光束静态/动态概率估计)
├── gazebo_worlds/          # Gazebo 世界模型与地图资源
└── maps/                   # 预建地图 (pgm/yaml/pbstream)
```

## 编译与环境设置

```bash
# 依赖安装
sudo apt update && sudo apt install -y \
  ros-humble-topic-tools \
  ros-humble-robot-localization \
  ros-humble-cartographer* \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-twist-stamper \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ackermann-steering-controller \
  libompl-dev \
  libeigen3-dev \
  libceres-dev

# 编译
colcon build --symlink-install
source install/setup.bash
```

## 集中配置

所有运行时设置集中在 **`src/robot_slam/config/sim_config.yaml`**，编辑此文件即可切换场景，无需每次在命令行拼参数。

```yaml
# sim_config.yaml
simulation:
  world: "mini.world"          # 可选: mini, world, world_dynamic, office_world
  use_sim_time: true
  start_rviz: true

scan_filter:
  enabled: false               # 设为 true 启用 Bayesian 动态障碍物滤波器

navigation:
  map_yaml: "src/maps/my_new_map.yaml"
  map_pgm: "src/maps/my_new_map.pgm"
  pbstream: "src/maps/my_new_map.pbstream"
```

> CLI 参数仍可覆盖配置文件的值，适合临时切换：`ros2 launch ... world:=world_dynamic.world`

## 运行

### 建图 (Cartographer SLAM)

```bash
# 一步启动 Gazebo + 机器人 + Cartographer SLAM
ros2 launch robot_slam slam.launch.py start_gz:=true

# 键盘控制（需要时手动启动）
ros2 launch ackermann_robot keyboard_control.launch.py
```

> 也可以分开启动：终端1 `ros2 launch ackermann_robot map.launch.py`，终端2 `ros2 launch robot_slam slam.launch.py`。

建图完成后同步保存 pbstream + PGM + YAML：
```bash
bash scripts/save_map.sh

# 或指定输出路径
bash scripts/save_map.sh src/maps/my_new_map
```

### 动态障碍物滤波器 (Bayesian Scan Filter)

在人群/动态障碍物场景下，LiDAR 扫描中的移动物体会产生与静态地图不匹配的射线，导致 Cartographer 定位精度下降。`scan_filter` 包提供逐光束 Bayesian 静态/动态概率估计，在扫描输入 Cartographer 之前清除动态光束。

**使用方法**: 在 `sim_config.yaml` 中设置 `scan_filter.enabled: true`，或命令行覆盖：

```bash
# 建图模式（带滤波）
ros2 launch robot_slam slam.launch.py start_gz:=true use_scan_filter:=true

# 导航模式（带滤波）—— 配合动态世界
ros2 launch robot_slam navigation_carto.launch.py use_scan_filter:=true
```

**核心机制**:
- **冻结参考 (Frozen Reference)**: 维护每条光束的静态背景参考距离，一旦检测到动态障碍物即冻结参考值，避免参考帧被动态物体污染
- **Bayesian 更新**: 基于当前读数与冻结参考的对比，逐帧更新 P(static) / P(dynamic)
- **运动保护**: 快速旋转时自动跳过更新，避免将自身运动误判为动态物体
- **零侵入**: 不修改 Cartographer 核心代码，仅在 `/scan` → `/scan_filtered` 链路中插入滤波节点

> **参数标定**: Ackermann 最高 1.5 m/s，10 Hz LiDAR → 最大 15 cm/帧位移。`eps_d=0.30 m` 提供 2× 安全余量。滤波器每 50 帧输出诊断日志，显示移除光束百分比——静态场景应接近 0%，动态场景 2-5%。

### 导航

**架构**: Cartographer 纯定位 + Hybrid A* 全局规划器 + NeuPAN 局部规划器

```bash
# 终端1: Gazebo + 机器人 + Cartographer 纯定位 + Hybrid A*
bash scripts/nav_carto_neupan.sh

# 终端2: NeuPAN 节点 (conda neupan)
bash scripts/run_neupan.sh

# 使用动态滤波器版本
bash scripts/nav_carto_neupan_filtered.sh
```

### 键盘控制

```bash
ros2 launch ackermann_robot keyboard_control.launch.py
```

## Gazebo 世界

| 世界文件 | 说明 |
|---------|------|
| `mini.world` (默认) | 小型多房间测试环境，含墙壁、桌子和静态障碍物 |
| `world.world` | 大型走廊式环境，多个房间和门口 |
| `world_dynamic.world` | 大型多房间环境 (60m×24m) + 7 个随机移动方块 → 测试动态滤波器 |
| `office_world.world` | 开放式办公环境，含家具、书架、站立人物 |

在 `sim_config.yaml` 中修改 `simulation.world` 即可切换默认世界，或 CLI 临时覆盖：
```bash
ros2 launch ackermann_robot map.launch.py world:=world_dynamic.world
```

## 车辆参数

| 参数 | 值 |
|------|-----|
| 运动模型 | Ackermann |
| 尺寸 | 0.70m × 0.52m |
| 轴距 | 0.593m |
| 最大转向角 | ±0.52 rad |
| 最小转弯半径 | 1.05m |
| 最大速度 | 1.5 m/s |
| 激光雷达 | 单线 360°，0.15-25m，10Hz |
| IMU | 100Hz |

## 里程计

- `ros2_control` 原始里程计重映射：`odom → /odom_wheel`
- EKF 融合后输出：`/odometry/filtered`
