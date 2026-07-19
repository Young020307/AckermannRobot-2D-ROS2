# AckermannRobot-2D-ROS2

ROS2 阿克曼底盘 2D 激光雷达导航与建图仿真项目。

## 项目结构

```
src/
├── ackermann_robot/        # 机器人模型 (XACRO/URDF)、Gazebo 仿真、控制器配置、键盘控制、cmd_vel_mux
├── hybrid_astar_planner/   # Standalone Hybrid A* 全局规划器 (直读 PGM，无 Nav2 依赖)
├── robot_slam/             # Cartographer SLAM/纯定位配置、导航 launch 文件
├── neupan_ros2/            # NeuPAN 神经网络局部规划器 + DUNE 模型训练
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

## 运行

### 建图 (Cartographer SLAM)

```bash
# 一步启动 Gazebo + 机器人 + Cartographer SLAM
ros2 launch robot_slam slam.launch.py start_gz:=true

# 键盘控制（需要时手动启动）
ros2 launch ackermann_robot keyboard_control.launch.py
```

> 也可以分开启动：终端1 `ros2 launch ackermann_robot map.launch.py`（Gazebo+机器人），终端2 `ros2 launch robot_slam slam.launch.py`（SLAM）。

建图完成后同步保存 pbstream + PGM + YAML：
```bash
bash scripts/save_map.sh

# 或指定输出路径
bash scripts/save_map.sh src/maps/my_new_map
```

### 导航

**架构**: Cartographer 纯定位 + Hybrid A* 全局规划器 + NeuPAN 局部规划器

```bash
# 终端1: Gazebo + 机器人 + Cartographer 纯定位 + Hybrid A*
bash scripts/nav_carto_neupan.sh

# 终端2: NeuPAN 节点 (conda neupan)
bash scripts/run_neupan.sh
```

### 键盘控制

```bash
# 建图或导航时需要手动控制时，单独启动
ros2 launch ackermann_robot keyboard_control.launch.py
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
