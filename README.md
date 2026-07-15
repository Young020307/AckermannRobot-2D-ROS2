# AckermannRobot-2D-ROS2

ROS2 阿克曼底盘 2D 激光雷达导航与建图仿真项目。

## 项目结构

```
src/
├── ackermann_robot/    # 机器人模型 (XACRO/URDF)、Gazebo 仿真、控制器配置、键盘控制、cmd_vel_mux
├── robot_slam/         # Nav2 导航参数、Cartographer SLAM/纯定位、AMCL 定位、launch 文件
├── neupan_ros2/        # NeuPAN 神经网络局部规划器 + DUNE 模型训练
├── gazebo_worlds/      # Gazebo 世界模型与地图资源
└── maps/               # 预建地图 (pgm/yaml/pbstream)
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
  ros-humble-ackermann-steering-controller

# 编译
colcon build --symlink-install
source install/setup.bash
```

## 运行

### 建图 (Cartographer SLAM)

```bash
# 终端1: Gazebo + 机器人 (不含键盘控制)
ros2 launch ackermann_robot map.launch.py

# 终端2: 键盘控制（需要时手动启动）
ros2 launch ackermann_robot keyboard_control.launch.py
```

建图完成后保存 pbstream：
```bash
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/home/young/AckermannRobot-2D/src/maps/my_map.pbstream'}"
```

### 导航

```bash
# Gazebo + 机器人
ros2 launch ackermann_robot map.launch.py

# AMCL + DWB
bash scripts/nav_amcl_dwb.sh

# AMCL + NeuPAN (需另开终端启动 NeuPAN 节点)
bash scripts/nav_amcl_neupan.sh       # Gazebo + AMCL + Nav2
bash scripts/run_neupan.sh            # NeuPAN 节点 (conda neupan)

# Cartographer 纯定位 + DWB（需先有 pbstream 地图）
bash scripts/nav_carto_dwb.sh

# 也可手动组合参数：
#   localization_engine: "amcl" (默认) 或 "cartographer"
#   use_neupan: true/false
#
#   ros2 launch robot_slam navigation.launch.py \
#     localization_engine:=cartographer use_neupan:=true \
#     pbstream_file:=/path/to/my_map.pbstream
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
