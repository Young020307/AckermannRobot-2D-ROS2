"""
Cartographer 2D SLAM — 建图模式

完成后一键保存 pbstream + PGM + YAML:
    bash scripts/save_map.sh
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'
    pkg_carto = 'cartographer_ros'

    # 加载集中配置文件
    config_path = os.path.join(
        get_package_share_directory(pkg_slam), 'config', 'sim_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # ====== 命令行参数 (默认值来自 sim_config.yaml，CLI 可覆盖) ======
    sim_cfg = config.get('simulation', {})
    filter_cfg = config.get('scan_filter', {})

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=str(sim_cfg.get('use_sim_time', True)).lower(),
        description='使用仿真时间 (Gazebo 下必须为 true)'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value=str(sim_cfg.get('start_rviz', True)).lower(),
        description='是否启动 RViz2'
    )
    start_gz_arg = DeclareLaunchArgument(
        'start_gz',
        default_value='false',
        description='是否同时启动 Gazebo + 机器人 (单独使用 Cartographer 时设为 false)'
    )
    use_scan_filter_arg = DeclareLaunchArgument(
        'use_scan_filter',
        default_value=str(filter_cfg.get('enabled', False)).lower(),
        description='是否启用 Bayesian 动态障碍物扫描滤波器'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    start_gz = LaunchConfiguration('start_gz')
    use_scan_filter = LaunchConfiguration('use_scan_filter')

    # ====== 0) (可选) 启动 Gazebo + 机器人 map.launch.py ======
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_robot), 'launch', 'map.launch.py'
            ])
        ]),
        condition=IfCondition(start_gz)
    )

    # ====== 1) Bayesian 动态障碍物扫描滤波器 (可选) ======
    scan_filter_node = Node(
        package='scan_filter',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[os.path.join(
            get_package_share_directory(pkg_slam), 'config', 'scan_filter_params.yaml'
        )],
        condition=IfCondition(use_scan_filter),
    )

    # ====== 2) Cartographer Node ======
    # 当 use_scan_filter=true 时，Cartographer 订阅 /scan_filtered 代替 /scan
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            os.path.join(get_package_share_directory(pkg_slam), 'config'),
            '-configuration_basename', 'my_robot_cartographer_2d.lua'
        ],
        remappings=[
            ('odom', '/odometry/filtered'),
            ('scan', '/scan_filtered'),
        ],
        condition=IfCondition(use_scan_filter),
    )
    cartographer_node_raw = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            os.path.join(get_package_share_directory(pkg_slam), 'config'),
            '-configuration_basename', 'my_robot_cartographer_2d.lua'
        ],
        remappings=[
            ('odom', '/odometry/filtered'),
        ],
        condition=UnlessCondition(use_scan_filter),
    )

    # ====== 3) Cartographer Occupancy Grid Node ======
    # 将 /submap_list 拼接为栅格地图 /map
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'resolution': 0.05},
        ]
    )

    # ====== 4) RViz2 ======
    rviz_config = os.path.join(
        get_package_share_directory(pkg_robot), 'rviz', 'slam_config.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(start_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        start_gz_arg,
        use_scan_filter_arg,
        gazebo_launch,
        scan_filter_node,
        cartographer_node,
        cartographer_node_raw,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])
