"""
Cartographer 2D SLAM — 建图模式

完成后一键保存 pbstream + PGM + YAML:
    bash scripts/save_map.sh
"""

import os
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

    # ====== 命令行参数 ======
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用仿真时间 (Gazebo 下必须为 true)'
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='是否启动 RViz2'
    )
    start_gz_arg = DeclareLaunchArgument(
        'start_gz',
        default_value='false',
        description='是否同时启动 Gazebo + 机器人 (单独使用 Cartographer 时设为 false)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    start_gz = LaunchConfiguration('start_gz')

    # ====== 0) (可选) 启动 Gazebo + 机器人 map.launch.py ======
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_robot), 'launch', 'map.launch.py'
            ])
        ]),
        condition=IfCondition(start_gz)
    )

    # ====== 1) Cartographer Node ======
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
            # 与纯定位一致：使用 EKF 融合后的里程计作为 Cartographer 运动先验
            ('odom', '/odometry/filtered'),
        ]
    )

    # ====== 2) Cartographer Occupancy Grid Node ======
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

    # ====== 3) RViz2 ======
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
        gazebo_launch,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])
