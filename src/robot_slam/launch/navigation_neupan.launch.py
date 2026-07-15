"""
AMCL + NeuPAN 导航 — DWB 后台待命，NeuPAN 负责实际控制
用法:
  终端1: ros2 launch robot_slam navigation_neupan.launch.py
  终端2: bash scripts/run_neupan.sh
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'

    map_file = os.path.join('/home/young/AckermannRobot-2D', 'src', 'maps', 'my_map.yaml')
    nav_param_file = os.path.join(get_package_share_directory(pkg_slam), 'config', 'nav2_params.yaml')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Nav2 全栈 — DWB 完整运行 (progress/goal checker 正常工作)，
    # 但 cmd_vel 由 mux 决定走 DWB 还是 NeuPAN
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments=[
            ('map', map_file),
            ('use_sim_time', 'True'),
            ('params_file', nav_param_file),
            ('use_composition', 'False'),
        ],
    )

    # cmd_vel_mux — DWB + NeuPAN 双路，默认走 NeuPAN
    mux = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'active_planner': 'neupan',
        }],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    return LaunchDescription([bringup, mux, rviz])
