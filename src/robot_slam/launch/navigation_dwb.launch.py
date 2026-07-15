"""
AMCL + DWB 导航 — 纯 Nav2，无 NeuPAN
用法: ros2 launch robot_slam navigation_dwb.launch.py
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

    # Nav2 全栈 (AMCL + map_server + DWB + planner + bt + ...)
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments=[
            ('map', map_file),
            ('use_sim_time', 'True'),
            ('params_file', nav_param_file),
            ('use_composition', 'False'),
        ],
    )

    # DWB /cmd_vel (Twist) → /ackermann_steering_controller/reference (TwistStamped)
    bridge = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    return LaunchDescription([bringup, bridge, rviz])
