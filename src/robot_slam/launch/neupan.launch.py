"""
NeuPAN Local Planner + Controller — Ackermann Robot

Launches neupan_node alongside the existing navigation stack.
NeuPAN publishes to /neupan_cmd_vel (separate from DWB /cmd_vel).
Use cmd_vel_mux to switch between DWB and NeuPAN at runtime.

Usage:
    # Start with DWB active (default):
    ros2 launch robot_slam neupan.launch.py

    # Start with NeuPAN active:
    ros2 launch robot_slam neupan.launch.py start_active:=neupan
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Robot config directory for NeuPAN
    neupan_config_dir = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config', 'robots', 'ackermann_robot'
    )
    robot_config = os.path.join(neupan_config_dir, 'robot.yaml')

    # Launch arguments
    start_active_arg = DeclareLaunchArgument(
        'start_active',
        default_value='dwb',
        description='Active planner on startup: "dwb" or "neupan"'
    )

    # ====== NeuPAN Node ======
    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_config,
            {'robot_config_dir': neupan_config_dir},
        ],
        # cmd_vel stays on /neupan_cmd_vel (no remap)
        # The mux decides which source to forward
    )

    # ====== cmd_vel Mux (DWB /cmd_vel + NeuPAN /neupan_cmd_vel → controller) ======
    cmd_vel_mux_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'active_planner': LaunchConfiguration('start_active'),
        }]
    )

    return LaunchDescription([
        start_active_arg,
        neupan_node,
        cmd_vel_mux_node,
    ])
