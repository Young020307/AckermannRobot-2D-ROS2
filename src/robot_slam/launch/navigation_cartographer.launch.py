"""
Cartographer 纯定位 + 导航 launch — DWB / NeuPAN

需要先有 pbstream 地图:
  ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
  ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/young/AckermannRobot-2D/src/maps/my_map.pbstream'}"

用法:
  # Cartographer 纯定位 + DWB
  ros2 launch robot_slam navigation_cartographer.launch.py

  # Cartographer 纯定位 + NeuPAN (另开终端跑 run_neupan.sh)
  ros2 launch robot_slam navigation_cartographer.launch.py use_neupan:=true

  # 指定 pbstream 路径
  ros2 launch robot_slam navigation_cartographer.launch.py \
    pbstream_file:=/path/to/my_map.pbstream
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'

    pbstream_file = os.path.join(
        '/home/young/AckermannRobot-2D', 'src', 'maps', 'my_map.pbstream'
    )
    nav_param_file = os.path.join(
        get_package_share_directory(pkg_slam),
        'config',
        'nav2_params.yaml'
    )
    nav2_bringup_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    # ====== Launch Arguments ======
    use_neupan_arg = DeclareLaunchArgument(
        'use_neupan',
        default_value='false',
        description='Use NeuPAN as local planner (switchable via cmd_vel_mux)'
    )
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=pbstream_file,
        description='Absolute path to .pbstream map'
    )

    use_neupan = LaunchConfiguration('use_neupan')

    # ====== 1) Cartographer 纯定位 ======
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory',
            os.path.join(get_package_share_directory(pkg_slam), 'config'),
            '-configuration_basename', 'my_robot_cartographer_2d_localization.lua',
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
        ],
        remappings=[('odom', '/odom_wheel')],
    )

    # ====== 2) Cartographer 占据栅格地图 (/map) ======
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}, {'resolution': 0.05}],
    )

    # ====== 3) Nav2 导航核心 (不含 AMCL / map_server) ======
    nav2_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/navigation_launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav_param_file,
            'autostart': 'True',
            'use_composition': 'False',
        },
    )

    # ====== 4) cmd_vel Bridge (DWB → ackermann controller) ======
    bridge = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(['"', use_neupan, '" == "false"']),
    )

    # ====== 5) cmd_vel Mux (DWB + NeuPAN, default neupan) ======
    mux = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{'use_sim_time': True, 'active_planner': 'neupan'}],
        condition=IfCondition(['"', use_neupan, '" == "true"']),
    )

    # ====== 6) NeuPAN Node ======
    neupan_config_dir = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config', 'robots', 'ackermann_robot'
    )
    neupan_node = Node(
        package='neupan_ros2',
        executable='neupan_node',
        name='neupan_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(neupan_config_dir, 'robot.yaml'),
            {'robot_config_dir': neupan_config_dir},
        ],
        condition=IfCondition(['"', use_neupan, '" == "true"']),
    )

    # ====== 7) RViz2 ======
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz')],
        output='screen',
    )

    return LaunchDescription([
        pbstream_file_arg,
        use_neupan_arg,
        cartographer_node,
        cartographer_occupancy_grid_node,
        nav2_core,
        bridge,
        mux,
        neupan_node,
        rviz,
    ])
