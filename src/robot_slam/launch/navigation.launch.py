"""
统一导航 launch — 支持 AMCL / Cartographer 纯定位 + DWB / NeuPAN

用法:
  # AMCL + DWB (默认)
  ros2 launch robot_slam navigation.launch.py

  # AMCL + NeuPAN (另开终端跑 run_neupan.sh)
  ros2 launch robot_slam navigation.launch.py use_neupan:=true

  # Cartographer 纯定位 + DWB
  ros2 launch robot_slam navigation.launch.py localization_engine:=cartographer

  # Cartographer + NeuPAN
  ros2 launch robot_slam navigation.launch.py localization_engine:=cartographer use_neupan:=true

  # 指定 pbstream 路径
  ros2 launch robot_slam navigation.launch.py \
    localization_engine:=cartographer \
    pbstream_file:=/path/to/my_map.pbstream
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_slam = 'robot_slam'
    pkg_robot = 'ackermann_robot'

    map_file = os.path.join(
        '/home/young/AckermannRobot-2D', 'src', 'maps', 'my_map.yaml'
    )
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

    localization_engine_arg = DeclareLaunchArgument(
        'localization_engine',
        default_value='amcl',
        description='Localization engine: "amcl" (particle filter) or "cartographer" (scan-to-submap)'
    )

    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value=pbstream_file,
        description='Absolute path to .pbstream map for Cartographer localization mode'
    )

    loc_engine = LaunchConfiguration('localization_engine')
    use_neupan = LaunchConfiguration('use_neupan')

    # Humble: LaunchConfiguration('x') == 'y' 返回 bool，不可用于 IfCondition
    # 必须用 PythonExpression 字符串比较
    is_amcl = PythonExpression(['"', loc_engine, '" == "amcl"'])
    is_carto = PythonExpression(['"', loc_engine, '" == "cartographer"'])
    is_neupan = PythonExpression(['"', use_neupan, '" == "true"'])
    is_dwb_only = PythonExpression(['"', use_neupan, '" == "false"'])

    # ==================================================================
    # AMCL 模式: Nav2 bringup（含 AMCL + map_server + 全部导航节点）
    # ==================================================================
    navigation_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/bringup_launch.py']
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav_param_file,
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(is_amcl),
    )

    # ==================================================================
    # Cartographer 纯定位模式
    # ==================================================================

    # --- Cartographer 纯定位节点 ---
    cartographer_localization_node = Node(
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
        remappings=[
            ('odom', '/odom_wheel'),
        ],
        condition=IfCondition(is_carto),
    )

    # --- Cartographer 占据栅格地图发布节点 (/map) ---
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05},
        ],
        condition=IfCondition(is_carto),
    )

    # --- Nav2 导航核心（不含 AMCL / map_server）---
    navigation_core_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/navigation_launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'params_file': nav_param_file,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
        condition=IfCondition(is_carto),
    )

    # ====== cmd_vel Bridge (DWB only) ======
    cmd_vel_bridge_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(is_dwb_only),
    )

    # ====== cmd_vel Mux (DWB + NeuPAN switchable) ======
    cmd_vel_mux_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'active_planner': 'neupan',
        }],
        condition=IfCondition(is_neupan),
    )

    # ====== NeuPAN Node ======
    neupan_config_dir = os.path.join(
        get_package_share_directory('neupan_ros2'),
        'config', 'robots', 'ackermann_robot'
    )
    robot_config = os.path.join(neupan_config_dir, 'robot.yaml')

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
        condition=IfCondition(is_neupan),
    )

    # ====== RViz2 ======
    rviz_config = os.path.join(
        get_package_share_directory(pkg_robot), 'rviz', 'nav2_default_view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        use_neupan_arg,
        localization_engine_arg,
        pbstream_file_arg,
        navigation_bringup_cmd,
        cartographer_localization_node,
        cartographer_occupancy_grid_node,
        navigation_core_cmd,
        cmd_vel_bridge_node,
        cmd_vel_mux_node,
        neupan_node,
        rviz_node,
    ])
