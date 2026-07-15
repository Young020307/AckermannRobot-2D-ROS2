import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        description='Use NeuPAN as local planner (with DWB as backup, switchable)'
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

    # ==================================================================
    # AMCL 模式: 使用 Nav2 bringup（含 AMCL + map_server + 全部导航节点）
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
        condition=IfCondition(
            LaunchConfiguration('localization_engine') == 'amcl'
        ),
    )

    # ==================================================================
    # Cartographer 纯定位模式
    #   - cartographer_node: 加载 .pbstream，scan-to-submap 匹配 → 发布 map→odom TF
    #   - cartographer_occupancy_grid_node: 从 pbstream 子图生成 /map (OccupancyGrid)
    #   - navigation_launch.py: planner + controller + bt_navigator + smoother
    #     (不含 AMCL 和 map_server，定位和地图由 Cartographer 提供)
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
        condition=IfCondition(
            LaunchConfiguration('localization_engine') == 'cartographer'
        ),
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
        condition=IfCondition(
            LaunchConfiguration('localization_engine') == 'cartographer'
        ),
    )

    # --- Nav2 导航节点（不含 AMCL / map_server）---
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
        condition=IfCondition(
            LaunchConfiguration('localization_engine') == 'cartographer'
        ),
    )

    # ====== cmd_vel Bridge (DWB only, no NeuPAN) ======
    cmd_vel_bridge_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_stamper.py',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            LaunchConfiguration('use_neupan') == 'false'
        ),
    )

    # ====== cmd_vel Mux (DWB + NeuPAN switchable) ======
    cmd_vel_mux_node = Node(
        package='ackermann_robot',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'active_planner': 'dwb',
        }],
        condition=IfCondition(
            LaunchConfiguration('use_neupan') == 'true'
        ),
    )

    # ====== NeuPAN Node (local planner + controller) ======
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
        condition=IfCondition(
            LaunchConfiguration('use_neupan') == 'true'
        ),
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
