"""
Cartographer 纯定位 + Hybrid A* + NeuPAN 导航

对标 AckermannRobot-3D 的 navigation_hdl.launch.py，定位方式用 Cartographer
纯定位替代 hdl_localization。全局规划器 Hybrid A* 直读 PGM，局部规划器 NeuPAN。

TF 链路:
    map ←(Cartographer 纯定位)← odom ←(EKF)← base_link ←(URDF)← laser_link

组件:
  - cartographer_node: 纯定位模式，加载 pbstream → map→odom TF
  - scan_filter_node (可选): Bayesian 动态障碍物滤波器 → /scan_filtered
  - hybrid_astar_planner: 加载 PGM → /plan (Path) + /map (OccupancyGrid)
  - cmd_vel_mux: NeuPAN Twist → TwistStamped → 阿克曼控制器

默认配置来自 sim_config.yaml，CLI 参数可覆盖：
  ros2 launch robot_slam navigation_carto.launch.py \
      map:=src/maps/other_map.yaml \
      pbstream:=src/maps/other_map.pbstream
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _resolve_path(raw_path, base_dir):
    """根据前缀将配置中的路径解析为绝对路径。"""
    if not raw_path:
        return ''
    if raw_path.startswith('./'):
        return os.path.normpath(os.path.join(base_dir, raw_path[2:]))
    if raw_path.startswith('src/'):
        # 项目根: install/robot_slam/share/robot_slam → 四层回到项目根
        ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(base_dir))))
        return os.path.normpath(os.path.join(ws_root, raw_path))
    return raw_path


def parse_map_yaml(context):
    """在运行时解析 map.yaml，提取 resolution 和 origin 给 hybrid_astar_planner。"""
    map_yaml_path = LaunchConfiguration('map').perform(context)
    map_pgm_path = LaunchConfiguration('map_pgm').perform(context)
    pbstream_path = LaunchConfiguration('pbstream').perform(context)
    use_scan_filter_val = LaunchConfiguration('use_scan_filter').perform(context)

    resolution = 0.05
    origin_x = 0.0
    origin_y = 0.0

    if map_yaml_path and os.path.exists(map_yaml_path):
        try:
            with open(map_yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            resolution = float(data.get('resolution', resolution))
            origin = data.get('origin', [0.0, 0.0, 0.0])
            origin_x = float(origin[0])
            origin_y = float(origin[1])
        except Exception as e:
            print(f"[WARN] Failed to parse {map_yaml_path}: {e}, using defaults")

    # 如果没指定 map_pgm，从 map.yaml 同目录推断
    if not map_pgm_path and map_yaml_path:
        map_dir = os.path.dirname(map_yaml_path)
        with open(map_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        image_rel = data.get('image', 'map.pgm')
        map_pgm_path = os.path.join(map_dir, image_rel)

    pkg_slam = get_package_share_directory('robot_slam')
    pkg_robot = get_package_share_directory('ackermann_robot')
    pkg_hybrid = get_package_share_directory('hybrid_astar_planner')

    use_filter = use_scan_filter_val.lower() == 'true'

    # ====== (可选) Bayesian 动态障碍物扫描滤波器 ======
    nodes = []

    if use_filter:
        scan_filter_node = Node(
            package='scan_filter',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_slam, 'config', 'scan_filter_params.yaml')],
        )
        nodes.append(scan_filter_node)

    # ====== 1) Cartographer 纯定位 → map→odom TF ======
    carto_remappings = [
        ('odom', '/odometry/filtered'),
        ('imu', '/imu/data'),
    ]
    if use_filter:
        carto_remappings.append(('scan', '/scan_filtered'))

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', os.path.join(pkg_slam, 'config'),
            '-configuration_basename', 'my_robot_cartographer_2d_localization.lua',
            '-load_state_filename', pbstream_path,
        ],
        remappings=carto_remappings,
    )
    nodes.append(cartographer_node)

    # ====== 2) Hybrid A* 全局规划器 ======
    hybrid_planner = Node(
        package='hybrid_astar_planner',
        executable='hybrid_astar_planner_node',
        name='hybrid_astar_planner',
        output='screen',
        parameters=[os.path.join(pkg_hybrid, 'config', 'planner_params.yaml'), {
            'use_sim_time': True,
            'map_path': map_pgm_path,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
        }],
    )
    nodes.append(hybrid_planner)

    # ====== 3) cmd_vel_mux (NeuPAN Twist → TwistStamped) ======
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
    nodes.append(mux)

    # ====== 4) RViz 多航点任务管理 ======
    waypoint_mission = Node(
        package='waypoint_mission',
        executable='waypoint_mission_node',
        name='waypoint_mission',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    nodes.append(waypoint_mission)

    # ====== 5) RViz ======
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_robot, 'rviz', 'nav2_default_view.rviz')],
    )
    nodes.append(rviz)

    return nodes


def generate_launch_description():
    # 加载集中配置文件
    pkg_slam = get_package_share_directory('robot_slam')
    config_path = os.path.join(pkg_slam, 'config', 'sim_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    nav_cfg = config.get('navigation', {})
    filter_cfg = config.get('scan_filter', {})

    # 解析路径
    map_default = _resolve_path(nav_cfg.get('map_yaml', ''), pkg_slam)
    pgm_default = _resolve_path(nav_cfg.get('map_pgm', ''), pkg_slam)
    pbstream_default = _resolve_path(nav_cfg.get('pbstream', ''), pkg_slam)
    filter_default = str(filter_cfg.get('enabled', False)).lower()

    map_yaml_arg = DeclareLaunchArgument(
        'map', default_value=map_default,
        description='Path to map.yaml (读 resolution/origin，默认来自 sim_config.yaml)'
    )
    map_pgm_arg = DeclareLaunchArgument(
        'map_pgm', default_value=pgm_default,
        description='Path to map.pgm (不指定则从 map.yaml 推断，默认来自 sim_config.yaml)'
    )
    pbstream_arg = DeclareLaunchArgument(
        'pbstream', default_value=pbstream_default,
        description='Path to .pbstream file (Cartographer 纯定位加载，默认来自 sim_config.yaml)'
    )
    use_scan_filter_arg = DeclareLaunchArgument(
        'use_scan_filter', default_value=filter_default,
        description='是否启用 Bayesian 动态障碍物扫描滤波器 (默认来自 sim_config.yaml)'
    )

    ld = LaunchDescription([
        map_yaml_arg,
        map_pgm_arg,
        pbstream_arg,
        use_scan_filter_arg,
        OpaqueFunction(function=parse_map_yaml),
    ])

    return ld
