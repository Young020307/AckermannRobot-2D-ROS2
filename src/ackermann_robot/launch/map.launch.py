import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    package_name = 'ackermann_robot'
    pkg_share = get_package_share_directory(package_name)

    # 加载集中配置文件
    slam_pkg_share = get_package_share_directory('robot_slam')
    config_path = os.path.join(slam_pkg_share, 'config', 'sim_config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 1. 解析 URDF (XACRO)
    xacro_file = os.path.join(pkg_share, 'xacro', 'robot.xacro')
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # ====== 世界文件 (默认值来自 sim_config.yaml，CLI 可覆盖) ======
    world_default = config.get('simulation', {}).get('world', 'mini.world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_default,
        description='Gazebo 世界文件名 (位于 gazebo_worlds/worlds/ 目录下，编辑 sim_config.yaml 可改默认值)'
    )
    world_file_path = PathJoinSubstitution([
        FindPackageShare('gazebo_worlds'), 'worlds', LaunchConfiguration('world')
    ])

    # 设置 GAZEBO_MODEL_PATH 环境变量
    pkg_share_env = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_env
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share_env

    # 2. 启动 Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_content
        }]
    )
    
    # 3. 启动 Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file_path,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # 4. 在 Gazebo 中生成机器人模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ackermann_robot',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '0.0', 
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # ================= NEW: 加载 ROS 2 Controllers =================
    
    # 加载关节状态广播器 (负责发布 /joint_states)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # 加载阿克曼控制器 (负责底盘运动)
    load_ackermann_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # ================= EKF 节点 =================
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered')] 
    )

    # ================= 返回 Launch Description =================
    return LaunchDescription([
        world_arg,
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        ekf_node,

        # 使用事件处理器确保控制器在模型生成后启动
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller],
            )
        ),
    ])