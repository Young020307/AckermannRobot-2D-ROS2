from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def read(relative_path):
    return (ROOT / relative_path).read_text(encoding='utf-8')


def test_navigation_launch_starts_waypoint_mission_node():
    launch = read('src/robot_slam/launch/navigation_carto.launch.py')

    assert "package='waypoint_mission'" in launch
    assert "executable='waypoint_mission_node'" in launch
    assert "name='waypoint_mission'" in launch
    assert "parameters=[{'use_sim_time': True}]" in launch


def test_robot_slam_declares_waypoint_mission_runtime_dependency():
    package_xml = read('src/robot_slam/package.xml')

    assert '<exec_depend>waypoint_mission</exec_depend>' in package_xml


def test_rviz_has_live_preview_and_marker_displays():
    rviz = read('src/ackermann_robot/rviz/nav2_default_view.rviz')

    assert 'Name: Waypoint Preview' in rviz
    assert 'Value: /waypoint_mission/preview_path' in rviz
    assert 'Name: Waypoint Mission' in rviz
    assert 'Value: /waypoint_mission/markers' in rviz
    assert 'Class: rviz_default_plugins/MarkerArray' in rviz


def test_rviz_goal_tool_defaults_to_waypoint_mission_input():
    rviz = read('src/ackermann_robot/rviz/nav2_default_view.rviz')
    set_goal = rviz.split(
        '- Class: rviz_default_plugins/SetGoal', maxsplit=1
    )[1].split('Transformation:', maxsplit=1)[0]

    assert 'Value: /waypoint_mission/goal' in set_goal
