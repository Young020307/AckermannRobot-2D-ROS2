from math import atan, tan
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]


def load_yaml(relative_path):
    with (ROOT / relative_path).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def steering_joint_limit():
    xacro = ET.parse(ROOT / 'src/ackermann_robot/xacro/chassis.xacro')
    steering_joint = next(
        joint for joint in xacro.getroot().iter('joint')
        if joint.get('name') == '${prefix}_steering_joint'
    )
    limit = steering_joint.find('limit')
    return float(limit.get('lower')), float(limit.get('upper'))


def test_planner_curvature_respects_physical_wheel_limit():
    runtime = load_yaml(
        'src/neupan_ros2/config/robots/ackermann_robot/planner.yaml'
    )
    training = load_yaml('src/neupan_ros2/train/train_config.yaml')
    controller = load_yaml(
        'src/ackermann_robot/config/ackermann_controllers.yaml'
    )['ackermann_steering_controller']['ros__parameters']
    hybrid = load_yaml(
        'src/hybrid_astar_planner/standalone_planner/config/planner_params.yaml'
    )['/**']['ros__parameters']

    lower, physical_wheel_limit = steering_joint_limit()
    wheelbase = runtime['robot']['wheelbase']
    track = controller['front_wheel_track']
    equivalent_center = atan(
        wheelbase
        / (wheelbase / tan(physical_wheel_limit) + track / 2.0)
    )
    runtime_center_limit = runtime['robot']['max_speed'][1]
    runtime_min_radius = runtime['ipath']['min_radius']

    assert lower == pytest.approx(-0.52)
    assert physical_wheel_limit == pytest.approx(0.52)
    assert runtime_center_limit == pytest.approx(equivalent_center, abs=0.005)
    assert runtime['robot']['min_speed'][1] == pytest.approx(
        -runtime_center_limit
    )
    assert training['robot']['max_speed'][1] == runtime_center_limit
    assert training['robot']['min_speed'][1] == pytest.approx(
        -runtime_center_limit
    )
    assert runtime_min_radius == pytest.approx(1.30)
    assert training['ipath']['min_radius'] == runtime_min_radius
    assert hybrid['minimum_turning_radius'] == runtime_min_radius
    assert wheelbase / tan(runtime_center_limit) == pytest.approx(
        runtime_min_radius, abs=0.02
    )


def test_ros2_control_limits_match_physical_wheel_limit():
    lower, upper = steering_joint_limit()
    control = ET.parse(ROOT / 'src/ackermann_robot/xacro/control.xacro')
    steering_joints = [
        joint for joint in control.getroot().iter('joint')
        if joint.get('name') in {
            'left_steering_joint',
            'right_steering_joint',
        }
    ]

    assert len(steering_joints) == 2
    for joint in steering_joints:
        command = joint.find("command_interface[@name='position']")
        parameters = {
            parameter.get('name'): float(parameter.text)
            for parameter in command.findall('param')
        }
        assert parameters == {'min': lower, 'max': upper}
