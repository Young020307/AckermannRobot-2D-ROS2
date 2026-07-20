from math import tan
from pathlib import Path

import pytest

from neupan_ros2.utils import control_to_yaw_rate


def test_ackermann_control_is_converted_from_steering_angle():
    assert control_to_yaw_rate(1.0, 0.52, 'acker', 0.593) == pytest.approx(
        tan(0.52) / 0.593
    )


def test_ackermann_reverse_control_preserves_vehicle_yaw_rate_sign():
    assert control_to_yaw_rate(
        -0.7907854319, -0.0617321469, 'acker', 0.593
    ) == pytest.approx(0.0824, abs=1e-4)


def test_ackermann_zero_speed_has_zero_yaw_rate():
    assert control_to_yaw_rate(0.0, 0.52, 'acker', 0.593) == 0.0


def test_ackermann_requires_positive_wheelbase():
    with pytest.raises(ValueError, match='wheelbase must be positive'):
        control_to_yaw_rate(1.0, 0.2, 'acker', 0.0)


def test_differential_control_is_already_yaw_rate():
    assert control_to_yaw_rate(-0.5, 0.7, 'diff', None) == 0.7


def test_neupan_node_converts_turn_control_at_twist_boundary():
    source = (
        Path(__file__).parents[1] / 'neupan_ros2' / 'neupan_node.py'
    ).read_text(encoding='utf-8')

    assert 'control_to_yaw_rate(' in source
    assert 'action.angular.z = steer' not in source
