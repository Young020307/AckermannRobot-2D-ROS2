#!/usr/bin/env python

"""
Utility functions for NeuPAN ROS2 package.

Contains coordinate transformation and mathematical utilities.

Developer: Li Chengyang <kevinladlee@gmail.com>
Date: 2025.11.15
"""

from math import sin, cos, atan2
from typing import Optional

from geometry_msgs.msg import Quaternion


class ArrivalReporter:
    """Track and report changes to a segment's arrival state."""

    def __init__(self) -> None:
        self._last: Optional[bool] = None

    def reset(self) -> bool:
        """Start a new segment and force a non-arrived report."""
        self._last = False
        return False

    def update(self, value: bool) -> Optional[bool]:
        """Return a changed value, or None when it is unchanged."""
        value = bool(value)
        if value == self._last:
            return None
        self._last = value
        return value


def yaw_to_quat(yaw: float) -> Quaternion:
    """Convert yaw angle to quaternion representation.

    Args:
        yaw: Yaw angle in radians

    Returns:
        Quaternion message representing the rotation

    """
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = sin(yaw / 2)
    quaternion.w = cos(yaw / 2)
    return quaternion


def quat_to_yaw(quaternion: Quaternion) -> float:
    """Extract yaw angle from quaternion representation.

    Args:
        quaternion: Quaternion message

    Returns:
        Yaw angle in radians

    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return yaw
