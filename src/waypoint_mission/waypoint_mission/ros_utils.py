"""ROS message helpers for waypoint mission previews and markers."""

import copy
import math
from typing import List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_mission.model import Pose2D


Color = Tuple[float, float, float, float]
IdentifiedPose = Tuple[int, Pose2D]

PENDING_COLOR: Color = (1.0, 0.55, 0.0, 0.95)
VALID_COLOR: Color = (0.1, 0.8, 0.1, 0.95)
FAILED_COLOR: Color = (0.9, 0.1, 0.1, 0.95)
CURRENT_COLOR: Color = (1.0, 0.9, 0.0, 1.0)
TEXT_COLOR: Color = (1.0, 1.0, 1.0, 1.0)


def pose_to_model(message: PoseStamped) -> Pose2D:
    """Convert a ROS PoseStamped into a planar mission pose."""
    orientation = message.pose.orientation
    sin_yaw = 2.0 * (
        orientation.w * orientation.z
        + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y
        + orientation.z * orientation.z
    )
    return Pose2D(
        message.pose.position.x,
        message.pose.position.y,
        math.atan2(sin_yaw, cos_yaw),
    )


def model_to_pose_stamped(
    pose: Pose2D,
    frame_id: str,
    stamp: Time,
) -> PoseStamped:
    """Convert a planar mission pose into a ROS PoseStamped."""
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = copy.deepcopy(stamp)
    message.pose.position.x = pose.x
    message.pose.position.y = pose.y
    message.pose.orientation.z = math.sin(pose.yaw / 2.0)
    message.pose.orientation.w = math.cos(pose.yaw / 2.0)
    return message


def pose_delta(first: Pose2D, second: Pose2D) -> Tuple[float, float]:
    """Return planar distance and normalized absolute yaw difference."""
    distance = math.hypot(second.x - first.x, second.y - first.y)
    delta = second.yaw - first.yaw
    yaw = abs(math.atan2(math.sin(delta), math.cos(delta)))
    return distance, yaw


def stitch_paths(paths: Sequence[Path], header: Header) -> Path:
    """Copy and join path segments while removing duplicate boundaries."""
    result = Path()
    result.header = copy.deepcopy(header)

    for path in paths:
        poses = path.poses
        start_index = 0
        if result.poses and poses:
            previous = result.poses[-1].pose.position
            current = poses[0].pose.position
            if math.hypot(previous.x - current.x, previous.y - current.y) <= 1e-6:
                start_index = 1
        result.poses.extend(copy.deepcopy(poses[start_index:]))
    return result


def _set_color(marker: Marker, color: Color) -> None:
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color


def _sphere_marker(
    namespace: str,
    marker_id: int,
    pose: Pose2D,
    color: Color,
    scale: float,
    frame_id: str,
    stamp: Time,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = copy.deepcopy(stamp)
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.15
    marker.pose.orientation.z = math.sin(pose.yaw / 2.0)
    marker.pose.orientation.w = math.cos(pose.yaw / 2.0)
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    _set_color(marker, color)
    return marker


def _text_marker(
    namespace: str,
    marker_id: int,
    pose: Pose2D,
    text: str,
    frame_id: str,
    stamp: Time,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = copy.deepcopy(stamp)
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.55
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.28
    marker.text = text
    _set_color(marker, TEXT_COLOR)
    return marker


def delete_all_markers(frame_id: str, stamp: Time) -> MarkerArray:
    """Build a MarkerArray that clears every mission marker."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = copy.deepcopy(stamp)
    marker.ns = 'clear'
    marker.id = 0
    marker.action = Marker.DELETEALL
    return MarkerArray(markers=[marker])


def build_markers(
    pending: Sequence[IdentifiedPose],
    valid: Sequence[Pose2D],
    failed: Sequence[IdentifiedPose],
    current_index: Optional[int],
    frame_id: str,
    stamp: Time,
) -> MarkerArray:
    """Build a complete, self-clearing waypoint visualization snapshot."""
    markers: List[Marker] = delete_all_markers(frame_id, stamp).markers

    for click_id, pose in pending:
        markers.append(
            _sphere_marker(
                'pending', click_id, pose, PENDING_COLOR, 0.30,
                frame_id, stamp,
            )
        )

    for index, pose in enumerate(valid):
        is_current = current_index == index
        markers.append(
            _sphere_marker(
                'valid', index, pose,
                CURRENT_COLOR if is_current else VALID_COLOR,
                0.45 if is_current else 0.30,
                frame_id, stamp,
            )
        )
        markers.append(
            _text_marker(
                'valid_text', index, pose, str(index + 1),
                frame_id, stamp,
            )
        )

    for click_id, pose in failed:
        markers.append(
            _sphere_marker(
                'failed', click_id, pose, FAILED_COLOR, 0.30,
                frame_id, stamp,
            )
        )
        markers.append(
            _text_marker(
                'failed_text', click_id, pose, f'X{click_id}',
                frame_id, stamp,
            )
        )
    return MarkerArray(markers=markers)
