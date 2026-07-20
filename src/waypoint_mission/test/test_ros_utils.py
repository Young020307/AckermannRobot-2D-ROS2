import math

import pytest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from waypoint_mission.model import Pose2D
from waypoint_mission.ros_utils import (
    CURRENT_COLOR,
    FAILED_COLOR,
    PENDING_COLOR,
    VALID_COLOR,
    build_markers,
    delete_all_markers,
    model_to_pose_stamped,
    pose_delta,
    pose_to_model,
    stitch_paths,
)


def make_path(points):
    path = Path()
    path.header.frame_id = 'map'
    for x, y in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


def rgba(marker):
    return (
        marker.color.r,
        marker.color.g,
        marker.color.b,
        marker.color.a,
    )


def find_marker(markers, namespace, marker_id):
    return next(
        marker for marker in markers.markers
        if marker.ns == namespace and marker.id == marker_id
    )


def test_stitch_paths_drops_duplicate_join_without_mutating_inputs():
    first = make_path([(0.0, 0.0), (1.0, 0.0)])
    second = make_path([(1.0, 0.0), (2.0, 1.0)])
    header = Header(frame_id='map', stamp=Time(sec=7))

    stitched = stitch_paths([first, second], header)

    assert [
        (pose.pose.position.x, pose.pose.position.y)
        for pose in stitched.poses
    ] == [(0.0, 0.0), (1.0, 0.0), (2.0, 1.0)]
    assert len(first.poses) == 2
    assert len(second.poses) == 2
    assert stitched.header.frame_id == 'map'
    assert stitched.header.stamp.sec == 7


def test_stitch_paths_keeps_nonmatching_segment_start_and_handles_empty():
    first = make_path([(0.0, 0.0), (1.0, 0.0)])
    second = make_path([(1.01, 0.0), (2.0, 0.0)])
    header = Header(frame_id='map')

    stitched = stitch_paths([Path(), first, second], header)

    assert len(stitched.poses) == 4
    assert stitch_paths([], header).poses == []


def test_pose_conversion_and_delta_normalize_yaw():
    source = Pose2D(1.0, 2.0, math.pi - 0.1)
    stamp = Time(sec=3)
    message = model_to_pose_stamped(source, 'map', stamp)

    restored = pose_to_model(message)
    distance, yaw = pose_delta(source, Pose2D(1.2, 2.0, -math.pi + 0.1))

    assert restored.x == 1.0
    assert restored.y == 2.0
    assert restored.yaw == pytest.approx(source.yaw)
    assert distance == pytest.approx(0.2)
    assert yaw == pytest.approx(0.2)
    assert message.header.frame_id == 'map'
    assert message.header.stamp.sec == 3


def test_markers_show_pending_valid_failed_and_current_semantics():
    markers = build_markers(
        pending=[(3, Pose2D(3.0, 0.0, 0.0))],
        valid=[Pose2D(1.0, 0.0, 0.0), Pose2D(2.0, 0.0, 0.0)],
        failed=[(2, Pose2D(9.0, 0.0, 0.0))],
        current_index=1,
        frame_id='map',
        stamp=Time(sec=5),
    )

    assert find_marker(markers, 'clear', 0).action == Marker.DELETEALL
    assert rgba(find_marker(markers, 'pending', 3)) == pytest.approx(
        PENDING_COLOR
    )
    assert rgba(find_marker(markers, 'valid', 0)) == pytest.approx(VALID_COLOR)
    current = find_marker(markers, 'valid', 1)
    assert rgba(current) == pytest.approx(CURRENT_COLOR)
    assert current.scale.x == pytest.approx(0.45)
    assert rgba(find_marker(markers, 'failed', 2)) == pytest.approx(
        FAILED_COLOR
    )
    assert find_marker(markers, 'valid_text', 0).text == '1'
    assert find_marker(markers, 'valid_text', 1).text == '2'
    assert find_marker(markers, 'failed_text', 2).text == 'X2'


def test_delete_all_markers_contains_only_deleteall():
    markers = delete_all_markers('map', Time(sec=8))

    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL
    assert markers.markers[0].header.frame_id == 'map'
