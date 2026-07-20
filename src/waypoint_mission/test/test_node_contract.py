import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.task import Future
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from waypoint_mission.model import MissionState, Pose2D
from waypoint_mission.waypoint_mission_node import WaypointMissionNode


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class ControlledPlanClient:
    def __init__(self):
        self.requests = []
        self.futures = []

    def call_async(self, request):
        future = Future()
        self.requests.append(request)
        self.futures.append(future)
        return future

    def service_is_ready(self):
        return True


def make_pose(x, y, yaw=0.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def make_path(*points):
    path = Path()
    path.header.frame_id = 'map'
    path.poses = [make_pose(x, y) for x, y in points]
    return path


def resolve_plan(client, path):
    response = GetPlan.Response()
    response.plan = path
    client.futures[-1].set_result(response)


@pytest.fixture
def node():
    rclpy.init()
    mission = WaypointMissionNode()
    yield mission
    mission.destroy_node()
    rclpy.shutdown()


def install_fakes(node, robot_pose=Pose2D(0.0, 0.0, 0.0)):
    client = ControlledPlanClient()
    execution = RecordingPublisher()
    preview = RecordingPublisher()
    markers = RecordingPublisher()
    status = RecordingPublisher()
    node.plan_client = client
    node.execution_path_pub = execution
    node.preview_path_pub = preview
    node.marker_pub = markers
    node.status_pub = status
    node._get_robot_pose = lambda: robot_pose
    node._transform_goal = lambda message: message
    return client, execution, preview, markers, status


def test_node_declares_exact_ros_contract(node):
    assert node.goal_sub.topic_name == '/waypoint_mission/goal'
    assert node.arrival_sub.topic_name == '/neupan_arrived'
    assert node.execution_path_pub.topic_name == '/plan'
    assert node.preview_path_pub.topic_name == '/waypoint_mission/preview_path'
    assert node.marker_pub.topic_name == '/waypoint_mission/markers'
    assert node.status_pub.topic_name == '/waypoint_mission/status'
    assert node.plan_client.srv_name == '/plan'
    assert node.plan_service.srv_name == '/waypoint_mission/plan'
    assert node.continue_service.srv_name == '/waypoint_mission/continue'
    assert node.clear_service.srv_name == '/waypoint_mission/clear'
    assert node.get_parameter('first_segment_replan_distance').value == 0.20
    assert node.get_parameter('first_segment_replan_yaw').value == 0.17
    assert node.latched_qos.durability.name == 'TRANSIENT_LOCAL'
    assert node.latched_qos.reliability.name == 'RELIABLE'


def test_clicks_plan_fifo_and_publish_preview_without_executing(node):
    client, execution, preview, _, _ = install_fakes(node)

    node.goal_callback(make_pose(1.0, 0.0))
    node.goal_callback(make_pose(2.0, 0.0))

    assert len(client.requests) == 1
    assert client.requests[0].start.pose.position.x == 0.0
    assert client.requests[0].goal.pose.position.x == 1.0
    assert execution.messages == []

    resolve_plan(client, make_path((0.0, 0.0), (1.0, 0.0)))
    assert len(client.requests) == 2
    assert client.requests[1].start.pose.position.x == 1.0
    assert client.requests[1].goal.pose.position.x == 2.0

    resolve_plan(client, make_path((1.0, 0.0), (2.0, 0.0)))
    assert node.model.state is MissionState.IDLE
    assert len(preview.messages[-1].poses) == 3
    assert execution.messages == []


def test_failed_click_is_marked_and_next_click_uses_last_valid_anchor(node):
    client, _, _, _, _ = install_fakes(node)
    node.goal_callback(make_pose(1.0, 0.0))
    node.goal_callback(make_pose(9.0, 0.0))
    node.goal_callback(make_pose(2.0, 0.0))

    resolve_plan(client, make_path((0.0, 0.0), (1.0, 0.0)))
    resolve_plan(client, Path())

    assert client.requests[-1].start.pose.position.x == 1.0
    assert client.requests[-1].goal.pose.position.x == 2.0
    resolve_plan(client, make_path((1.0, 0.0), (2.0, 0.0)))
    assert node.model.failed_click_ids == [2]
    assert [pose.x for pose in node.model.waypoints] == [1.0, 2.0]


def test_plan_arrival_and_continue_publish_one_segment_at_a_time(node):
    client, execution, _, _, _ = install_fakes(node)
    node.goal_callback(make_pose(1.0, 0.0))
    resolve_plan(client, make_path((0.0, 0.0), (1.0, 0.0)))
    node.goal_callback(make_pose(2.0, 0.0))
    resolve_plan(client, make_path((1.0, 0.0), (2.0, 0.0)))

    response = node.plan_callback(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert len(execution.messages) == 1
    assert execution.messages[-1].poses[-1].pose.position.x == 1.0

    node.arrival_callback(Bool(data=True))
    assert node.model.state is MissionState.NAVIGATING
    node.arrival_callback(Bool(data=False))
    node.arrival_callback(Bool(data=True))
    assert node.model.state is MissionState.WAITING

    response = node.continue_callback(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert len(execution.messages) == 2
    assert execution.messages[-1].poses[-1].pose.position.x == 2.0


def test_plan_replans_first_segment_when_robot_moved(node):
    client, execution, preview, _, _ = install_fakes(node)
    node.goal_callback(make_pose(1.0, 0.0))
    resolve_plan(client, make_path((0.0, 0.0), (1.0, 0.0)))
    node._get_robot_pose = lambda: Pose2D(0.5, 0.0, 0.0)

    response = node.plan_callback(Trigger.Request(), Trigger.Response())
    assert response.success is True
    assert 'replan' in response.message.lower()
    assert execution.messages == []
    assert client.requests[-1].start.pose.position.x == 0.5

    replacement = make_path((0.5, 0.0), (1.0, 0.0))
    resolve_plan(client, replacement)
    assert execution.messages[-1] is replacement
    assert preview.messages[-1].poses[0].pose.position.x == 0.5
