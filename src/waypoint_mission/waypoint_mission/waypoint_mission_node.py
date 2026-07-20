"""ROS 2 adapter for live-preview manual-release waypoint missions."""

import copy
import threading
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros
from visualization_msgs.msg import MarkerArray

from waypoint_mission.model import (
    MissionError,
    MissionModel,
    MissionState,
    PlanningRequest,
    Pose2D,
)
from waypoint_mission.ros_utils import (
    build_markers,
    delete_all_markers,
    model_to_pose_stamped,
    pose_delta,
    pose_to_model,
    stitch_paths,
)


class WaypointMissionNode(Node):
    """Plan clicked waypoints and execute them with manual release."""

    def __init__(self) -> None:
        super().__init__('waypoint_mission')
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self.model = MissionModel()

        defaults = {
            'global_frame': 'map',
            'base_frame': 'base_link',
            'goal_topic': '/waypoint_mission/goal',
            'planner_service': '/plan',
            'execution_path_topic': '/plan',
            'preview_path_topic': '/waypoint_mission/preview_path',
            'marker_topic': '/waypoint_mission/markers',
            'status_topic': '/waypoint_mission/status',
            'arrival_topic': '/neupan_arrived',
            'first_segment_replan_distance': 0.20,
            'first_segment_replan_yaw': 0.17,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.replan_distance = self.get_parameter(
            'first_segment_replan_distance'
        ).value
        self.replan_yaw = self.get_parameter(
            'first_segment_replan_yaw'
        ).value

        self.latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.execution_path_pub = self.create_publisher(
            Path,
            self.get_parameter('execution_path_topic').value,
            10,
        )
        self.preview_path_pub = self.create_publisher(
            Path,
            self.get_parameter('preview_path_topic').value,
            self.latched_qos,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.get_parameter('marker_topic').value,
            self.latched_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter('status_topic').value,
            self.latched_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('goal_topic').value,
            self.goal_callback,
            10,
            callback_group=self.callback_group,
        )
        self.arrival_sub = self.create_subscription(
            Bool,
            self.get_parameter('arrival_topic').value,
            self.arrival_callback,
            self.latched_qos,
            callback_group=self.callback_group,
        )
        self.plan_client = self.create_client(
            GetPlan,
            self.get_parameter('planner_service').value,
            callback_group=self.callback_group,
        )
        self.plan_service = self.create_service(
            Trigger,
            '/waypoint_mission/plan',
            self.plan_callback,
            callback_group=self.callback_group,
        )
        self.continue_service = self.create_service(
            Trigger,
            '/waypoint_mission/continue',
            self.continue_callback,
            callback_group=self.callback_group,
        )
        self.clear_service = self.create_service(
            Trigger,
            '/waypoint_mission/clear',
            self.clear_callback,
            callback_group=self.callback_group,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.clicked_poses: Dict[int, Pose2D] = {}
        self.failed_poses: Dict[int, Pose2D] = {}
        self.first_segment_start: Optional[Pose2D] = None
        self._preview_in_flight = False
        self._preview_future = None
        self._start_replan_future = None

        self._publish_status()
        self._publish_markers()

    def _now(self):
        return self.get_clock().now().to_msg()

    def _transform_goal(self, goal: PoseStamped) -> PoseStamped:
        if not goal.header.frame_id or goal.header.frame_id == self.global_frame:
            transformed = copy.deepcopy(goal)
            transformed.header.frame_id = self.global_frame
            return transformed
        return self.tf_buffer.transform(
            goal,
            self.global_frame,
            timeout=Duration(seconds=1.0),
        )

    def _get_robot_pose(self) -> Pose2D:
        transform = self.tf_buffer.lookup_transform(
            self.global_frame,
            self.base_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.5),
        )
        pose = PoseStamped()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.orientation = transform.transform.rotation
        return pose_to_model(pose)

    def _request_message(self, request: PlanningRequest) -> GetPlan.Request:
        message = GetPlan.Request()
        stamp = self._now()
        message.start = model_to_pose_stamped(
            request.start, self.global_frame, stamp
        )
        message.goal = model_to_pose_stamped(
            request.goal, self.global_frame, stamp
        )
        message.tolerance = 0.0
        return message

    def goal_callback(self, goal: PoseStamped) -> None:
        """Queue a transformed RViz click and start its preview plan."""
        try:
            transformed = self._transform_goal(goal)
            pose = pose_to_model(transformed)
            with self._lock:
                click_id = self.model.enqueue_goal(pose)
                self.clicked_poses[click_id] = pose
        except (MissionError, Exception) as error:
            self.get_logger().warning(f'Waypoint rejected: {error}')
            return

        self._publish_markers()
        self._start_next_preview()

    def _start_next_preview(self) -> None:
        with self._lock:
            if self._preview_in_flight or not self.model.pending:
                return

        try:
            robot_pose = self._get_robot_pose()
        except Exception as error:
            self.get_logger().warning(f'Cannot get robot pose: {error}')
            return

        with self._lock:
            if self._preview_in_flight:
                return
            request = self.model.next_planning_request(robot_pose)
            if request is None:
                return
            self._preview_in_flight = True

        self._publish_status()
        self._publish_markers()
        if not self.plan_client.service_is_ready():
            self._finish_preview_without_path('Hybrid A* /plan is unavailable')
            return

        try:
            future = self.plan_client.call_async(
                self._request_message(request)
            )
            self._preview_future = future
            future.add_done_callback(self._preview_done)
        except Exception as error:
            self._finish_preview_without_path(str(error))

    def _finish_preview_without_path(self, reason: str) -> None:
        with self._lock:
            request = self.model.finish_planning(None)
            self.failed_poses[request.click_id] = request.goal
            self._preview_in_flight = False
        self.get_logger().warning(
            f'Waypoint click #{request.click_id} failed: {reason}'
        )
        self._publish_status()
        self._publish_markers()
        self._start_next_preview()

    def _preview_done(self, future) -> None:
        try:
            response = future.result()
            path = response.plan if response.plan.poses else None
        except Exception as error:
            self._finish_preview_without_path(str(error))
            return

        with self._lock:
            request = self.model.finish_planning(path)
            if path is None:
                self.failed_poses[request.click_id] = request.goal
            elif len(self.model.segments) == 1:
                self.first_segment_start = request.start
            self._preview_in_flight = False
        if path is None:
            self.get_logger().warning(
                f'Waypoint click #{request.click_id} is unreachable'
            )
        self._publish_preview()
        self._publish_status()
        self._publish_markers()
        self._start_next_preview()

    def _publish_preview(self) -> None:
        with self._lock:
            segments = list(self.model.segments)
        header = segments[0].header if segments else Path().header
        header = copy.deepcopy(header)
        header.frame_id = self.global_frame
        header.stamp = self._now()
        self.preview_path_pub.publish(stitch_paths(segments, header))

    def _publish_status(self) -> None:
        with self._lock:
            state = self.model.state.value
        self.status_pub.publish(String(data=state))

    def _publish_markers(self) -> None:
        with self._lock:
            pending_ids = [queued.click_id for queued in self.model.pending]
            if self.model.active_request is not None:
                pending_ids.insert(0, self.model.active_request.click_id)
            pending = [
                (click_id, self.clicked_poses[click_id])
                for click_id in pending_ids
            ]
            valid = list(self.model.waypoints)
            failed = sorted(self.failed_poses.items())
            current = (
                self.model.current_segment_index
                if self.model.state in (
                    MissionState.NAVIGATING,
                    MissionState.WAITING,
                    MissionState.COMPLETED,
                )
                else None
            )
        self.marker_pub.publish(
            build_markers(
                pending, valid, failed, current,
                self.global_frame, self._now(),
            )
        )

    def plan_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Confirm cached previews and begin the first segment."""
        with self._lock:
            if self.model.pending_count:
                response.success = False
                response.message = (
                    f'{self.model.pending_count} waypoint request(s) remain'
                )
                return response
            if not self.model.segments:
                response.success = False
                response.message = 'No valid waypoint is available'
                return response
            if self.model.execution_locked:
                response.success = False
                response.message = 'Mission execution has already started'
                return response
            first_start = self.first_segment_start

        try:
            current_pose = self._get_robot_pose()
        except Exception as error:
            response.success = False
            response.message = f'Cannot get robot pose: {error}'
            return response

        needs_replan = first_start is None
        if first_start is not None:
            distance, yaw = pose_delta(first_start, current_pose)
            needs_replan = (
                distance > self.replan_distance or yaw > self.replan_yaw
            )

        if needs_replan:
            return self._begin_start_replan(current_pose, response)

        try:
            with self._lock:
                path = self.model.begin_execution()
        except MissionError as error:
            response.success = False
            response.message = str(error)
            return response
        self.execution_path_pub.publish(path)
        self._publish_status()
        self._publish_markers()
        response.success = True
        response.message = 'Mission started'
        return response

    def _begin_start_replan(
        self,
        current_pose: Pose2D,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._lock:
            try:
                self.model.begin_first_segment_replan()
            except MissionError as error:
                response.success = False
                response.message = str(error)
                return response
            goal = self.model.waypoints[0]
        request = PlanningRequest(0, current_pose, goal)

        if not self.plan_client.service_is_ready():
            with self._lock:
                self.model.finish_first_segment_replan(None)
            self._publish_status()
            response.success = False
            response.message = 'Hybrid A* /plan is unavailable'
            return response

        future = self.plan_client.call_async(self._request_message(request))
        self._start_replan_future = future
        future.add_done_callback(
            lambda result: self._start_replan_done(result, current_pose)
        )
        self._publish_status()
        response.success = True
        response.message = 'First segment replan started'
        return response

    def _start_replan_done(self, future, current_pose: Pose2D) -> None:
        try:
            result = future.result()
            path = result.plan if result.plan.poses else None
        except Exception as error:
            path = None
            self.get_logger().warning(f'First segment replan failed: {error}')

        with self._lock:
            executable = self.model.finish_first_segment_replan(path)
            if executable is not None:
                self.first_segment_start = current_pose
        self._publish_preview()
        self._publish_status()
        self._publish_markers()
        if executable is not None:
            self.execution_path_pub.publish(executable)

    def arrival_callback(self, message: Bool) -> None:
        """Advance to WAITING or COMPLETED on an armed arrival."""
        with self._lock:
            changed = self.model.observe_arrival(message.data)
        if changed:
            self._publish_status()
            self._publish_markers()

    def continue_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Release a waiting mission to its next segment."""
        try:
            with self._lock:
                path = self.model.continue_execution()
        except MissionError as error:
            response.success = False
            response.message = str(error)
            return response
        self.execution_path_pub.publish(path)
        self._publish_status()
        self._publish_markers()
        response.success = True
        response.message = 'Next segment started'
        return response

    def clear_callback(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """Clear a non-moving mission and all preview visuals."""
        try:
            with self._lock:
                self.model.clear()
                self.clicked_poses.clear()
                self.failed_poses.clear()
                self.first_segment_start = None
        except MissionError as error:
            response.success = False
            response.message = str(error)
            return response

        stamp = self._now()
        empty_path = Path()
        empty_path.header.frame_id = self.global_frame
        empty_path.header.stamp = stamp
        self.preview_path_pub.publish(empty_path)
        self.marker_pub.publish(delete_all_markers(self.global_frame, stamp))
        self._publish_status()
        response.success = True
        response.message = 'Mission cleared'
        return response


def main(args=None) -> None:
    """Run the waypoint mission node with a multithreaded executor."""
    rclpy.init(args=args)
    node = WaypointMissionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
