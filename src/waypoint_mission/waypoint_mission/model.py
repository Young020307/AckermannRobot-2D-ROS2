"""ROS-independent state model for manual-release waypoint missions."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Any, Deque, List, Optional


class MissionState(str, Enum):
    """Stable mission states published by the ROS adapter."""

    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    NAVIGATING = 'NAVIGATING'
    WAITING = 'WAITING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'


class MissionError(RuntimeError):
    """Raised when a mission command is invalid in the current state."""


@dataclass(frozen=True)
class Pose2D:
    """Planar pose used by the mission model."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class QueuedGoal:
    """A clicked goal waiting for its preview plan."""

    click_id: int
    pose: Pose2D


@dataclass(frozen=True)
class PlanningRequest:
    """A single segment request with a stable click identifier."""

    click_id: int
    start: Pose2D
    goal: Pose2D


class MissionModel:
    """Own waypoint queueing and execution state without ROS dependencies."""

    def __init__(self) -> None:
        self.state = MissionState.IDLE
        self.pending: Deque[QueuedGoal] = deque()
        self.active_request: Optional[PlanningRequest] = None
        self.waypoints: List[Pose2D] = []
        self.segments: List[Any] = []
        self.failed_click_ids: List[int] = []
        self.current_segment_index: Optional[int] = None
        self.arrival_armed = False
        self.execution_locked = False
        self._next_click_id = 1

    @property
    def pending_count(self) -> int:
        """Return queued plus currently planning click count."""
        return len(self.pending) + int(self.active_request is not None)

    def enqueue_goal(self, pose: Pose2D) -> int:
        """Queue a goal and return its monotonic click identifier."""
        if self.execution_locked:
            raise MissionError('mission execution has started')
        click_id = self._next_click_id
        self._next_click_id += 1
        self.pending.append(QueuedGoal(click_id, pose))
        return click_id

    def next_planning_request(
        self,
        robot_pose: Pose2D,
    ) -> Optional[PlanningRequest]:
        """Begin the next queued preview request when none is active."""
        if self.active_request is not None or not self.pending:
            return None
        queued = self.pending.popleft()
        start = self.waypoints[-1] if self.waypoints else robot_pose
        request = PlanningRequest(queued.click_id, start, queued.pose)
        self.active_request = request
        self.state = MissionState.PLANNING
        return request

    def finish_planning(self, path: Any) -> PlanningRequest:
        """Finish the active preview, accepting only a non-None path."""
        if self.active_request is None:
            raise MissionError('no planning request is active')

        request = self.active_request
        self.active_request = None
        if path is None:
            self.failed_click_ids.append(request.click_id)
            self.state = (
                MissionState.PLANNING if self.pending else MissionState.FAILED
            )
        else:
            self.waypoints.append(request.goal)
            self.segments.append(path)
            self.state = (
                MissionState.PLANNING if self.pending else MissionState.IDLE
            )
        return request

    def _validate_ready_for_execution(self) -> None:
        if self.execution_locked:
            raise MissionError('mission execution has started')
        if self.pending:
            raise MissionError(
                f'{len(self.pending)} pending click requests remain'
            )
        if self.active_request is not None:
            raise MissionError('a click is still planning')
        if not self.segments:
            raise MissionError('no valid waypoint is available')

    def begin_execution(self) -> Any:
        """Freeze editing and return the first executable segment."""
        self._validate_ready_for_execution()
        self.execution_locked = True
        self.current_segment_index = 0
        self.arrival_armed = False
        self.state = MissionState.NAVIGATING
        return self.segments[0]

    def begin_first_segment_replan(self) -> None:
        """Freeze editing while refreshing a stale first segment."""
        self._validate_ready_for_execution()
        self.execution_locked = True
        self.state = MissionState.PLANNING

    def finish_first_segment_replan(self, path: Any) -> Optional[Any]:
        """Complete first-segment refresh and start, or unlock on failure."""
        if not self.execution_locked or self.state is not MissionState.PLANNING:
            raise MissionError('first-segment replan is not active')
        if path is None:
            self.execution_locked = False
            self.state = MissionState.FAILED
            return None

        self.segments[0] = path
        self.current_segment_index = 0
        self.arrival_armed = False
        self.state = MissionState.NAVIGATING
        return path

    def observe_arrival(self, arrived: bool) -> bool:
        """Consume arrival state and report a completed-segment transition."""
        if self.state is not MissionState.NAVIGATING:
            return False
        if not arrived:
            self.arrival_armed = True
            return False
        if not self.arrival_armed:
            return False

        if self.current_segment_index == len(self.segments) - 1:
            self.state = MissionState.COMPLETED
        else:
            self.state = MissionState.WAITING
        return True

    def continue_execution(self) -> Any:
        """Advance from a waiting waypoint and return the next segment."""
        if self.state is not MissionState.WAITING:
            raise MissionError(
                f'continue is invalid in {self.state.value}'
            )
        assert self.current_segment_index is not None
        self.current_segment_index += 1
        self.arrival_armed = False
        self.state = MissionState.NAVIGATING
        return self.segments[self.current_segment_index]

    def clear(self) -> None:
        """Reset an editable, waiting, failed, or completed mission."""
        if self.state in (MissionState.PLANNING, MissionState.NAVIGATING):
            raise MissionError(f'clear is invalid in {self.state.value}')
        self.__init__()
