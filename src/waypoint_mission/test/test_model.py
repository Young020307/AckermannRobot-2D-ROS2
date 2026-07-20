import pytest

from waypoint_mission.model import (
    MissionError,
    MissionModel,
    MissionState,
    Pose2D,
)


ROBOT = Pose2D(0.0, 0.0, 0.0)
FIRST = Pose2D(1.0, 0.0, 0.0)
SECOND = Pose2D(2.0, 0.0, 0.0)
THIRD = Pose2D(3.0, 1.0, 0.5)


def finish_next(model, path, robot=ROBOT):
    request = model.next_planning_request(robot)
    assert request is not None
    model.finish_planning(path)
    return request


def model_with_two_segments():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    finish_next(model, 'segment-1')
    model.enqueue_goal(SECOND)
    finish_next(model, 'segment-2')
    return model


def test_click_ids_and_planning_requests_are_fifo():
    model = MissionModel()

    assert model.enqueue_goal(FIRST) == 1
    assert model.enqueue_goal(SECOND) == 2

    first_request = model.next_planning_request(ROBOT)
    assert first_request.click_id == 1
    assert first_request.start == ROBOT
    assert first_request.goal == FIRST
    model.finish_planning('segment-1')

    second_request = model.next_planning_request(ROBOT)
    assert second_request.click_id == 2
    assert second_request.start == FIRST
    assert second_request.goal == SECOND


def test_failed_click_does_not_become_next_anchor():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    model.enqueue_goal(SECOND)
    model.enqueue_goal(THIRD)

    finish_next(model, 'segment-1')
    blocked_request = finish_next(model, None)
    next_request = model.next_planning_request(ROBOT)

    assert blocked_request.goal == SECOND
    assert next_request.start == FIRST
    assert next_request.goal == THIRD
    model.finish_planning('segment-3')
    assert model.waypoints == [FIRST, THIRD]
    assert model.segments == ['segment-1', 'segment-3']
    assert model.failed_click_ids == [2]
    assert model.state is MissionState.IDLE


def test_failure_without_more_clicks_enters_failed_and_can_recover():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    finish_next(model, None)
    assert model.state is MissionState.FAILED

    model.enqueue_goal(SECOND)
    request = finish_next(model, 'recovered')
    assert request.start == ROBOT
    assert model.state is MissionState.IDLE


def test_cannot_start_empty_pending_or_active_mission():
    model = MissionModel()
    with pytest.raises(MissionError, match='no valid waypoint'):
        model.begin_execution()

    model.enqueue_goal(FIRST)
    with pytest.raises(MissionError, match='pending'):
        model.begin_execution()

    model.next_planning_request(ROBOT)
    with pytest.raises(MissionError, match='still planning'):
        model.begin_execution()


def test_arrival_requires_false_then_true_for_each_segment():
    model = model_with_two_segments()

    assert model.begin_execution() == 'segment-1'
    assert model.state is MissionState.NAVIGATING
    assert model.observe_arrival(True) is False
    assert model.observe_arrival(False) is False
    assert model.observe_arrival(True) is True
    assert model.state is MissionState.WAITING

    assert model.continue_execution() == 'segment-2'
    assert model.observe_arrival(True) is False
    assert model.observe_arrival(False) is False
    assert model.observe_arrival(True) is True
    assert model.state is MissionState.COMPLETED


def test_continue_is_rejected_before_arrival_and_after_completion():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    finish_next(model, 'segment-1')
    model.begin_execution()

    with pytest.raises(MissionError, match='NAVIGATING'):
        model.continue_execution()

    model.observe_arrival(False)
    model.observe_arrival(True)
    with pytest.raises(MissionError, match='COMPLETED'):
        model.continue_execution()


def test_first_segment_replan_success_starts_and_failure_unlocks():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    finish_next(model, 'old-segment')

    model.begin_first_segment_replan()
    assert model.state is MissionState.PLANNING
    with pytest.raises(MissionError, match='execution has started'):
        model.enqueue_goal(SECOND)
    assert model.finish_first_segment_replan('new-segment') == 'new-segment'
    assert model.state is MissionState.NAVIGATING
    assert model.segments[0] == 'new-segment'

    failed = MissionModel()
    failed.enqueue_goal(FIRST)
    finish_next(failed, 'old-segment')
    failed.begin_first_segment_replan()
    assert failed.finish_first_segment_replan(None) is None
    assert failed.state is MissionState.FAILED
    assert failed.enqueue_goal(SECOND) == 2


def test_clear_obeys_state_rules_and_resets_click_ids():
    model = MissionModel()
    model.enqueue_goal(FIRST)
    finish_next(model, 'segment-1')
    model.clear()
    assert model.state is MissionState.IDLE
    assert model.waypoints == []
    assert model.enqueue_goal(FIRST) == 1

    navigating = MissionModel()
    navigating.enqueue_goal(FIRST)
    finish_next(navigating, 'segment-1')
    navigating.begin_execution()
    with pytest.raises(MissionError, match='NAVIGATING'):
        navigating.clear()


def test_finish_planning_requires_an_active_request():
    with pytest.raises(MissionError, match='no planning request'):
        MissionModel().finish_planning('unexpected')
