from neupan_ros2.utils import ArrivalReporter


def test_reset_always_reports_false_and_true_is_edge_triggered():
    reporter = ArrivalReporter()

    assert reporter.reset() is False
    assert reporter.update(False) is None
    assert reporter.update(True) is True
    assert reporter.update(True) is None
    assert reporter.reset() is False
    assert reporter.update(True) is True


def test_reporter_emits_nonarrival_transition_after_true():
    reporter = ArrivalReporter()
    reporter.reset()
    reporter.update(True)

    assert reporter.update(False) is False
    assert reporter.update(False) is None
