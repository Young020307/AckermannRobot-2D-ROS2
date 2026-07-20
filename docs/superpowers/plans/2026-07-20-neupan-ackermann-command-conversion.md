# NeuPAN Ackermann Command Conversion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a standards-compliant yaw rate from NeuPAN Ackermann steering commands so forward and reverse paths reach the curvature planned by Hybrid A*.

**Architecture:** Put the kinematic conversion in a pure utility in `utils.py`, then call it at the ROS message boundary in `NeupanCore.generate_twist_msg`. Ackermann controls use `v * tan(steering_angle) / wheelbase`; differential-drive controls remain unchanged.

**Tech Stack:** Python 3.10, ROS 2 Humble `geometry_msgs/Twist`, pytest, NumPy.

## Global Constraints

- Do not change NeuPAN planner weights, speed limits, Hybrid A* settings, mux behavior, or URDF steering limits.
- Preserve zero Twist output for missing actions, emergency stop, and arrival.
- Preserve differential-drive command semantics.
- Use the wheelbase already loaded by NeuPAN's robot model.

---

### Task 1: Kinematic command conversion

**Files:**
- Modify: `src/neupan_ros2/neupan_ros2/utils.py`
- Create: `src/neupan_ros2/test/test_command_conversion.py`

**Interfaces:**
- Consumes: `speed: float`, `turn_control: float`, `kinematics: str`, `wheelbase: Optional[float]`.
- Produces: `control_to_yaw_rate(speed, turn_control, kinematics, wheelbase) -> float`.

- [ ] **Step 1: Write the failing utility tests**

```python
from math import tan

import pytest

from neupan_ros2.utils import control_to_yaw_rate


def test_ackermann_control_is_converted_from_steering_angle():
    assert control_to_yaw_rate(1.0, 0.52, 'acker', 0.593) == pytest.approx(
        tan(0.52) / 0.593
    )


def test_ackermann_reverse_control_preserves_vehicle_yaw_rate_sign():
    assert control_to_yaw_rate(-0.7907854319, -0.0617321469, 'acker', 0.593) \
        == pytest.approx(0.0824, abs=1e-4)


def test_ackermann_zero_speed_has_zero_yaw_rate():
    assert control_to_yaw_rate(0.0, 0.52, 'acker', 0.593) == 0.0


def test_ackermann_requires_positive_wheelbase():
    with pytest.raises(ValueError, match='wheelbase must be positive'):
        control_to_yaw_rate(1.0, 0.2, 'acker', 0.0)


def test_differential_control_is_already_yaw_rate():
    assert control_to_yaw_rate(-0.5, 0.7, 'diff', None) == 0.7
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/neupan_ros2 python3 -m pytest -q src/neupan_ros2/test/test_command_conversion.py
```

Expected: collection fails because `control_to_yaw_rate` does not exist.

- [ ] **Step 3: Add the minimal pure conversion**

```python
from math import atan2, cos, sin, tan


def control_to_yaw_rate(
    speed: float,
    turn_control: float,
    kinematics: str,
    wheelbase: Optional[float],
) -> float:
    """Convert NeuPAN's second control component to ROS yaw rate."""
    if kinematics != 'acker':
        return float(turn_control)
    if wheelbase is None or wheelbase <= 0.0:
        raise ValueError('Ackermann wheelbase must be positive')
    return float(speed) * tan(float(turn_control)) / float(wheelbase)
```

- [ ] **Step 4: Run the focused test and verify GREEN**

Run the command from Step 2. Expected: `5 passed`.

### Task 2: ROS message integration

**Files:**
- Modify: `src/neupan_ros2/neupan_ros2/neupan_node.py:47,805-832`
- Modify: `src/neupan_ros2/test/test_command_conversion.py`

**Interfaces:**
- Consumes: `control_to_yaw_rate` from Task 1 and `self.neupan_planner.robot.{kinematics,L}`.
- Produces: `Twist.angular.z` containing yaw rate for all robot kinematics.

- [ ] **Step 1: Add a source-level integration regression test**

```python
from pathlib import Path


def test_neupan_node_converts_turn_control_at_twist_boundary():
    source = (
        Path(__file__).parents[1] / 'neupan_ros2' / 'neupan_node.py'
    ).read_text(encoding='utf-8')
    assert 'control_to_yaw_rate(' in source
    assert 'action.angular.z = steer' not in source
```

- [ ] **Step 2: Run the focused test and verify RED**

Run the command from Task 1 Step 2. Expected: the integration test fails because the node still assigns the steering angle directly.

- [ ] **Step 3: Use the conversion in `generate_twist_msg`**

Import `control_to_yaw_rate`, rename the local second control variable to `turn_control`, and set:

```python
robot = self.neupan_planner.robot
action.angular.z = control_to_yaw_rate(
    speed,
    turn_control,
    robot.kinematics,
    robot.L,
)
```

Update the method documentation to describe `[linear_speed, turn_control]` and clarify that Ackermann input is steering angle while differential input is yaw rate.

- [ ] **Step 4: Run focused and package tests**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/neupan_ros2 python3 -m pytest -q src/neupan_ros2/test
```

Expected: all `neupan_ros2` tests pass.

- [ ] **Step 5: Verify syntax, style, and whitespace**

```bash
python3 -m compileall -q src/neupan_ros2/neupan_ros2
git diff --check
```

Expected: both commands exit with status 0 and no output.

- [ ] **Step 6: Commit the focused fix**

```bash
git add src/neupan_ros2/neupan_ros2/utils.py \
  src/neupan_ros2/neupan_ros2/neupan_node.py \
  src/neupan_ros2/test/test_command_conversion.py \
  docs/superpowers/specs/2026-07-20-neupan-ackermann-command-conversion-design.md \
  docs/superpowers/plans/2026-07-20-neupan-ackermann-command-conversion.md
git commit -m "fix: convert NeuPAN steering commands to yaw rate"
```
