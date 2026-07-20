# Ackermann Steering Limit Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the simulated front steering joints capable of executing the `0.52 rad` center steering angle assumed by Hybrid A* and NeuPAN.

**Architecture:** Add a cross-configuration geometry test to the `ackermann_robot` package, then raise the symmetric XACRO steering-joint limits to `±0.66 rad`. Keep all planner and controller geometry parameters unchanged.

**Tech Stack:** ROS 2 Humble, XACRO/URDF, Python 3.10, pytest, ament_cmake_pytest, PyYAML.

## Global Constraints

- Keep Hybrid A* minimum turning radius at `1.05 m`.
- Keep NeuPAN center steering limit at `±0.52 rad` and initial-path radius at `1.05 m`.
- Keep controller wheelbase `0.593 m` and front track `0.510 m`.
- Do not change NeuPAN arrival thresholds or tracking weights.
- Set both simulated steering joints to symmetric limits of exactly `±0.66 rad`.

---

### Task 1: Enforce and align steering geometry

**Files:**
- Create: `src/ackermann_robot/test/test_steering_geometry.py`
- Modify: `src/ackermann_robot/CMakeLists.txt`
- Modify: `src/ackermann_robot/package.xml`
- Modify: `src/ackermann_robot/xacro/chassis.xacro:115`

**Interfaces:**
- Consumes: NeuPAN center steering limit and wheelbase, Hybrid A* minimum radius, controller front track, and XACRO steering-joint limits.
- Produces: a build-time geometry consistency test and XACRO joint range `[-0.66, 0.66] rad`.

- [ ] **Step 1: Add the failing geometry test and register it**

```python
from math import atan, tan
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[3]


def load_yaml(relative_path):
    with (ROOT / relative_path).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_joint_limit_covers_planners_center_steering_geometry():
    neupan = load_yaml(
        'src/neupan_ros2/config/robots/ackermann_robot/planner.yaml'
    )
    controller = load_yaml(
        'src/ackermann_robot/config/ackermann_controllers.yaml'
    )['ackermann_steering_controller']['ros__parameters']
    hybrid = load_yaml(
        'src/hybrid_astar_planner/standalone_planner/config/planner_params.yaml'
    )['/**']['ros__parameters']

    wheelbase = neupan['robot']['wheelbase']
    center_limit = neupan['robot']['max_speed'][1]
    track = controller['front_wheel_track']
    center_radius = wheelbase / tan(center_limit)
    inside_angle = atan(wheelbase / (center_radius - track / 2.0))

    xacro = ET.parse(ROOT / 'src/ackermann_robot/xacro/chassis.xacro')
    steering_joint = next(
        joint for joint in xacro.getroot().iter('joint')
        if joint.get('name') == '${prefix}_steering_joint'
    )
    limit = steering_joint.find('limit')

    assert float(limit.get('upper')) == pytest.approx(0.66)
    assert float(limit.get('lower')) == pytest.approx(-0.66)
    assert float(limit.get('upper')) >= inside_angle
    assert neupan['ipath']['min_radius'] == pytest.approx(center_radius, abs=0.01)
    assert hybrid['minimum_turning_radius'] == pytest.approx(
        center_radius, abs=0.01
    )
```

Register it in `CMakeLists.txt`:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_steering_geometry
    test/test_steering_geometry.py
  )
endif()
```

Add the test dependencies to `package.xml`:

```xml
<test_depend>ament_cmake_pytest</test_depend>
<test_depend>python3-yaml</test_depend>
```

- [ ] **Step 2: Run the test and verify RED**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/ackermann_robot/test/test_steering_geometry.py
```

Expected: FAIL because the current upper limit is `0.52`, not `0.66`.

- [ ] **Step 3: Raise the XACRO steering limits**

Change the steering joint limit to:

```xml
<limit lower="-0.66" upper="0.66" effort="25" velocity="10" />
```

- [ ] **Step 4: Run focused tests and verify GREEN**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/ackermann_robot/test/test_steering_geometry.py
PYTHONPATH=src/neupan_ros2:$PYTHONPATH python3 -m pytest -q \
  src/neupan_ros2/test/test_command_conversion.py
```

Expected: steering geometry test passes and all six command-conversion tests pass.

- [ ] **Step 5: Build and run the registered package test**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ackermann_robot --symlink-install
colcon test --packages-select ackermann_robot --event-handlers console_direct+
colcon test-result --verbose
```

Expected: build succeeds and `test_steering_geometry` passes.

- [ ] **Step 6: Verify generated URDF and workspace diff**

```bash
source /opt/ros/humble/setup.bash
xacro src/ackermann_robot/xacro/robot.xacro > /tmp/ackermann_robot_steering_limit.urdf
python3 -m compileall -q src/ackermann_robot/test
git diff --check
```

Expected: XACRO exits successfully, Python compilation succeeds, and `git diff --check` produces no output.

- [ ] **Step 7: Commit the focused change**

```bash
git add src/ackermann_robot/CMakeLists.txt \
  src/ackermann_robot/package.xml \
  src/ackermann_robot/xacro/chassis.xacro \
  src/ackermann_robot/test/test_steering_geometry.py \
  docs/superpowers/plans/2026-07-20-ackermann-steering-limit-alignment.md
git commit -m "fix: align Ackermann steering joint limits"
```
