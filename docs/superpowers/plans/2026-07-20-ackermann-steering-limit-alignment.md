# Ackermann Steering Limit Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Hybrid A* and NeuPAN plan only curvature that the vehicle's `±0.52 rad` front steering joints can physically execute.

**Architecture:** Keep the physical and `ros2_control` joint limits at `±0.52 rad`. Derive the bicycle-model center limit from wheelbase and track, then align NeuPAN runtime/training bounds and Hybrid A* minimum radius to rounded values of `±0.43 rad` and `1.30 m`.

**Tech Stack:** ROS 2 Humble, XACRO/URDF, YAML, Python 3.10, pytest, ament_cmake_pytest.

## Global Constraints

- Keep both physical steering joints and command interfaces at `±0.52 rad`.
- Set NeuPAN runtime and training center steering bounds to `±0.43 rad`.
- Set NeuPAN runtime/training and Hybrid A* minimum radius to `1.30 m`.
- Keep wheelbase `0.593 m`, front track `0.510 m`, arrival thresholds, reference speed, acceleration bounds, and tuning weights unchanged.
- Preserve the steering-angle-to-yaw-rate conversion in `neupan_node`.

---

### Task 1: Enforce physically consistent steering geometry

**Files:**
- Modify: `src/ackermann_robot/test/test_steering_geometry.py`
- Modify: `src/ackermann_robot/CMakeLists.txt`
- Modify: `src/ackermann_robot/package.xml`
- Modify: `src/ackermann_robot/xacro/chassis.xacro`
- Modify: `src/ackermann_robot/xacro/control.xacro`
- Modify: `src/neupan_ros2/config/robots/ackermann_robot/planner.yaml`
- Modify: `src/neupan_ros2/train/train_config.yaml`
- Modify: `src/hybrid_astar_planner/standalone_planner/config/planner_params.yaml`

**Interfaces:**
- Consumes: physical inside-wheel limit, controller wheelbase/front track, and planner bicycle-model bounds.
- Produces: build-time consistency tests and feasible planner curvature limits.

- [ ] **Step 1: Rewrite the geometry tests for physical-limit semantics**

The tests must parse all listed XACRO/YAML files and verify:

```python
equivalent_center = atan(
    wheelbase / (wheelbase / tan(physical_wheel_limit) + track / 2.0)
)

assert physical_wheel_limit == pytest.approx(0.52)
assert ros2_control_limits == {'min': -0.52, 'max': 0.52}
assert runtime_center_limit == pytest.approx(equivalent_center, abs=0.005)
assert training_center_limit == runtime_center_limit
assert runtime_min_radius == pytest.approx(1.30)
assert training_min_radius == runtime_min_radius
assert hybrid_min_radius == runtime_min_radius
assert wheelbase / tan(runtime_center_limit) == pytest.approx(1.30, abs=0.02)
```

Keep the existing `ament_cmake_pytest` registration and test dependencies.

- [ ] **Step 2: Run the test and verify RED**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/ackermann_robot/test/test_steering_geometry.py
```

Expected: FAIL because the worktree still contains the superseded `±0.66 rad` joint changes and planner center limit `0.52 rad`.

- [ ] **Step 3: Apply the physical-limit configuration**

Restore XACRO and `ros2_control` limits:

```xml
lower/min: -0.52
upper/max: 0.52
```

Set both NeuPAN YAML files to:

```yaml
max_speed: [1.5, 0.43]
min_speed: [-0.5, -0.43]
min_radius: 1.30
```

Set Hybrid A* to:

```yaml
minimum_turning_radius: 1.30
```

Update adjacent comments to describe the single-wheel physical limit and center-equivalent limit.

- [ ] **Step 4: Run focused tests and verify GREEN**

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/ackermann_robot/test/test_steering_geometry.py
PYTHONPATH=src/neupan_ros2:$PYTHONPATH python3 -m pytest -q \
  src/neupan_ros2/test/test_command_conversion.py
```

Expected: geometry tests and all six command-conversion tests pass.

- [ ] **Step 5: Build and run registered tests**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ackermann_robot hybrid_astar_planner neupan_ros2 \
  --symlink-install
colcon test --packages-select ackermann_robot --event-handlers console_direct+
colcon test-result --verbose
```

Expected: all three packages build and the registered steering geometry tests pass.

- [ ] **Step 6: Verify generated URDF and workspace diff**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
xacro src/ackermann_robot/xacro/robot.xacro | rg -n -A8 \
  'joint name="(left|right)_steering_joint"'
python3 -m compileall -q src/ackermann_robot/test
git diff --check
```

Expected: generated joint and command-interface limits are `±0.52`, compilation succeeds, and the diff check is clean.

- [ ] **Step 7: Commit the focused change**

```bash
git add docs/superpowers/specs/2026-07-20-ackermann-steering-limit-alignment-design.md \
  docs/superpowers/plans/2026-07-20-ackermann-steering-limit-alignment.md \
  src/ackermann_robot/CMakeLists.txt src/ackermann_robot/package.xml \
  src/ackermann_robot/xacro/chassis.xacro \
  src/ackermann_robot/xacro/control.xacro \
  src/ackermann_robot/test/test_steering_geometry.py \
  src/neupan_ros2/config/robots/ackermann_robot/planner.yaml \
  src/neupan_ros2/train/train_config.yaml \
  src/hybrid_astar_planner/standalone_planner/config/planner_params.yaml
git commit -m "fix: align planners with physical steering limits"
```
