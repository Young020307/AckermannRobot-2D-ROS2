# Manual-Release Waypoint Mission Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RViz-driven multi-waypoint navigation with live Hybrid A* path previews, per-waypoint stops, and manual release to the next waypoint.

**Architecture:** A new `waypoint_mission` ROS 2 Python package owns a tested ROS-independent mission model and a thin ROS adapter. The adapter serializes RViz clicks into `/plan` GetPlan calls, publishes a transient-local preview, and sends one cached segment at a time to NeuPAN. NeuPAN publishes its arrival state so the mission can stop at each target and accept a manual continue request.

**Tech Stack:** ROS 2 Humble, Python 3.10, rclpy, TF2, RViz2, pytest, ament_python.

## Global Constraints

- Preserve `/goal_pose` single-goal navigation; multi-waypoint input is `/waypoint_mission/goal`.
- Never publish a preview to NeuPAN's `/plan` input before the mission is confirmed.
- Process rapid clicks FIFO with only one GetPlan request in flight.
- A failed click is red, excluded from execution, and never becomes the next segment anchor.
- NeuPAN remains the only velocity-command producer.
- Accept arrival only after observing `false` and then `true` for the current segment.
- Preserve all existing uncommitted navigation and scan-filter changes.

## File Structure

- `src/waypoint_mission/waypoint_mission/model.py`: pure queue and mission state.
- `src/waypoint_mission/waypoint_mission/ros_utils.py`: pose, path, and marker helpers.
- `src/waypoint_mission/waypoint_mission/waypoint_mission_node.py`: ROS adapter.
- `src/waypoint_mission/test/`: model, helper, node-contract, and integration tests.
- `src/neupan_ros2/neupan_ros2/utils.py`: arrival edge reporter.
- `src/neupan_ros2/neupan_ros2/neupan_node.py`: arrival publisher wiring.
- `src/robot_slam/launch/navigation_carto.launch.py`: mission node startup.
- `src/ackermann_robot/rviz/nav2_default_view.rviz`: preview and marker displays.
- `README.md`: operator workflow.

---

### Task 1: Scaffold the Package and Build the Mission Model

**Files:**
- Create: `src/waypoint_mission/package.xml`
- Create: `src/waypoint_mission/setup.py`
- Create: `src/waypoint_mission/setup.cfg`
- Create: `src/waypoint_mission/resource/waypoint_mission`
- Create: `src/waypoint_mission/waypoint_mission/__init__.py`
- Create: `src/waypoint_mission/waypoint_mission/model.py`
- Test: `src/waypoint_mission/test/test_model.py`

**Interfaces:**
- Produces `Pose2D`, `PlanningRequest`, `MissionState`, `MissionError`, and `MissionModel`.
- `MissionModel` owns queued goals, valid waypoints, opaque path segments, failed click IDs, current segment, and arrival gating.
- Exact methods: `enqueue_goal(Pose2D) -> int`, `next_planning_request(Pose2D) -> Optional[PlanningRequest]`, `finish_planning(Any) -> PlanningRequest`, `begin_execution() -> Any`, `begin_first_segment_replan() -> None`, `finish_first_segment_replan(Any) -> Optional[Any]`, `observe_arrival(bool) -> bool`, `continue_execution() -> Any`, and `clear() -> None`.

- [ ] **Step 1: Write failing model tests**

Cover FIFO IDs; success; failure excluded from the anchor chain; recovery after failure; pending/active start rejection; successful and failed first-segment replan; `IDLE → NAVIGATING → WAITING → NAVIGATING → COMPLETED`; false-before-true arrival; duplicate continue; and clear state rules. The key anchor test is:

```python
model.enqueue_goal(first)
model.enqueue_goal(blocked)
model.enqueue_goal(third)
req = model.next_planning_request(robot)
assert (req.start, req.goal) == (robot, first)
model.finish_planning('segment-1')
req = model.next_planning_request(robot)
assert (req.start, req.goal) == (first, blocked)
model.finish_planning(None)
req = model.next_planning_request(robot)
assert (req.start, req.goal) == (first, third)
```

- [ ] **Step 2: Prove tests fail before implementation**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_model.py`

Expected: `ModuleNotFoundError` for `waypoint_mission.model`.

- [ ] **Step 3: Implement the exact model API**

```python
class MissionState(str, Enum):
    IDLE = 'IDLE'
    PLANNING = 'PLANNING'
    NAVIGATING = 'NAVIGATING'
    WAITING = 'WAITING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PlanningRequest:
    click_id: int
    start: Pose2D
    goal: Pose2D
```

Implement `MissionModel` with the exact signatures listed in the Interfaces block. `next_planning_request()` uses the last valid waypoint or robot pose; `finish_planning(None)` records failure without changing the anchor; `begin_execution()` rejects pending/active/empty missions. `begin_first_segment_replan()` performs the same readiness validation, freezes new goals, and enters PLANNING. `finish_first_segment_replan(path)` replaces segment 0 and enters NAVIGATING when nonempty; a `None` path enters FAILED and unlocks goal editing. Each published segment resets `arrival_armed`; `observe_arrival(False)` arms and a later true changes to WAITING or COMPLETED; clear rejects PLANNING/NAVIGATING.

- [ ] **Step 4: Run model tests**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_model.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/waypoint_mission
git commit -m "feat: add waypoint mission state model"
```

---

### Task 2: Add Path and Marker Utilities

**Files:**
- Create: `src/waypoint_mission/waypoint_mission/ros_utils.py`
- Test: `src/waypoint_mission/test/test_ros_utils.py`

**Interfaces:**
- Consumes `Pose2D`.
- Produces `pose_to_model()`, `pose_delta()`, `stitch_paths()`, `build_markers()`, and `delete_all_markers()`.

- [ ] **Step 1: Write failing utility tests**

Test that two paths `[(0,0),(1,0)]` and `[(1,0),(2,1)]` stitch to three poses; the inputs remain unchanged; yaw difference is normalized; marker IDs remain stable; current overrides valid color; pending is orange; failed is red and text `X<click-id>`; and clear creates DELETEALL.

- [ ] **Step 2: Prove tests fail**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_ros_utils.py`

Expected: missing `ros_utils` import.

- [ ] **Step 3: Implement helpers**

Use exact colors pending `(1.0,0.55,0.0,0.95)`, valid `(0.1,0.8,0.1,0.95)`, failed `(0.9,0.1,0.1,0.95)`, current `(1.0,0.9,0.0,1.0)`, and white text. Use SPHERE markers at z=0.15, diameter 0.30 m, current diameter 0.45 m, text at z=0.55. Drop the first pose of a later path only when its XY matches the previous output within `1e-6 m`.

```python
def pose_delta(first: Pose2D, second: Pose2D) -> Tuple[float, float]:
    distance = math.hypot(second.x - first.x, second.y - first.y)
    delta = second.yaw - first.yaw
    yaw = abs(math.atan2(math.sin(delta), math.cos(delta)))
    return distance, yaw
```

- [ ] **Step 4: Run both test files**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_model.py src/waypoint_mission/test/test_ros_utils.py`

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/waypoint_mission/waypoint_mission/ros_utils.py src/waypoint_mission/test/test_ros_utils.py
git commit -m "feat: add waypoint preview helpers"
```

---

### Task 3: Implement the ROS Mission Node

**Files:**
- Create: `src/waypoint_mission/waypoint_mission/waypoint_mission_node.py`
- Modify: `src/waypoint_mission/package.xml`
- Modify: `src/waypoint_mission/setup.py`
- Test: `src/waypoint_mission/test/test_node_contract.py`

**Interfaces:**
- Consumes `/plan` as `nav_msgs/srv/GetPlan` and `/neupan_arrived` as Bool.
- Produces `/plan`, `/waypoint_mission/preview_path`, `/waypoint_mission/markers`, `/waypoint_mission/status`.
- Produces Trigger services `/waypoint_mission/plan`, `/waypoint_mission/continue`, `/waypoint_mission/clear`.

- [ ] **Step 1: Write failing contract/callback tests**

Using fake publishers, TF, GetPlan futures, and model paths, verify exact interface names; transient-local preview/markers/status QoS; FIFO requests; failed click exclusion; plan rejection while requests remain; first-segment replan; continue rejection before arrival; and one segment per execution publish.

- [ ] **Step 2: Prove contract tests fail**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_node_contract.py`

Expected: node module missing.

- [ ] **Step 3: Implement node construction and live preview**

Declare parameters with exact defaults:

```python
global_frame = 'map'
base_frame = 'base_link'
goal_topic = '/waypoint_mission/goal'
planner_service = '/plan'
execution_path_topic = '/plan'
preview_path_topic = '/waypoint_mission/preview_path'
marker_topic = '/waypoint_mission/markers'
status_topic = '/waypoint_mission/status'
arrival_topic = '/neupan_arrived'
first_segment_replan_distance = 0.20
first_segment_replan_yaw = 0.17
```

Use `ReentrantCallbackGroup`, a four-thread executor, one lock, and one in-flight future. Goal callback transforms to map, records the click pose, enqueues it, publishes orange marker, then starts planning. Completion stores a nonempty Path or failure, publishes the stitched preview and markers, then starts the next queued request. Never hold the lock during TF, publish, or `call_async()`.

- [ ] **Step 4: Implement services and arrival callback**

`plan` rejects empty/pending/active/already-started missions. It compares current TF against the cached first start; over 0.20 m or 0.17 rad calls `begin_first_segment_replan()` and starts an async replacement request. Pass its result to `finish_first_segment_replan()`; a failed replan publishes nothing to `/plan`, while success updates preview and publishes only segment 0. If no replan is needed, call `begin_execution()` and publish only segment 0. `continue` publishes exactly the segment returned by `continue_execution()`. `clear` resets model/caches and publishes DELETEALL plus an empty preview. Arrival passes Bool to `observe_arrival()` and updates status/markers only on transition.

- [ ] **Step 5: Finish package metadata**

Add exec dependencies `rclpy`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `std_srvs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`; install package/resource XML; expose `waypoint_mission_node = waypoint_mission.waypoint_mission_node:main`.

- [ ] **Step 6: Test and build**

```bash
PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test
colcon build --symlink-install --packages-select waypoint_mission
```

Expected: PASS and package builds.

- [ ] **Step 7: Commit**

```bash
git add src/waypoint_mission
git commit -m "feat: add live-preview waypoint mission node"
```

---

### Task 4: Publish NeuPAN Arrival State

**Files:**
- Modify: `src/neupan_ros2/neupan_ros2/utils.py`
- Modify: `src/neupan_ros2/neupan_ros2/neupan_node.py`
- Modify: `src/neupan_ros2/config/robots/ackermann_robot/robot.yaml`
- Modify: `src/neupan_ros2/package.xml`
- Create: `src/neupan_ros2/test/test_arrival_reporter.py`

**Interfaces:**
- Produces `ArrivalReporter.reset() -> bool`, `update(bool) -> Optional[bool]`.
- Produces `/neupan_arrived`, reliable/transient-local/depth 1.

- [ ] **Step 1: Write failing reporter test**

```python
reporter = ArrivalReporter()
assert reporter.reset() is False
assert reporter.update(False) is None
assert reporter.update(True) is True
assert reporter.update(True) is None
assert reporter.reset() is False
assert reporter.update(True) is True
```

- [ ] **Step 2: Prove it fails**

Run: `PYTHONPATH=src/neupan_ros2 pytest -q src/neupan_ros2/test/test_arrival_reporter.py`

Expected: import failure.

- [ ] **Step 3: Implement reporter and wiring**

```python
class ArrivalReporter:
    def __init__(self) -> None:
        self._last: Optional[bool] = None

    def reset(self) -> bool:
        self._last = False
        return False

    def update(self, value: bool) -> Optional[bool]:
        value = bool(value)
        if value == self._last:
            return None
        self._last = value
        return value
```

Declare `arrival_topic` default `/neupan_arrived`, publish false at startup, force false after accepting every nonempty new Path, and publish only changed planning arrival values. Publish outside `_state_lock`.

- [ ] **Step 4: Add YAML and package dependency**

Add `arrival_topic: '/neupan_arrived'` and `<exec_depend>std_msgs</exec_depend>`.

- [ ] **Step 5: Test and syntax-check**

```bash
PYTHONPATH=src/neupan_ros2 pytest -q src/neupan_ros2/test/test_arrival_reporter.py
python3 -m py_compile src/neupan_ros2/neupan_ros2/utils.py src/neupan_ros2/neupan_ros2/neupan_node.py
```

Expected: PASS and exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/neupan_ros2
git commit -m "feat: publish NeuPAN segment arrival state"
```

---

### Task 5: Integrate Launch and RViz

**Files:**
- Modify: `src/robot_slam/launch/navigation_carto.launch.py`
- Modify: `src/robot_slam/package.xml`
- Modify: `src/ackermann_robot/rviz/nav2_default_view.rviz`
- Test: `src/waypoint_mission/test/test_integration_files.py`

**Interfaces:**
- Launches `waypoint_mission_node`.
- Adds preview Path and mission MarkerArray; sets SetGoal topic `/waypoint_mission/goal`.

- [ ] **Step 1: Write failing integration assertions**

Assert launch package/executable/use_sim_time; robot_slam exec dependency; RViz Path topic `waypoint_mission/preview_path`; MarkerArray topic `waypoint_mission/markers`; SetGoal topic `/waypoint_mission/goal`.

- [ ] **Step 2: Prove failure**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_integration_files.py`

Expected: missing integration assertions fail.

- [ ] **Step 3: Append mission node without rewriting existing work**

```python
waypoint_mission = Node(
    package='waypoint_mission',
    executable='waypoint_mission_node',
    name='waypoint_mission',
    output='screen',
    parameters=[{'use_sim_time': True}],
)
nodes.append(waypoint_mission)
```

Add `<exec_depend>waypoint_mission</exec_depend>` only; retain scan-filter/config changes.

- [ ] **Step 4: Add RViz displays**

Add enabled cyan `Waypoint Preview` Path and enabled `Waypoint Mission` MarkerArray with reliable/transient-local QoS. Change only the SetGoal tool topic.

- [ ] **Step 5: Verify**

```bash
PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_integration_files.py
python3 -m py_compile src/robot_slam/launch/navigation_carto.launch.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/robot_slam/launch/navigation_carto.launch.py src/robot_slam/package.xml src/ackermann_robot/rviz/nav2_default_view.rviz src/waypoint_mission/test/test_integration_files.py
git commit -m "feat: launch and visualize waypoint missions"
```

---

### Task 6: Document and Verify the Workflow

**Files:**
- Modify: `README.md`
- Modify: `src/waypoint_mission/test/test_integration_files.py`

**Interfaces:**
- Documents all goal/status/service names and marker meanings.

- [ ] **Step 1: Add failing README contract**

Assert README contains `/waypoint_mission/goal`, `plan`, `continue`, `clear`, `status`, all four color meanings, and warns against simultaneous `/goal_pose` use.

- [ ] **Step 2: Prove failure**

Run: `PYTHONPATH=src/waypoint_mission pytest -q src/waypoint_mission/test/test_integration_files.py`

Expected: documentation assertions fail.

- [ ] **Step 3: Document exact operation**

```bash
ros2 topic echo --qos-durability transient_local /waypoint_mission/status
ros2 service call /waypoint_mission/plan std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_mission/continue std_srvs/srv/Trigger '{}'
ros2 service call /waypoint_mission/clear std_srvs/srv/Trigger '{}'
```

Explain orange pending, green valid, red rejected, yellow current; actual Hybrid A* previews; indefinite waiting; failed-point exclusion; and single/multi input separation.

- [ ] **Step 4: Run full focused verification**

```bash
PYTHONPATH=src/waypoint_mission:src/neupan_ros2 pytest -q src/waypoint_mission/test src/neupan_ros2/test/test_arrival_reporter.py
python3 -m py_compile src/waypoint_mission/waypoint_mission/*.py src/neupan_ros2/neupan_ros2/utils.py src/neupan_ros2/neupan_ros2/neupan_node.py src/robot_slam/launch/navigation_carto.launch.py
colcon build --symlink-install --packages-select waypoint_mission neupan_ros2 robot_slam
```

Expected: tests and syntax checks pass; affected packages build. If the NeuPAN environment is unavailable, record it and separately require waypoint_mission and robot_slam builds to pass.

- [ ] **Step 5: Inspect user-change safety**

```bash
git status --short
git diff --check
git diff -- src/robot_slam/launch/navigation_carto.launch.py README.md
```

Expected: existing scan-filter/config changes remain intact; no whitespace errors.

- [ ] **Step 6: Commit documentation**

```bash
git add README.md src/waypoint_mission/test/test_integration_files.py
git commit -m "docs: explain manual-release waypoint navigation"
```

- [ ] **Step 7: Optional live smoke test when simulation is available**

```bash
ros2 node info /waypoint_mission
ros2 topic info /waypoint_mission/preview_path --verbose
ros2 topic echo --once --qos-durability transient_local /waypoint_mission/status
ros2 service list | rg '^/waypoint_mission/(plan|continue|clear)$'
```

Expected: documented interfaces exist, preview/status are transient local, and initial status is IDLE.
