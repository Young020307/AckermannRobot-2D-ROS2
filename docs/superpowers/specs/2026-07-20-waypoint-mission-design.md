# RViz 多航点人工放行导航设计

## 目标

在现有 ROS 2 导航链路中加入多航点任务能力。用户在 RViz 中连续添加多个目标点，系统使用 Hybrid A* 预先规划全部航段，并让 NeuPAN 按航段依次执行。车辆到达每个航点后保持停车，只有收到人工放行命令才前往下一个航点。

本功能参考 `~/navigation/navigation_location/src/mutitarget_nav` 的多点缓存、分段规划和 RViz 标记设计，但适配当前项目的 ROS 2、Standalone Hybrid A* 与 NeuPAN 接口。

## 范围

第一版支持：

- 从 RViz 按顺序添加任意多个航点。
- 在任务开始前规划当前位置到第一个航点以及所有相邻航点之间的路径。
- 逐段向 NeuPAN 发布规划结果。
- 到达航点后停车并等待人工放行。
- 清空未执行、等待中或已结束的任务。
- 发布任务状态和 RViz 航点标记。
- 保留原有 `/goal_pose` 单点导航入口。

第一版不支持：

- 导航途中添加或修改航点。
- 车辆行驶过程中强制清空、暂停或跳过当前航点。
- 自动等待固定时间后放行。
- 航点脚本、话题发布或服务调用等扩展动作。
- 任务持久化、CSV 导入或循环巡航。

## 总体架构

新增独立的 ROS 2 Python 包 `waypoint_mission`。任务节点负责航点缓存、分段规划、任务状态和逐段调度，不直接发布速度命令。

现有 Hybrid A* 的 `/plan` `nav_msgs/srv/GetPlan` 服务用于规划每一段路径。任务节点只有在所有航段均成功后才发布第一段路径；这样可避免车辆启动后才发现后续航段不可达。

任务节点将当前航段以 `nav_msgs/msg/Path` 发布到现有 `/plan` 话题。NeuPAN继续作为唯一的局部规划与车辆控制节点。NeuPAN新增到达状态输出，让任务节点以局部规划器的实际判定作为航段完成依据。

```text
RViz /waypoint_mission/goal
          │
          ▼
 waypoint_mission ── GetPlan ──► Hybrid A* /plan service
          │                         （预先规划全部航段）
          │ Path /plan
          ▼
       NeuPAN ───────────────► cmd_vel_mux ─► Ackermann controller
          │
          └── Bool /neupan_arrived ──► waypoint_mission

人工调用 /waypoint_mission/continue 后发布下一航段
```

单点模式仍使用 `/goal_pose`。多点模式的 RViz Goal Tool 必须改为 `/waypoint_mission/goal`，避免每次点击被现有 Hybrid A* 和 NeuPAN立即当作单点导航命令。

## 组件职责

### WaypointMissionNode

任务节点具有以下职责：

- 接收并按顺序保存 RViz 航点。
- 使用 TF 获取 `map` 坐标系下的当前 `base_link` 位姿。
- 依次请求“当前位置到航点 1”及“航点 N 到航点 N+1”的路径。
- 验证全部航段非空后保存规划结果。
- 发布当前航段并跟踪执行序号。
- 根据 NeuPAN 到达状态进入等待或完成状态。
- 校验人工放行和清空请求是否合法。
- 发布状态、航点编号、连接线与当前目标标记。

任务状态机与 ROS 通信代码分离。纯 Python 状态机不依赖 ROS 消息，以便单元测试覆盖边界条件。

### Hybrid A*

现有 Hybrid A* 节点保持规划算法不变。任务节点调用其 `/plan` 服务，并显式提供每段的起点和终点。服务返回的路径只作为规划结果，不会自动触发 NeuPAN。

### NeuPAN

NeuPAN保留当前订阅 `/plan` 并在新路径到来时重置 `arrive` 的行为，同时新增 `std_msgs/msg/Bool` 到达状态发布器：

- 新路径被接受时发布 `false`。
- 本航段首次到达时发布 `true`。
- 状态值发生变化时立即发布，使用 reliable、transient-local、depth 1 QoS，使后启动的任务节点也能获得当前值。
- NeuPAN到达后继续使用现有零速度输出逻辑保持停车。

任务节点不通过 TF 距离自行重复判断到达，避免任务层阈值与 NeuPAN实际停车阈值不一致。

## ROS 2 接口

### 订阅

| 名称 | 类型 | 用途 |
|---|---|---|
| `/waypoint_mission/goal` | `geometry_msgs/msg/PoseStamped` | RViz添加航点 |
| `/neupan_arrived` | `std_msgs/msg/Bool` | 当前 NeuPAN 航段到达状态 |

非 `map` 坐标系的目标点由任务节点通过 TF 转换到 `map`；转换失败的点不会加入队列。

### 发布

| 名称 | 类型 | 用途 |
|---|---|---|
| `/plan` | `nav_msgs/msg/Path` | 向 NeuPAN发送当前航段 |
| `/waypoint_mission/markers` | `visualization_msgs/msg/MarkerArray` | 显示航点、编号、连线和当前目标 |
| `/waypoint_mission/status` | `std_msgs/msg/String` | 发布当前状态名称 |

`status` 使用 reliable、transient-local、depth 1 QoS，并仅发布以下稳定值：`IDLE`、`PLANNING`、`NAVIGATING`、`WAITING`、`COMPLETED`、`FAILED`。

### 服务

三个服务均使用 `std_srvs/srv/Trigger`，通过响应中的 `success` 和 `message` 明确表示命令是否被接受。

| 名称 | 有效状态 | 行为 |
|---|---|---|
| `/waypoint_mission/plan` | `IDLE`、`FAILED` | 规划全部航段并启动第一段 |
| `/waypoint_mission/continue` | `WAITING` | 发布下一航段 |
| `/waypoint_mission/clear` | `IDLE`、`WAITING`、`COMPLETED`、`FAILED` | 清空航点、路径和标记，回到 `IDLE` |

`clear` 在 `PLANNING` 或 `NAVIGATING` 时拒绝请求。`continue` 在任何非 `WAITING` 状态拒绝请求，因此重复调用不会跳过航点。

`plan` 在校验航点和状态、成功启动异步规划流程后立即返回 `success=true`，响应消息表示“规划已开始”。最终规划成功或失败通过 `/waypoint_mission/status` 和节点日志报告，避免 Trigger 回调在等待多个 Hybrid A* 服务响应时阻塞 executor。

## 数据流与状态流转

1. 节点启动后处于 `IDLE`。
2. 用户将 RViz 的 2D Goal Pose 话题设为 `/waypoint_mission/goal` 并连续点击航点。
3. 每个有效航点加入队列后更新 MarkerArray。`IDLE` 状态允许继续添加航点。
4. 用户调用 `/waypoint_mission/plan`。
5. 节点进入 `PLANNING`，查询当前机器人位姿并按顺序调用 Hybrid A* 服务。
6. 任一航段失败时丢弃本次产生的全部航段，进入 `FAILED`，车辆不启动；已添加航点保留以便修正外部条件后重试或清空。
7. 全部航段成功后，节点保存航段列表，将执行索引设为 0，进入 `NAVIGATING` 并发布第一段。
8. NeuPAN接受新路径并发布 `false`，沿当前航段行驶。
9. NeuPAN首次发布 `true` 时：
   - 若当前航段不是最后一段，任务进入 `WAITING`，车辆保持零速度。
   - 若当前航段是最后一段，任务进入 `COMPLETED`，车辆保持零速度。
10. `WAITING` 时用户调用 `/waypoint_mission/continue`。任务节点递增执行索引、进入 `NAVIGATING` 并发布下一段路径。
11. 新路径使 NeuPAN重置到达状态并发布 `false`，随后重复步骤 8 至 10。

为了防止 transient-local 保存的旧 `true` 在任务开始时被误认为新航段已到达，任务节点在每次发布航段后必须先观察到一次 `false`，之后才接受该航段的 `true`。该门控同样消除消息乱序和重复 `true` 的影响。

## RViz 可视化

任务节点在固定命名空间下发布：

- 每个航点一个球形 Marker。
- 每个航点一个从 1 开始的文本编号。
- 一条按添加顺序连接航点的折线。
- 当前正在导航或等待的目标点使用不同颜色和更大尺寸。

调用 `clear` 时发布 `DELETEALL`。规划失败时保留航点标记；用户可以检查航点后重试或清空。规划服务不发布拼接路径，因为执行时以航段为单位；现有 `/plan_path` 和 NeuPAN可视化继续显示当前路径。

## 并发与一致性

服务、订阅和规划结果可能由 ROS 2 executor 并发触发。任务节点使用互斥锁保护状态、航点列表、航段列表、执行索引和到达门控。锁内只修改内存状态，不调用 ROS 服务或发布消息。

规划过程中使用航点快照。进入 `PLANNING` 后拒绝新航点，即使规划服务响应较慢也不会改变本次任务。服务调用必须异步推进，不能在回调内同步等待同一 executor 的响应。每次响应只触发下一段请求，直到全部成功或任一失败。

## 错误处理

- 未添加航点：`plan` 返回失败，保持 `IDLE`。
- TF不可用或目标坐标转换失败：记录明确原因，不发送路径。
- Hybrid A* 服务不可用：进入 `FAILED`，保留航点。
- 任一航段返回空路径或服务异常：进入 `FAILED`，不执行任何部分路径。
- 提前或重复调用 `continue`：返回失败和当前状态，不改变执行索引。
- 在禁止状态调用 `clear`：返回失败，不改变任务。
- 收到重复的 NeuPAN到达消息：由“先见 false、再接受 true”的门控与状态检查忽略。
- 最终航点到达：进入 `COMPLETED`；此后 `continue` 返回任务已完成。

## 启动与使用

多航点任务节点加入现有 `navigation_carto.launch.py`，默认启动并使用仿真时间。原有导航脚本无需增加新的终端。

典型使用流程：

```bash
# 启动现有导航与 NeuPAN
bash scripts/nav_carto_neupan.sh
bash scripts/run_neupan.sh

# RViz 中将 2D Goal Pose 的 Topic 改为 /waypoint_mission/goal，连续添加航点

# 规划全部航段并开始第一段
ros2 service call /waypoint_mission/plan std_srvs/srv/Trigger {}

# 每次车辆到点停车后，人工放行
ros2 service call /waypoint_mission/continue std_srvs/srv/Trigger {}

# 等待中或任务结束后清空
ros2 service call /waypoint_mission/clear std_srvs/srv/Trigger {}
```

README需要补充 RViz 话题设置、服务命令、状态查询以及不要同时使用单点和多点入口的提示。

## 测试与验收

### 单元测试

状态机测试覆盖：

- 空任务不能规划。
- 合法状态流 `IDLE → PLANNING → NAVIGATING → WAITING → NAVIGATING → COMPLETED`。
- 规划失败进入 `FAILED` 且不产生可执行航段。
- 非 `WAITING` 状态拒绝 `continue`。
- 重复 `continue` 不会跳过航段。
- 未观察到 `false` 时忽略陈旧的 `true`。
- `clear` 的允许与拒绝状态。
- 最终航点到达后不能继续。

航点与路径辅助函数测试覆盖：

- 多航点生成正确数量和顺序的航段请求。
- 空路径导致整体失败。
- Marker ID、编号和删除操作稳定。

NeuPAN测试覆盖：

- 接收新路径时发布 `false` 并重置旧到达状态。
- 到达状态首次变化为 `true` 时发布一次有效状态。
- 重复规划循环中的持续 `true` 不产生状态洪泛。

### 集成验收

1. 在 RViz 添加三个航点，调用 `plan` 后确认 Hybrid A* 完成三段规划并只发布第一段。
2. 车辆到达航点 1 后保持静止至少 30 秒，且未自动发布第二段。
3. 调用一次 `continue` 后车辆前往航点 2；重复调用被拒绝。
4. 在航点 2 重复停车和人工放行流程。
5. 到达航点 3 后状态为 `COMPLETED`，车辆持续静止，`continue` 被拒绝。
6. 验证某一航段不可达时车辆不启动、状态为 `FAILED`，错误信息包含航段编号。
7. 验证原有 `/goal_pose` 单点导航仍可独立使用。

## 完成标准

- 新包和 NeuPAN接口通过构建及单元测试。
- 三航点仿真流程满足逐点停车和人工放行要求。
- 失败路径不会导致部分任务启动。
- RViz标记和任务状态与实际执行航段一致。
- README提供可复制执行的完整操作命令。
