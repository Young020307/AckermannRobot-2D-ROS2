# RViz 多航点人工放行导航设计

## 目标

在现有 ROS 2 导航链路中加入多航点任务能力。用户在 RViz 中连续添加多个目标点，系统使用 Hybrid A* 预先规划全部航段，并让 NeuPAN 按航段依次执行。车辆到达每个航点后保持停车，只有收到人工放行命令才前往下一个航点。

本功能参考 `~/navigation/navigation_location/src/mutitarget_nav` 的多点缓存、分段规划和 RViz 标记设计，但适配当前项目的 ROS 2、Standalone Hybrid A* 与 NeuPAN 接口。

## 范围

第一版支持：

- 从 RViz 按顺序添加任意多个航点。
- 每次点击后立即规划并显示当前位置到第一个航点或相邻航点之间的真实路径。
- 按点击顺序排队处理尚未完成的规划请求。
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

现有 Hybrid A* 的 `/plan` `nav_msgs/srv/GetPlan` 服务用于规划每一段路径。每次 RViz 点击都会立即产生一个异步分段规划请求：第一段从点击时的车辆当前位置出发，后续段从上一个有效航点出发。成功航段被追加到预览路径，不可达的点击不会进入有效任务序列。

用户完成设置后调用 `/waypoint_mission/plan` 确认任务。任务节点等待所有点击请求处理完毕，并在启动前复核第一段起点。只有全部有效航段准备完成后才向 NeuPAN发布第一段路径。

任务节点将当前航段以 `nav_msgs/msg/Path` 发布到现有 `/plan` 话题。NeuPAN继续作为唯一的局部规划与车辆控制节点。NeuPAN新增到达状态输出，让任务节点以局部规划器的实际判定作为航段完成依据。

```text
RViz /waypoint_mission/goal
          │
          ▼
 waypoint_mission ── GetPlan ──► Hybrid A* /plan service
          │                         （逐点击规划航段）
          ├── Path /waypoint_mission/preview_path ─► RViz
          │
          │ Path /plan（确认任务后）
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
- 将快速连续点击形成的请求排队，依次规划“当前位置到航点 1”及“航点 N 到航点 N+1”的路径。
- 保存成功航段并发布拼接后的只读预览路径；标记但不接纳不可达航点。
- 启动任务前复核车辆位置，必要时重新规划第一航段。
- 发布当前航段并跟踪执行序号。
- 根据 NeuPAN 到达状态进入等待或完成状态。
- 校验人工放行和清空请求是否合法。
- 发布状态、航点编号、真实路径预览与当前目标标记。

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
| `/waypoint_mission/preview_path` | `nav_msgs/msg/Path` | 向 RViz发布已确认航段拼接成的真实路径预览 |
| `/waypoint_mission/markers` | `visualization_msgs/msg/MarkerArray` | 显示航点、编号、连线和当前目标 |
| `/waypoint_mission/status` | `std_msgs/msg/String` | 发布当前状态名称 |

`status` 使用 reliable、transient-local、depth 1 QoS，并仅发布以下稳定值：`IDLE`、`PLANNING`、`NAVIGATING`、`WAITING`、`COMPLETED`、`FAILED`。

`preview_path` 和 `markers` 同样使用 reliable、transient-local、depth 1 QoS，确保 RViz晚于任务节点启动时仍能立即看到当前预览。

### 服务

三个服务均使用 `std_srvs/srv/Trigger`，通过响应中的 `success` 和 `message` 明确表示命令是否被接受。

| 名称 | 有效状态 | 行为 |
|---|---|---|
| `/waypoint_mission/plan` | `IDLE`、`FAILED` | 确认预览结果，复核首段起点并启动第一段 |
| `/waypoint_mission/continue` | `WAITING` | 发布下一航段 |
| `/waypoint_mission/clear` | `IDLE`、`WAITING`、`COMPLETED`、`FAILED` | 清空航点、路径和标记，回到 `IDLE` |

`clear` 在 `PLANNING` 或 `NAVIGATING` 时拒绝请求。`continue` 在任何非 `WAITING` 状态拒绝请求，因此重复调用不会跳过航点。

若仍有点击请求正在排队或规划，`plan` 返回 `success=false` 并提示剩余请求数量，用户稍后重试。若第一航段起点需要更新，`plan` 在成功启动异步重规划后立即返回 `success=true`，响应消息表示“首段重规划已开始”；最终成功或失败通过 `/waypoint_mission/status` 和节点日志报告。若无需更新首段，`plan` 直接进入 `NAVIGATING` 并发布第一航段。

## 数据流与状态流转

1. 节点启动后处于 `IDLE`。
2. 用户将 RViz 的 2D Goal Pose 话题设为 `/waypoint_mission/goal` 并连续点击航点。
3. 每个点击先进入先进先出队列并显示为橙色待处理航点。`IDLE` 状态允许继续添加航点。
4. 队首请求开始时，节点将状态短暂切换为 `PLANNING`，并调用 Hybrid A*：第一个有效点的起点是此时的机器人位姿，其余点的起点是上一个有效航点。
5. 规划成功后，将航点和航段加入有效任务序列，航点变为绿色，更新 `/waypoint_mission/preview_path`，然后继续处理队列。队列清空后回到 `IDLE`。
6. 规划失败后，该点击显示为红色失败点但不加入有效序列；后续点击仍从上一个有效航点开始规划。队列清空后状态为 `FAILED`，用户可清空失败标记或继续添加新点；新增规划成功后回到 `IDLE`。
7. 用户确认预览路线后调用 `/waypoint_mission/plan`。
8. 若车辆当前位置与第一段保存的起点之间的位置偏差超过 `first_segment_replan_distance`（默认 0.20 m），或航向偏差超过 `first_segment_replan_yaw`（默认 0.17 rad），节点进入 `PLANNING` 并从当前位姿重新规划第一段。重规划失败则进入 `FAILED` 且车辆不启动。
9. 首段重规划成功时替换缓存中的第一航段并更新完整预览路径。航段就绪后，节点将执行索引设为 0，进入 `NAVIGATING` 并发布第一段。
10. NeuPAN接受新路径并发布 `false`，沿当前航段行驶。
11. NeuPAN首次发布 `true` 时：
   - 若当前航段不是最后一段，任务进入 `WAITING`，车辆保持零速度。
   - 若当前航段是最后一段，任务进入 `COMPLETED`，车辆保持零速度。
12. `WAITING` 时用户调用 `/waypoint_mission/continue`。任务节点递增执行索引、进入 `NAVIGATING` 并发布下一段路径。
13. 新路径使 NeuPAN重置到达状态并发布 `false`，随后重复步骤 10 至 12。

为了防止 transient-local 保存的旧 `true` 在任务开始时被误认为新航段已到达，任务节点在每次发布航段后必须先观察到一次 `false`，之后才接受该航段的 `true`。该门控同样消除消息乱序和重复 `true` 的影响。

## RViz 可视化

任务节点发布真实规划路径而非目标点之间的几何直线。成功航段去除相邻段重复端点后拼接为 `/waypoint_mission/preview_path`；该话题仅供 RViz显示，不被 NeuPAN订阅。

任务节点在固定命名空间下发布：

- 每个点击一个形状 Marker：待处理为橙色、有效为绿色、规划失败为红色。
- 每个航点一个从 1 开始的文本编号。
- 有效航点的编号按实际执行顺序连续排列；失败点使用 `X` 和失败点击序号，不占用执行编号。
- 当前正在导航或等待的目标点使用黄色和更大尺寸。
- RViz中的 Path display 订阅 `/waypoint_mission/preview_path`，完整显示从车辆预览起点到最后一个有效航点的 Hybrid A* 路径。

调用 `clear` 时发布 `DELETEALL` 和空预览路径。规划失败时保留红色失败标记，便于用户调整下一个点击；失败点不参与路径拼接。执行时仍以航段为单位，现有 `/plan_path` 和 NeuPAN可视化继续显示当前执行路径。

## 并发与一致性

服务、订阅和规划结果可能由 ROS 2 executor 并发触发。任务节点使用互斥锁保护状态、航点列表、航段列表、执行索引和到达门控。锁内只修改内存状态，不调用 ROS 服务或发布消息。

点击回调只负责坐标转换、创建 Marker 和入队，不等待规划结果。队列只允许一个在途 GetPlan 请求，每次响应按请求序号提交结果并触发下一个请求，从而保持快速连续点击的原始顺序。服务调用必须异步推进，不能在回调内同步等待同一 executor 的响应。

执行 `/waypoint_mission/plan` 后冻结航点队列并拒绝新点击。第一段重规划也使用航点与车辆位姿快照。任务回到 `IDLE`、`FAILED` 后且尚未启动时，才允许继续添加点击。

## 错误处理

- 未添加有效航点：`plan` 返回失败，保持当前状态。
- TF不可用或目标坐标转换失败：记录明确原因，不发送路径。
- Hybrid A* 服务不可用：进入 `FAILED`，保留航点。
- 新点击对应航段返回空路径或服务异常：保留红色失败标记，不加入有效航点和预览路径。
- 首段启动前重规划失败：进入 `FAILED`，不执行任何部分路径。
- 点击队列尚未处理完时调用 `plan`：拒绝启动并返回剩余请求数量。
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

# 确认已经预览的航段并开始第一段
ros2 service call /waypoint_mission/plan std_srvs/srv/Trigger {}

# 每次车辆到点停车后，人工放行
ros2 service call /waypoint_mission/continue std_srvs/srv/Trigger {}

# 等待中或任务结束后清空
ros2 service call /waypoint_mission/clear std_srvs/srv/Trigger {}
```

RViz配置需要增加 `/waypoint_mission/preview_path` 的 Path display 和 `/waypoint_mission/markers` 的 MarkerArray display。README需要补充 RViz 话题设置、标记颜色含义、服务命令、状态查询以及不要同时使用单点和多点入口的提示。

## 测试与验收

### 单元测试

状态机测试覆盖：

- 空任务不能规划。
- 合法状态流 `IDLE → PLANNING → IDLE → NAVIGATING → WAITING → NAVIGATING → COMPLETED`。
- 快速连续点击按原始顺序串行规划。
- 点击航段规划失败进入 `FAILED`，失败点不改变有效航点序列。
- 后续点击从上一个有效航点而非失败点开始规划。
- 队列未清空时不能启动任务。
- 起点位姿超过阈值时重规划第一段，失败时不启动任务。
- 非 `WAITING` 状态拒绝 `continue`。
- 重复 `continue` 不会跳过航段。
- 未观察到 `false` 时忽略陈旧的 `true`。
- `clear` 的允许与拒绝状态。
- 最终航点到达后不能继续。

航点与路径辅助函数测试覆盖：

- 多航点生成正确数量和顺序的航段请求。
- 成功航段正确拼接预览路径并去除连接处重复点。
- 空路径只拒绝对应点击，不污染有效任务路径。
- Marker ID、编号和删除操作稳定。

NeuPAN测试覆盖：

- 接收新路径时发布 `false` 并重置旧到达状态。
- 到达状态首次变化为 `true` 时发布一次有效状态。
- 重复规划循环中的持续 `true` 不产生状态洪泛。

### 集成验收

1. 在 RViz 快速添加三个航点，确认每次规划完成后依次显示真实航段、绿色形状和连续编号。
2. 添加一个不可达点，确认显示红色失败标记且下一有效点仍连接到上一个绿色航点。
3. 调用 `plan` 后确认只向 NeuPAN发布第一段；移动过车辆时先自动重规划第一段。
4. 车辆到达航点 1 后保持静止至少 30 秒，且未自动发布第二段。
5. 调用一次 `continue` 后车辆前往航点 2；重复调用被拒绝。
6. 在航点 2 重复停车和人工放行流程。
7. 到达航点 3 后状态为 `COMPLETED`，车辆持续静止，`continue` 被拒绝。
8. 验证首段重规划失败时车辆不启动、状态为 `FAILED`。
9. 验证原有 `/goal_pose` 单点导航仍可独立使用。

## 完成标准

- 新包和 NeuPAN接口通过构建及单元测试。
- 三航点仿真流程满足逐点停车和人工放行要求。
- RViz在设置阶段逐段显示 Hybrid A* 真实路径、有效航点顺序和失败点。
- 失败路径不会导致部分任务启动。
- RViz标记和任务状态与实际执行航段一致。
- README提供可复制执行的完整操作命令。
