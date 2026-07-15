-- ============================================================
-- my_robot_cartographer_2d_localization.lua
-- 基于 my_robot_cartographer_2d.lua 的纯定位模式配置
--
-- 核心差异:
--   1. pure_localization_trimer: 只保留最近 3 个子图，不创建新子图
--   2. optimize_every_n_nodes: 降低优化频率（定位不需要频繁全局优化）
--   3. 运行时通过 -load_state_filename 加载已有 .pbstream 地图
--
-- 官方参考: backpack_2d_localization.lua
-- ============================================================

include "my_robot_cartographer_2d.lua"

-- ====== 纯定位模式: 只保留最近 3 个子图，不新增子图 ======
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- ====== 降低全局优化频率（定位不需要频繁回环优化）======
POSE_GRAPH.optimize_every_n_nodes = 20

return options
