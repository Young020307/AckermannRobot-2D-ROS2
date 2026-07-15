-- ============================================================
-- my_robot_cartographer_2d.lua
-- 基于 cartographer 官方 backpack_2d.lua 修改
-- 适配单线 360° 激光雷达的 Ackermann 小车
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ====== 坐标系 ======
  map_frame = "map",
  tracking_frame = "laser_link",          -- 激光雷达 frame
  published_frame = "base_link",          -- 底盘 frame，cartographer 发布的 TF 将连接到此
  odom_frame = "odom",
  provide_odom_frame = true,              -- 为其他节点提供 odom 坐标系
  publish_frame_projected_to_2d = true,   -- 锁定 2D（无 roll/pitch/z），符合 2D 建图

  -- ====== 传感器选项 ======
  use_odometry = true,                    -- 订阅 /odom (nav_msgs/Odometry)
  use_nav_sat = false,
  use_landmarks = false,
  use_pose_extrapolator = false,          -- 不使用先验位姿估计（除非 IMU+里程计特别准）
  num_laser_scans = 1,                    -- 1 个 2D 激光扫描
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,    -- 不分割扫描
  num_point_clouds = 0,                   -- 不使用 3D 点云

  -- ====== 发布周期 ======
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,         -- 200 Hz
  trajectory_publish_period_sec = 30e-3,  -- ~33 Hz

  -- ====== 采样比例 ======
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ====== 使用 2D 轨迹构建器 ======
MAP_BUILDER.use_trajectory_builder_2d = true

-- ====== 2D 轨迹构建器参数 ======
TRAJECTORY_BUILDER_2D.min_range = 0.15          -- 激光最小距离 (m)，匹配 LiDAR 配置
TRAJECTORY_BUILDER_2D.max_range = 25.0          -- 激光最大距离 (m)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false       -- 不使用 IMU（可根据需要改为 true）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ====== 子图 ======
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

-- ====== 位姿图优化 ======
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

-- ====== 回环检测 ======
-- 启用全局回环检测（2D 激光雷达可以可靠回环）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
