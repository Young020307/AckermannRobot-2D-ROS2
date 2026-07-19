-- ============================================================
-- assets_writer_ros_map.lua
-- 从 pbstream 导出 PGM + YAML (高精度 origin)
--
-- 使用 cartographer_assets_writer:
--   cartographer_assets_writer \
--     -configuration_directory <dir> \
--     -configuration_basename assets_writer_ros_map.lua \
--     -pose_graph_filename <path/to/map.pbstream>
--
-- 输出: map.pgm + map.yaml (origin 带完整精度)
-- ============================================================

options = {
  tracking_frame = "laser_link",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.15,
      max_range = 25.0,
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filestem = "map",
      resolution = 0.05,
    }
  }
}

return options
