cmake_minimum_required(VERSION 3.8)
project(scan_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_filter_node src/scan_filter_node.cpp)
target_compile_features(scan_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(scan_filter_node rclcpp sensor_msgs)

install(TARGETS scan_filter_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
