cmake_minimum_required(VERSION 3.8)
project(ackermann_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/arrow_key_control.py
  scripts/cmd_vel_stamper.py
  scripts/cmd_vel_mux.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  meshes
  rviz
  maps
  xacro
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_steering_geometry
    test/test_steering_geometry.py
  )
endif()

ament_package()
