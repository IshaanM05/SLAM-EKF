cmake_minimum_required(VERSION 3.8)
project(motion_update_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(dv_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories (removed include since directory doesn't exist)
include_directories(${EIGEN3_INCLUDE_DIRS})

# Build executables
add_executable(ekf_visualizer_node src/ekf_visualizer_node.cpp)

# Compile data_assoc_trainee.cpp directly with ekf_localization_node
add_executable(ekf_localization_node 
  src/ekf_localization_node.cpp
  src/data_assoc_trainee.cpp
)

# Dependencies for executables
ament_target_dependencies(ekf_visualizer_node
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs nav_msgs eufs_msgs dv_msgs Eigen3
)

ament_target_dependencies(ekf_localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
  Eigen3
  dv_msgs
  eufs_msgs  # ADDED: Required for WheelSpeedsStamped message
)

# Installation
install(TARGETS
  ekf_visualizer_node
  ekf_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rosbag folder
install(DIRECTORY rosbag_localisation
  DESTINATION share/${PROJECT_NAME}
)

# Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
