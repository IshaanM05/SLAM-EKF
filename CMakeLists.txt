cmake_minimum_required(VERSION 3.8)
project(motion_update_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
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

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Build utility library
add_library(load_landmarks STATIC src/load_landmarks.cpp)
target_include_directories(load_landmarks PUBLIC include)

# Nodes
add_executable(motion_update_node src/motion_update_node.cpp)
target_link_libraries(motion_update_node load_landmarks)

add_executable(ekf_visualizer_node src/ekf_visualizer_node.cpp)

# Dependencies
ament_target_dependencies(motion_update_node
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  eufs_msgs dv_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs
)

ament_target_dependencies(ekf_visualizer_node
  rclcpp std_msgs sensor_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs nav_msgs
  eufs_msgs dv_msgs
)

ament_target_dependencies(load_landmarks
  rclcpp Eigen3
)

# Installation
install(TARGETS
  motion_update_node
  ekf_visualizer_node
  load_landmarks
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rosbag folder (optional, if you want it deployed with the package)
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
