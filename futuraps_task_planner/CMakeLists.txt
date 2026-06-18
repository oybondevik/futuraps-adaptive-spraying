cmake_minimum_required(VERSION 3.8)
project(futuraps_task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(futuraps_perception REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(simple_task_planner src/simple_task_planner.cpp)

ament_target_dependencies(simple_task_planner
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(vertical_pose_planner src/vertical_pose_planner.cpp)

ament_target_dependencies(vertical_pose_planner
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  moveit_ros_planning_interface
  futuraps_perception
  std_srvs
  std_msgs
)

add_executable(vertical_spray_servo src/vertical_spray_servo.cpp)

ament_target_dependencies(vertical_spray_servo
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  futuraps_perception
  std_srvs
  std_msgs
  sensor_msgs
)

target_include_directories(vertical_spray_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(pose_plan_execute src/pose_plan_execute.cpp)
ament_target_dependencies(pose_plan_execute
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(horizontal_spray
  src/horizontal_spray_node.cpp
  src/horizontal_path_builder.cpp
  src/perception_client.cpp
  src/arm_motion_interface.cpp
  src/trajectory_visualizer.cpp
)

ament_target_dependencies(horizontal_spray
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  std_srvs
  std_msgs
  futuraps_perception
  moveit_ros_planning_interface
  moveit_msgs
  nav_msgs
  visualization_msgs
)

add_executable(horizontal_spray_servo
  src/horizontal_spray_servo_node.cpp
  src/perception_client.cpp
  src/platform_moving.cpp
  src/path/path_builder.cpp
  src/path/path_follower.cpp
  src/arm_motion_interface.cpp
  src/path/reachability_filter.cpp
  src/trajectory_visualizer.cpp
)

ament_target_dependencies(horizontal_spray_servo
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  futuraps_perception
  std_srvs
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_msgs
  moveit_core
  moveit_ros_planning
)

target_include_directories(horizontal_spray PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(horizontal_spray_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS 
  simple_task_planner
  vertical_pose_planner
  vertical_spray_servo
  pose_plan_execute
  horizontal_spray
  horizontal_spray_servo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
