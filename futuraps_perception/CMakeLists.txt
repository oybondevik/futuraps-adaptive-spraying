cmake_minimum_required(VERSION 3.8)
project(futuraps_perception)

# ---------------------------------------------------------------------------
# General setup
# ---------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# NOTE: include filters here to fix VoxelGrid / CropBox / PassThrough etc.
find_package(PCL REQUIRED COMPONENTS common kdtree filters)
find_package(pcl_conversions REQUIRED)

find_package(rosidl_default_generators REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Project-wide include dirs: public headers + generated msg/srv headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
)

# ---------------------------------------------------------------------------
# Messages / Services
# ---------------------------------------------------------------------------
set(MSG_FILES
  "msg/CanopyDensityGrid.msg"
)

set(SRV_FILES
  "srv/GetCanopyDensityGrid.srv"
  "srv/GetClosestGrid.srv"
  "srv/GetGlobalNormal.srv"
  "srv/GetCloudBounds.srv"
  "srv/GetTopCanopyPoint.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# C++ typesupport target for our own msgs/srvs
rosidl_get_typesupport_target(rosidl_typesupport_cpp_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

# Custom target name for the C++ generator
set(MSG_DEP_TARGET ${PROJECT_NAME}__rosidl_generator_cpp)

# ---------------------------------------------------------------------------
# Filters
# ---------------------------------------------------------------------------

# Crop box filter as a component + standalone node
add_library(crop_box_filter SHARED
  src/filters/crop_box_filter.cpp
)
target_include_directories(crop_box_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(crop_box_filter
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  visualization_msgs
)
target_link_libraries(crop_box_filter ${PCL_LIBRARIES})
rclcpp_components_register_nodes(crop_box_filter "futuraps::CropBoxFilterNode")

add_executable(crop_box_filter_node
  src/filters/crop_box_filter_node.cpp
)
target_link_libraries(crop_box_filter_node
  crop_box_filter
)
ament_target_dependencies(crop_box_filter_node
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  visualization_msgs
)

# Local map filter as a component + standalone node
add_library(local_map_filter SHARED
  src/filters/local_map_filter.cpp
)
target_include_directories(local_map_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(local_map_filter
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)
target_link_libraries(local_map_filter ${PCL_LIBRARIES})
rclcpp_components_register_nodes(local_map_filter "futuraps::LocalMapFilterNode")

add_executable(local_map_filter_node
  src/filters/local_map_filter_node.cpp
)
target_link_libraries(local_map_filter_node
  local_map_filter
)
ament_target_dependencies(local_map_filter_node
  rclcpp
  sensor_msgs
)

# ---------------------------------------------------------------------------
# Closest points library + nodes
# ---------------------------------------------------------------------------

# Utility library with closest grid functionality
add_library(closest_grid_utils SHARED
  src/closest_points/closest_grid.cpp
)
target_include_directories(closest_grid_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(closest_grid_utils PRIVATE ${PCL_DEFINITIONS})
target_link_libraries(closest_grid_utils ${PCL_LIBRARIES})

# Closest grid server (offers GetClosestGrid)
add_executable(closest_grid_server
  src/closest_points/closest_grid_server.cpp
)
ament_target_dependencies(closest_grid_server
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
  geometry_msgs
)
target_link_libraries(closest_grid_server
  closest_grid_utils
  Eigen3::Eigen
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(closest_grid_server ${MSG_DEP_TARGET})

# Grid visualization node (calls GetClosestGrid)
add_executable(closest_point_visualizer
  src/closest_points/closest_point_visualizer.cpp
)
ament_target_dependencies(closest_point_visualizer
  rclcpp
  visualization_msgs
)
target_link_libraries(closest_point_visualizer
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(closest_point_visualizer ${MSG_DEP_TARGET})

# Cloud bounds server (offers GetCloudBounds)
add_executable(cloud_bounds_server
  src/closest_points/cloud_bounds_server.cpp
)
ament_target_dependencies(cloud_bounds_server
  rclcpp
  sensor_msgs
)
target_link_libraries(cloud_bounds_server
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(cloud_bounds_server ${MSG_DEP_TARGET})

add_executable(top_canopy_point_server
  src/closest_points/top_canopy_point_server.cpp
)

ament_target_dependencies(top_canopy_point_server
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  tf2_eigen
)

target_link_libraries(top_canopy_point_server
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${rosidl_typesupport_cpp_target}
)

add_dependencies(top_canopy_point_server ${MSG_DEP_TARGET})

# ---------------------------------------------------------------------------
# Canopy density
# ---------------------------------------------------------------------------

add_executable(canopy_density_server
  src/canopy_density/canopy_density_server.cpp
)
ament_target_dependencies(canopy_density_server
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)
target_include_directories(canopy_density_server PUBLIC
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(canopy_density_server
  ${PCL_LIBRARIES}
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(canopy_density_server ${MSG_DEP_TARGET})

add_executable(canopy_density_visualizer
  src/canopy_density/canopy_density_visualizer.cpp
)
ament_target_dependencies(canopy_density_visualizer
  rclcpp
  visualization_msgs
)
target_link_libraries(canopy_density_visualizer
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(canopy_density_visualizer ${MSG_DEP_TARGET})

add_executable(canopy_density_colorizer_node
  src/canopy_density/canopy_density_map_colorizer_node.cpp
)
ament_target_dependencies(canopy_density_colorizer_node
  rclcpp
  sensor_msgs
)
target_link_libraries(canopy_density_colorizer_node
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(canopy_density_colorizer_node ${MSG_DEP_TARGET})

# ---------------------------------------------------------------------------
# Surface normals
# ---------------------------------------------------------------------------

add_executable(surface_normal_server
  src/surface_normals/surface_normal_server.cpp
)
ament_target_dependencies(surface_normal_server
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_eigen
)
target_include_directories(surface_normal_server PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(surface_normal_server
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(surface_normal_server ${MSG_DEP_TARGET})

add_executable(surface_normal_visualizer
  src/surface_normals/surface_normal_visualizer.cpp
)
ament_target_dependencies(surface_normal_visualizer
  rclcpp
  visualization_msgs
)
target_link_libraries(surface_normal_visualizer
  ${rosidl_typesupport_cpp_target}
)
add_dependencies(surface_normal_visualizer ${MSG_DEP_TARGET})

# ---------------------------------------------------------------------------
# Installation
# ---------------------------------------------------------------------------

install(TARGETS
  # Libraries
  crop_box_filter
  local_map_filter
  closest_grid_utils

  # Executables
  crop_box_filter_node
  local_map_filter_node
  closest_grid_server
  closest_point_visualizer
  cloud_bounds_server
  top_canopy_point_server
  canopy_density_server
  canopy_density_visualizer
  canopy_density_colorizer_node
  surface_normal_server
  surface_normal_visualizer

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  rviz
  resource
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Exports
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(
  closest_grid_utils
  crop_box_filter
  local_map_filter
)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_eigen
  pcl_conversions
  rosidl_default_runtime
)

# ---------------------------------------------------------------------------
# Finalize
# ---------------------------------------------------------------------------
ament_package()
