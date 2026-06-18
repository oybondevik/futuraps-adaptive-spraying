#pragma once

#include <vector>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/time.hpp"
#include <Eigen/Core>


namespace futuraps_task_planner
{
struct SprayWaypoint
{
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3d tangent;
  double s{0.0};
  double v_path{0.0};
};

struct SprayPath
{
  std::vector<SprayWaypoint> waypoints;
  std::string frame_id;
  rclcpp::Time stamp;
};

} // namespace futuraps_task_planner