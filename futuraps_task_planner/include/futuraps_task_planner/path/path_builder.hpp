#pragma once

#include <cstddef>

#include <rclcpp/rclcpp.hpp>

#include "futuraps_task_planner/path/path_types.hpp"
#include "futuraps_task_planner/perception_client.hpp"
#include "futuraps_task_planner/types.hpp"
#include "futuraps_task_planner/path/spline.h"
#include "futuraps_task_planner/path/reachability_filter.hpp"

namespace futuraps_task_planner
{

class PathBuilder
{
public:
  PathBuilder() = default;

  void configure(const HorizontalPathConfig& config);

  void setReachabilityFilter(std::shared_ptr<ReachabilityFilter> filter);

  SprayPath buildPath(const PerceptionResult& perception) const;

private:
  HorizontalPathConfig config_;

  std::vector<geometry_msgs::msg::Pose> extractWaypoints(
    const PerceptionResult& perception) const;

  SprayPath makeSprayPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) const;

  std::shared_ptr<ReachabilityFilter> reachability_filter_;

  double max_inward_angle_deg_{25.0};
  int spray_side_{1}; // +1 or -1

  double sample_spacing_{0.02};

  mutable size_t debug_segment_index_{0};

  rclcpp::Logger logger_{rclcpp::get_logger("PathBuilder")};
};
} // namespace futuraps_task_planner
