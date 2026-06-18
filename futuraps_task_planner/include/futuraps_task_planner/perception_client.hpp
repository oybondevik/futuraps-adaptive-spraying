#pragma once

#include <future>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>

#include "futuraps_perception/srv/get_closest_grid.hpp"
#include "futuraps_perception/srv/get_global_normal.hpp"
#include "futuraps_perception/srv/get_top_canopy_point.hpp"
#include "futuraps_task_planner/types.hpp"

namespace futuraps_task_planner
{

class PerceptionClient
{
public:
  explicit PerceptionClient(rclcpp::Node* node);

  void configure(const PerceptionConfig& config);

  bool sendRequest();
  bool checkReady();

  PerceptionResult getResult() const;
  void clearResult();

private:
  rclcpp::Node* node_;
  PerceptionConfig config_;

  rclcpp::Client<futuraps_perception::srv::GetClosestGrid>::SharedPtr closest_grid_client_;
  rclcpp::Client<futuraps_perception::srv::GetGlobalNormal>::SharedPtr global_normal_client_;
  rclcpp::Client<futuraps_perception::srv::GetTopCanopyPoint>::SharedPtr top_point_client_;

  using ClosestFuture = rclcpp::Client<futuraps_perception::srv::GetClosestGrid>::SharedFuture;
  using NormalFuture  = rclcpp::Client<futuraps_perception::srv::GetGlobalNormal>::SharedFuture;
  using TopPointFuture = rclcpp::Client<futuraps_perception::srv::GetTopCanopyPoint>::SharedFuture;

  ClosestFuture closest_future_;
  std::vector<NormalFuture> normal_futures_;
  TopPointFuture top_point_future_;

  bool closest_pending_{false};
  bool normal_pending_{false};
  bool top_point_pending_{false};

  rclcpp::Time request_start_time_{0, 0, RCL_ROS_TIME};

  PerceptionResult latest_result_;

  void createClientsIfNeeded();
};

}  // namespace futuraps_task_planner