#pragma once

#include <fstream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <rclcpp/rclcpp.hpp>

#include "futuraps_task_planner/types.hpp"
#include "futuraps_task_planner/path/path_types.hpp"

namespace futuraps_task_planner
{
class PathFollower
{
public:
  explicit PathFollower(rclcpp::Node * node);

  void configure(const PathFollowerConfig & config);

  bool startPathFollowing(const SprayPath& spray_path);
    
  void update();

  void updateMoveToPose();

  void stop();

  bool isFinished() const; 

  geometry_msgs::msg::Pose getCurrentPose() const;

  geometry_msgs::msg::Pose transformPoseToCommandFrame(
    const geometry_msgs::msg::Pose & pose_in_path_frame) const;

private:
  void resetSpeedDebugLog();

  void logSpeedDebug(
    const geometry_msgs::msg::Pose& current_pose,
    size_t current_index,
    size_t target_index,
    double s_target,
    double s_stroke,
    double v_path,
    double err_norm,
    double vx,
    double vy,
    double vz);

  rclcpp::Node * node_{nullptr};

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  SprayPath active_path_;
  size_t current_index_{0};
  bool active_{false};
  bool finished_{false};

  double pos_gain_{1.5};
  double ori_gain_{3.0};
  double max_linear_speed_{1.5};
  double max_angular_speed_{1.5};
  double pos_tolerance_{0.05};
  double ori_tolerance_{0.30};
  int lookahead_points_{3}; // 6 cm at 2 cm sampling distance

  double v_min_{0.25};  // m/s
  double v_max_{0.40};  // m/s

  std::string path_frame_{"platform_link"};
  std::string command_frame_{"base_link"};
  std::string ee_frame_{"tool0"};

  std::ofstream speed_debug_file_;
  bool speed_debug_have_last_pose_{false};
  geometry_msgs::msg::Pose speed_debug_last_pose_;
  rclcpp::Time speed_debug_last_time_;

};

} // namespace futuraps_task_planner
