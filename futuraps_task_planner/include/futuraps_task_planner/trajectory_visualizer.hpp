#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>

namespace futuraps_task_planner
{

class TrajectoryVisualizer
{
public:
  explicit TrajectoryVisualizer(rclcpp::Node * node);

  void configure(
    const std::string & fixed_frame,
    const std::string & ee_frame);

  void reset();

  void publishGoalPoints(const std::vector<geometry_msgs::msg::Pose> & waypoints);

  void publishDesiredPath(const std::vector<geometry_msgs::msg::Pose> & waypoints);

  bool appendActualPoseFromTF(
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer);

  void publishDesiredPose(const geometry_msgs::msg::Pose & pose);

  void appendActualPose(const geometry_msgs::msg::Pose & pose);

private:
  geometry_msgs::msg::PoseStamped makePoseStamped(
    const geometry_msgs::msg::Pose & pose) const;

  rclcpp::Node * node_{nullptr};

  std::string fixed_frame_{"platform_link"};
  std::string ee_frame_{"tool0"};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actual_pose_pub_;

  nav_msgs::msg::Path desired_path_msg_;
  nav_msgs::msg::Path actual_path_msg_;
};

}  // namespace futuraps_task_planner