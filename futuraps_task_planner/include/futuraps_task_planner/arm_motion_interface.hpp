#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "futuraps_task_planner/types.hpp"

namespace futuraps_task_planner
{

class ArmMotionInterface
{
public:
  explicit ArmMotionInterface(rclcpp::Node * node);
  ~ArmMotionInterface();

  void configure(const ArmMotionConfig & config);

  bool initialize();
  bool isInitialized() const;

  bool waitForStateUpdate(double timeout_s) const;
  bool hasCurrentState() const;
  bool syncStartStateToCurrentState();

  bool getCurrentPose(geometry_msgs::msg::PoseStamped & pose_stamped) const;

  bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose> & waypoints);
  bool executePlannedPath();

  bool moveHome();

  bool moveToPoseCartesian(
    const geometry_msgs::msg::Pose & target_pose,
    double eef_step = 0.005,
    double jump_threshold = 0.0);

  bool moveToPoseSeeded(
    const geometry_msgs::msg::Pose & target_pose,
    const std::string & ee_link = "",
    bool allow_approximate_ik = false);

private:
  rclcpp::Node * node_{nullptr};
  ArmMotionConfig config_{};

  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
  std::thread moveit_spin_thread_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit_msgs::msg::RobotTrajectory planned_trajectory_;

  bool initialized_{false};
};

}  // namespace futuraps_task_planner