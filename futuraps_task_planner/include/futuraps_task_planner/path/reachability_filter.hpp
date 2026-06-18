#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>

namespace futuraps_task_planner
{

class ReachabilityFilter
{
public:
  ReachabilityFilter(
    const rclcpp::Node::SharedPtr& node,
    const std::string& planning_group,
    const std::string& tip_link);

  bool initialize();

  std::vector<geometry_msgs::msg::Pose> filterReachableSegment(
    const std::vector<geometry_msgs::msg::Pose>& waypoints) const;

private:
  struct CandidateResult
  {
    CandidateResult(
      const geometry_msgs::msg::Pose& pose_in,
      const moveit::core::RobotState& state_in,
      bool repaired_workspace_in,
      bool relaxed_orientation_in)
    : pose(pose_in),
      state(state_in),
      repaired_workspace(repaired_workspace_in),
      relaxed_orientation(relaxed_orientation_in)
    {
    }

    geometry_msgs::msg::Pose pose;
    moveit::core::RobotState state;
    bool repaired_workspace{false};
    bool relaxed_orientation{false};
  };

  void loadParameters();

  std::optional<CandidateResult> repairAndCheckPose(
    const geometry_msgs::msg::Pose& desired_pose,
    const moveit::core::RobotState& seed_state,
    std::size_t waypoint_index,
    bool enforce_joint_jump_check) const;

  bool checkPoseWithSeed(
    const geometry_msgs::msg::Pose& pose,
    const moveit::core::RobotState& seed_state,
    moveit::core::RobotState& solved_state,
    std::size_t waypoint_index,
    const std::string& candidate_name) const;

  bool isStateCollisionFree(
    const moveit::core::RobotState& state,
    std::size_t waypoint_index,
    const std::string& candidate_name) const;

  bool jointJumpAcceptable(
    const moveit::core::RobotState& previous_state,
    const moveit::core::RobotState& next_state) const;

  std::vector<geometry_msgs::msg::Pose> makePoseCandidates(
    const geometry_msgs::msg::Pose& desired_pose) const;

  geometry_msgs::msg::Pose repairWorkspaceEnvelope(
    const geometry_msgs::msg::Pose& pose,
    bool& was_repaired) const;

  double minAllowedZForY(double y) const;

  geometry_msgs::msg::Pose rotatePoseAboutToolAxis(
    const geometry_msgs::msg::Pose& pose,
    double angle_rad) const;

  bool isInsidePlatformKeepout(
    const geometry_msgs::msg::Pose& pose) const;

  double platformSafeY(
    double y,
    double offset) const;

  bool poseDiffers(
    const geometry_msgs::msg::Pose& a,
    const geometry_msgs::msg::Pose& b) const;

  rclcpp::Node::SharedPtr node_;

  std::string planning_group_;
  std::string tip_link_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  const moveit::core::JointModelGroup* joint_model_group_{nullptr};

  double ik_timeout_{0.05};

  // Workspace repair.
  bool workspace_repair_enabled_{true};

  double yz_start_y_{0.90};
  double yz_end_y_{1.10};
  double z_min_at_start_y_{0.05};
  double z_min_at_end_y_{0.30};

  // Low-Z lateral limit. This pulls the TCP inward when it is too low.
  bool low_z_y_limit_enabled_{true};
  double low_z_y_limit_max_z_{0.50};
  double low_z_y_limit_max_abs_y_{1.10};

  // Wide clamps.
  double min_y_{0.0};
  double max_y_{2.0};
  double min_z_{0.0};
  double max_z_{2.0};

  // Existing orientation relaxation around the desired pose.
  bool orientation_relaxation_enabled_{true};

  std::vector<double> roll_relaxation_angles_rad_{
    0.0,
    0.17453292519943295,
    -0.17453292519943295,
    0.3490658503988659,
    -0.3490658503988659,
    0.5235987755982988,
    -0.5235987755982988,
    0.7853981633974483,
    -0.7853981633974483
  };

  // Safe/non-folding orientation repair.
  bool safe_orientation_enabled_{false};
  geometry_msgs::msg::Quaternion safe_orientation_;

  std::vector<double> safe_orientation_roll_offsets_rad_{
    0.0,
    0.17453292519943295,
    -0.17453292519943295,
    0.3490658503988659,
    -0.3490658503988659,
    0.5235987755982988,
    -0.5235987755982988
  };

  // Simple y repair around robot-base height.
  bool base_clearance_repair_enabled_{true};
  double base_clearance_z_min_{0.25};
  double base_clearance_z_max_{1.60};
  double base_clearance_min_y_{0.68};

  std::vector<double> base_clearance_y_offsets_m_{
    0.0,
    0.05,
    0.10,
    0.15,
    0.20
  };

  // Platform keepout.
  // Conservative TCP y-z keepout around platform_link.
  // This avoids targets that tend to make forearm_link / boom_link collide with platform_link.
  bool platform_keepout_enabled_{true};

  // From robot_description:
  // platform_width = 0.824 -> half width = 0.412
  double platform_half_width_y_{0.412};

  // Extra clearance outside the platform side.
  double platform_y_margin_{0.25};

  // Apply keepout over a tall z range because arm links can collide with the platform
  // even when the TCP is above the actual platform top.
  double platform_z_min_{0.0};
  double platform_z_max_{1.60};

  std::vector<double> platform_y_offsets_m_{
    0.0,
    0.05,
    0.10,
    0.15,
    0.20,
    0.25
  };

  // Joint continuity.
  bool joint_jump_check_enabled_{true};
  double max_joint_step_rad_{0.35};

  // Debug logging.
  bool log_rejected_candidates_{true};
  bool log_collision_contacts_{true};
};

}  // namespace futuraps_task_planner
