#include "futuraps_task_planner/path/reachability_filter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/collision_detection/collision_common.h>

namespace futuraps_task_planner
{

namespace
{

double degToRad(double deg)
{
  return deg * M_PI / 180.0;
}

std::vector<double> degVectorToRadVector(const std::vector<double>& deg_values)
{
  std::vector<double> rad_values;
  rad_values.reserve(deg_values.size());

  for (const double deg : deg_values) {
    rad_values.push_back(degToRad(deg));
  }

  return rad_values;
}

bool appendUniqueDouble(std::vector<double>& values, double value, double tolerance = 1e-6)
{
  for (const double existing : values) {
    if (std::abs(existing - value) < tolerance) {
      return false;
    }
  }

  values.push_back(value);
  return true;
}

}  // namespace

ReachabilityFilter::ReachabilityFilter(
  const rclcpp::Node::SharedPtr& node,
  const std::string& planning_group,
  const std::string& tip_link)
: node_(node),
  planning_group_(planning_group),
  tip_link_(tip_link)
{
  safe_orientation_.x = 0.0;
  safe_orientation_.y = 0.0;
  safe_orientation_.z = 0.0;
  safe_orientation_.w = 1.0;
}

bool ReachabilityFilter::initialize()
{
  loadParameters();

  robot_model_loader_ =
    std::make_shared<robot_model_loader::RobotModelLoader>(
      node_,
      "robot_description");

  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_) {
    RCLCPP_ERROR(node_->get_logger(), "ReachabilityFilter: failed to load robot model");
    return false;
  }

  joint_model_group_ = robot_model_->getJointModelGroup(planning_group_);

  if (!joint_model_group_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ReachabilityFilter: planning group '%s' not found",
      planning_group_.c_str());
    return false;
  }

  planning_scene_monitor_ =
    std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_,
      "robot_description");

  if (!planning_scene_monitor_->getPlanningScene()) {
    RCLCPP_ERROR(node_->get_logger(), "ReachabilityFilter: failed to create planning scene");
    return false;
  }

  planning_scene_monitor_->startStateMonitor();
  planning_scene_monitor_->startSceneMonitor();
  planning_scene_monitor_->startWorldGeometryMonitor();

  RCLCPP_INFO(
    node_->get_logger(),
    "ReachabilityFilter initialized with group='%s', tip_link='%s'",
    planning_group_.c_str(),
    tip_link_.c_str());

  RCLCPP_INFO(
    node_->get_logger(),
    "ReachabilityFilter config: safe_orientation=%s, low_z_y_limit=%s(z<%.3f -> |y|<=%.3f), base_clearance=%s, base_min_abs_y=%.3f, base_z=[%.3f, %.3f], platform_keepout=%s, platform_safe_abs_y=%.3f, platform_z=[%.3f, %.3f], max_joint_step=%.3f",
    safe_orientation_enabled_ ? "true" : "false",
    low_z_y_limit_enabled_ ? "true" : "false",
    low_z_y_limit_max_z_,
    low_z_y_limit_max_abs_y_,
    base_clearance_repair_enabled_ ? "true" : "false",
    base_clearance_min_y_,
    base_clearance_z_min_,
    base_clearance_z_max_,
    platform_keepout_enabled_ ? "true" : "false",
    platform_half_width_y_ + platform_y_margin_,
    platform_z_min_,
    platform_z_max_,
    max_joint_step_rad_);

  return true;
}

void ReachabilityFilter::loadParameters()
{
  const std::string prefix = "reachability_filter.";

  if (!node_->has_parameter(prefix + "ik_timeout")) {
    node_->declare_parameter<double>(prefix + "ik_timeout", ik_timeout_);
  }
  node_->get_parameter(prefix + "ik_timeout", ik_timeout_);

  if (!node_->has_parameter(prefix + "workspace_repair_enabled")) {
    node_->declare_parameter<bool>(prefix + "workspace_repair_enabled", workspace_repair_enabled_);
  }
  node_->get_parameter(prefix + "workspace_repair_enabled", workspace_repair_enabled_);

  if (!node_->has_parameter(prefix + "yz_start_y")) {
    node_->declare_parameter<double>(prefix + "yz_start_y", yz_start_y_);
  }
  node_->get_parameter(prefix + "yz_start_y", yz_start_y_);

  if (!node_->has_parameter(prefix + "yz_end_y")) {
    node_->declare_parameter<double>(prefix + "yz_end_y", yz_end_y_);
  }
  node_->get_parameter(prefix + "yz_end_y", yz_end_y_);

  if (!node_->has_parameter(prefix + "z_min_at_start_y")) {
    node_->declare_parameter<double>(prefix + "z_min_at_start_y", z_min_at_start_y_);
  }
  node_->get_parameter(prefix + "z_min_at_start_y", z_min_at_start_y_);

  if (!node_->has_parameter(prefix + "z_min_at_end_y")) {
    node_->declare_parameter<double>(prefix + "z_min_at_end_y", z_min_at_end_y_);
  }
  node_->get_parameter(prefix + "z_min_at_end_y", z_min_at_end_y_);

  if (!node_->has_parameter(prefix + "low_z_y_limit_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "low_z_y_limit_enabled",
      low_z_y_limit_enabled_);
  }
  node_->get_parameter(
    prefix + "low_z_y_limit_enabled",
    low_z_y_limit_enabled_);

  if (!node_->has_parameter(prefix + "low_z_y_limit_max_z")) {
    node_->declare_parameter<double>(
      prefix + "low_z_y_limit_max_z",
      low_z_y_limit_max_z_);
  }
  node_->get_parameter(
    prefix + "low_z_y_limit_max_z",
    low_z_y_limit_max_z_);

  if (!node_->has_parameter(prefix + "low_z_y_limit_max_abs_y")) {
    node_->declare_parameter<double>(
      prefix + "low_z_y_limit_max_abs_y",
      low_z_y_limit_max_abs_y_);
  }
  node_->get_parameter(
    prefix + "low_z_y_limit_max_abs_y",
    low_z_y_limit_max_abs_y_);

  if (!node_->has_parameter(prefix + "min_y")) {
    node_->declare_parameter<double>(prefix + "min_y", min_y_);
  }
  node_->get_parameter(prefix + "min_y", min_y_);

  if (!node_->has_parameter(prefix + "max_y")) {
    node_->declare_parameter<double>(prefix + "max_y", max_y_);
  }
  node_->get_parameter(prefix + "max_y", max_y_);

  if (!node_->has_parameter(prefix + "min_z")) {
    node_->declare_parameter<double>(prefix + "min_z", min_z_);
  }
  node_->get_parameter(prefix + "min_z", min_z_);

  if (!node_->has_parameter(prefix + "max_z")) {
    node_->declare_parameter<double>(prefix + "max_z", max_z_);
  }
  node_->get_parameter(prefix + "max_z", max_z_);

  if (!node_->has_parameter(prefix + "orientation_relaxation_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "orientation_relaxation_enabled",
      orientation_relaxation_enabled_);
  }
  node_->get_parameter(
    prefix + "orientation_relaxation_enabled",
    orientation_relaxation_enabled_);

  std::vector<double> roll_relaxation_angles_deg{
    0.0,
    10.0,
    -10.0,
    20.0,
    -20.0,
    30.0,
    -30.0,
    45.0,
    -45.0
  };

  if (!node_->has_parameter(prefix + "roll_relaxation_angles_deg")) {
    node_->declare_parameter<std::vector<double>>(
      prefix + "roll_relaxation_angles_deg",
      roll_relaxation_angles_deg);
  }
  node_->get_parameter(prefix + "roll_relaxation_angles_deg", roll_relaxation_angles_deg);
  roll_relaxation_angles_rad_ = degVectorToRadVector(roll_relaxation_angles_deg);

  if (!node_->has_parameter(prefix + "safe_orientation_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "safe_orientation_enabled",
      safe_orientation_enabled_);
  }
  node_->get_parameter(prefix + "safe_orientation_enabled", safe_orientation_enabled_);

  std::vector<double> safe_orientation_xyzw{
    safe_orientation_.x,
    safe_orientation_.y,
    safe_orientation_.z,
    safe_orientation_.w
  };

  if (!node_->has_parameter(prefix + "safe_orientation_xyzw")) {
    node_->declare_parameter<std::vector<double>>(
      prefix + "safe_orientation_xyzw",
      safe_orientation_xyzw);
  }
  node_->get_parameter(prefix + "safe_orientation_xyzw", safe_orientation_xyzw);

  if (safe_orientation_xyzw.size() == 4) {
    tf2::Quaternion q(
      safe_orientation_xyzw[0],
      safe_orientation_xyzw[1],
      safe_orientation_xyzw[2],
      safe_orientation_xyzw[3]);

    if (q.length2() > 1e-12) {
      q.normalize();
      safe_orientation_ = tf2::toMsg(q);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "ReachabilityFilter: safe_orientation_xyzw has near-zero norm. Disabling safe orientation repair.");
      safe_orientation_enabled_ = false;
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "ReachabilityFilter: safe_orientation_xyzw must contain exactly 4 values [x, y, z, w]. Disabling safe orientation repair.");
    safe_orientation_enabled_ = false;
  }

  std::vector<double> safe_orientation_roll_offsets_deg{
    0.0,
    10.0,
    -10.0,
    20.0,
    -20.0,
    30.0,
    -30.0
  };

  if (!node_->has_parameter(prefix + "safe_orientation_roll_offsets_deg")) {
    node_->declare_parameter<std::vector<double>>(
      prefix + "safe_orientation_roll_offsets_deg",
      safe_orientation_roll_offsets_deg);
  }
  node_->get_parameter(
    prefix + "safe_orientation_roll_offsets_deg",
    safe_orientation_roll_offsets_deg);
  safe_orientation_roll_offsets_rad_ =
    degVectorToRadVector(safe_orientation_roll_offsets_deg);

  if (!node_->has_parameter(prefix + "base_clearance_repair_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "base_clearance_repair_enabled",
      base_clearance_repair_enabled_);
  }
  node_->get_parameter(
    prefix + "base_clearance_repair_enabled",
    base_clearance_repair_enabled_);

  if (!node_->has_parameter(prefix + "base_clearance_z_min")) {
    node_->declare_parameter<double>(
      prefix + "base_clearance_z_min",
      base_clearance_z_min_);
  }
  node_->get_parameter(prefix + "base_clearance_z_min", base_clearance_z_min_);

  if (!node_->has_parameter(prefix + "base_clearance_z_max")) {
    node_->declare_parameter<double>(
      prefix + "base_clearance_z_max",
      base_clearance_z_max_);
  }
  node_->get_parameter(prefix + "base_clearance_z_max", base_clearance_z_max_);

  if (!node_->has_parameter(prefix + "base_clearance_min_y")) {
    node_->declare_parameter<double>(
      prefix + "base_clearance_min_y",
      base_clearance_min_y_);
  }
  node_->get_parameter(prefix + "base_clearance_min_y", base_clearance_min_y_);

  if (!node_->has_parameter(prefix + "base_clearance_y_offsets_m")) {
    node_->declare_parameter<std::vector<double>>(
      prefix + "base_clearance_y_offsets_m",
      base_clearance_y_offsets_m_);
  }
  node_->get_parameter(
    prefix + "base_clearance_y_offsets_m",
    base_clearance_y_offsets_m_);

  if (!node_->has_parameter(prefix + "platform_keepout_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "platform_keepout_enabled",
      platform_keepout_enabled_);
  }
  node_->get_parameter(
    prefix + "platform_keepout_enabled",
    platform_keepout_enabled_);

  if (!node_->has_parameter(prefix + "platform_half_width_y")) {
    node_->declare_parameter<double>(
      prefix + "platform_half_width_y",
      platform_half_width_y_);
  }
  node_->get_parameter(
    prefix + "platform_half_width_y",
    platform_half_width_y_);

  if (!node_->has_parameter(prefix + "platform_y_margin")) {
    node_->declare_parameter<double>(
      prefix + "platform_y_margin",
      platform_y_margin_);
  }
  node_->get_parameter(
    prefix + "platform_y_margin",
    platform_y_margin_);

  if (!node_->has_parameter(prefix + "platform_z_min")) {
    node_->declare_parameter<double>(
      prefix + "platform_z_min",
      platform_z_min_);
  }
  node_->get_parameter(
    prefix + "platform_z_min",
    platform_z_min_);

  if (!node_->has_parameter(prefix + "platform_z_max")) {
    node_->declare_parameter<double>(
      prefix + "platform_z_max",
      platform_z_max_);
  }
  node_->get_parameter(
    prefix + "platform_z_max",
    platform_z_max_);

  if (!node_->has_parameter(prefix + "platform_y_offsets_m")) {
    node_->declare_parameter<std::vector<double>>(
      prefix + "platform_y_offsets_m",
      platform_y_offsets_m_);
  }
  node_->get_parameter(
    prefix + "platform_y_offsets_m",
    platform_y_offsets_m_);

  if (!node_->has_parameter(prefix + "joint_jump_check_enabled")) {
    node_->declare_parameter<bool>(
      prefix + "joint_jump_check_enabled",
      joint_jump_check_enabled_);
  }
  node_->get_parameter(prefix + "joint_jump_check_enabled", joint_jump_check_enabled_);

  if (!node_->has_parameter(prefix + "max_joint_step_rad")) {
    node_->declare_parameter<double>(
      prefix + "max_joint_step_rad",
      max_joint_step_rad_);
  }
  node_->get_parameter(prefix + "max_joint_step_rad", max_joint_step_rad_);

  if (!node_->has_parameter(prefix + "log_rejected_candidates")) {
    node_->declare_parameter<bool>(
      prefix + "log_rejected_candidates",
      log_rejected_candidates_);
  }
  node_->get_parameter(prefix + "log_rejected_candidates", log_rejected_candidates_);

  if (!node_->has_parameter(prefix + "log_collision_contacts")) {
    node_->declare_parameter<bool>(
      prefix + "log_collision_contacts",
      log_collision_contacts_);
  }
  node_->get_parameter(prefix + "log_collision_contacts", log_collision_contacts_);
}

std::vector<geometry_msgs::msg::Pose> ReachabilityFilter::filterReachableSegment(
  const std::vector<geometry_msgs::msg::Pose>& waypoints) const
{
  std::vector<geometry_msgs::msg::Pose> reachable;
  reachable.reserve(waypoints.size());

  if (!robot_model_ || !joint_model_group_ || !planning_scene_monitor_) {
    RCLCPP_ERROR(node_->get_logger(), "ReachabilityFilter: not initialized");
    return reachable;
  }

  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);

  if (!scene) {
    RCLCPP_ERROR(node_->get_logger(), "ReachabilityFilter: no planning scene");
    return reachable;
  }

  moveit::core::RobotState seed_state(scene->getCurrentState());
  seed_state.update();

  bool have_previous_accepted_state = false;

  std::size_t rejected_count = 0;
  std::size_t repaired_count = 0;
  std::size_t relaxed_count = 0;

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const bool enforce_joint_jump_check =
      have_previous_accepted_state && joint_jump_check_enabled_;

    const auto result = repairAndCheckPose(
      waypoints[i],
      seed_state,
      i,
      enforce_joint_jump_check);

    if (!result) {
      ++rejected_count;

      RCLCPP_WARN(
        node_->get_logger(),
        "ReachabilityFilter: rejected waypoint %zu at y=%.3f z=%.3f",
        i,
        waypoints[i].position.y,
        waypoints[i].position.z);

      continue;
    }

    reachable.push_back(result->pose);

    seed_state = result->state;
    have_previous_accepted_state = true;

    if (result->repaired_workspace) {
      ++repaired_count;
    }

    if (result->relaxed_orientation) {
      ++relaxed_count;
    }
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "ReachabilityFilter: kept %zu/%zu waypoints, rejected=%zu, workspace_repaired=%zu, orientation_or_pose_repaired=%zu",
    reachable.size(),
    waypoints.size(),
    rejected_count,
    repaired_count,
    relaxed_count);

  return reachable;
}

std::optional<ReachabilityFilter::CandidateResult> ReachabilityFilter::repairAndCheckPose(
  const geometry_msgs::msg::Pose& desired_pose,
  const moveit::core::RobotState& seed_state,
  std::size_t waypoint_index,
  bool enforce_joint_jump_check) const
{
  bool workspace_repaired = false;
  const auto workspace_pose = repairWorkspaceEnvelope(desired_pose, workspace_repaired);

  const auto candidates = makePoseCandidates(workspace_pose);

  for (std::size_t candidate_idx = 0; candidate_idx < candidates.size(); ++candidate_idx) {
    const auto& candidate_pose = candidates[candidate_idx];

    moveit::core::RobotState solved_state(seed_state);

    std::ostringstream candidate_name;
    candidate_name << "candidate_" << candidate_idx;

    if (!checkPoseWithSeed(
          candidate_pose,
          seed_state,
          solved_state,
          waypoint_index,
          candidate_name.str()))
    {
      continue;
    }

    if (enforce_joint_jump_check &&
        !jointJumpAcceptable(seed_state, solved_state))
    {
      if (log_rejected_candidates_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "ReachabilityFilter: wp=%zu %s rejected due to joint jump",
          waypoint_index,
          candidate_name.str().c_str());
      }

      continue;
    }

    const bool orientation_or_pose_repaired =
      poseDiffers(candidate_pose, workspace_pose);

    if (workspace_repaired || orientation_or_pose_repaired) {
      RCLCPP_INFO(
        node_->get_logger(),
        "ReachabilityFilter: wp=%zu accepted after repair: workspace=%s orientation_or_pose_repaired=%s candidate=%zu y=%.3f z=%.3f",
        waypoint_index,
        workspace_repaired ? "true" : "false",
        orientation_or_pose_repaired ? "true" : "false",
        candidate_idx,
        candidate_pose.position.y,
        candidate_pose.position.z);
    }

    return CandidateResult(
      candidate_pose,
      solved_state,
      workspace_repaired,
      orientation_or_pose_repaired);
  }

  return std::nullopt;
}

bool ReachabilityFilter::checkPoseWithSeed(
  const geometry_msgs::msg::Pose& pose,
  const moveit::core::RobotState& seed_state,
  moveit::core::RobotState& solved_state,
  std::size_t waypoint_index,
  const std::string& candidate_name) const
{
  solved_state = seed_state;

  const bool ik_ok = solved_state.setFromIK(
    joint_model_group_,
    pose,
    tip_link_,
    ik_timeout_);

  if (!ik_ok) {
    if (log_rejected_candidates_) {
      RCLCPP_WARN(
        node_->get_logger(),
        "ReachabilityFilter: wp=%zu %s rejected: IK failed",
        waypoint_index,
        candidate_name.c_str());
    }

    return false;
  }

  solved_state.update();

  if (!isStateCollisionFree(solved_state, waypoint_index, candidate_name)) {
    return false;
  }

  return true;
}

bool ReachabilityFilter::isStateCollisionFree(
  const moveit::core::RobotState& state,
  std::size_t waypoint_index,
  const std::string& candidate_name) const
{
  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);

  if (!scene) {
    RCLCPP_ERROR(node_->get_logger(), "ReachabilityFilter: no planning scene during collision check");
    return false;
  }

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;

  collision_request.group_name = planning_group_;
  collision_request.contacts = true;
  collision_request.max_contacts = 20;
  collision_request.max_contacts_per_pair = 5;

  scene->checkCollision(
    collision_request,
    collision_result,
    state);

  if (!collision_result.collision) {
    return true;
  }

  if (log_collision_contacts_) {
    for (const auto& contact_pair : collision_result.contacts) {
      const auto& a = contact_pair.first.first;
      const auto& b = contact_pair.first.second;

      RCLCPP_WARN(
        node_->get_logger(),
        "ReachabilityFilter: wp=%zu %s collision pair: %s <-> %s",
        waypoint_index,
        candidate_name.c_str(),
        a.c_str(),
        b.c_str());
    }
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "ReachabilityFilter: wp=%zu %s rejected due to collision",
      waypoint_index,
      candidate_name.c_str());
  }

  return false;
}

bool ReachabilityFilter::jointJumpAcceptable(
  const moveit::core::RobotState& previous_state,
  const moveit::core::RobotState& next_state) const
{
  std::vector<double> q_prev;
  std::vector<double> q_next;

  previous_state.copyJointGroupPositions(joint_model_group_, q_prev);
  next_state.copyJointGroupPositions(joint_model_group_, q_next);

  if (q_prev.size() != q_next.size()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ReachabilityFilter: joint vector size mismatch");
    return false;
  }

  for (std::size_t i = 0; i < q_prev.size(); ++i) {
    const double dq = std::abs(q_next[i] - q_prev[i]);

    if (dq > max_joint_step_rad_) {
      if (log_rejected_candidates_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "ReachabilityFilter: joint jump too large at joint index %zu: dq=%.3f rad, limit=%.3f rad",
          i,
          dq,
          max_joint_step_rad_);
      }

      return false;
    }
  }

  return true;
}

std::vector<geometry_msgs::msg::Pose> ReachabilityFilter::makePoseCandidates(
  const geometry_msgs::msg::Pose& desired_pose) const
{
  std::vector<geometry_msgs::msg::Pose> candidates;
  std::vector<double> y_values;

  const bool inside_platform_keepout =
    isInsidePlatformKeepout(desired_pose);

  const bool near_base_height =
    base_clearance_repair_enabled_ &&
    desired_pose.position.z >= base_clearance_z_min_ &&
    desired_pose.position.z <= base_clearance_z_max_;

  const bool too_close_to_base_in_y =
    near_base_height &&
    std::abs(desired_pose.position.y) < base_clearance_min_y_;

  if (inside_platform_keepout) {
    // Important:
    // Do not try the original y. It is inside the platform keepout zone.
    for (const double offset : platform_y_offsets_m_) {
      appendUniqueDouble(
        y_values,
        platformSafeY(desired_pose.position.y, offset));
    }
  } else if (too_close_to_base_in_y) {
    // Important:
    // Do not try the original y. It is inside the base-clearance danger zone.
    const double sign = (desired_pose.position.y >= 0.0) ? 1.0 : -1.0;

    for (const double offset : base_clearance_y_offsets_m_) {
      const double repaired_abs_y =
        std::max(
          std::abs(desired_pose.position.y) + offset,
          base_clearance_min_y_);

      const double repaired_y =
        std::clamp(
          sign * repaired_abs_y,
          min_y_,
          max_y_);

      appendUniqueDouble(y_values, repaired_y);
    }
  } else {
    appendUniqueDouble(y_values, desired_pose.position.y);
  }

  for (const double y_value : y_values) {
    geometry_msgs::msg::Pose y_pose = desired_pose;
    y_pose.position.y = y_value;

    const bool repaired_y =
      std::abs(y_value - desired_pose.position.y) > 1e-6;

    const bool risky_pose =
      inside_platform_keepout ||
      too_close_to_base_in_y ||
      repaired_y;

    if (risky_pose && safe_orientation_enabled_) {
      // For platform/base-risky poses, prefer safe orientation first.
      geometry_msgs::msg::Pose safe_pose = y_pose;
      safe_pose.orientation = safe_orientation_;

      candidates.push_back(safe_pose);

      for (const double roll_offset : safe_orientation_roll_offsets_rad_) {
        if (std::abs(roll_offset) < 1e-9) {
          continue;
        }

        candidates.push_back(rotatePoseAboutToolAxis(safe_pose, roll_offset));
      }

      // Try the original orientation only after the safe orientations.
      candidates.push_back(y_pose);
    } else {
      // Normal case: original pose first.
      candidates.push_back(y_pose);

      if (safe_orientation_enabled_) {
        geometry_msgs::msg::Pose safe_pose = y_pose;
        safe_pose.orientation = safe_orientation_;

        candidates.push_back(safe_pose);

        for (const double roll_offset : safe_orientation_roll_offsets_rad_) {
          if (std::abs(roll_offset) < 1e-9) {
            continue;
          }

          candidates.push_back(rotatePoseAboutToolAxis(safe_pose, roll_offset));
        }
      }
    }

    if (orientation_relaxation_enabled_) {
      for (const double roll_offset : roll_relaxation_angles_rad_) {
        if (std::abs(roll_offset) < 1e-9) {
          continue;
        }

        candidates.push_back(rotatePoseAboutToolAxis(y_pose, roll_offset));
      }
    }
  }

  return candidates;
}

geometry_msgs::msg::Pose ReachabilityFilter::repairWorkspaceEnvelope(
  const geometry_msgs::msg::Pose& pose,
  bool& was_repaired) const
{
  was_repaired = false;

  if (!workspace_repair_enabled_) {
    return pose;
  }

  geometry_msgs::msg::Pose repaired = pose;

  auto& y = repaired.position.y;
  auto& z = repaired.position.z;

  if (y < min_y_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: workspace repair y %.3f -> %.3f",
      y,
      min_y_);

    y = min_y_;
    was_repaired = true;
  }

  if (y > max_y_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: workspace repair y %.3f -> %.3f",
      y,
      max_y_);

    y = max_y_;
    was_repaired = true;
  }

  if (z < min_z_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: workspace repair z %.3f -> %.3f",
      z,
      min_z_);

    z = min_z_;
    was_repaired = true;
  }

  if (z > max_z_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: workspace repair z %.3f -> %.3f",
      z,
      max_z_);

    z = max_z_;
    was_repaired = true;
  }

  if (
    low_z_y_limit_enabled_ &&
    z < low_z_y_limit_max_z_ &&
    std::abs(y) > low_z_y_limit_max_abs_y_)
  {
    const double repaired_y =
      std::clamp(
        std::copysign(low_z_y_limit_max_abs_y_, y),
        min_y_,
        max_y_);

    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: low-z y repair at z=%.3f: y %.3f -> %.3f",
      z,
      y,
      repaired_y);

    y = repaired_y;
    was_repaired = true;
  }

  const double z_min_for_y = minAllowedZForY(y);

  if (z < z_min_for_y) {
    RCLCPP_INFO(
      node_->get_logger(),
      "ReachabilityFilter: low/far repair at y=%.3f: z %.3f -> %.3f",
      y,
      z,
      z_min_for_y);

    z = z_min_for_y;
    was_repaired = true;
  }

  return repaired;
}

double ReachabilityFilter::minAllowedZForY(double y) const
{
  if (y <= yz_start_y_) {
    return z_min_at_start_y_;
  }

  if (y >= yz_end_y_) {
    return z_min_at_end_y_;
  }

  const double denominator = yz_end_y_ - yz_start_y_;

  if (std::abs(denominator) < 1e-9) {
    return z_min_at_end_y_;
  }

  const double t = (y - yz_start_y_) / denominator;

  return z_min_at_start_y_ +
         t * (z_min_at_end_y_ - z_min_at_start_y_);
}

bool ReachabilityFilter::isInsidePlatformKeepout(
  const geometry_msgs::msg::Pose& pose) const
{
  if (!platform_keepout_enabled_) {
    return false;
  }

  const double z = pose.position.z;
  const double y = pose.position.y;

  if (z < platform_z_min_ || z > platform_z_max_) {
    return false;
  }

  const double required_abs_y =
    platform_half_width_y_ + platform_y_margin_;

  return std::abs(y) < required_abs_y;
}

double ReachabilityFilter::platformSafeY(
  double y,
  double offset) const
{
  const double required_abs_y =
    platform_half_width_y_ + platform_y_margin_ + offset;

  const double sign = (y >= 0.0) ? 1.0 : -1.0;

  return std::clamp(
    sign * required_abs_y,
    min_y_,
    max_y_);
}

bool ReachabilityFilter::poseDiffers(
  const geometry_msgs::msg::Pose& a,
  const geometry_msgs::msg::Pose& b) const
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;

  const double position_diff_sq = dx * dx + dy * dy + dz * dz;

  if (position_diff_sq > 1e-10) {
    return true;
  }

  tf2::Quaternion qa;
  tf2::Quaternion qb;
  tf2::fromMsg(a.orientation, qa);
  tf2::fromMsg(b.orientation, qb);

  if (qa.length2() < 1e-12 || qb.length2() < 1e-12) {
    return true;
  }

  qa.normalize();
  qb.normalize();

  const double dot =
    std::abs(
      qa.x() * qb.x() +
      qa.y() * qb.y() +
      qa.z() * qb.z() +
      qa.w() * qb.w());

  return dot < 0.999999;
}

geometry_msgs::msg::Pose ReachabilityFilter::rotatePoseAboutToolAxis(
  const geometry_msgs::msg::Pose& pose,
  double angle_rad) const
{
  if (std::abs(angle_rad) < 1e-9) {
    return pose;
  }

  tf2::Quaternion q_original;
  tf2::fromMsg(pose.orientation, q_original);
  q_original.normalize();

  // IMPORTANT:
  // This assumes that the spray/nozzle direction is the local X-axis of tip_link/tool0.
  //
  // If your PathBuilder aligns another axis with the spray direction,
  // change this vector.
  //
  // Examples:
  //   local Z:  tf2::Vector3 local_tool_axis(0.0, 0.0, 1.0);
  //   local -Z: tf2::Vector3 local_tool_axis(0.0, 0.0, -1.0);
  //   local Y:  tf2::Vector3 local_tool_axis(0.0, 1.0, 0.0);
  tf2::Vector3 local_tool_axis(1.0, 0.0, 0.0);

  tf2::Vector3 world_tool_axis = tf2::quatRotate(q_original, local_tool_axis);

  if (world_tool_axis.length2() < 1e-12) {
    return pose;
  }

  world_tool_axis.normalize();

  tf2::Quaternion q_roll(world_tool_axis, angle_rad);
  q_roll.normalize();

  tf2::Quaternion q_new = q_roll * q_original;
  q_new.normalize();

  geometry_msgs::msg::Pose rotated = pose;
  rotated.orientation = tf2::toMsg(q_new);

  return rotated;
}

}  // namespace futuraps_task_planner
