#include "futuraps_task_planner/arm_motion_interface.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace futuraps_task_planner
{

ArmMotionInterface::ArmMotionInterface(rclcpp::Node * node)
: node_(node)
{
}

ArmMotionInterface::~ArmMotionInterface()
{
  if (moveit_executor_) {
    moveit_executor_->cancel();
  }

  if (moveit_spin_thread_.joinable()) {
    moveit_spin_thread_.join();
  }
}

void ArmMotionInterface::configure(const ArmMotionConfig & config)
{
  config_ = config;
}

bool ArmMotionInterface::initialize()
{
  if (initialized_) {
    return true;
  }

  if (node_ == nullptr) {
    return false;
  }

  try {
    bool use_sim_time = false;
    (void)node_->get_parameter("use_sim_time", use_sim_time);

    rclcpp::NodeOptions moveit_node_options;
    moveit_node_options.automatically_declare_parameters_from_overrides(true);
    moveit_node_options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", use_sim_time)});

    moveit_node_ = std::make_shared<rclcpp::Node>(
      "horizontal_spray_moveit_client",
      moveit_node_options);

    moveit_executor_ =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);

    moveit_spin_thread_ = std::thread([this]() {
      moveit_executor_->spin();
    });

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      moveit_node_,
      config_.planning_group);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to initialize MoveGroupInterface: %s",
      e.what());
    return false;
  }

  move_group_->setPlanningTime(10.0);
  move_group_->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.max_acceleration_scaling);
  move_group_->startStateMonitor();

  auto warm_state = move_group_->getCurrentState(2.0);
  if (!warm_state) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "MoveIt state monitor did not initialize with a fresh robot state");
    return false;
  }

  initialized_ = true;
  return true;
}

bool ArmMotionInterface::isInitialized() const
{
  return initialized_ && static_cast<bool>(move_group_);
}

bool ArmMotionInterface::waitForStateUpdate(double timeout_s) const
{
  if (!isInitialized()) {
    return false;
  }

  auto state = move_group_->getCurrentState(timeout_s);
  return static_cast<bool>(state);
}

bool ArmMotionInterface::hasCurrentState() const
{
  if (!isInitialized()) {
    return false;
  }

  auto state = move_group_->getCurrentState(0.1);
  return static_cast<bool>(state);
}

bool ArmMotionInterface::syncStartStateToCurrentState()
{
  if (!isInitialized()) {
    return false;
  }

  auto state = move_group_->getCurrentState(0.25);
  if (!state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state to sync start state");
    return false;
  }

  move_group_->setStartState(*state);
  return true;
}

bool ArmMotionInterface::getCurrentPose(geometry_msgs::msg::PoseStamped & pose_stamped) const
{
  if (!isInitialized()) {
    return false;
  }

  auto current_state = move_group_->getCurrentState(0.25);
  if (!current_state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state before getCurrentPose()");
    return false;
  }

  try {
    pose_stamped = move_group_->getCurrentPose();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to get current pose: %s",
      e.what());
    return false;
  }

  const bool bad_pose =
    !std::isfinite(pose_stamped.pose.position.x) ||
    !std::isfinite(pose_stamped.pose.position.y) ||
    !std::isfinite(pose_stamped.pose.position.z) ||
    !std::isfinite(pose_stamped.pose.orientation.x) ||
    !std::isfinite(pose_stamped.pose.orientation.y) ||
    !std::isfinite(pose_stamped.pose.orientation.z) ||
    !std::isfinite(pose_stamped.pose.orientation.w);

  if (bad_pose) {
    RCLCPP_ERROR(node_->get_logger(), "Current pose contains non-finite values");
    return false;
  }

  return true;
}

bool ArmMotionInterface::planCartesianPath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt not initialized");
    return false;
  }

  if (waypoints.size() < 2) {
    RCLCPP_WARN(node_->get_logger(), "Too few waypoints for Cartesian planning");
    return false;
  }

  auto current_state = move_group_->getCurrentState(1.0);
  if (!current_state) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to fetch current robot state before Cartesian planning");
    return false;
  }

  move_group_->setStartState(*current_state);

  moveit_msgs::msg::RobotTrajectory traj_msg;
  const double fraction = move_group_->computeCartesianPath(
    waypoints,
    config_.eef_step,
    config_.jump_threshold,
    traj_msg,
    config_.avoid_collisions);

  RCLCPP_INFO(node_->get_logger(), "Cartesian path fraction: %.3f", fraction);

  if (fraction < 0.999) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Cartesian path fraction too low for execution: %.3f",
      fraction);
    return false;
  }

  if (traj_msg.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Cartesian path returned fraction %.3f but trajectory has no points",
      fraction);
    return false;
  }

  const auto robot_model = move_group_->getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt robot model is null");
    return false;
  }

  robot_trajectory::RobotTrajectory rt(robot_model, config_.planning_group);

  try {
    rt.setRobotTrajectoryMsg(*current_state, traj_msg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to convert Cartesian trajectory to RobotTrajectory: %s",
      e.what());
    return false;
  }

  if (rt.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "RobotTrajectory is empty after conversion");
    return false;
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const bool timed_ok = totg.computeTimeStamps(
    rt,
    config_.cartesian_velocity_scaling,
    config_.cartesian_acceleration_scaling);

  if (!timed_ok) {
    RCLCPP_WARN(node_->get_logger(), "Failed to retime Cartesian trajectory");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);

  if (traj_msg.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Retimed Cartesian trajectory is empty");
    return false;
  }

  planned_trajectory_ = traj_msg;

  const auto & last_pt = planned_trajectory_.joint_trajectory.points.back();
  const double total_time =
    static_cast<double>(last_pt.time_from_start.sec) +
    1e-9 * static_cast<double>(last_pt.time_from_start.nanosec);

  RCLCPP_INFO(
    node_->get_logger(),
    "Retimed Cartesian trajectory with vel_scale=%.3f acc_scale=%.3f total_time=%.3f s points=%zu",
    config_.cartesian_velocity_scaling,
    config_.cartesian_acceleration_scaling,
    total_time,
    planned_trajectory_.joint_trajectory.points.size());

  return true;
}

bool ArmMotionInterface::executePlannedPath()
{
  if (!isInitialized()) {
    return false;
  }

  if (planned_trajectory_.joint_trajectory.points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No planned trajectory available to execute");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = planned_trajectory_;

  const auto result = move_group_->execute(plan);
  return result == moveit::core::MoveItErrorCode::SUCCESS;
}

bool ArmMotionInterface::moveHome()
{
  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt not initialized");
    return false;
  }

  if (config_.home_joint_values.size() != 6) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Home joint target must have 6 values, got %zu",
      config_.home_joint_values.size());
    return false;
  }

  auto current_state = move_group_->getCurrentState(0.2);
  if (!current_state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state for moveHome()");
    return false;
  }

  move_group_->stop();
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();
  move_group_->setStartState(*current_state);

  // Make homing more robust than default planning
  move_group_->setPlanningTime(5.0);
  move_group_->setNumPlanningAttempts(10);
  move_group_->setMaxVelocityScalingFactor(0.15);
  move_group_->setMaxAccelerationScalingFactor(0.15);

  move_group_->setJointValueTarget(config_.home_joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success =
    (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!success) {
    RCLCPP_WARN(node_->get_logger(), "moveHome() first planning attempt failed, retrying...");

    current_state = move_group_->getCurrentState(0.2);
    if (!current_state) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to refresh current state before moveHome() retry");
      return false;
    }

    move_group_->stop();
    move_group_->clearPoseTargets();
    move_group_->clearPathConstraints();
    move_group_->setStartState(*current_state);
    move_group_->setJointValueTarget(config_.home_joint_values);

    success =
      (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  }

  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to plan moveHome()");
    return false;
  }

  const auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute moveHome()");
    return false;
  }

  return true;
}


bool ArmMotionInterface::moveToPoseCartesian(
  const geometry_msgs::msg::Pose & target_pose,
  double eef_step,
  double jump_threshold)
{
  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt not initialized");
    return false;
  }

  auto current_state = move_group_->getCurrentState(0.2);
  if (!current_state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state for Cartesian move");
    return false;
  }

  move_group_->stop();
  move_group_->clearPoseTargets();
  move_group_->clearPathConstraints();
  move_group_->setStartState(*current_state);

  geometry_msgs::msg::PoseStamped current_pose_stamped;
  if (!getCurrentPose(current_pose_stamped)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current pose for Cartesian move");
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(current_pose_stamped.pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory traj_msg;
  const double fraction = move_group_->computeCartesianPath(
    waypoints,
    eef_step,
    jump_threshold,
    traj_msg,
    true);

  RCLCPP_INFO(
    node_->get_logger(),
    "Move-to-start Cartesian fraction: %.3f",
    fraction);

  if (fraction < 0.999) {
    RCLCPP_WARN(node_->get_logger(), "Cartesian move-to-start fraction too low");
    return false;
  }

  if (traj_msg.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Cartesian move-to-start returned empty trajectory");
    return false;
  }

  robot_trajectory::RobotTrajectory rt(
    move_group_->getRobotModel(),
    config_.planning_group);

  rt.setRobotTrajectoryMsg(*current_state, traj_msg);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, 0.08, 0.08)) {
    RCLCPP_WARN(node_->get_logger(), "Failed to retime Cartesian move-to-start");
    return false;
  }

  rt.getRobotTrajectoryMsg(traj_msg);

  if (traj_msg.joint_trajectory.points.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Retimed Cartesian move-to-start is empty");
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = traj_msg;

  const auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to execute Cartesian move-to-start");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "Cartesian move-to-start succeeded");
  return true;
}

bool ArmMotionInterface::moveToPoseSeeded(
  const geometry_msgs::msg::Pose & target_pose,
  const std::string & ee_link,
  bool /*allow_approximate_ik*/)
{
  if (!isInitialized()) {
    RCLCPP_ERROR(node_->get_logger(), "MoveIt not initialized");
    return false;
  }

  const std::string link_name =
    ee_link.empty() ? move_group_->getEndEffectorLink() : ee_link;

  if (link_name.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No end-effector link available for IK");
    return false;
  }

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(1.0);
  if (!current_state) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to fetch current robot state for seeded IK");
    return false;
  }

  move_group_->setStartState(*current_state);

  const moveit::core::JointModelGroup * jmg =
    current_state->getJointModelGroup(config_.planning_group);

  if (!jmg) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "JointModelGroup '%s' not found",
      config_.planning_group.c_str());
    return false;
  }

  moveit::core::RobotState target_state(*current_state);
  const bool ik_ok = target_state.setFromIK(
    jmg,
    target_pose,
    link_name,
    0.1);

  if (!ik_ok) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Seeded IK failed for link '%s'",
      link_name.c_str());
    return false;
  }

  std::vector<double> joint_values;
  target_state.copyJointGroupPositions(jmg, joint_values);

  move_group_->setMaxVelocityScalingFactor(config_.max_velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(config_.max_acceleration_scaling);
  move_group_->setJointValueTarget(joint_values);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group_->plan(plan);

  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to plan seeded IK move-to-pose for link '%s'",
      link_name.c_str());
    return false;
  }

  const auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to execute seeded IK move-to-pose for link '%s'",
      link_name.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Seeded IK move-to-pose succeeded for link '%s'",
    link_name.c_str());
  return true;
}

}  // namespace futuraps_task_planner