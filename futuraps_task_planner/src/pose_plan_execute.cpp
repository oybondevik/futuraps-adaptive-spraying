#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

class PosePlanExecuteNode : public rclcpp::Node
{
public:
  PosePlanExecuteNode() : Node("pose_plan_execute")
  {
    // ---- Parameters (compatible with your launch) ----
    declare_parameter<std::string>("planning_group", "ur10_arm");
    declare_parameter<std::string>("target_frame", "platform_link");

    declare_parameter<double>("x", 0.45);
    declare_parameter<double>("y", 0.0);
    declare_parameter<double>("z", 0.55);

    // Euler angles (radians)
    declare_parameter<double>("roll", 0.0);
    declare_parameter<double>("pitch", 0.0);
    declare_parameter<double>("yaw", 0.0);

    declare_parameter<bool>("execute", true);

    declare_parameter<double>("planning_time", 5.0);
    declare_parameter<int>("planning_attempts", 5);

    declare_parameter<std::string>("pipeline_id", "ompl");
    declare_parameter<std::string>("planner_id", "");

    declare_parameter<double>("max_velocity_scaling", 0.2);
    declare_parameter<double>("max_acceleration_scaling", 0.2);

    declare_parameter<double>("wait_for_state_timeout_s", 3.0);
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<double>("max_joint_state_age_s", 1.0);

    declare_parameter<double>("ik_timeout_s", 0.05);
    declare_parameter<int>("ik_attempts", 4);

    joint_states_topic_ = get_parameter("joint_states_topic").as_string();

    // ---- Callback groups (IMPORTANT) ----
    // Default callback group is MutuallyExclusive. If the timer callback blocks waiting,
    // the JointState callback would not run. So we place them in different groups.
    joint_state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    one_shot_group_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ---- JointState subscriber (in its own group) ----
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = joint_state_group_;

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;
        latest_joint_state_rx_time_ = now();
      },
      sub_opts);

    // ---- One-shot timer (in its own group) ----
    one_shot_timer_ = this->create_wall_timer(
      50ms,
      [this]() {
        one_shot_timer_->cancel();
        run_once();
      },
      one_shot_group_);
  }

private:
  bool have_fresh_joint_state(double max_age_s) const
  {
    if (!latest_joint_state_) return false;
    const double age = (now() - latest_joint_state_rx_time_).seconds();
    return age <= max_age_s;
  }

  bool wait_for_joint_state_or_timeout(double timeout_s, double max_age_s)
  {
    const rclcpp::Time start = now();
    while (rclcpp::ok()) {
      if (have_fresh_joint_state(max_age_s)) return true;
      if ((now() - start).seconds() >= timeout_s) return false;
      rclcpp::sleep_for(20ms);
    }
    return false;
  }

  [[noreturn]] void exit_tool(int code)
  {
    rclcpp::sleep_for(50ms);  // let logs flush
    std::exit(code);
  }

  geometry_msgs::msg::Pose read_target_pose_from_params() const
  {
    const double x = get_parameter("x").as_double();
    const double y = get_parameter("y").as_double();
    const double z = get_parameter("z").as_double();

    const double roll  = get_parameter("roll").as_double();
    const double pitch = get_parameter("pitch").as_double();
    const double yaw   = get_parameter("yaw").as_double();

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  moveit::core::RobotState build_start_state_from_joint_state(
    const moveit::core::RobotModelConstPtr& robot_model,
    const sensor_msgs::msg::JointState& js_msg) const
  {
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();

    const size_t n = std::min(js_msg.name.size(), js_msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      const std::string& joint_name = js_msg.name[i];
      const double joint_pos = js_msg.position[i];

      const auto* joint_model = robot_model->getJointModel(joint_name);
      if (!joint_model) continue;
      if (joint_model->getVariableCount() != 1) continue;  // ignore multi-DOF joints

      state.setVariablePosition(joint_name, joint_pos);
    }

    state.update();
    return state;
  }

  bool seeded_ik_to_joint_goal(
    const moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::RobotState& seed_state,
    const geometry_msgs::msg::Pose& target_pose_in_planning_frame,
    double ik_timeout_s,
    int ik_attempts,
    std::vector<double>& joint_goal_out) const
  {
    auto robot_model = move_group.getRobotModel();
    if (!robot_model) return false;

    const auto* joint_group = robot_model->getJointModelGroup(move_group.getName());
    if (!joint_group) return false;

    moveit::core::RobotState ik_state = seed_state;

    const std::string ee_link = move_group.getEndEffectorLink();
    const int attempts = std::max(1, ik_attempts);

    bool ok = false;
    for (int i = 0; i < attempts; ++i) {
      ok = ik_state.setFromIK(joint_group, target_pose_in_planning_frame, ee_link, ik_timeout_s);
      if (ok) break;
    }
    if (!ok) return false;

    ik_state.copyJointGroupPositions(joint_group, joint_goal_out);
    return true;
  }

  void run_once()
  {
    const std::string planning_group = get_parameter("planning_group").as_string();
    const std::string target_frame   = get_parameter("target_frame").as_string();
    const bool do_execute            = get_parameter("execute").as_bool();

    const double planning_time       = get_parameter("planning_time").as_double();
    const int planning_attempts      = get_parameter("planning_attempts").as_int();

    const std::string pipeline_id    = get_parameter("pipeline_id").as_string();
    const std::string planner_id     = get_parameter("planner_id").as_string();

    const double vel_scale           = get_parameter("max_velocity_scaling").as_double();
    const double acc_scale           = get_parameter("max_acceleration_scaling").as_double();

    const double wait_timeout_s      = get_parameter("wait_for_state_timeout_s").as_double();
    const double max_joint_age_s     = get_parameter("max_joint_state_age_s").as_double();

    const double ik_timeout_s        = get_parameter("ik_timeout_s").as_double();
    const int ik_attempts            = get_parameter("ik_attempts").as_int();

    RCLCPP_INFO(get_logger(), "Waiting for JointState on '%s' (timeout %.2fs)...",
                joint_states_topic_.c_str(), wait_timeout_s);

    if (!wait_for_joint_state_or_timeout(wait_timeout_s, max_joint_age_s)) {
      RCLCPP_ERROR(get_logger(), "No fresh JointState received.");
      exit_tool(1);
    }

    auto js_msg = latest_joint_state_;
    if (!js_msg || js_msg->name.empty() ||
        js_msg->position.size() < js_msg->name.size()) {
      RCLCPP_ERROR(get_logger(), "JointState invalid (name/position mismatch).");
      exit_tool(1);
    }

    // MoveIt interface
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), planning_group);
    move_group.setPlanningTime(planning_time);
    move_group.setNumPlanningAttempts(planning_attempts);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);
    if (!pipeline_id.empty()) move_group.setPlanningPipelineId(pipeline_id);
    if (!planner_id.empty())  move_group.setPlannerId(planner_id);

    const std::string planning_frame = move_group.getPlanningFrame();
    if (target_frame != planning_frame) {
      RCLCPP_ERROR(get_logger(),
                   "target_frame='%s' must match MoveIt planning_frame='%s' (this tool does no TF transform).",
                   target_frame.c_str(), planning_frame.c_str());
      exit_tool(1);
    }

    auto robot_model = move_group.getRobotModel();
    if (!robot_model) {
      RCLCPP_ERROR(get_logger(), "RobotModel missing (robot_description / SRDF not loaded).");
      exit_tool(1);
    }

    // Start state from /joint_states
    moveit::core::RobotState start_state =
      build_start_state_from_joint_state(robot_model, *js_msg);
    move_group.setStartState(start_state);

    // Target pose from params
    const geometry_msgs::msg::Pose target_pose = read_target_pose_from_params();

    // Seeded IK -> joint goal
    std::vector<double> joint_goal;
    if (!seeded_ik_to_joint_goal(move_group, start_state, target_pose, ik_timeout_s, ik_attempts, joint_goal)) {
      RCLCPP_ERROR(get_logger(), "Seeded IK failed (no IK solution).");
      exit_tool(1);
    }

    move_group.setJointValueTarget(joint_goal);

    // Plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Planning failed.");
      exit_tool(1);
    }

    if (!do_execute) {
      RCLCPP_INFO(get_logger(), "Planned successfully (execute=false).");
      exit_tool(0);
    }

    // Execute
    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Execution failed/aborted.");
      exit_tool(1);
    }

    RCLCPP_INFO(get_logger(), "Done.");
    exit_tool(0);
  }

  // ---- JointState data ----
  std::string joint_states_topic_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Time latest_joint_state_rx_time_{0, 0, RCL_ROS_TIME};

  // ---- Callback groups / timer ----
  rclcpp::CallbackGroup::SharedPtr joint_state_group_;
  rclcpp::CallbackGroup::SharedPtr one_shot_group_;
  rclcpp::TimerBase::SharedPtr one_shot_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PosePlanExecuteNode>();

  // IMPORTANT: MultiThreadedExecutor is required since run_once() can "wait/sleep"
  // while JointState callbacks must still be processed concurrently.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}