// vertical_pose_planner.cpp
//
// Uses ClosestGrid + GlobalNormal perception and plans/executes poses with MoveGroupInterface.
// Fixes included:
//   - use_sim_time-friendly timers
//   - persistent MoveGroupInterface (not recreated every plan cycle)
//   - PID auto-enable once system is ready
//   - async perception response handling fixed
//   - replan deadband
//   - failed/success cooldowns
//   - optional simple look-ahead

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "futuraps_perception/srv/get_closest_grid.hpp"
#include "futuraps_perception/srv/get_global_normal.hpp"

using namespace std::chrono_literals;

class VerticalPosePlannerNode : public rclcpp::Node
{
public:
  VerticalPosePlannerNode()
  : Node("vertical_pose_planner")
  {
    declare_parameters();
    load_parameters();

    joint_state_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    perception_group_  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    planning_group_    = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = joint_state_group_;
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(10),
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_joint_state_ = msg;
        latest_joint_state_rx_time_ = now();
        for (const auto & n : msg->name) {
          seen_joints_.insert(n);
        }
      },
      sub_opts);

    closest_grid_client_ =
      this->create_client<futuraps_perception::srv::GetClosestGrid>(closest_srv_);
    global_normal_client_ =
      this->create_client<futuraps_perception::srv::GetGlobalNormal>(normal_srv_);
    enable_pid_client_ =
      this->create_client<std_srvs::srv::SetBool>("/pid_controller/enable");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ee_actual_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/spray/ee_actual_pose", 10);
    ee_desired_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/spray/ee_desired_pose", 10);
    actual_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/spray/actual_path", 10);
    desired_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/spray/desired_path", 10);
    spray_enabled_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(spray_enabled_topic_, 10);

    actual_path_msg_.header.frame_id = target_frame_;
    desired_path_msg_.header.frame_id = target_frame_;
    actual_path_msg_.header.stamp = now();
    desired_path_msg_.header.stamp = now();
    publish_spray_enabled(false, true);

    perception_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(perception_dt_s_)),
      std::bind(&VerticalPosePlannerNode::perception_update, this),
      perception_group_);

    plan_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(plan_period_s_)),
      std::bind(&VerticalPosePlannerNode::plan_cycle, this),
      planning_group_);

    RCLCPP_INFO(
      get_logger(),
      "VerticalPosePlanner started.\n"
      " planning_group=%s plan_period=%.2fs perception_dt=%.2fs\n"
      " standoff=%.3f z_target=%.3f\n"
      " map_frame=%s base_frame=%s target_frame=%s",
      planning_group_name_.c_str(), plan_period_s_, perception_dt_s_,
      standoff_m_, z_target_,
      map_frame_.c_str(), base_frame_.c_str(), target_frame_.c_str());
  }

private:
  // -----------------------------
  // Parameters
  // -----------------------------
  std::string planning_group_name_{"ur10_arm"};
  std::string target_frame_{"platform_link"};
  std::string map_frame_{"map"};
  std::string base_frame_{"platform_link"};

  bool execute_{true};
  double planning_time_{2.0};
  int planning_attempts_{3};
  std::string pipeline_id_{"ompl"};
  std::string planner_id_{""};
  double max_velocity_scaling_{0.15};
  double max_acceleration_scaling_{0.15};

  double perception_dt_s_{0.10};
  double plan_period_s_{0.50};

  std::string joint_states_topic_{"/joint_states"};
  double wait_for_state_timeout_s_{3.0};
  double max_joint_state_age_s_{1.0};
  std::vector<std::string> required_joints_{
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  double ik_timeout_s_{0.05};
  int ik_attempts_{4};

  std::string closest_srv_{"/get_closest_grid"};
  std::string normal_srv_{"/get_global_normal"};
  double perception_timeout_s_{0.3};

  double cell_size_{0.3};
  int rows_{5};
  int cols_{1};
  double x0_{-0.15};
  double z0_{0.2};
  double y_left_max_{2.0};
  double y_right_max_{2.0};
  int side_{0};
  double front_percentile_{0.01};
  int min_points_per_cell_{20};

  double normal_min_x_{-0.5};
  double normal_max_x_{0.5};
  double normal_min_y_{-2.0};
  double normal_max_y_{2.0};
  double normal_min_z_{0.2};
  double normal_max_z_{2.0};

  double standoff_m_{0.20};
  double z_target_{1.0};
  double filter_tau_s_{0.20};
  int tool_axis_forward_{2};
  double tool_roll_orientation_{-1.57};
  bool point_tool_into_surface_{true};

  double replan_pos_threshold_m_{0.03};
  double replan_angle_threshold_rad_{0.20};
  double failed_plan_cooldown_s_{1.0};
  double successful_plan_cooldown_s_{0.25};

  double platform_speed_mps_{0.0};
  double lookahead_time_s_{0.0};
  bool auto_enable_spray_{true};
  std::string spray_enabled_topic_{"/spray/enabled"};

  // -----------------------------
  // ROS interfaces
  // -----------------------------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<futuraps_perception::srv::GetClosestGrid>::SharedPtr closest_grid_client_;
  rclcpp::Client<futuraps_perception::srv::GetGlobalNormal>::SharedPtr global_normal_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_pid_client_;

  rclcpp::TimerBase::SharedPtr perception_timer_;
  rclcpp::TimerBase::SharedPtr plan_timer_;

  rclcpp::CallbackGroup::SharedPtr joint_state_group_;
  rclcpp::CallbackGroup::SharedPtr perception_group_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_actual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_desired_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_enabled_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  moveit::core::RobotModelConstPtr robot_model_;

  // -----------------------------
  // State
  // -----------------------------
  nav_msgs::msg::Path actual_path_msg_;
  nav_msgs::msg::Path desired_path_msg_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Time latest_joint_state_rx_time_{0, 0, RCL_ROS_TIME};
  std::unordered_set<std::string> seen_joints_;

  std::mutex meas_mtx_;
  bool have_measurement_{false};
  tf2::Vector3 p_cp_filt_{0.0, 0.5, 1.0};
  tf2::Vector3 n_filt_{0, -1, 0};
  rclcpp::Time last_target_update_time_{0, 0, RCL_ROS_TIME};

  using ClosestFuture = rclcpp::Client<futuraps_perception::srv::GetClosestGrid>::SharedFuture;
  using NormalFuture  = rclcpp::Client<futuraps_perception::srv::GetGlobalNormal>::SharedFuture;
  using ClosestResponse = futuraps_perception::srv::GetClosestGrid::Response;
  using NormalResponse  = futuraps_perception::srv::GetGlobalNormal::Response;

  bool closest_pending_{false};
  bool normal_pending_{false};
  ClosestFuture closest_future_;
  NormalFuture  normal_future_;
  ClosestResponse::SharedPtr closest_response_;
  NormalResponse::SharedPtr normal_response_;
  rclcpp::Time perception_start_time_{0, 0, RCL_ROS_TIME};

  bool planning_busy_{false};
  bool platform_motion_started_{false};
  bool last_spray_enabled_{false};
  rclcpp::Time last_spray_enabled_publish_time_{0, 0, RCL_ROS_TIME};
  bool initial_pose_logged_{false};
  bool move_group_initialized_{false};

  geometry_msgs::msg::Pose last_commanded_pose_;
  bool have_last_commanded_pose_{false};
  rclcpp::Time last_plan_attempt_time_{0, 0, RCL_ROS_TIME};
  bool last_attempt_failed_{false};

  // -----------------------------
  // Utilities
  // -----------------------------
  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  static bool finite3(double a, double b, double c)
  {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
  }

  geometry_msgs::msg::PoseStamped make_pose_stamped(
    const geometry_msgs::msg::Pose & pose,
    const std::string & frame_id) const
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = frame_id;
    ps.pose = pose;
    return ps;
  }

  void publish_actual_pose_and_path(const geometry_msgs::msg::PoseStamped & actual_ps)
  {
    ee_actual_pose_pub_->publish(actual_ps);

    actual_path_msg_.header.frame_id = actual_ps.header.frame_id;
    actual_path_msg_.header.stamp = now();
    actual_path_msg_.poses.push_back(actual_ps);
    actual_path_pub_->publish(actual_path_msg_);
  }

  void publish_desired_pose_and_path(
    const geometry_msgs::msg::Pose & desired_pose,
    const std::string & frame_id)
  {
    auto desired_ps = make_pose_stamped(desired_pose, frame_id);
    ee_desired_pose_pub_->publish(desired_ps);

    desired_path_msg_.header.frame_id = frame_id;
    desired_path_msg_.header.stamp = now();
    desired_path_msg_.poses.push_back(desired_ps);
    desired_path_pub_->publish(desired_path_msg_);
  }

  bool have_fresh_joint_state(double max_age_s) const
  {
    if (!latest_joint_state_) {
      return false;
    }
    const double age = (now() - latest_joint_state_rx_time_).seconds();
    return age <= max_age_s;
  }

  bool have_required_joints() const
  {
    if (required_joints_.empty()) {
      return true;
    }
    for (const auto & j : required_joints_) {
      if (seen_joints_.find(j) == seen_joints_.end()) {
        return false;
      }
    }
    return true;
  }

  moveit::core::RobotState build_start_state_from_joint_state(
    const moveit::core::RobotModelConstPtr & robot_model,
    const sensor_msgs::msg::JointState & js_msg) const
  {
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();

    const size_t n = std::min(js_msg.name.size(), js_msg.position.size());
    for (size_t i = 0; i < n; ++i) {
      const std::string & joint_name = js_msg.name[i];
      const double joint_pos = js_msg.position[i];

      const auto * joint_model = robot_model->getJointModel(joint_name);
      if (!joint_model) {
        continue;
      }
      if (joint_model->getVariableCount() != 1) {
        continue;
      }
      state.setVariablePosition(joint_name, joint_pos);
    }

    state.update();
    return state;
  }

  bool seeded_ik_to_joint_goal(
    const moveit::planning_interface::MoveGroupInterface & move_group,
    const moveit::core::RobotState & seed_state,
    const geometry_msgs::msg::Pose & target_pose_in_planning_frame,
    double ik_timeout_s,
    int ik_attempts,
    std::vector<double> & joint_goal_out) const
  {
    auto robot_model = move_group.getRobotModel();
    if (!robot_model) {
      return false;
    }

    const auto * joint_group = robot_model->getJointModelGroup(move_group.getName());
    if (!joint_group) {
      return false;
    }

    moveit::core::RobotState ik_state = seed_state;
    const std::string ee_link = move_group.getEndEffectorLink();
    const int attempts = std::max(1, ik_attempts);

    bool ok = false;
    for (int i = 0; i < attempts; ++i) {
      ok = ik_state.setFromIK(joint_group, target_pose_in_planning_frame, ee_link, ik_timeout_s);
      if (ok) {
        break;
      }
    }
    if (!ok) {
      return false;
    }

    ik_state.copyJointGroupPositions(joint_group, joint_goal_out);
    return true;
  }

  static tf2::Quaternion quatAlignToolAxisToForward(
    const tf2::Vector3 & forward_in,
    const tf2::Vector3 & up_hint_in,
    int tool_axis)
  {
    tf2::Vector3 forward = forward_in;
    if (forward.length2() < 1e-12) {
      forward = tf2::Vector3(1, 0, 0);
    }
    forward.normalize();

    tf2::Vector3 up_hint = up_hint_in;
    if (up_hint.length2() < 1e-12) {
      up_hint = tf2::Vector3(0, 0, 1);
    }
    up_hint.normalize();

    if (std::fabs(forward.dot(up_hint)) > 0.95) {
      up_hint = tf2::Vector3(1, 0, 0);
    }

    tf2::Vector3 b = up_hint.cross(forward);
    if (b.length2() < 1e-12) {
      b = tf2::Vector3(0, 1, 0);
    }
    b.normalize();
    tf2::Vector3 a = forward.cross(b);
    a.normalize();

    tf2::Vector3 X, Y, Z;
    if (tool_axis == 0) {
      X = forward; Y = b; Z = a;
    } else if (tool_axis == 1) {
      Y = forward; Z = b; X = a;
    } else {
      Z = forward; X = b; Y = a;
    }

    tf2::Matrix3x3 R(
      X.x(), Y.x(), Z.x(),
      X.y(), Y.y(), Z.y(),
      X.z(), Y.z(), Z.z());

    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    return q;
  }

  double pose_position_error(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b) const
  {
    const double dx = a.position.x - b.position.x;
    const double dy = a.position.y - b.position.y;
    const double dz = a.position.z - b.position.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double pose_angle_error(
    const geometry_msgs::msg::Pose & a,
    const geometry_msgs::msg::Pose & b) const
  {
    tf2::Quaternion qa(a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w);
    tf2::Quaternion qb(b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w);
    qa.normalize();
    qb.normalize();

    double dot = std::fabs(qa.dot(qb));
    dot = std::min(1.0, std::max(-1.0, dot));
    return 2.0 * std::acos(dot);
  }

  bool should_replan(const geometry_msgs::msg::Pose & target_pose) const
  {
    if (!have_last_commanded_pose_) {
      return true;
    }

    const double pos_err = pose_position_error(target_pose, last_commanded_pose_);
    const double ang_err = pose_angle_error(target_pose, last_commanded_pose_);

    return (pos_err >= replan_pos_threshold_m_) ||
           (ang_err >= replan_angle_threshold_rad_);
  }

  bool cooldown_active() const
  {
    if (last_plan_attempt_time_.nanoseconds() == 0) {
      return false;
    }

    const double dt = (now() - last_plan_attempt_time_).seconds();
    const double cooldown = last_attempt_failed_ ? failed_plan_cooldown_s_
                                                 : successful_plan_cooldown_s_;
    return dt < cooldown;
  }

  // -----------------------------
  // Parameters
  // -----------------------------
  void declare_parameters()
  {
    declare_parameter<std::string>("planning_group", planning_group_name_);
    declare_parameter<std::string>("target_frame", target_frame_);
    declare_parameter<std::string>("map_frame", map_frame_);
    declare_parameter<std::string>("base_frame", base_frame_);

    declare_parameter<bool>("execute", execute_);
    declare_parameter<double>("planning_time", planning_time_);
    declare_parameter<int>("planning_attempts", planning_attempts_);
    declare_parameter<std::string>("pipeline_id", pipeline_id_);
    declare_parameter<std::string>("planner_id", planner_id_);
    declare_parameter<double>("max_velocity_scaling", max_velocity_scaling_);
    declare_parameter<double>("max_acceleration_scaling", max_acceleration_scaling_);

    declare_parameter<double>("perception_dt_s", perception_dt_s_);
    declare_parameter<double>("plan_period_s", plan_period_s_);

    declare_parameter<double>("wait_for_state_timeout_s", wait_for_state_timeout_s_);
    declare_parameter<std::string>("joint_states_topic", joint_states_topic_);
    declare_parameter<double>("max_joint_state_age_s", max_joint_state_age_s_);
    declare_parameter<std::vector<std::string>>("required_joints", required_joints_);

    declare_parameter<double>("ik_timeout_s", ik_timeout_s_);
    declare_parameter<int>("ik_attempts", ik_attempts_);

    declare_parameter<std::string>("closest_srv", closest_srv_);
    declare_parameter<std::string>("normal_srv", normal_srv_);
    declare_parameter<double>("perception_timeout_s", perception_timeout_s_);

    declare_parameter("cell_size", cell_size_);
    declare_parameter("rows", rows_);
    declare_parameter("cols", cols_);
    declare_parameter("x0", x0_);
    declare_parameter("z0", z0_);
    declare_parameter("y_left_max", y_left_max_);
    declare_parameter("y_right_max", y_right_max_);
    declare_parameter("side", side_);
    declare_parameter("front_percentile", front_percentile_);
    declare_parameter("min_points_per_cell", min_points_per_cell_);

    declare_parameter("normal_min_x", normal_min_x_);
    declare_parameter("normal_max_x", normal_max_x_);
    declare_parameter("normal_min_y", normal_min_y_);
    declare_parameter("normal_max_y", normal_max_y_);
    declare_parameter("normal_min_z", normal_min_z_);
    declare_parameter("normal_max_z", normal_max_z_);

    declare_parameter("standoff_m", standoff_m_);
    declare_parameter("z_target", z_target_);
    declare_parameter("filter_tau_s", filter_tau_s_);
    declare_parameter("tool_axis_forward", tool_axis_forward_);
    declare_parameter("tool_roll_orientation", tool_roll_orientation_);
    declare_parameter("point_tool_into_surface", point_tool_into_surface_);

    declare_parameter("replan_pos_threshold_m", replan_pos_threshold_m_);
    declare_parameter("replan_angle_threshold_rad", replan_angle_threshold_rad_);
    declare_parameter("failed_plan_cooldown_s", failed_plan_cooldown_s_);
    declare_parameter("successful_plan_cooldown_s", successful_plan_cooldown_s_);

    declare_parameter("platform_speed_mps", platform_speed_mps_);
    declare_parameter("lookahead_time_s", lookahead_time_s_);
    declare_parameter("auto_enable_spray", auto_enable_spray_);
    declare_parameter("spray_enabled_topic", spray_enabled_topic_);
  }

  void load_parameters()
  {
    planning_group_name_ = get_parameter("planning_group").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    execute_ = get_parameter("execute").as_bool();
    planning_time_ = get_parameter("planning_time").as_double();
    planning_attempts_ = get_parameter("planning_attempts").as_int();
    pipeline_id_ = get_parameter("pipeline_id").as_string();
    planner_id_ = get_parameter("planner_id").as_string();
    max_velocity_scaling_ = get_parameter("max_velocity_scaling").as_double();
    max_acceleration_scaling_ = get_parameter("max_acceleration_scaling").as_double();

    perception_dt_s_ = get_parameter("perception_dt_s").as_double();
    plan_period_s_ = get_parameter("plan_period_s").as_double();

    wait_for_state_timeout_s_ = get_parameter("wait_for_state_timeout_s").as_double();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    max_joint_state_age_s_ = get_parameter("max_joint_state_age_s").as_double();
    required_joints_ = get_parameter("required_joints").as_string_array();

    ik_timeout_s_ = get_parameter("ik_timeout_s").as_double();
    ik_attempts_ = get_parameter("ik_attempts").as_int();

    closest_srv_ = get_parameter("closest_srv").as_string();
    normal_srv_ = get_parameter("normal_srv").as_string();
    perception_timeout_s_ = get_parameter("perception_timeout_s").as_double();

    cell_size_ = get_parameter("cell_size").as_double();
    rows_ = get_parameter("rows").as_int();
    cols_ = get_parameter("cols").as_int();
    x0_ = get_parameter("x0").as_double();
    z0_ = get_parameter("z0").as_double();
    y_left_max_ = get_parameter("y_left_max").as_double();
    y_right_max_ = get_parameter("y_right_max").as_double();
    side_ = get_parameter("side").as_int();
    front_percentile_ = get_parameter("front_percentile").as_double();
    min_points_per_cell_ = get_parameter("min_points_per_cell").as_int();

    normal_min_x_ = get_parameter("normal_min_x").as_double();
    normal_max_x_ = get_parameter("normal_max_x").as_double();
    normal_min_y_ = get_parameter("normal_min_y").as_double();
    normal_max_y_ = get_parameter("normal_max_y").as_double();
    normal_min_z_ = get_parameter("normal_min_z").as_double();
    normal_max_z_ = get_parameter("normal_max_z").as_double();

    standoff_m_ = get_parameter("standoff_m").as_double();
    z_target_ = get_parameter("z_target").as_double();
    filter_tau_s_ = get_parameter("filter_tau_s").as_double();
    tool_axis_forward_ = get_parameter("tool_axis_forward").as_int();
    tool_roll_orientation_ = get_parameter("tool_roll_orientation").as_double();
    point_tool_into_surface_ = get_parameter("point_tool_into_surface").as_bool();

    replan_pos_threshold_m_ = get_parameter("replan_pos_threshold_m").as_double();
    replan_angle_threshold_rad_ = get_parameter("replan_angle_threshold_rad").as_double();
    failed_plan_cooldown_s_ = get_parameter("failed_plan_cooldown_s").as_double();
    successful_plan_cooldown_s_ = get_parameter("successful_plan_cooldown_s").as_double();

    platform_speed_mps_ = get_parameter("platform_speed_mps").as_double();
    lookahead_time_s_ = get_parameter("lookahead_time_s").as_double();
    auto_enable_spray_ = get_parameter("auto_enable_spray").as_bool();
    spray_enabled_topic_ = get_parameter("spray_enabled_topic").as_string();
  }

  // -----------------------------
  // MoveGroup init
  // -----------------------------
  bool ensure_move_group_ready()
  {
    if (move_group_initialized_) {
      return true;
    }

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_name_);

      move_group_->setPlanningTime(planning_time_);
      move_group_->setNumPlanningAttempts(planning_attempts_);
      move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
      move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

      if (!pipeline_id_.empty()) {
        move_group_->setPlanningPipelineId(pipeline_id_);
      }
      if (!planner_id_.empty()) {
        move_group_->setPlannerId(planner_id_);
      }

      robot_model_ = move_group_->getRobotModel();
      if (!robot_model_) {
        RCLCPP_ERROR(get_logger(), "RobotModel missing (robot_description/SRDF not loaded).");
        return false;
      }

      const std::string planning_frame = move_group_->getPlanningFrame();
      if (planning_frame != target_frame_) {
        RCLCPP_ERROR(
          get_logger(),
          "target_frame='%s' must match MoveIt planning_frame='%s' (no TF transform in this node).",
          target_frame_.c_str(), planning_frame.c_str());
        return false;
      }

      move_group_initialized_ = true;
      RCLCPP_INFO(get_logger(), "Persistent MoveGroupInterface initialized.");
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to initialize MoveGroupInterface: %s", ex.what());
      return false;
    }
  }

  // -----------------------------
  // Platform / TF helpers
  // -----------------------------
  bool ensure_platform_tf_ready()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Platform TF lookup failed: %s", ex.what());
      return false;
    }

    if (!initial_pose_logged_) {
      RCLCPP_INFO(
        get_logger(),
        "Initial platform pose found in %s: [x=%.3f y=%.3f z=%.3f]",
        map_frame_.c_str(),
        tf.transform.translation.x,
        tf.transform.translation.y,
        tf.transform.translation.z);
      initial_pose_logged_ = true;
    }

    return true;
  }

  void publish_spray_enabled(bool enabled, bool force = false)
  {
    if (!auto_enable_spray_) {
      return;
    }

    const bool state_changed = enabled != last_spray_enabled_;
    const bool should_republish =
      last_spray_enabled_publish_time_.nanoseconds() == 0 ||
      (now() - last_spray_enabled_publish_time_).seconds() >= 1.0;

    if (!force && !state_changed && !should_republish) {
      return;
    }

    std_msgs::msg::Bool msg;
    msg.data = enabled;
    spray_enabled_pub_->publish(msg);

    last_spray_enabled_ = enabled;
    last_spray_enabled_publish_time_ = now();

    if (state_changed || force) {
      RCLCPP_INFO(
        get_logger(),
        "Published %s = %s",
        spray_enabled_topic_.c_str(),
        enabled ? "true" : "false");
    }
  }

  bool start_platform_motion()
  {
    if (platform_motion_started_) {
      return true;
    }

    if (!enable_pid_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PID enable service not ready");
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    enable_pid_client_->async_send_request(req);

    platform_motion_started_ = true;
    RCLCPP_INFO(get_logger(), "Sent request to enable PID controller for vertical spraying");
    if (execute_) {
      publish_spray_enabled(true, true);
    }
    return true;
  }

  // -----------------------------
  // Perception
  // -----------------------------
  void perception_update()
  {
    if (!closest_grid_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s", closest_srv_.c_str());
      return;
    }
    if (!global_normal_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s", normal_srv_.c_str());
      return;
    }

    if (!closest_pending_ && !normal_pending_ &&
        !closest_response_ && !normal_response_)
    {
      auto closest_req =
        std::make_shared<futuraps_perception::srv::GetClosestGrid::Request>();
      closest_req->cell_size = cell_size_;
      closest_req->rows = rows_;
      closest_req->cols = cols_;
      closest_req->x0 = x0_;
      closest_req->z0 = z0_;
      closest_req->y_left_max = y_left_max_;
      closest_req->y_right_max = y_right_max_;
      closest_req->side = side_;
      closest_req->front_percentile = front_percentile_;
      closest_req->min_points_per_cell = min_points_per_cell_;

      auto normal_req =
        std::make_shared<futuraps_perception::srv::GetGlobalNormal::Request>();
      normal_req->frame_id = target_frame_;
      normal_req->min_x = normal_min_x_;
      normal_req->max_x = normal_max_x_;
      normal_req->min_y = normal_min_y_;
      normal_req->max_y = normal_max_y_;
      normal_req->min_z = normal_min_z_;
      normal_req->max_z = normal_max_z_;

      closest_future_ = closest_grid_client_->async_send_request(closest_req).future.share();
      normal_future_ = global_normal_client_->async_send_request(normal_req).future.share();
      closest_pending_ = true;
      normal_pending_ = true;
      perception_start_time_ = now();
      return;
    }

    if ((closest_pending_ || normal_pending_) &&
        (now() - perception_start_time_).seconds() > perception_timeout_s_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Perception timeout (closest/normal)");

      closest_pending_ = false;
      normal_pending_ = false;
      closest_response_.reset();
      normal_response_.reset();

      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }

    if (closest_pending_ && closest_future_.valid() &&
        closest_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      closest_response_ = closest_future_.get();
      closest_pending_ = false;
    }

    if (normal_pending_ && normal_future_.valid() &&
        normal_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
    {
      normal_response_ = normal_future_.get();
      normal_pending_ = false;
    }

    if (closest_pending_ || normal_pending_) {
      return;
    }

    if (!closest_response_ || !normal_response_) {
      return;
    }

    auto closest_res = closest_response_;
    auto normal_res = normal_response_;
    closest_response_.reset();
    normal_response_.reset();

    if (closest_res->x.empty() || closest_res->y.empty() || closest_res->z.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ClosestGrid empty arrays");
      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }

    if (!(closest_res->x.size() == closest_res->y.size() &&
          closest_res->y.size() == closest_res->z.size()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ClosestGrid size mismatch: x=%zu y=%zu z=%zu",
        closest_res->x.size(), closest_res->y.size(), closest_res->z.size());
      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }

    const size_t i = closest_res->x.size() / 2;
    if (!finite3(closest_res->x[i], closest_res->y[i], closest_res->z[i])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Closest point not finite");
      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }

    tf2::Vector3 p_cp(closest_res->x[i], closest_res->y[i], closest_res->z[i]);

    if (!finite3(normal_res->nx, normal_res->ny, normal_res->nz)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Normal not finite: [%f %f %f]",
        normal_res->nx, normal_res->ny, normal_res->nz);
      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }

    tf2::Vector3 n_meas(normal_res->nx, normal_res->ny, normal_res->nz);
    if (n_meas.length2() < 1e-10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Normal too small");
      std::lock_guard<std::mutex> lk(meas_mtx_);
      have_measurement_ = false;
      return;
    }
    n_meas.normalize();

    const double dt = perception_dt_s_;
    const double alpha = clamp(dt / (filter_tau_s_ + dt), 0.0, 1.0);

    std::lock_guard<std::mutex> lk(meas_mtx_);

    if (n_meas.dot(n_filt_) < 0.0) {
      n_meas = -n_meas;
    }

    p_cp_filt_ = (1.0 - alpha) * p_cp_filt_ + alpha * p_cp;
    n_filt_ = (1.0 - alpha) * n_filt_ + alpha * n_meas;
    if (n_filt_.length2() > 1e-10) {
      n_filt_.normalize();
    }

    have_measurement_ = true;
    last_target_update_time_ = now();
  }

  // -----------------------------
  // Target pose computation
  // -----------------------------
  bool compute_target_pose(geometry_msgs::msg::Pose & target_pose_out)
  {
    if (!move_group_) {
      return false;
    }

    tf2::Vector3 p_cp, n;
    {
      std::lock_guard<std::mutex> lk(meas_mtx_);
      if (!have_measurement_) {
        return false;
      }
      p_cp = p_cp_filt_;
      n = n_filt_;
    }

    geometry_msgs::msg::PoseStamped cur = move_group_->getCurrentPose();
    if (cur.header.frame_id.empty()) {
      return false;
    }

    const std::string planning_frame = move_group_->getPlanningFrame();
    if (planning_frame != target_frame_) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "MoveIt planning_frame='%s' != target_frame='%s'. This node does no TF transform.",
        planning_frame.c_str(), target_frame_.c_str());
      return false;
    }

    const double y_cp = p_cp.y();
    const double sign_y = (y_cp >= 0.0) ? 1.0 : -1.0;
    const double y_cp_lookahead = y_cp + sign_y * platform_speed_mps_ * lookahead_time_s_;

    target_pose_out = cur.pose;
    target_pose_out.position.x = cur.pose.position.x;
    target_pose_out.position.y = y_cp_lookahead - sign_y * standoff_m_;
    target_pose_out.position.z = z_target_;

    tf2::Vector3 forward = n;
    if (forward.length2() > 1e-12) {
      forward.normalize();
    }

    if (point_tool_into_surface_) {
      forward = -forward;
    }

    tf2::Vector3 p_cur(cur.pose.position.x, cur.pose.position.y, cur.pose.position.z);
    tf2::Vector3 to_surface = (p_cp - p_cur);
    if (to_surface.length2() > 1e-10) {
      if (forward.dot(to_surface) < 0.0) {
        forward = -forward;
      }
    }

    tf2::Quaternion q_des =
      quatAlignToolAxisToForward(forward, tf2::Vector3(0, 0, 1), tool_axis_forward_);

    tf2::Vector3 axis_tool(1, 0, 0);
    if (tool_axis_forward_ == 1) {
      axis_tool = tf2::Vector3(0, 1, 0);
    }
    if (tool_axis_forward_ == 2) {
      axis_tool = tf2::Vector3(0, 0, 1);
    }

    tf2::Quaternion q_roll;
    q_roll.setRotation(axis_tool, tool_roll_orientation_);
    q_roll.normalize();

    q_des = q_des * q_roll;
    q_des.normalize();

    target_pose_out.orientation.x = q_des.x();
    target_pose_out.orientation.y = q_des.y();
    target_pose_out.orientation.z = q_des.z();
    target_pose_out.orientation.w = q_des.w();

    return true;
  }

  // -----------------------------
  // Planning / execution
  // -----------------------------
  void mark_plan_attempt(bool failed)
  {
    last_plan_attempt_time_ = now();
    last_attempt_failed_ = failed;
  }

  void plan_cycle()
  {
    if (planning_busy_) {
      return;
    }

    if (!ensure_move_group_ready()) {
      return;
    }

    if (cooldown_active()) {
      return;
    }

    if (!have_required_joints()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for required joints on %s ...", joint_states_topic_.c_str());
      return;
    }

    if (!have_fresh_joint_state(max_joint_state_age_s_)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for fresh JointState ...");
      return;
    }

    {
      std::lock_guard<std::mutex> lk(meas_mtx_);
      if (!have_measurement_) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for perception measurement ...");
        return;
      }
    }

    if (!ensure_platform_tf_ready()) {
      return;
    }

    if (!platform_motion_started_) {
      if (!start_platform_motion()) {
        return;
      }
    } else if (execute_) {
      publish_spray_enabled(true);
    }

    auto js_msg = latest_joint_state_;
    if (!js_msg || js_msg->name.empty() || js_msg->position.size() < js_msg->name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "JointState invalid.");
      return;
    }

    planning_busy_ = true;
    auto & move_group = *move_group_;

    moveit::core::RobotState start_state =
      build_start_state_from_joint_state(robot_model_, *js_msg);
    move_group.setStartState(start_state);

    geometry_msgs::msg::Pose target_pose;
    if (!compute_target_pose(target_pose)) {
      planning_busy_ = false;
      return;
    }

    if (!should_replan(target_pose)) {
      planning_busy_ = false;
      return;
    }

    geometry_msgs::msg::PoseStamped actual_pose_stamped = move_group.getCurrentPose();
    if (!actual_pose_stamped.header.frame_id.empty()) {
      publish_actual_pose_and_path(actual_pose_stamped);
    }
    publish_desired_pose_and_path(target_pose, target_frame_);

    std::vector<double> joint_goal;
    if (!seeded_ik_to_joint_goal(
          move_group, start_state, target_pose, ik_timeout_s_, ik_attempts_, joint_goal))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Seeded IK failed (no solution) - skipping this cycle.");
      mark_plan_attempt(true);
      planning_busy_ = false;
      return;
    }

    move_group.setJointValueTarget(joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto plan_result = move_group.plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Planning failed - skipping this cycle.");
      mark_plan_attempt(true);
      planning_busy_ = false;
      return;
    }

    if (!execute_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Planned successfully (execute=false).");
      last_commanded_pose_ = target_pose;
      have_last_commanded_pose_ = true;
      mark_plan_attempt(false);
      planning_busy_ = false;
      return;
    }

    auto exec_result = move_group.execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Execution failed/aborted.");
      mark_plan_attempt(true);
      planning_busy_ = false;
      return;
    }

    last_commanded_pose_ = target_pose;
    have_last_commanded_pose_ = true;
    mark_plan_attempt(false);
    planning_busy_ = false;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VerticalPosePlannerNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
