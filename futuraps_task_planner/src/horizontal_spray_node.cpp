#include <chrono>
#include <memory>
#include <string>
#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "futuraps_task_planner/horizontal_path_builder.hpp"
#include "futuraps_task_planner/types.hpp"
#include "futuraps_task_planner/perception_client.hpp"
#include "futuraps_task_planner/arm_motion_interface.hpp"
#include "futuraps_task_planner/trajectory_visualizer.hpp"

using namespace std::chrono_literals;

namespace
{

void normalizeQuaternion(geometry_msgs::msg::Quaternion & q)
{
  const double n = std::sqrt(
    q.x * q.x +
    q.y * q.y +
    q.z * q.z +
    q.w * q.w);

  if (n > 1e-9) {
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
  } else {
    q.x = 0.0;
    q.y = 0.0;
    q.z = 0.0;
    q.w = 1.0;
  }
}

double quaternionDot(
  const geometry_msgs::msg::Quaternion & a,
  const geometry_msgs::msg::Quaternion & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

void enforceQuaternionContinuity(
  std::vector<geometry_msgs::msg::Pose> & waypoints,
  const geometry_msgs::msg::Quaternion & reference_q)
{
  geometry_msgs::msg::Quaternion prev = reference_q;
  normalizeQuaternion(prev);

  for (auto & pose : waypoints) {
    normalizeQuaternion(pose.orientation);

    if (quaternionDot(prev, pose.orientation) < 0.0) {
      pose.orientation.x *= -1.0;
      pose.orientation.y *= -1.0;
      pose.orientation.z *= -1.0;
      pose.orientation.w *= -1.0;
    }

    prev = pose.orientation;
  }
}

}  // namespace

class HorizontalSprayPlanExecuteNode : public rclcpp::Node
{
public:
  HorizontalSprayPlanExecuteNode()
  : Node("horizontal_spray"),
    perception_client_(this),
    arm_motion_(this),
    trajectory_visualizer_(this)
  {
    declare_parameters();
    load_parameters();

    perception_client_.configure(make_perception_config());
    arm_motion_.configure(make_arm_motion_config());
    trajectory_visualizer_.configure(base_frame_, ee_frame_);

    enable_pid_client_ =
      this->create_client<std_srvs::srv::SetBool>("/pid_controller/enable");
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/diff_drive_controller/cmd_vel", 10);
    cmd_vel_unstamped_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel_unstamped", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ee_desired_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/spray/ee_desired_pose", 10);

    ee_actual_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/spray/ee_actual_pose", 10);

    platform_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("/spray/platform_pose", 10);

    spray_enabled_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/spray/enabled", 10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&HorizontalSprayPlanExecuteNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "HorizontalSprayPlanExecuteNode started in state: %s",
      state_name(state_).c_str());
  }

private:
  enum class PlannerState
  {
    IDLE,
    MOVE_FORWARD_START,
    MOVE_FORWARD_WAIT,
    WAIT_FOR_ARM_STATE_AFTER_PLATFORM,
    PERCEPTION_REQUEST,
    PERCEPTION_WAIT,
    PLAN_SPRAY,
    MOVE_TO_SPRAY_START,
    WAIT_FOR_ARM_STATE_AFTER_MOVE_TO_START,
    EXECUTE_SPRAY,
    EXECUTE_WAIT,
    ADVANCE_SEGMENT,
    ERROR
  };

  PlannerState state_{PlannerState::IDLE};
  rclcpp::TimerBase::SharedPtr control_timer_;

  double boom_length_{1.4};
  bool use_platform_pid_{true};
  double standoff_m_{0.4};
  int tool_axis_forward_{2};
  double tool_roll_orientation_{3.1415};
  bool point_tool_into_surface_{true};

  std::string closest_srv_{"/get_closest_grid"};
  std::string normal_srv_{"/get_global_normal"};

  std::string map_frame_{"map"};
  std::string base_frame_{"platform_link"};
  std::string target_frame_{"platform_link"};

  double cell_size_{0.3};
  int rows_{5};
  int cols_{5};
  double x0_{-0.75};
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
  double normal_z_overlap_{0.3};

  double perception_timeout_s_{0.5};

  std::string planning_group_name_{"ur10_arm"};
  double eef_step_{0.01};
  double jump_threshold_{0.0};
  bool avoid_collisions_{true};
  double max_velocity_scaling_{0.2};
  double max_acceleration_scaling_{0.2};
  double cartesian_velocity_scaling_{0.15};
  double cartesian_acceleration_scaling_{0.15};

  std::string ee_frame_{"tool0"};

  std::vector<double> home_joint_values_{
    -1.57, -1.80, -2.00, 0.80, 1.57, 0.0
  };
  double home_joint_tolerance_{0.02};

  int plan_spray_retry_count_{0};
  int max_plan_spray_retries_{10};

  bool mission_started_{false};
  bool mission_completed_{false};
  bool error_logged_{false};

  double row_z_blend_{0.35};
  double max_row_delta_y_{0.25};
  double max_row_delta_z_{0.40};
  double max_abs_y_{1.20};
  double max_z_{1.60};

  int max_waypoint_repair_attempts_{5};
  double repair_position_blend_{0.5};
  double repair_first_point_blend_{0.4};
  bool drop_bad_waypoint_as_last_resort_{true};

  bool allow_stale_state_after_platform_motion_{true};
  bool allow_stale_state_for_planning_{true};
  double planning_retry_sleep_s_{0.20};

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_pid_client_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_unstamped_pub_;
  rclcpp::Time brake_hold_until_{0, 0, RCL_ROS_TIME};
  double brake_hold_duration_s_{1.0};
  bool brake_hold_active_{false};

  geometry_msgs::msg::Vector3 start_position_;
  rclcpp::Time state_wait_start_time_{0, 0, RCL_ROS_TIME};
  double state_wait_timeout_s_{1.5};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::Pose> half_waypoints_;
  std::vector<geometry_msgs::msg::Pose> waypoints_;

  futuraps_task_planner::HorizontalPathBuilder path_builder_;
  futuraps_task_planner::PerceptionClient perception_client_;
  futuraps_task_planner::PerceptionResult latest_perception_;
  futuraps_task_planner::ArmMotionInterface arm_motion_;
  futuraps_task_planner::TrajectoryVisualizer trajectory_visualizer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_desired_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_actual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr platform_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_enabled_pub_;

  void declare_parameters()
  {
    declare_parameter<double>("boom_length", boom_length_);
    declare_parameter<bool>("use_platform_pid", use_platform_pid_);
    declare_parameter<std::string>("closest_srv", closest_srv_);
    declare_parameter<std::string>("normal_srv", normal_srv_);
    declare_parameter<std::string>("map_frame", map_frame_);
    declare_parameter<std::string>("base_frame", base_frame_);
    declare_parameter<std::string>("target_frame", target_frame_);

    declare_parameter<double>("cell_size", cell_size_);
    declare_parameter<int>("rows", rows_);
    declare_parameter<int>("cols", cols_);
    declare_parameter<double>("x0", x0_);
    declare_parameter<double>("z0", z0_);
    declare_parameter<double>("y_left_max", y_left_max_);
    declare_parameter<double>("y_right_max", y_right_max_);
    declare_parameter<int>("side", side_);
    declare_parameter<double>("front_percentile", front_percentile_);
    declare_parameter<int>("min_points_per_cell", min_points_per_cell_);

    declare_parameter<double>("normal_min_x", normal_min_x_);
    declare_parameter<double>("normal_max_x", normal_max_x_);
    declare_parameter<double>("normal_min_y", normal_min_y_);
    declare_parameter<double>("normal_max_y", normal_max_y_);
    declare_parameter<double>("normal_z_overlap", normal_z_overlap_);

    declare_parameter<double>("perception_timeout_s", perception_timeout_s_);

    declare_parameter<double>("standoff_m", standoff_m_);
    declare_parameter<int>("tool_axis_forward", tool_axis_forward_);
    declare_parameter<double>("tool_roll_orientation", tool_roll_orientation_);
    declare_parameter<bool>("point_tool_into_surface", point_tool_into_surface_);

    declare_parameter<std::string>("planning_group", planning_group_name_);
    declare_parameter<double>("eef_step", eef_step_);
    declare_parameter<double>("jump_threshold", jump_threshold_);
    declare_parameter<bool>("avoid_collisions", avoid_collisions_);
    declare_parameter<double>("max_velocity_scaling", max_velocity_scaling_);
    declare_parameter<double>("max_acceleration_scaling", max_acceleration_scaling_);
    declare_parameter<double>("cartesian_velocity_scaling", cartesian_velocity_scaling_);
    declare_parameter<double>("cartesian_acceleration_scaling", cartesian_acceleration_scaling_);

    declare_parameter<std::vector<double>>("home_joint_values", home_joint_values_);
    declare_parameter<double>("home_joint_tolerance", home_joint_tolerance_);

    declare_parameter<int>("max_plan_spray_retries", max_plan_spray_retries_);

    declare_parameter<double>("row_z_blend", row_z_blend_);
    declare_parameter<double>("max_row_delta_y", max_row_delta_y_);
    declare_parameter<double>("max_row_delta_z", max_row_delta_z_);
    declare_parameter<double>("max_abs_y", max_abs_y_);
    declare_parameter<double>("max_z", max_z_);

    declare_parameter<int>("max_waypoint_repair_attempts", max_waypoint_repair_attempts_);
    declare_parameter<double>("repair_position_blend", repair_position_blend_);
    declare_parameter<double>("repair_first_point_blend", repair_first_point_blend_);
    declare_parameter<bool>("drop_bad_waypoint_as_last_resort", drop_bad_waypoint_as_last_resort_);

    declare_parameter<bool>(
      "allow_stale_state_after_platform_motion",
      allow_stale_state_after_platform_motion_);

    declare_parameter<bool>(
      "allow_stale_state_for_planning",
      allow_stale_state_for_planning_);

    declare_parameter<double>(
      "planning_retry_sleep_s",
      planning_retry_sleep_s_);

    declare_parameter<std::string>("ee_frame", ee_frame_);
  }

  void load_parameters()
  {
    boom_length_ = get_parameter("boom_length").as_double();
    use_platform_pid_ = get_parameter("use_platform_pid").as_bool();
    closest_srv_ = get_parameter("closest_srv").as_string();
    normal_srv_  = get_parameter("normal_srv").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    target_frame_ = get_parameter("target_frame").as_string();

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
    normal_z_overlap_ = get_parameter("normal_z_overlap").as_double();

    perception_timeout_s_ = get_parameter("perception_timeout_s").as_double();

    standoff_m_ = get_parameter("standoff_m").as_double();
    tool_axis_forward_ = get_parameter("tool_axis_forward").as_int();
    tool_roll_orientation_ = get_parameter("tool_roll_orientation").as_double();
    point_tool_into_surface_ = get_parameter("point_tool_into_surface").as_bool();

    planning_group_name_ = get_parameter("planning_group").as_string();
    eef_step_ = get_parameter("eef_step").as_double();
    jump_threshold_ = get_parameter("jump_threshold").as_double();
    avoid_collisions_ = get_parameter("avoid_collisions").as_bool();
    max_velocity_scaling_ = get_parameter("max_velocity_scaling").as_double();
    max_acceleration_scaling_ = get_parameter("max_acceleration_scaling").as_double();
    cartesian_velocity_scaling_ = get_parameter("cartesian_velocity_scaling").as_double();
    cartesian_acceleration_scaling_ = get_parameter("cartesian_acceleration_scaling").as_double();

    home_joint_values_ = get_parameter("home_joint_values").as_double_array();
    home_joint_tolerance_ = get_parameter("home_joint_tolerance").as_double();

    max_plan_spray_retries_ = get_parameter("max_plan_spray_retries").as_int();

    row_z_blend_ = get_parameter("row_z_blend").as_double();
    max_row_delta_y_ = get_parameter("max_row_delta_y").as_double();
    max_row_delta_z_ = get_parameter("max_row_delta_z").as_double();
    max_abs_y_ = get_parameter("max_abs_y").as_double();
    max_z_ = get_parameter("max_z").as_double();

    max_waypoint_repair_attempts_ = get_parameter("max_waypoint_repair_attempts").as_int();
    repair_position_blend_ = get_parameter("repair_position_blend").as_double();
    repair_first_point_blend_ = get_parameter("repair_first_point_blend").as_double();
    drop_bad_waypoint_as_last_resort_ = get_parameter("drop_bad_waypoint_as_last_resort").as_bool();

    allow_stale_state_after_platform_motion_ =
      get_parameter("allow_stale_state_after_platform_motion").as_bool();

    allow_stale_state_for_planning_ =
      get_parameter("allow_stale_state_for_planning").as_bool();

    planning_retry_sleep_s_ =
      get_parameter("planning_retry_sleep_s").as_double();

    ee_frame_ = get_parameter("ee_frame").as_string();
  }

  futuraps_task_planner::PerceptionConfig make_perception_config() const
  {
    futuraps_task_planner::PerceptionConfig config;
    config.closest_srv = closest_srv_;
    config.normal_srv = normal_srv_;
    config.target_frame = target_frame_;
    config.cell_size = cell_size_;
    config.rows = rows_;
    config.cols = cols_;
    config.x0 = x0_;
    config.z0 = z0_;
    config.y_left_max = y_left_max_;
    config.y_right_max = y_right_max_;
    config.side = side_;
    config.front_percentile = front_percentile_;
    config.min_points_per_cell = min_points_per_cell_;
    config.normal_min_x = normal_min_x_;
    config.normal_max_x = normal_max_x_;
    config.normal_min_y = normal_min_y_;
    config.normal_max_y = normal_max_y_;
    config.normal_z_overlap = normal_z_overlap_;
    config.timeout_s = perception_timeout_s_;
    return config;
  }

  futuraps_task_planner::ArmMotionConfig make_arm_motion_config() const
  {
    futuraps_task_planner::ArmMotionConfig config;
    config.planning_group = planning_group_name_;
    config.eef_step = eef_step_;
    config.jump_threshold = jump_threshold_;
    config.avoid_collisions = avoid_collisions_;
    config.max_velocity_scaling = max_velocity_scaling_;
    config.max_acceleration_scaling = max_acceleration_scaling_;
    config.cartesian_velocity_scaling = cartesian_velocity_scaling_;
    config.cartesian_acceleration_scaling = cartesian_acceleration_scaling_;
    config.home_joint_values = home_joint_values_;
    config.home_joint_tolerance = home_joint_tolerance_;
    return config;
  }

  void set_state(PlannerState new_state)
  {
    if (state_ != new_state) {
      RCLCPP_INFO(
        this->get_logger(),
        "State: %s -> %s",
        state_name(state_).c_str(),
        state_name(new_state).c_str());

      state_ = new_state;

      if (state_ != PlannerState::ERROR) {
        error_logged_ = false;
      }
    }
  }

  std::string state_name(PlannerState s) const
  {
    switch (s) {
      case PlannerState::IDLE: return "IDLE";
      case PlannerState::MOVE_FORWARD_START: return "MOVE_FORWARD_START";
      case PlannerState::MOVE_FORWARD_WAIT: return "MOVE_FORWARD_WAIT";
      case PlannerState::WAIT_FOR_ARM_STATE_AFTER_PLATFORM:
        return "WAIT_FOR_ARM_STATE_AFTER_PLATFORM";
      case PlannerState::PERCEPTION_REQUEST: return "PERCEPTION_REQUEST";
      case PlannerState::PERCEPTION_WAIT: return "PERCEPTION_WAIT";
      case PlannerState::PLAN_SPRAY: return "PLAN_SPRAY";
      case PlannerState::MOVE_TO_SPRAY_START: return "MOVE_TO_SPRAY_START";
      case PlannerState::WAIT_FOR_ARM_STATE_AFTER_MOVE_TO_START:
        return "WAIT_FOR_ARM_STATE_AFTER_MOVE_TO_START";
      case PlannerState::EXECUTE_SPRAY: return "EXECUTE_SPRAY";
      case PlannerState::EXECUTE_WAIT: return "EXECUTE_WAIT";
      case PlannerState::ADVANCE_SEGMENT: return "ADVANCE_SEGMENT";
      case PlannerState::ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
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

  std::vector<geometry_msgs::msg::Pose> mirror_waypoints(
    const std::vector<geometry_msgs::msg::Pose> & half_path) const
  {
    if (half_path.size() <= 1) {
      return half_path;
    }

    std::vector<geometry_msgs::msg::Pose> mirrored = half_path;
    for (int i = static_cast<int>(half_path.size()) - 2; i >= 0; --i) {
      mirrored.push_back(half_path[static_cast<size_t>(i)]);
    }
    return mirrored;
  }

  bool publish_platform_pose_from_tf()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = map_frame_;
    ps.pose.position.x = tf.transform.translation.x;
    ps.pose.position.y = tf.transform.translation.y;
    ps.pose.position.z = tf.transform.translation.z;
    ps.pose.orientation = tf.transform.rotation;
    platform_pose_pub_->publish(ps);
    return true;
  }

  bool publish_actual_ee_pose_from_tf()
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return false;
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = base_frame_;
    ps.pose.position.x = tf.transform.translation.x;
    ps.pose.position.y = tf.transform.translation.y;
    ps.pose.position.z = tf.transform.translation.z;
    ps.pose.orientation = tf.transform.rotation;
    ee_actual_pose_pub_->publish(ps);
    return true;
  }

  void publish_desired_waypoints_as_pose_stream()
  {
    for (const auto & wp : waypoints_) {
      ee_desired_pose_pub_->publish(make_pose_stamped(wp, base_frame_));
    }
  }

  void publish_visualized_waypoints()
  {
    trajectory_visualizer_.publishGoalPoints(waypoints_);
    trajectory_visualizer_.publishDesiredPath(waypoints_);
    publish_desired_waypoints_as_pose_stream();
  }

  void control_loop()
  {
    publish_brake_hold_if_active();

    trajectory_visualizer_.appendActualPoseFromTF(tf_buffer_);
    publish_actual_ee_pose_from_tf();
    publish_platform_pose_from_tf();

    switch (state_) {
      case PlannerState::IDLE:
        handle_idle();
        break;
      case PlannerState::MOVE_FORWARD_START:
        handle_move_forward_start();
        break;
      case PlannerState::MOVE_FORWARD_WAIT:
        handle_move_forward_wait();
        break;
      case PlannerState::WAIT_FOR_ARM_STATE_AFTER_PLATFORM:
        handle_wait_for_arm_state_after_platform();
        break;
      case PlannerState::PERCEPTION_REQUEST:
        handle_perception_request();
        break;
      case PlannerState::PERCEPTION_WAIT:
        handle_perception_wait();
        break;
      case PlannerState::PLAN_SPRAY:
        handle_plan_spray();
        break;
      case PlannerState::MOVE_TO_SPRAY_START:
        handle_move_to_spray_start();
        break;
      case PlannerState::WAIT_FOR_ARM_STATE_AFTER_MOVE_TO_START:
        handle_wait_for_arm_state_after_move_to_start();
        break;
      case PlannerState::EXECUTE_SPRAY:
        handle_execute_spray();
        break;
      case PlannerState::EXECUTE_WAIT:
        handle_execute_wait();
        break;
      case PlannerState::ADVANCE_SEGMENT:
        handle_advance_segment();
        break;
      case PlannerState::ERROR:
        handle_error();
        break;
    }
  }

  void begin_state_wait()
  {
    state_wait_start_time_ = now();
  }

  bool state_wait_timed_out() const
  {
    return (now() - state_wait_start_time_).seconds() > state_wait_timeout_s_;
  }

  void handle_wait_for_arm_state_after_platform()
  {
    if (arm_motion_.hasCurrentState()) {
      RCLCPP_INFO(get_logger(), "Fresh arm state received after platform motion.");
      set_state(PlannerState::PERCEPTION_REQUEST);
      return;
    }

    if (state_wait_timed_out()) {
      if (allow_stale_state_after_platform_motion_) {
        RCLCPP_WARN(
          get_logger(),
          "Robot state did not refresh after platform motion within %.2f s, continuing because allow_stale_state_after_platform_motion=true",
          state_wait_timeout_s_);
        set_state(PlannerState::PERCEPTION_REQUEST);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Robot state did not resume after platform motion within %.2f s. Check /joint_states and executor behavior.",
          state_wait_timeout_s_);
        set_state(PlannerState::ERROR);
      }
      return;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Waiting for fresh arm state after platform motion...");
  }

  void handle_wait_for_arm_state_after_move_to_start()
  {
    if (arm_motion_.hasCurrentState()) {
      RCLCPP_INFO(get_logger(), "Fresh arm state received after move-to-start.");
      if (!plan_cartesian_with_repairs()) {
        ++plan_spray_retry_count_;

        RCLCPP_WARN(
          get_logger(),
          "Could not plan Cartesian path even after repairs, retry %d/%d",
          plan_spray_retry_count_, max_plan_spray_retries_);

        if (plan_spray_retry_count_ >= max_plan_spray_retries_) {
          RCLCPP_ERROR(get_logger(), "Exceeded max retries while planning Cartesian path");
          set_state(PlannerState::ERROR);
        } else {
          set_state(PlannerState::PLAN_SPRAY);
        }
        return;
      }

      plan_spray_retry_count_ = 0;
      set_state(PlannerState::EXECUTE_SPRAY);
      return;
    }

    if (state_wait_timed_out()) {
      if (allow_stale_state_for_planning_) {
        RCLCPP_WARN(
          get_logger(),
          "Fresh state did not arrive after move-to-start within %.2f s, continuing because allow_stale_state_for_planning=true",
          state_wait_timeout_s_);

        if (!plan_cartesian_with_repairs()) {
          ++plan_spray_retry_count_;

          RCLCPP_WARN(
            get_logger(),
            "Could not plan Cartesian path even after repairs, retry %d/%d",
            plan_spray_retry_count_, max_plan_spray_retries_);

          if (plan_spray_retry_count_ >= max_plan_spray_retries_) {
            RCLCPP_ERROR(get_logger(), "Exceeded max retries while planning Cartesian path");
            set_state(PlannerState::ERROR);
          } else {
            set_state(PlannerState::PLAN_SPRAY);
          }
          return;
        }

        plan_spray_retry_count_ = 0;
        set_state(PlannerState::EXECUTE_SPRAY);
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Fresh state did not arrive after move-to-start within %.2f s.",
          state_wait_timeout_s_);
        set_state(PlannerState::ERROR);
      }
      return;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 500,
      "Waiting for fresh arm state after move-to-start...");
  }

  void handle_idle()
  {
    publish_spray_enabled(false);

    if (!arm_motion_.isInitialized()) {
      if (!arm_motion_.initialize()) {
        set_state(PlannerState::ERROR);
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "MoveIt initialized. Planning frame: %s",
        base_frame_.c_str());

      RCLCPP_INFO(
        get_logger(),
        "Joint planning scales: vel=%.3f acc=%.3f | Cartesian scales: vel=%.3f acc=%.3f",
        max_velocity_scaling_,
        max_acceleration_scaling_,
        cartesian_velocity_scaling_,
        cartesian_acceleration_scaling_);

      RCLCPP_INFO(
        get_logger(),
        "Platform motion mode: %s",
        use_platform_pid_ ? "PID-controlled" : "manual/TF-tracked");

      return;
    }

    if (!mission_started_ && !mission_completed_) {
      mission_started_ = true;
      RCLCPP_INFO(get_logger(), "Starting autonomous horizontal spraying mission.");
      set_state(PlannerState::MOVE_FORWARD_START);
    }
  }

  void handle_move_forward_start()
  {
    publish_spray_enabled(false);

    RCLCPP_INFO(get_logger(), "Starting forward movement...");
    trajectory_visualizer_.reset();

    if (!set_platform_start_position()) {
      RCLCPP_WARN(get_logger(), "Could not set platform start position yet");
      return;
    }

    if (!start_moving_forward()) {
      RCLCPP_WARN(get_logger(), "Could not start moving forward");
      return;
    }

    set_state(PlannerState::MOVE_FORWARD_WAIT);
  }

  void handle_move_forward_wait()
  {
    if (!robot_has_moved_boom_length()) {
      return;
    }

    stop_moving_forward();
    RCLCPP_INFO(get_logger(), "Robot moved required distance.");

    begin_state_wait();
    set_state(PlannerState::WAIT_FOR_ARM_STATE_AFTER_PLATFORM);
  }

  void handle_perception_request()
  {
    publish_spray_enabled(false);

    RCLCPP_INFO(get_logger(), "Sending perception requests...");

    if (!perception_client_.sendRequest()) {
      set_state(PlannerState::ERROR);
      return;
    }

    set_state(PlannerState::PERCEPTION_WAIT);
  }

  void handle_perception_wait()
  {
    if (perception_client_.checkReady()) {
      latest_perception_ = perception_client_.getResult();
      plan_spray_retry_count_ = 0;

      RCLCPP_INFO(
        get_logger(),
        "Perception ready. found_points=%zu, valid_normals=%zu, result.valid=%s",
        latest_perception_.x.size(),
        latest_perception_.normals.size(),
        latest_perception_.valid ? "true" : "false");

      set_state(PlannerState::PLAN_SPRAY);
    }
  }

  void handle_plan_spray()
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Planning spray trajectory...");

    const bool fresh_state_ok = arm_motion_.hasCurrentState();
    if (!fresh_state_ok && !allow_stale_state_for_planning_) {
      ++plan_spray_retry_count_;
      RCLCPP_WARN(
        get_logger(),
        "Waiting for state update before planning failed, retry %d/%d",
        plan_spray_retry_count_, max_plan_spray_retries_);

      if (plan_spray_retry_count_ >= max_plan_spray_retries_) {
        RCLCPP_ERROR(get_logger(), "Exceeded max retries waiting for state update");
        set_state(PlannerState::ERROR);
      }
      return;
    }

    geometry_msgs::msg::PoseStamped current_pose_stamped;
    if (!arm_motion_.getCurrentPose(current_pose_stamped)) {
      ++plan_spray_retry_count_;

      if (plan_spray_retry_count_ >= max_plan_spray_retries_) {
        RCLCPP_ERROR(get_logger(), "Exceeded max retries while fetching current pose");
        set_state(PlannerState::ERROR);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Could not get current pose yet, retry %d/%d",
          plan_spray_retry_count_, max_plan_spray_retries_);
      }
      return;
    }

    if (!build_half_pose_array_from_current_pose(current_pose_stamped.pose)) {
      ++plan_spray_retry_count_;
      RCLCPP_WARN(
        get_logger(),
        "Could not build spray half-path yet, retry %d/%d",
        plan_spray_retry_count_, max_plan_spray_retries_);

      if (plan_spray_retry_count_ >= max_plan_spray_retries_) {
        RCLCPP_ERROR(get_logger(), "Exceeded max retries while building spray half-path");
        set_state(PlannerState::ERROR);
      }
      return;
    }

    plan_spray_retry_count_ = 0;
    set_state(PlannerState::MOVE_TO_SPRAY_START);
  }

  void handle_move_to_spray_start()
  {
    if (half_waypoints_.empty()) {
      RCLCPP_ERROR(get_logger(), "Half spray path is empty");
      set_state(PlannerState::ERROR);
      return;
    }

    RCLCPP_INFO(get_logger(), "Moving arm to lowest spray start point...");
    publish_spray_enabled(false);

    if (!arm_motion_.moveToPoseCartesian(half_waypoints_.front(), 0.005, 0.0)) {
      RCLCPP_ERROR(get_logger(), "Failed to move to spray start pose with Cartesian motion");
      set_state(PlannerState::ERROR);
      return;
    }

    begin_state_wait();
    set_state(PlannerState::WAIT_FOR_ARM_STATE_AFTER_MOVE_TO_START);
  }

  void handle_execute_spray()
  {
    RCLCPP_INFO(get_logger(), "Starting spray execution...");
    publish_spray_enabled(true);

    const bool success = arm_motion_.executePlannedPath();

    publish_spray_enabled(false);

    if (success) {
      set_state(PlannerState::EXECUTE_WAIT);
    } else {
      set_state(PlannerState::ERROR);
    }
  }

  void handle_execute_wait()
  {
    if (spray_execution_done()) {
      RCLCPP_INFO(get_logger(), "Spray execution finished.");
      set_state(PlannerState::ADVANCE_SEGMENT);
    }
  }

  void handle_advance_segment()
  {
    publish_spray_enabled(false);

    RCLCPP_INFO(get_logger(), "Moving arm back to ready pose");
    const bool robot_homed = arm_motion_.moveHome();
    if (!robot_homed) {
      RCLCPP_ERROR(get_logger(), "Robot arm not able to return home");
      set_state(PlannerState::ERROR);
      return;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(get_logger(), "Robot returned to ready/home pose");
    RCLCPP_INFO(get_logger(), "Advancing to next segment...");

    const bool finished_all_segments = advance_segment();

    if (finished_all_segments) {
      RCLCPP_INFO(get_logger(), "All segments completed.");
      mission_completed_ = true;
      set_state(PlannerState::IDLE);
    } else {
      set_state(PlannerState::MOVE_FORWARD_START);
    }
  }

  void handle_error()
  {
    publish_spray_enabled(false);

    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "State machine entered ERROR state.");

    control_timer_->cancel();
  }

  bool start_moving_forward()
  {
    brake_hold_active_ = false;

    if (!use_platform_pid_) {
      RCLCPP_INFO(
        get_logger(),
        "Platform PID disabled by parameter; waiting for operator to move the robot");
      return true;
    }

    if (!enable_pid_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "PID enable service not ready");
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    enable_pid_client_->async_send_request(req);

    RCLCPP_INFO(get_logger(), "Sent request to enable PID controller (move forward)");
    return true;
  }

  bool stop_moving_forward()
  {
    if (!use_platform_pid_) {
      RCLCPP_INFO(
        get_logger(),
        "Platform PID disabled by parameter; segment distance reached while manually driving");
      return true;
    }

    if (!enable_pid_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "PID enable service not ready");
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = false;
    enable_pid_client_->async_send_request(req);
    start_brake_hold();

    RCLCPP_INFO(get_logger(), "Sent request to stop PID controller (stop moving forward)");
    return true;
  }

  void start_brake_hold()
  {
    brake_hold_active_ = true;
    brake_hold_until_ = now() + rclcpp::Duration::from_seconds(brake_hold_duration_s_);
    publish_zero_platform_command();
  }

  void publish_zero_platform_command()
  {
    geometry_msgs::msg::TwistStamped stamped;
    stamped.header.stamp = now();
    stamped.header.frame_id = base_frame_;
    cmd_vel_pub_->publish(stamped);

    geometry_msgs::msg::Twist unstamped;
    cmd_vel_unstamped_pub_->publish(unstamped);
  }

  void publish_brake_hold_if_active()
  {
    if (brake_hold_active_ || now() <= brake_hold_until_) {
      publish_zero_platform_command();
    }
  }

  bool set_platform_start_position()
  {
    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }

    start_position_ = tf.transform.translation;
    return true;
  }

  bool robot_has_moved_boom_length()
  {
    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }

    const auto current = tf.transform.translation;

    const double dx = current.x - start_position_.x;
    const double dy = current.y - start_position_.y;
    const double dz = current.z - start_position_.z;

    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Moved distance: %.3f / %.3f", dist, boom_length_);

    return dist >= boom_length_;
  }

  bool build_half_pose_array_from_current_pose(const geometry_msgs::msg::Pose & current_pose)
  {
    RCLCPP_INFO(
      get_logger(),
      "Current pose from MoveIt: x=%.3f y=%.3f z=%.3f q=[%.3f %.3f %.3f %.3f]",
      current_pose.position.x,
      current_pose.position.y,
      current_pose.position.z,
      current_pose.orientation.x,
      current_pose.orientation.y,
      current_pose.orientation.z,
      current_pose.orientation.w);

    futuraps_task_planner::HorizontalPathConfig config;
    config.rows = rows_;
    config.cols = cols_;
    config.standoff_m = standoff_m_;
    config.tool_axis_forward = tool_axis_forward_;
    config.tool_roll_orientation = tool_roll_orientation_;
    config.point_tool_into_surface = point_tool_into_surface_;
    config.cell_size = cell_size_;
    config.z0 = z0_;
    config.row_z_blend = row_z_blend_;
    config.max_row_delta_y = max_row_delta_y_;
    config.max_row_delta_z = max_row_delta_z_;
    config.max_abs_y = max_abs_y_;
    config.max_z = max_z_;

    half_waypoints_ = path_builder_.buildWaypoints(current_pose, latest_perception_, config);

    geometry_msgs::msg::PoseStamped current_pose_stamped;
    if (arm_motion_.getCurrentPose(current_pose_stamped)) {
      enforceQuaternionContinuity(
        half_waypoints_,
        current_pose_stamped.pose.orientation);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Could not fetch current pose for quaternion continuity enforcement");
    }

    RCLCPP_INFO(get_logger(), "Built %zu half-waypoints", half_waypoints_.size());

    for (size_t k = 0; k < half_waypoints_.size(); ++k) {
      const auto & p = half_waypoints_[k];
      RCLCPP_INFO(
        get_logger(),
        "Half waypoint %zu: x=%.3f y=%.3f z=%.3f q=[%.3f %.3f %.3f %.3f]",
        k,
        p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
    }

    return half_waypoints_.size() >= 2;
  }

  void update_full_path_from_half_path()
  {
    waypoints_ = mirror_waypoints(half_waypoints_);

    geometry_msgs::msg::PoseStamped current_pose_stamped;
    if (arm_motion_.getCurrentPose(current_pose_stamped)) {
      enforceQuaternionContinuity(
        waypoints_,
        current_pose_stamped.pose.orientation);
    }

    publish_visualized_waypoints();
  }

  bool plan_cartesian_path()
  {
    return arm_motion_.planCartesianPath(waypoints_);
  }

  double quat_dot(
    const geometry_msgs::msg::Quaternion & a,
    const geometry_msgs::msg::Quaternion & b) const
  {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
  }

  geometry_msgs::msg::Quaternion normalized_quat(geometry_msgs::msg::Quaternion q) const
  {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n < 1e-12) {
      q.x = 0.0;
      q.y = 0.0;
      q.z = 0.0;
      q.w = 1.0;
      return q;
    }
    q.x /= n;
    q.y /= n;
    q.z /= n;
    q.w /= n;
    return q;
  }

  geometry_msgs::msg::Quaternion blend_quat(
    geometry_msgs::msg::Quaternion a,
    geometry_msgs::msg::Quaternion b,
    double alpha) const
  {
    a = normalized_quat(a);
    b = normalized_quat(b);

    double d = quat_dot(a, b);
    if (d < 0.0) {
      b.x = -b.x;
      b.y = -b.y;
      b.z = -b.z;
      b.w = -b.w;
    }

    geometry_msgs::msg::Quaternion q;
    q.x = (1.0 - alpha) * a.x + alpha * b.x;
    q.y = (1.0 - alpha) * a.y + alpha * b.y;
    q.z = (1.0 - alpha) * a.z + alpha * b.z;
    q.w = (1.0 - alpha) * a.w + alpha * b.w;
    return normalized_quat(q);
  }

  double half_waypoint_outlier_score(
    const std::vector<geometry_msgs::msg::Pose> & wps,
    size_t i) const
  {
    if (wps.size() < 3 || i == 0 || i + 1 >= wps.size()) {
      return -1.0;
    }

    const auto & prev = wps[i - 1];
    const auto & cur  = wps[i];
    const auto & next = wps[i + 1];

    const double dy_prev = std::abs(cur.position.y - prev.position.y);
    const double dz_prev = std::abs(cur.position.z - prev.position.z);
    const double dy_next = std::abs(next.position.y - cur.position.y);
    const double dz_next = std::abs(next.position.z - cur.position.z);

    const double mid_y = 0.5 * (prev.position.y + next.position.y);
    const double mid_z = 0.5 * (prev.position.z + next.position.z);

    const double midpoint_dev =
      std::abs(cur.position.y - mid_y) +
      0.7 * std::abs(cur.position.z - mid_z);

    const auto q_prev = normalized_quat(prev.orientation);
    const auto q_cur  = normalized_quat(cur.orientation);
    const auto q_next = normalized_quat(next.orientation);

    const double qjump_prev = 1.0 - std::abs(quat_dot(q_prev, q_cur));
    const double qjump_next = 1.0 - std::abs(quat_dot(q_cur, q_next));

    double score =
      1.0 * (dy_prev + dy_next) +
      0.8 * (dz_prev + dz_next) +
      1.2 * midpoint_dev +
      1.5 * (qjump_prev + qjump_next);

    if (i == 1 || i + 2 == wps.size()) {
      score *= 1.5;
    }

    return score;
  }

  size_t find_worst_half_waypoint_index(
    const std::vector<geometry_msgs::msg::Pose> & wps) const
  {
    if (wps.size() < 3) {
      return 0;
    }

    double best_score = -1.0;
    size_t best_idx = 0;

    for (size_t i = 1; i + 1 < wps.size(); ++i) {
      const double score = half_waypoint_outlier_score(wps, i);
      if (score > best_score) {
        best_score = score;
        best_idx = i;
      }
    }

    return best_idx;
  }

  void repair_half_waypoint_in_place(
    std::vector<geometry_msgs::msg::Pose> & wps,
    size_t idx)
  {
    if (wps.size() < 3 || idx == 0 || idx + 1 >= wps.size()) {
      return;
    }

    auto & cur = wps[idx];
    const auto & prev = wps[idx - 1];
    const auto & next = wps[idx + 1];

    const double mid_y = 0.5 * (prev.position.y + next.position.y);
    const double mid_z = 0.5 * (prev.position.z + next.position.z);

    cur.position.y =
      (1.0 - repair_position_blend_) * cur.position.y +
      repair_position_blend_ * mid_y;

    cur.position.z =
      (1.0 - repair_position_blend_) * cur.position.z +
      repair_position_blend_ * mid_z;

    const auto q_mid = blend_quat(prev.orientation, next.orientation, 0.5);
    cur.orientation = blend_quat(cur.orientation, q_mid, repair_position_blend_);
  }

  bool drop_half_waypoint_in_place(std::vector<geometry_msgs::msg::Pose> & wps, size_t idx)
  {
    if (wps.size() <= 4) {
      return false;
    }
    if (idx == 0 || idx >= wps.size() - 1) {
      return false;
    }

    wps.erase(wps.begin() + static_cast<long>(idx));
    return true;
  }

  bool plan_cartesian_with_repairs()
  {
    update_full_path_from_half_path();

    if (plan_cartesian_path()) {
      RCLCPP_INFO(get_logger(), "Cartesian plan succeeded without waypoint repair.");
      return true;
    }

    std::vector<geometry_msgs::msg::Pose> candidate_half = half_waypoints_;

    for (int attempt = 0; attempt < max_waypoint_repair_attempts_; ++attempt) {
      const size_t worst_idx = find_worst_half_waypoint_index(candidate_half);
      if (worst_idx == 0) {
        break;
      }

      RCLCPP_WARN(
        get_logger(),
        "Cartesian planning failed. Repair attempt %d/%d on half-waypoint %zu",
        attempt + 1, max_waypoint_repair_attempts_, worst_idx);

      repair_half_waypoint_in_place(candidate_half, worst_idx);

      half_waypoints_ = candidate_half;
      update_full_path_from_half_path();

      if (plan_cartesian_path()) {
        RCLCPP_INFO(
          get_logger(),
          "Cartesian plan succeeded after repairing half-waypoint %zu",
          worst_idx);
        return true;
      }
    }

    if (drop_bad_waypoint_as_last_resort_) {
      std::vector<geometry_msgs::msg::Pose> candidate_drop = half_waypoints_;

      for (int attempt = 0; attempt < 2; ++attempt) {
        if (candidate_drop.size() < 3) {
          break;
        }

        size_t idx_to_drop = 1;
        if (attempt == 1 && candidate_drop.size() > 3) {
          idx_to_drop = candidate_drop.size() - 2;
        }

        if (!drop_half_waypoint_in_place(candidate_drop, idx_to_drop)) {
          break;
        }

        RCLCPP_WARN(
          get_logger(),
          "Dropping half-waypoint %zu as last resort and retrying Cartesian planning",
          idx_to_drop);

        half_waypoints_ = candidate_drop;
        update_full_path_from_half_path();

        if (plan_cartesian_path()) {
          RCLCPP_INFO(get_logger(), "Cartesian plan succeeded after dropping boundary half-waypoint.");
          return true;
        }
      }
    }

    return false;
  }

  bool spray_execution_done()
  {
    return true;
  }

  bool advance_segment()
  {
    return false;
  }

  void publish_spray_enabled(bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    spray_enabled_pub_->publish(msg);

    RCLCPP_INFO(
      get_logger(),
      "Published /spray/enabled = %s",
      enabled ? "true" : "false");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HorizontalSprayPlanExecuteNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
