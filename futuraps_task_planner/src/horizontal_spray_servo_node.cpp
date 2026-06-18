#include <chrono>
#include <memory>
#include <functional>
#include <future>

#include <rclcpp/rclcpp.hpp>
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


#include "futuraps_task_planner/platform_moving.hpp"
#include "futuraps_task_planner/perception_client.hpp"
#include "futuraps_task_planner/path/path_builder.hpp"
#include "futuraps_task_planner/arm_motion_interface.hpp"
#include "futuraps_task_planner/path/path_follower.hpp"
#include "futuraps_task_planner/path/reachability_filter.hpp"
#include "futuraps_task_planner/trajectory_visualizer.hpp"

class HorizontalSprayServoNode : public rclcpp::Node
{
public: 
  HorizontalSprayServoNode()
  : Node("horizontal_spray_servo"),
    platform_moving_(this),
    perception_client_(this),
    path_builder_(),
    arm_motion_(this),
    path_follower_(this),
    trajectory_visualizer_(this)
  {

    declare_parameters();
    load_parameters();

    platform_moving_.configure(make_platform_moving_config());
    perception_client_.configure(make_perception_config());
    arm_motion_.configure(make_arm_motion_config());
    path_builder_.configure(make_path_config());
    path_follower_.configure(make_path_follower_config());
    trajectory_visualizer_.configure(visualizer_fixed_frame_, visualizer_ee_frame_);

    start_servo_client_ =
      this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    spray_enabled_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/spray/enabled", 10);
    actual_spray_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/spray/horizontal/ee_actual_spray_pose", 10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(control_period_ms_),
      std::bind(&HorizontalSprayServoNode::control_loop, this));

  }

private:
  enum class PlannerState
  {
    INITIALIZING, 
    MOVE_FORWARD_START,
    MOVE_FORWARD_WAIT,
    PERCEPTION_REQUEST,
    PERCEPTION_WAIT,
    BUILD_PATH,
    MOVE_TO_INITIAL_SPRAY_POS_START,
    MOVE_TO_INITIAL_SPRAY_POS,
    EXECUTE_SPRAY_START,
    EXECUTE_SPRAY,
    MOVE_TO_HOME_POS_START,
    MOVE_TO_HOME_POS,
    ERROR,
  };

  PlannerState state_{PlannerState::INITIALIZING};

  futuraps_task_planner::PlatformMoving platform_moving_;

  futuraps_task_planner::PerceptionClient perception_client_;
  futuraps_task_planner::PerceptionResult latest_perception_;

  futuraps_task_planner::PathBuilder path_builder_;
  futuraps_task_planner::SprayPath latest_path_;
  futuraps_task_planner::HorizontalPathConfig path_config_;

  std::shared_ptr<futuraps_task_planner::ReachabilityFilter> reachability_filter_;
  bool reachability_filter_initialized_{false};

  geometry_msgs::msg::Pose home_pose_;
  bool home_pose_saved_{false};

  futuraps_task_planner::ArmMotionInterface arm_motion_;

  futuraps_task_planner::PathFollower path_follower_;

  futuraps_task_planner::TrajectoryVisualizer trajectory_visualizer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_enabled_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actual_spray_pose_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  bool start_servo_requested_{false};
  bool servo_started_{false};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture start_servo_future_;

  std::string orientation_mode_{"perception_normals"};
  bool use_perception_yaw_{false};
  bool use_platform_pid_{true};
  double predefined_tilt_up_deg_{12.0};
  double predefined_tilt_down_deg_{12.0};
  double max_inward_angle_deg_{25.0};
  double sample_spacing_{0.02};
  bool local_y_smoothing_enabled_{false};
  double local_y_smoothing_min_sharpness_{0.025};
  int local_y_smoothing_radius_{8};
  bool prune_low_z_outward_waypoints_enabled_{false};
  double prune_low_z_outward_max_z_{0.35};
  double prune_low_z_outward_min_abs_y_{1.0};
  int prune_low_z_outward_min_remaining_{5};

  bool error_logged_{false};

  // Spraying parameters
  double standoff_m_{0.4};
  double boom_length_{1.4};

  // Perception parameters
  std::string closest_srv_{"/get_closest_grid"};
  std::string normal_srv_{"/get_global_normal"};
  std::string map_frame_{"map"};
  std::string platform_motion_frame_{"odom"};
  std::string base_frame_{"platform_link"};
  std::string target_frame_{"platform_link"};
  std::string servo_command_frame_{"base_link"};
  double cell_size_{0.3};
  int rows_{7};
  int normal_rows_{5};
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
  double perception_timeout_s_{5.0};
  std::string top_point_srv_{"/get_top_canopy_point"};
  double top_band_height_{0.10};
  double top_front_percentile_{0.01};
  int top_min_points_{5};
  double top_min_x_{-0.5};
  double top_max_x_{0.5};
  double top_min_y_{-2.0};
  double top_max_y_{2.0};
  double top_min_z_{0.0};
  double top_max_z_{2.5};

  bool add_top_waypoint_{true};
  double top_waypoint_margin_{0.03};
  double min_top_extension_{0.03};
  double max_top_extension_{0.40};

  // Arm motion parameters
  std::string planning_group_name_{"ur10_arm"};
  double eef_step_{0.01};
  double jump_threshold_{0.0};
  bool avoid_collisions_{true};
  double max_velocity_scaling_{0.2};
  double max_acceleration_scaling_{0.2};
  double cartesian_velocity_scaling_{0.15};
  double cartesian_acceleration_scaling_{0.15};
  std::vector<double> home_joint_values_{};
  double home_joint_tolerance_{0.05};

  double pos_gain_{1.5};
  double ori_gain_{3.0};
  double max_linear_speed_{1.5};
  double max_angular_speed_{1.5};
  double pos_tolerance_{0.05};
  double ori_tolerance_{0.30};
  int lookahead_points_{3};
  double v_min_{0.25};
  double v_max_{0.40};

  std::string visualizer_fixed_frame_{"platform_link"};
  std::string visualizer_ee_frame_{"tool0"};
  int control_period_ms_{10};

  void declare_parameters()
  {
    declare_parameter<double>("boom_length", boom_length_);
    declare_parameter<double>("standoff_m", standoff_m_);

    declare_parameter<std::string>("orientation_mode", orientation_mode_);
    declare_parameter<bool>("use_perception_yaw", use_perception_yaw_);
    declare_parameter<bool>("use_platform_pid", use_platform_pid_);
    declare_parameter<double>("predefined_tilt_up_deg", predefined_tilt_up_deg_);
    declare_parameter<double>("predefined_tilt_down_deg", predefined_tilt_down_deg_);
    declare_parameter<double>("max_inward_angle_deg", max_inward_angle_deg_);
    declare_parameter<double>("sample_spacing", sample_spacing_);
    declare_parameter<bool>("local_y_smoothing_enabled", local_y_smoothing_enabled_);
    declare_parameter<double>("local_y_smoothing_min_sharpness", local_y_smoothing_min_sharpness_);
    declare_parameter<int>("local_y_smoothing_radius", local_y_smoothing_radius_);
    declare_parameter<bool>("prune_low_z_outward_waypoints_enabled", prune_low_z_outward_waypoints_enabled_);
    declare_parameter<double>("prune_low_z_outward_max_z", prune_low_z_outward_max_z_);
    declare_parameter<double>("prune_low_z_outward_min_abs_y", prune_low_z_outward_min_abs_y_);
    declare_parameter<int>("prune_low_z_outward_min_remaining", prune_low_z_outward_min_remaining_);
    
    //Perception params
    declare_parameter<std::string>("closest_srv", closest_srv_);
    declare_parameter<std::string>("normal_srv", normal_srv_);
    declare_parameter<std::string>("map_frame", map_frame_);
    declare_parameter<std::string>("platform_motion_frame", platform_motion_frame_);
    declare_parameter<std::string>("base_frame", base_frame_);
    declare_parameter<std::string>("target_frame", target_frame_);
    declare_parameter<std::string>("servo_command_frame", servo_command_frame_);
    declare_parameter<double>("cell_size", cell_size_);
    declare_parameter<int>("rows", rows_);
    declare_parameter<int>("normal_rows", normal_rows_);
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
    declare_parameter<std::string>("top_point_srv", top_point_srv_);
    declare_parameter<double>("top_band_height", top_band_height_);
    declare_parameter<double>("top_front_percentile", top_front_percentile_);
    declare_parameter<int>("top_min_points", top_min_points_);
    declare_parameter<double>("top_min_x", top_min_x_);
    declare_parameter<double>("top_max_x", top_max_x_);
    declare_parameter<double>("top_min_y", top_min_y_);
    declare_parameter<double>("top_max_y", top_max_y_);
    declare_parameter<double>("top_min_z", top_min_z_);
    declare_parameter<double>("top_max_z", top_max_z_);
    declare_parameter<bool>("add_top_waypoint", add_top_waypoint_);
    declare_parameter<double>("top_waypoint_margin", top_waypoint_margin_);
    declare_parameter<double>("min_top_extension", min_top_extension_);
    declare_parameter<double>("max_top_extension", max_top_extension_);
    // Arm motion params
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
    declare_parameter<double>("pos_gain", pos_gain_);
    declare_parameter<double>("ori_gain", ori_gain_);
    declare_parameter<double>("max_linear_speed", max_linear_speed_);
    declare_parameter<double>("max_angular_speed", max_angular_speed_);
    declare_parameter<double>("pos_tolerance", pos_tolerance_);
    declare_parameter<double>("ori_tolerance", ori_tolerance_);
    declare_parameter<int>("lookahead_points", lookahead_points_);
    declare_parameter<double>("v_min", v_min_);
    declare_parameter<double>("v_max", v_max_);
    declare_parameter<std::string>("visualizer_fixed_frame", visualizer_fixed_frame_);
    declare_parameter<std::string>("visualizer_ee_frame", visualizer_ee_frame_);
    declare_parameter<int>("control_period_ms", control_period_ms_);
  }

  void load_parameters()
  {
    boom_length_ = get_parameter("boom_length").as_double();
    standoff_m_ = get_parameter("standoff_m").as_double();
    orientation_mode_ = get_parameter("orientation_mode").as_string();
    use_perception_yaw_ = get_parameter("use_perception_yaw").as_bool();
    use_platform_pid_ = get_parameter("use_platform_pid").as_bool();
    predefined_tilt_up_deg_ = get_parameter("predefined_tilt_up_deg").as_double();
    predefined_tilt_down_deg_ = get_parameter("predefined_tilt_down_deg").as_double();
    max_inward_angle_deg_ = get_parameter("max_inward_angle_deg").as_double();
    sample_spacing_ = get_parameter("sample_spacing").as_double();
    local_y_smoothing_enabled_ = get_parameter("local_y_smoothing_enabled").as_bool();
    local_y_smoothing_min_sharpness_ = get_parameter("local_y_smoothing_min_sharpness").as_double();
    local_y_smoothing_radius_ = get_parameter("local_y_smoothing_radius").as_int();
    prune_low_z_outward_waypoints_enabled_ =
      get_parameter("prune_low_z_outward_waypoints_enabled").as_bool();
    prune_low_z_outward_max_z_ = get_parameter("prune_low_z_outward_max_z").as_double();
    prune_low_z_outward_min_abs_y_ = get_parameter("prune_low_z_outward_min_abs_y").as_double();
    prune_low_z_outward_min_remaining_ =
      get_parameter("prune_low_z_outward_min_remaining").as_int();
    
    //Perception params
    closest_srv_ = get_parameter("closest_srv").as_string();
    normal_srv_  = get_parameter("normal_srv").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    platform_motion_frame_ = get_parameter("platform_motion_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    servo_command_frame_ = get_parameter("servo_command_frame").as_string();
    cell_size_ = get_parameter("cell_size").as_double();
    rows_ = get_parameter("rows").as_int();
    normal_rows_ = get_parameter("normal_rows").as_int();
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
    top_point_srv_ = get_parameter("top_point_srv").as_string();
    top_band_height_ = get_parameter("top_band_height").as_double();
    top_front_percentile_ = get_parameter("top_front_percentile").as_double();
    top_min_points_ = get_parameter("top_min_points").as_int();
    top_min_x_ = get_parameter("top_min_x").as_double();
    top_max_x_ = get_parameter("top_max_x").as_double();
    top_min_y_ = get_parameter("top_min_y").as_double();
    top_max_y_ = get_parameter("top_max_y").as_double();
    top_min_z_ = get_parameter("top_min_z").as_double();
    top_max_z_ = get_parameter("top_max_z").as_double();
    add_top_waypoint_ = get_parameter("add_top_waypoint").as_bool();
    top_waypoint_margin_ = get_parameter("top_waypoint_margin").as_double();
    min_top_extension_ = get_parameter("min_top_extension").as_double();
    max_top_extension_ = get_parameter("max_top_extension").as_double();
    // Arm motion params
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
    pos_gain_ = get_parameter("pos_gain").as_double();
    ori_gain_ = get_parameter("ori_gain").as_double();
    max_linear_speed_ = get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    pos_tolerance_ = get_parameter("pos_tolerance").as_double();
    ori_tolerance_ = get_parameter("ori_tolerance").as_double();
    lookahead_points_ = get_parameter("lookahead_points").as_int();
    v_min_ = get_parameter("v_min").as_double();
    v_max_ = get_parameter("v_max").as_double();
    visualizer_fixed_frame_ = get_parameter("visualizer_fixed_frame").as_string();
    visualizer_ee_frame_ = get_parameter("visualizer_ee_frame").as_string();
    control_period_ms_ = get_parameter("control_period_ms").as_int();
  }

  futuraps_task_planner::PlatformMovingConfig make_platform_moving_config() const
  {
    futuraps_task_planner::PlatformMovingConfig config;
    config.base_frame = base_frame_;
    config.map_frame = platform_motion_frame_;
    config.use_pid = use_platform_pid_;
    return config;
  }

  futuraps_task_planner::PerceptionConfig make_perception_config() const
  {
    futuraps_task_planner::PerceptionConfig config;
    config.closest_srv = closest_srv_;
    config.normal_srv = normal_srv_;
    config.target_frame = target_frame_;
    config.cell_size = cell_size_;
    config.rows = rows_;
    config.normal_rows = normal_rows_;
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
    config.top_point_srv = top_point_srv_;
    config.top_band_height = top_band_height_;
    config.top_front_percentile = top_front_percentile_;
    config.top_min_points = top_min_points_;
    config.top_min_x = top_min_x_;
    config.top_max_x = top_max_x_;
    config.top_min_y = top_min_y_;
    config.top_max_y = top_max_y_;
    config.top_min_z = top_min_z_;
    config.top_max_z = top_max_z_;
    return config;
  }
  
  futuraps_task_planner::HorizontalPathConfig make_path_config() const
  {
    futuraps_task_planner::HorizontalPathConfig config;
    config.rows = rows_;
    config.cols = cols_;
    config.standoff_m = standoff_m_;
    config.cell_size = cell_size_;
    config.z0 = z0_;
    config.add_top_waypoint = add_top_waypoint_;
    config.top_waypoint_margin = top_waypoint_margin_;
    config.min_top_extension = min_top_extension_;
    config.max_top_extension = max_top_extension_;
    config.orientation_mode = orientation_mode_;
    config.use_perception_yaw = use_perception_yaw_;
    config.predefined_tilt_up_deg = predefined_tilt_up_deg_;
    config.predefined_tilt_down_deg = predefined_tilt_down_deg_;
    config.max_inward_angle_deg = max_inward_angle_deg_;
    config.sample_spacing = sample_spacing_;
    config.local_y_smoothing_enabled = local_y_smoothing_enabled_;
    config.local_y_smoothing_min_sharpness = local_y_smoothing_min_sharpness_;
    config.local_y_smoothing_radius = local_y_smoothing_radius_;
    config.prune_low_z_outward_waypoints_enabled = prune_low_z_outward_waypoints_enabled_;
    config.prune_low_z_outward_max_z = prune_low_z_outward_max_z_;
    config.prune_low_z_outward_min_abs_y = prune_low_z_outward_min_abs_y_;
    config.prune_low_z_outward_min_remaining = prune_low_z_outward_min_remaining_;
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

  futuraps_task_planner::PathFollowerConfig make_path_follower_config() const
  {
    futuraps_task_planner::PathFollowerConfig config;
    config.pos_gain = pos_gain_;
    config.ori_gain = ori_gain_;
    config.max_linear_speed = max_linear_speed_;
    config.max_angular_speed = max_angular_speed_;
    config.pos_tolerance = pos_tolerance_;
    config.ori_tolerance = ori_tolerance_;
    config.lookahead_points = lookahead_points_;
    config.v_min = v_min_;
    config.v_max = v_max_;
    config.path_frame = target_frame_;
    config.command_frame = servo_command_frame_;
    config.ee_frame = visualizer_ee_frame_;
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
      case PlannerState::INITIALIZING: return "INITIALIZING";
      case PlannerState::MOVE_FORWARD_START: return "MOVE_FORWARD_START";
      case PlannerState::MOVE_FORWARD_WAIT: return "MOVE_FORWARD_WAIT";
      case PlannerState::PERCEPTION_REQUEST: return "PERCEPTION_REQUEST";
      case PlannerState::PERCEPTION_WAIT: return "PERCEPTION_WAIT";
      case PlannerState::BUILD_PATH: return "BUILD_PATH";
      case PlannerState::MOVE_TO_INITIAL_SPRAY_POS_START: return "MOVE_TO_INITIAL_SPRAY_POS_START";
      case PlannerState::MOVE_TO_INITIAL_SPRAY_POS: return "MOVE_TO_INITIAL_SPRAY_POS";
      case PlannerState::EXECUTE_SPRAY_START: return "EXECUTE_SPRAY_START";
      case PlannerState::EXECUTE_SPRAY: return "EXECUTE_SPRAY";
      case PlannerState::MOVE_TO_HOME_POS_START: return "MOVE_TO_HOME_POS_START";
      case PlannerState::MOVE_TO_HOME_POS: return "MOVE_TO_HOME_POS";
      case PlannerState::ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }

  void control_loop()
  {
    switch (state_) {
      case PlannerState::INITIALIZING:
        handle_initializing();
        break;
      case PlannerState::MOVE_FORWARD_START:
        handle_move_forward_start();
        break;
      case PlannerState::MOVE_FORWARD_WAIT:
        handle_move_forward_wait();
        break;
      case PlannerState::PERCEPTION_REQUEST:
        handle_perception_request();
        break;
      case PlannerState::PERCEPTION_WAIT:
        handle_perception_wait();
        break;
      case PlannerState::BUILD_PATH:
        handle_build_path();
        break;
      case PlannerState::MOVE_TO_INITIAL_SPRAY_POS_START:
        handle_move_to_initial_spray_pos_start();
        break;
      case PlannerState::MOVE_TO_INITIAL_SPRAY_POS:
        handle_move_to_initial_spray_pos();
        break;
      case PlannerState::EXECUTE_SPRAY_START:
        handle_execute_spray_start();
        break;
      case PlannerState::EXECUTE_SPRAY:
        handle_execute_spray();
        break;
      case PlannerState::MOVE_TO_HOME_POS_START:
        handle_move_to_home_pos_start();
        break;
      case PlannerState::MOVE_TO_HOME_POS:
        handle_move_to_home_pos();
        break;
      case PlannerState::ERROR:
        handle_error();
        break;
    }
  }

  void handle_initializing()
  {
    if (!start_servo_client_->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for /servo_node/start_servo service...");
      return;
    }

    if (!start_servo_requested_) {
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      start_servo_future_ = start_servo_client_->async_send_request(req).future.share();
      start_servo_requested_ = true;
      RCLCPP_INFO(get_logger(), "Sent request to start MoveIt Servo");
      return;
    }

    if (!servo_started_) {
      if (start_servo_future_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready)
      {
        auto res = start_servo_future_.get();
        if (!res->success) {
          RCLCPP_ERROR(get_logger(), "Failed to start servo: %s", res->message.c_str());
          set_state(PlannerState::ERROR);
          return;
        }

        servo_started_ = true;
        RCLCPP_INFO(get_logger(), "MoveIt Servo started successfully");
      } else {
        return;
      }
    }

    if (!reachability_filter_initialized_) {
      reachability_filter_ =
        std::make_shared<futuraps_task_planner::ReachabilityFilter>(
          shared_from_this(),
          planning_group_name_,
          "tool0");

      if (!reachability_filter_->initialize()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize ReachabilityFilter");
        set_state(PlannerState::ERROR);
        return;
      }

      path_builder_.setReachabilityFilter(reachability_filter_);

      reachability_filter_initialized_ = true;
    }

    if (!home_pose_saved_) {
      home_pose_ = path_follower_.getCurrentPose();
      home_pose_saved_ = true;

      RCLCPP_INFO(
        get_logger(),
        "Saved home tool pose: y=%.3f z=%.3f",
        home_pose_.position.y,
        home_pose_.position.z);
    }

    set_state(PlannerState::MOVE_FORWARD_START);
  } 

  void handle_move_forward_start()
  {
    if (platform_moving_.startMovePlatformDistance(boom_length_)) {
      set_state(PlannerState::MOVE_FORWARD_WAIT);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for platform motion TF before starting move");
    }
  }

  void handle_move_forward_wait()
  {
    if (platform_moving_.motionFinished()){
      set_state(PlannerState::PERCEPTION_REQUEST);
    }
  }

  void handle_perception_request()
  {
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

      RCLCPP_INFO(
        get_logger(),
        "Perception ready. found_points=%zu, valid_normals=%zu, result.valid=%s",
        latest_perception_.x.size(),
        latest_perception_.normals.size(),
        latest_perception_.valid ? "true" : "false");

      set_state(PlannerState::BUILD_PATH);
    }
  }

  void handle_build_path()
  {
    latest_path_ = path_builder_.buildPath(latest_perception_);

    if (latest_path_.waypoints.empty()) {
      RCLCPP_WARN(get_logger(), "Path builder returned empty path");
      set_state(PlannerState::ERROR);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Path built with %zu waypoints",
      latest_path_.waypoints.size());

    trajectory_visualizer_.reset();

    const auto poses = sprayPathToPoses(latest_path_);

    trajectory_visualizer_.publishDesiredPath(poses);
    trajectory_visualizer_.publishGoalPoints(poses);

    set_state(PlannerState::MOVE_TO_INITIAL_SPRAY_POS_START);
  }

  void handle_move_to_initial_spray_pos_start()
  {
    if (latest_path_.waypoints.empty()) {
      RCLCPP_ERROR(get_logger(), "Cannot move to initial spray pose: path is empty");
      set_state(PlannerState::ERROR);
      return;
    }

    futuraps_task_planner::SprayPath initial_path;
    initial_path.frame_id = latest_path_.frame_id;
    initial_path.stamp = this->now();
    initial_path.waypoints.push_back(latest_path_.waypoints.front());

    if (!path_follower_.startPathFollowing(initial_path)) {
      RCLCPP_ERROR(get_logger(), "Failed to start path following to initial spray pose");
      set_state(PlannerState::ERROR);
      return;
    }

    RCLCPP_INFO(get_logger(), "Started Servo motion to initial spray pose");
    set_state(PlannerState::MOVE_TO_INITIAL_SPRAY_POS);
  }

  void handle_move_to_initial_spray_pos()
  {
    path_follower_.updateMoveToPose();
    trajectory_visualizer_.appendActualPose(path_follower_.getCurrentPose());

    if (path_follower_.isFinished()){
      set_state(PlannerState::EXECUTE_SPRAY_START);
    }
  }

  void handle_execute_spray_start()
  {
    if (!path_follower_.startPathFollowing(latest_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to start path following");
      set_state(PlannerState::ERROR);
      return;
    }
    publish_spray_enabled(true);

    set_state(PlannerState::EXECUTE_SPRAY);
  }

  void handle_execute_spray()
  {
    path_follower_.update();
    const auto current_pose = path_follower_.getCurrentPose();
    trajectory_visualizer_.appendActualPose(current_pose);
    publish_actual_spray_pose(current_pose);

    if (path_follower_.isFinished()){
      set_state(PlannerState::MOVE_TO_HOME_POS_START);
    }
  }

  void handle_move_to_home_pos_start()
  {
    publish_spray_enabled(false);

    futuraps_task_planner::SprayPath home_path;
    home_path.frame_id = servo_command_frame_;
    home_path.stamp = this->now();

    geometry_msgs::msg::Pose current_pose;
    current_pose = path_follower_.getCurrentPose();


    futuraps_task_planner::SprayWaypoint home_wp;
    home_wp.pose = home_pose_;
    home_wp.pose.orientation = current_pose.orientation;
    home_wp.s = 0.0;
    home_wp.tangent = Eigen::Vector3d::Zero();

    home_path.waypoints.push_back(home_wp);

    path_follower_.startPathFollowing(home_path);

    set_state(PlannerState::MOVE_TO_HOME_POS);
  }

  void handle_move_to_home_pos()
  {
    path_follower_.updateMoveToPose();

    if (path_follower_.isFinished()) {
      set_state(PlannerState::MOVE_FORWARD_START);
    }
  }

  void handle_error()
  {
    if (!error_logged_) {
      RCLCPP_ERROR(get_logger(), "Planner entered ERROR state");
      error_logged_ = true;
    }
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

  void publish_actual_spray_pose(const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = now();
    ps.header.frame_id = "platform_link";
    ps.pose = pose;

    actual_spray_pose_pub_->publish(ps);
  }

  std::vector<geometry_msgs::msg::Pose> sprayPathToPoses(
    const futuraps_task_planner::SprayPath& path) const
  {
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(path.waypoints.size());

    for (const auto& wp : path.waypoints) {
      poses.push_back(wp.pose);
    }

    return poses;
  }

};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HorizontalSprayServoNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
