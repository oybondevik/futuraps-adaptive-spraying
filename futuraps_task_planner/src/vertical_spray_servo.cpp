#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_set>
#include <rclcpp/create_timer.hpp>

#include "futuraps_task_planner/path/spline.h"
#include "futuraps_perception/srv/get_closest_grid.hpp"
#include "futuraps_perception/srv/get_global_normal.hpp"

#include <chrono>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <limits>

using namespace std::chrono_literals;

class VerticalSprayServo : public rclcpp::Node
{
public:
  VerticalSprayServo() : Node("vertical_spray_servo")
  {
    declare_parameters();
    load_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    closest_grid_client_ = this->create_client<futuraps_perception::srv::GetClosestGrid>(closest_srv_);
    global_normal_client_ = this->create_client<futuraps_perception::srv::GetGlobalNormal>(normal_srv_);

    // Match Servo subscriber QoS (sensor data)
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(servo_twist_topic_, qos);

    // Logging publishers
    actual_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/spray/vertical/ee_actual_pose", 10);
    desired_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/spray/vertical/ee_desired_pose", 10);
    horizon_target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/spray/vertical/horizon_target_pose", 10);
    pos_error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
      "/spray/vertical/ee_pos_error", 10);
    actual_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/actual_path", 10);
    desired_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/desired_path", 10);
    perception_surface_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/perception_surface_path", 10);
    perception_offset_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/perception_offset_path", 10);
    perception_raw_surface_waypoints_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/perception_raw_surface_waypoints", 10);
    perception_raw_offset_waypoints_pub_ = this->create_publisher<nav_msgs::msg::Path>(
      "/spray/vertical/perception_raw_offset_waypoints", 10);
    spray_enabled_pub_ =
      this->create_publisher<std_msgs::msg::Bool>(spray_enabled_topic_, 10);

    actual_path_msg_.header.frame_id = base_frame_;
    desired_path_msg_.header.frame_id = base_frame_;
    publish_spray_enabled(false, true);
    open_pose_debug_csv();
    open_horizon_debug_csv();

    // --- Joint states (for readiness gating) ---
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(10),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        for (const auto& n : msg->name) {
          seen_joints_.insert(n);
        }
      }
    );

    // --- Servo start client + timer ---
    start_servo_client_ = this->create_client<std_srvs::srv::Trigger>(servo_start_service_);
    enable_pid_client_ = this->create_client<std_srvs::srv::SetBool>(pid_enable_service_);

    if (auto_start_servo_) {
      auto_start_begin_time_ = now();

      start_servo_timer_ = rclcpp::create_timer(
        this,
        this->get_clock(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(auto_start_check_dt_s_)),
        [this]() { this->try_auto_start_servo(); }
      );
    }

    perception_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(perception_dt_s_)),
      std::bind(&VerticalSprayServo::perception_update, this));

    control_timer_ = rclcpp::create_timer(
      this,
      this->get_clock(),
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(control_dt_s_)),
      std::bind(&VerticalSprayServo::control_update, this));

    RCLCPP_INFO(get_logger(),
      "VerticalSprayServo started. control_dt=%.4f perception_dt=%.4f standoff=%.3f z_target=%.3f base_frame=%s planning_frame=%s closest_grid_frame=%s orientation_mode=%s log_every_n=%d",
      control_dt_s_, perception_dt_s_, standoff_m_, z_target_,
      base_frame_.c_str(), planning_frame_.c_str(), closest_grid_frame_.c_str(),
      orientation_mode_.c_str(), log_every_n_);
  }

private:
  // ---------------- Params ----------------
  double control_dt_s_{0.01};     // 100 Hz
  double perception_dt_s_{0.10};  // 10 Hz

  double standoff_m_{0.20};       // 20 cm toward robot from closest point (toward y=0)
  double z_target_{1.0};          // fixed z height (half plant height placeholder)
  double platform_speed_mps_{0.0};
  double lookahead_time_s_{0.0};
  bool path_tracking_enabled_{false};
  double path_horizon_m_{1.0};
  double path_waypoint_spacing_m_{0.20};
  double path_follow_lookahead_m_{0.08};
  double path_rebuild_period_s_{0.10};

  double filter_tau_s_{0.20};     // filter time constant for point/normal

  double kp_pos_{1.0};            // linear velocity gain (m/s per m)
  double kp_rot_{2.0};            // angular velocity gain (rad/s per rad)
  double v_max_{0.10};
  double w_max_{0.80};

  int tool_axis_forward_{2}; // 0=X, 1=Y, 2=Z
  double tool_roll_orientation_{0.0};
  std::string orientation_mode_{"perception_normal_world_up"};
  bool orientation_clamp_enabled_{false};
  double orientation_clamp_max_deviation_deg_{25.0};
  bool platform_y_keepout_enabled_{false};
  double platform_min_abs_y_{0.60};
  bool canopy_gap_guard_enabled_{true};
  double canopy_gap_max_outward_jump_m_{0.12};
  double canopy_gap_outward_speed_mps_{0.08};
  double canopy_gap_inward_speed_mps_{0.40};

  bool auto_start_servo_{true};
  double auto_start_check_dt_s_{0.2};
  double auto_start_timeout_s_{10.0};
  std::string servo_start_service_{"/servo_node/start_servo"};
  bool auto_start_pid_{true};
  std::string pid_enable_service_{"/pid_controller/enable"};
  bool auto_enable_spray_{true};
  std::string spray_enabled_topic_{"/spray/enabled"};
  std::vector<std::string> required_joints_{
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
  };

  // Logging decimation. 1 = every control step, 50 = every 50th control step.
  int log_every_n_{50};
  int max_path_history_{500};
  std::string pose_debug_csv_path_{"/tmp/vertical_servo_pose_debug.csv"};
  std::string horizon_debug_csv_path_{"/tmp/vertical_servo_horizon_path.csv"};
  std::string horizon_debug_frame_{"map"};
  std::size_t log_counter_{0};

  // --- ClosestGrid request params ---
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

  // --- GlobalNormal request params ---
  double normal_min_x_{-0.5};
  double normal_max_x_{0.5};
  double normal_min_y_{-2.0};
  double normal_max_y_{2.0};
  double normal_min_z_{0.2};
  double normal_max_z_{2.0};

  std::string base_frame_{"base_link"};
  std::string planning_frame_{"map"};
  std::string closest_grid_frame_{""};
  std::string ee_frame_{"tool0"};

  std::string servo_twist_topic_{"/servo_node/delta_twist_cmds"};
  std::string closest_srv_{"/get_closest_grid"};
  std::string normal_srv_{"/get_global_normal"};
  double perception_timeout_s_{0.3};

  // If true, tool forward axis points INTO surface (forward = -normal)
  bool point_tool_into_surface_{true};

  // ---------------- ROS ----------------
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr actual_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr horizon_target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pos_error_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr desired_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr perception_surface_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr perception_offset_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr perception_raw_surface_waypoints_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr perception_raw_offset_waypoints_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_enabled_pub_;

  nav_msgs::msg::Path actual_path_msg_;
  nav_msgs::msg::Path desired_path_msg_;
  std::ofstream pose_debug_file_;
  std::ofstream horizon_debug_file_;

  struct HorizonPoint
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  bool have_horizon_path_{false};
  std::vector<double> horizon_s_;
  std::vector<double> horizon_x_;
  std::vector<double> horizon_y_;
  std::vector<double> horizon_z_;
  tk::spline horizon_x_spline_;
  tk::spline horizon_y_spline_;
  tk::spline horizon_z_spline_;
  std::vector<HorizonPoint> perception_surface_horizon_points_;  // planning_frame_
  std::vector<HorizonPoint> perception_horizon_points_;          // standoff targets in planning_frame_
  bool horizon_update_pending_{false};
  rclcpp::Time last_horizon_rebuild_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Client<futuraps_perception::srv::GetClosestGrid>::SharedPtr closest_grid_client_;
  rclcpp::Client<futuraps_perception::srv::GetGlobalNormal>::SharedPtr global_normal_client_;

  rclcpp::TimerBase::SharedPtr perception_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_pid_client_;
  rclcpp::TimerBase::SharedPtr start_servo_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  std::unordered_set<std::string> seen_joints_;
  bool servo_started_{false};
  bool auto_start_finished_{false};
  bool platform_pid_started_{false};
  bool last_spray_enabled_{false};
  rclcpp::Time last_spray_enabled_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time auto_start_begin_time_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture start_servo_future_;
  bool start_servo_pending_{false};

  // ---------------- Filtered state from perception ----------------
  bool have_measurement_{false};
  tf2::Vector3 p_cp_filt_{0.0, 0.5, 1.0}; // filtered closest point in base_frame_
  tf2::Vector3 n_filt_{0, -1, 0};         // filtered unit normal in base_frame_
  bool have_guarded_target_y_{false};
  double guarded_target_y_{0.0};

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

  // ---------------- Utility ----------------
  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  static double deg_to_rad(double deg)
  {
    constexpr double pi = 3.14159265358979323846;
    return deg * pi / 180.0;
  }

  static bool finite3(double a, double b, double c)
  {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
  }

  static bool finite_twist(const geometry_msgs::msg::TwistStamped& c)
  {
    return std::isfinite(c.twist.linear.x)  && std::isfinite(c.twist.linear.y)  && std::isfinite(c.twist.linear.z) &&
           std::isfinite(c.twist.angular.x) && std::isfinite(c.twist.angular.y) && std::isfinite(c.twist.angular.z);
  }

  static tf2::Quaternion quatAlignToolAxisToForward(
    const tf2::Vector3& forward_in,
    const tf2::Vector3& up_hint_in,
    int tool_axis)
  {
    tf2::Vector3 forward = forward_in;
    if (forward.length2() < 1e-12) forward = tf2::Vector3(1,0,0);
    forward.normalize();

    tf2::Vector3 up_hint = up_hint_in;
    if (up_hint.length2() < 1e-12) up_hint = tf2::Vector3(0,0,1);
    up_hint.normalize();

    if (std::fabs(forward.dot(up_hint)) > 0.95) up_hint = tf2::Vector3(1,0,0);

    tf2::Vector3 b = up_hint.cross(forward);
    if (b.length2() < 1e-12) b = tf2::Vector3(0,1,0);
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
      X.z(), Y.z(), Z.z()
    );

    tf2::Quaternion q;
    R.getRotation(q);
    q.normalize();
    return q;
  }

  static tf2::Vector3 clampDirectionToCone(
    const tf2::Vector3 & direction_in,
    const tf2::Vector3 & reference_in,
    double max_angle_rad,
    double * original_angle_out = nullptr)
  {
    tf2::Vector3 direction = direction_in;
    tf2::Vector3 reference = reference_in;

    if (direction.length2() < 1e-12 || reference.length2() < 1e-12) {
      if (original_angle_out) {
        *original_angle_out = 0.0;
      }
      return direction_in;
    }

    direction.normalize();
    reference.normalize();

    const double dot = clamp(direction.dot(reference), -1.0, 1.0);
    const double angle = std::acos(dot);
    if (original_angle_out) {
      *original_angle_out = angle;
    }

    if (angle <= max_angle_rad) {
      return direction;
    }

    tf2::Vector3 axis = reference.cross(direction);
    if (axis.length2() < 1e-12) {
      return reference;
    }
    axis.normalize();

    tf2::Quaternion q;
    q.setRotation(axis, max_angle_rad);
    q.normalize();

    tf2::Vector3 clamped = tf2::quatRotate(q, reference);
    clamped.normalize();
    return clamped;
  }

  bool usePathTangentOrientation() const
  {
    return orientation_mode_ == "perception_normal_path_tangent" ||
           orientation_mode_ == "perception_normals_path_tangent" ||
           orientation_mode_ == "path_tangent";
  }

  bool useStaticYawOrientation() const
  {
    return orientation_mode_ == "perception_normal_static_yaw" ||
           orientation_mode_ == "static_yaw";
  }

  static tf2::Vector3 keepOnlyVerticalTilt(
    const tf2::Vector3 & forward_in,
    double sign_y)
  {
    tf2::Vector3 forward = forward_in;
    if (forward.length2() < 1e-12) {
      return tf2::Vector3(0, sign_y, 0);
    }
    forward.normalize();

    tf2::Vector3 adjusted(
      0.0,
      sign_y * std::abs(forward.y()),
      forward.z());

    if (adjusted.length2() < 1e-12) {
      adjusted = tf2::Vector3(0.0, sign_y, 0.0);
    }

    adjusted.normalize();
    return adjusted;
  }

  static tf2::Vector3 removeForwardComponentAlongTangent(
    const tf2::Vector3 & forward_in,
    const tf2::Vector3 & tangent_in)
  {
    tf2::Vector3 forward = forward_in;
    if (forward.length2() < 1e-12) {
      forward = tf2::Vector3(0, 1, 0);
    }
    forward.normalize();

    tf2::Vector3 tangent = tangent_in;
    if (tangent.length2() < 1e-12) {
      tangent = tf2::Vector3(0, 0, 1);
    }
    tangent.normalize();

    tf2::Vector3 adjusted = forward - tangent * forward.dot(tangent);
    if (adjusted.length2() < 1e-12) {
      return forward;
    }

    adjusted.normalize();
    return adjusted;
  }

  double applyPlatformYKeepout(double requested_y, double sign_y, bool warn)
  {
    double target_y = requested_y;

    if (platform_y_keepout_enabled_ &&
        platform_min_abs_y_ > 0.0 &&
        std::abs(target_y) < platform_min_abs_y_)
    {
      target_y = sign_y * platform_min_abs_y_;
      if (warn) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Platform y keepout active: requested y=%.3f clamped to %.3f",
          requested_y, target_y);
      }
    }

    return target_y;
  }

  double applyCanopyGapGuard(double requested_y, double sign_y)
  {
    if (!canopy_gap_guard_enabled_) {
      guarded_target_y_ = requested_y;
      have_guarded_target_y_ = true;
      return requested_y;
    }

    if (!have_guarded_target_y_ || sign_y * guarded_target_y_ <= 0.0) {
      guarded_target_y_ = requested_y;
      have_guarded_target_y_ = true;
      return requested_y;
    }

    const double previous_abs_y = std::abs(guarded_target_y_);
    const double requested_abs_y = std::abs(requested_y);
    const double delta_abs_y = requested_abs_y - previous_abs_y;

    double max_step = canopy_gap_inward_speed_mps_ * control_dt_s_;
    if (delta_abs_y > 0.0) {
      max_step = canopy_gap_outward_speed_mps_ * control_dt_s_;

      if (delta_abs_y > canopy_gap_max_outward_jump_m_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Canopy gap guard holding outward target jump: requested |y| %.3f from %.3f",
          requested_abs_y, previous_abs_y);
      }
    }

    const double limited_abs_y =
      previous_abs_y + clamp(delta_abs_y, -max_step, max_step);
    guarded_target_y_ = sign_y * limited_abs_y;
    return guarded_target_y_;
  }

  bool have_required_joints() const
  {
    if (required_joints_.empty()) return true;

    for (const auto& j : required_joints_) {
      if (seen_joints_.find(j) == seen_joints_.end()) {
        return false;
      }
    }
    return true;
  }

  bool have_required_tf()
  {
    try {
      (void)tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
      if (path_tracking_enabled_ && planning_frame_ != base_frame_) {
        (void)tf_buffer_->lookupTransform(planning_frame_, base_frame_, tf2::TimePointZero);
        (void)tf_buffer_->lookupTransform(base_frame_, planning_frame_, tf2::TimePointZero);
      }
      return true;
    } catch (...) {
      return false;
    }
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

  bool try_start_platform_pid()
  {
    if (!auto_start_pid_ || platform_pid_started_) {
      return true;
    }

    if (!enable_pid_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "PID enable service not ready: %s", pid_enable_service_.c_str());
      return false;
    }

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = true;
    enable_pid_client_->async_send_request(req);

    platform_pid_started_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Sent request to enable PID controller for vertical spraying (%s)",
      pid_enable_service_.c_str());
    return true;
  }

  std::vector<HorizonPoint> extractHorizonPoints(const ClosestResponse & closest_res) const
  {
    std::vector<HorizonPoint> points;
    const std::size_t expected_size =
      static_cast<std::size_t>(std::max(0, rows_)) *
      static_cast<std::size_t>(std::max(0, cols_));

    if (expected_size == 0 ||
        closest_res.found.size() < expected_size ||
        closest_res.x.size() < expected_size ||
        closest_res.y.size() < expected_size ||
        closest_res.z.size() < expected_size)
    {
      return points;
    }

    points.reserve(static_cast<std::size_t>(cols_));

    for (int col = 0; col < cols_; ++col) {
      bool have_best = false;
      HorizonPoint best;
      const double requested_side = side_ == 0 ? 0.0 : static_cast<double>(side_);

      for (int row = 0; row < rows_; ++row) {
        const std::size_t idx =
          static_cast<std::size_t>(row) * static_cast<std::size_t>(cols_) +
          static_cast<std::size_t>(col);

        if (!closest_res.found[idx]) {
          continue;
        }
        if (!finite3(closest_res.x[idx], closest_res.y[idx], closest_res.z[idx])) {
          continue;
        }

        const HorizonPoint candidate{
          static_cast<double>(closest_res.x[idx]),
          static_cast<double>(closest_res.y[idx]),
          static_cast<double>(closest_res.z[idx])};
        if (requested_side > 0.0 && candidate.y < 0.0) {
          continue;
        }
        if (requested_side < 0.0 && candidate.y > 0.0) {
          continue;
        }

        if (!have_best || std::abs(candidate.y) < std::abs(best.y)) {
          best = candidate;
          have_best = true;
        }
      }

      if (have_best) {
        points.push_back(best);
      }
    }

    const bool forward_order = platform_speed_mps_ >= 0.0;
    std::sort(
      points.begin(),
      points.end(),
      [forward_order](const HorizonPoint & a, const HorizonPoint & b) {
        return forward_order ? a.x < b.x : a.x > b.x;
      });

    return points;
  }

  static HorizonPoint transformHorizonPoint(
    const HorizonPoint & point,
    const geometry_msgs::msg::TransformStamped & transform)
  {
    geometry_msgs::msg::PointStamped in;
    geometry_msgs::msg::PointStamped out;
    in.header = transform.header;
    in.point.x = point.x;
    in.point.y = point.y;
    in.point.z = point.z;
    tf2::doTransform(in, out, transform);
    return HorizonPoint{out.point.x, out.point.y, out.point.z};
  }

  bool transformHorizonPoints(
    const std::vector<HorizonPoint> & in,
    const std::string & source_frame,
    const std::string & target_frame,
    std::vector<HorizonPoint> & out,
    const rclcpp::Time & stamp = rclcpp::Time(0, 0, RCL_ROS_TIME),
    bool allow_latest_fallback = false)
  {
    out.clear();
    if (source_frame == target_frame) {
      out = in;
      return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
    } catch (const tf2::TransformException & ex) {
      if (allow_latest_fallback && stamp.nanoseconds() != 0) {
        try {
          transform = tf_buffer_->lookupTransform(
            target_frame, source_frame, tf2::TimePointZero);
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Horizon point TF %s <- %s unavailable at %.3fs (%s); using latest TF fallback",
            target_frame.c_str(),
            source_frame.c_str(),
            stamp.seconds(),
            ex.what());
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Horizon point latest TF fallback %s <- %s unavailable: %s",
            target_frame.c_str(),
            source_frame.c_str(),
            latest_ex.what());
          return false;
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "Horizon point TF %s <- %s unavailable: %s",
          target_frame.c_str(),
          source_frame.c_str(),
          ex.what());
        return false;
      }
    }

    out.reserve(in.size());
    for (const auto & point : in) {
      out.push_back(transformHorizonPoint(point, transform));
    }
    return true;
  }

  bool transformHorizonPoint(
    const HorizonPoint & in,
    const std::string & source_frame,
    const std::string & target_frame,
    HorizonPoint & out,
    const rclcpp::Time & stamp = rclcpp::Time(0, 0, RCL_ROS_TIME),
    bool allow_latest_fallback = false)
  {
    if (source_frame == target_frame) {
      out = in;
      return true;
    }

    try {
      const auto transform =
        tf_buffer_->lookupTransform(target_frame, source_frame, stamp);
      out = transformHorizonPoint(in, transform);
      return true;
    } catch (const tf2::TransformException & ex) {
      if (allow_latest_fallback && stamp.nanoseconds() != 0) {
        try {
          const auto transform =
            tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
          out = transformHorizonPoint(in, transform);
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Horizon target TF %s <- %s unavailable at %.3fs (%s); using latest TF fallback",
            target_frame.c_str(),
            source_frame.c_str(),
            stamp.seconds(),
            ex.what());
          return true;
        } catch (const tf2::TransformException & latest_ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Horizon target latest TF fallback %s <- %s unavailable: %s",
            target_frame.c_str(),
            source_frame.c_str(),
            latest_ex.what());
          return false;
        }
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Horizon target TF %s <- %s unavailable: %s",
        target_frame.c_str(),
        source_frame.c_str(),
        ex.what());
      return false;
    }
  }

  std::size_t selectCurrentClosestIndex(const ClosestResponse & closest_res) const
  {
    std::size_t best_idx = 0;
    bool have_best = false;
    const std::size_t expected_size =
      std::min(
        closest_res.found.size(),
        std::min(
          closest_res.x.size(),
          std::min(closest_res.y.size(), closest_res.z.size())));

    for (std::size_t idx = 0; idx < expected_size; ++idx) {
      if (!closest_res.found[idx]) {
        continue;
      }
      if (!finite3(closest_res.x[idx], closest_res.y[idx], closest_res.z[idx])) {
        continue;
      }

      const double y = static_cast<double>(closest_res.y[idx]);
      if (side_ > 0 && y < 0.0) {
        continue;
      }
      if (side_ < 0 && y > 0.0) {
        continue;
      }

      const double best_y = static_cast<double>(closest_res.y[best_idx]);
      if (!have_best || std::abs(y) < std::abs(best_y)) {
        best_idx = idx;
        have_best = true;
      }
    }

    return best_idx;
  }

  void try_auto_start_servo()
  {
    if (servo_started_ || auto_start_finished_) {
      return;
    }

    if (this->get_clock()->now().nanoseconds() == 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for non-zero ROS clock before auto-starting Servo...");
      return;
    }

    if (auto_start_begin_time_.nanoseconds() == 0) {
      auto_start_begin_time_ = now();
    }

    const double elapsed = (now() - auto_start_begin_time_).seconds();
    if (elapsed > auto_start_timeout_s_) {
      RCLCPP_WARN(
        get_logger(),
        "Auto-start Servo timed out after %.1fs. You can still call %s manually.",
        auto_start_timeout_s_, servo_start_service_.c_str());
      auto_start_finished_ = true;
      return;
    }

    if (!start_servo_client_->service_is_ready()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s ...", servo_start_service_.c_str());
      return;
    }

    if (!have_required_tf()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for TF %s -> %s ...", base_frame_.c_str(), ee_frame_.c_str());
      return;
    }

    if (!have_required_joints()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for required joints on /joint_states ...");
      return;
    }

    if (start_servo_pending_) {
      if (start_servo_future_.valid() &&
          start_servo_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
      {
        auto res = start_servo_future_.get();
        if (res->success) {
          RCLCPP_INFO(get_logger(), "Servo started automatically (%s).", servo_start_service_.c_str());
          servo_started_ = true;
          try_start_platform_pid();
        } else {
          RCLCPP_WARN(get_logger(), "Servo start returned success=false: %s", res->message.c_str());
        }
        start_servo_pending_ = false;
      }
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_servo_future_ = start_servo_client_->async_send_request(req).future.share();
    start_servo_pending_ = true;

    RCLCPP_INFO(get_logger(), "Calling %s ...", servo_start_service_.c_str());
  }

  void declare_parameters()
  {
    this->declare_parameter("control_dt_s", control_dt_s_);
    this->declare_parameter("perception_dt_s", perception_dt_s_);
    this->declare_parameter("standoff_m", standoff_m_);
    this->declare_parameter("z_target", z_target_);
    this->declare_parameter("platform_speed_mps", platform_speed_mps_);
    this->declare_parameter("lookahead_time_s", lookahead_time_s_);
    this->declare_parameter("path_tracking_enabled", path_tracking_enabled_);
    this->declare_parameter("path_horizon_m", path_horizon_m_);
    this->declare_parameter("path_waypoint_spacing_m", path_waypoint_spacing_m_);
    this->declare_parameter("path_follow_lookahead_m", path_follow_lookahead_m_);
    this->declare_parameter("path_rebuild_period_s", path_rebuild_period_s_);
    this->declare_parameter("filter_tau_s", filter_tau_s_);

    this->declare_parameter("kp_pos", kp_pos_);
    this->declare_parameter("kp_rot", kp_rot_);
    this->declare_parameter("v_max", v_max_);
    this->declare_parameter("w_max", w_max_);

    this->declare_parameter("base_frame", base_frame_);
    this->declare_parameter("planning_frame", planning_frame_);
    this->declare_parameter("closest_grid_frame", closest_grid_frame_);
    this->declare_parameter("ee_frame", ee_frame_);
    this->declare_parameter("servo_twist_topic", servo_twist_topic_);
    this->declare_parameter("closest_srv", closest_srv_);
    this->declare_parameter("normal_srv", normal_srv_);
    this->declare_parameter("perception_timeout_s", perception_timeout_s_);
    this->declare_parameter("point_tool_into_surface", point_tool_into_surface_);
    this->declare_parameter("tool_axis_forward", tool_axis_forward_);
    this->declare_parameter("tool_roll_orientation", tool_roll_orientation_);
    this->declare_parameter("orientation_mode", orientation_mode_);
    this->declare_parameter("orientation_clamp_enabled", orientation_clamp_enabled_);
    this->declare_parameter("orientation_clamp_max_deviation_deg", orientation_clamp_max_deviation_deg_);
    this->declare_parameter("platform_y_keepout_enabled", platform_y_keepout_enabled_);
    this->declare_parameter("platform_min_abs_y", platform_min_abs_y_);
    this->declare_parameter("canopy_gap_guard_enabled", canopy_gap_guard_enabled_);
    this->declare_parameter("canopy_gap_max_outward_jump_m", canopy_gap_max_outward_jump_m_);
    this->declare_parameter("canopy_gap_outward_speed_mps", canopy_gap_outward_speed_mps_);
    this->declare_parameter("canopy_gap_inward_speed_mps", canopy_gap_inward_speed_mps_);

    this->declare_parameter("cell_size", cell_size_);
    this->declare_parameter("rows", rows_);
    this->declare_parameter("cols", cols_);
    this->declare_parameter("x0", x0_);
    this->declare_parameter("z0", z0_);
    this->declare_parameter("y_left_max", y_left_max_);
    this->declare_parameter("y_right_max", y_right_max_);
    this->declare_parameter("side", side_);
    this->declare_parameter("front_percentile", front_percentile_);
    this->declare_parameter("min_points_per_cell", min_points_per_cell_);

    this->declare_parameter("normal_min_x", normal_min_x_);
    this->declare_parameter("normal_max_x", normal_max_x_);
    this->declare_parameter("normal_min_y", normal_min_y_);
    this->declare_parameter("normal_max_y", normal_max_y_);
    this->declare_parameter("normal_min_z", normal_min_z_);
    this->declare_parameter("normal_max_z", normal_max_z_);

    this->declare_parameter("auto_start_servo", auto_start_servo_);
    this->declare_parameter("auto_start_check_dt_s", auto_start_check_dt_s_);
    this->declare_parameter("auto_start_timeout_s", auto_start_timeout_s_);
    this->declare_parameter("servo_start_service", servo_start_service_);
    this->declare_parameter("auto_start_pid", auto_start_pid_);
    this->declare_parameter("pid_enable_service", pid_enable_service_);
    this->declare_parameter("auto_enable_spray", auto_enable_spray_);
    this->declare_parameter("spray_enabled_topic", spray_enabled_topic_);
    this->declare_parameter("required_joints", required_joints_);

    this->declare_parameter("log_every_n", log_every_n_);
    this->declare_parameter("max_path_history", max_path_history_);
    this->declare_parameter("pose_debug_csv_path", pose_debug_csv_path_);
    this->declare_parameter("horizon_debug_csv_path", horizon_debug_csv_path_);
    this->declare_parameter("horizon_debug_frame", horizon_debug_frame_);
  }

  void load_parameters()
  {
    this->get_parameter("control_dt_s", control_dt_s_);
    this->get_parameter("perception_dt_s", perception_dt_s_);
    this->get_parameter("standoff_m", standoff_m_);
    this->get_parameter("z_target", z_target_);
    this->get_parameter("platform_speed_mps", platform_speed_mps_);
    this->get_parameter("lookahead_time_s", lookahead_time_s_);
    this->get_parameter("path_tracking_enabled", path_tracking_enabled_);
    this->get_parameter("path_horizon_m", path_horizon_m_);
    this->get_parameter("path_waypoint_spacing_m", path_waypoint_spacing_m_);
    this->get_parameter("path_follow_lookahead_m", path_follow_lookahead_m_);
    this->get_parameter("path_rebuild_period_s", path_rebuild_period_s_);
    this->get_parameter("filter_tau_s", filter_tau_s_);

    this->get_parameter("kp_pos", kp_pos_);
    this->get_parameter("kp_rot", kp_rot_);
    this->get_parameter("v_max", v_max_);
    this->get_parameter("w_max", w_max_);

    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("planning_frame", planning_frame_);
    this->get_parameter("closest_grid_frame", closest_grid_frame_);
    if (planning_frame_.empty()) {
      planning_frame_ = base_frame_;
    }
    if (closest_grid_frame_.empty()) {
      closest_grid_frame_ = base_frame_;
    }
    this->get_parameter("ee_frame", ee_frame_);
    this->get_parameter("servo_twist_topic", servo_twist_topic_);
    this->get_parameter("closest_srv", closest_srv_);
    this->get_parameter("normal_srv", normal_srv_);
    this->get_parameter("perception_timeout_s", perception_timeout_s_);
    this->get_parameter("point_tool_into_surface", point_tool_into_surface_);
    this->get_parameter("tool_axis_forward", tool_axis_forward_);
    this->get_parameter("tool_roll_orientation", tool_roll_orientation_);
    this->get_parameter("orientation_mode", orientation_mode_);
    this->get_parameter("orientation_clamp_enabled", orientation_clamp_enabled_);
    this->get_parameter("orientation_clamp_max_deviation_deg", orientation_clamp_max_deviation_deg_);
    this->get_parameter("platform_y_keepout_enabled", platform_y_keepout_enabled_);
    this->get_parameter("platform_min_abs_y", platform_min_abs_y_);
    this->get_parameter("canopy_gap_guard_enabled", canopy_gap_guard_enabled_);
    this->get_parameter("canopy_gap_max_outward_jump_m", canopy_gap_max_outward_jump_m_);
    this->get_parameter("canopy_gap_outward_speed_mps", canopy_gap_outward_speed_mps_);
    this->get_parameter("canopy_gap_inward_speed_mps", canopy_gap_inward_speed_mps_);

    this->get_parameter("cell_size", cell_size_);
    this->get_parameter("rows", rows_);
    this->get_parameter("cols", cols_);
    this->get_parameter("x0", x0_);
    this->get_parameter("z0", z0_);
    this->get_parameter("y_left_max", y_left_max_);
    this->get_parameter("y_right_max", y_right_max_);
    this->get_parameter("side", side_);
    this->get_parameter("front_percentile", front_percentile_);
    this->get_parameter("min_points_per_cell", min_points_per_cell_);

    this->get_parameter("normal_min_x", normal_min_x_);
    this->get_parameter("normal_max_x", normal_max_x_);
    this->get_parameter("normal_min_y", normal_min_y_);
    this->get_parameter("normal_max_y", normal_max_y_);
    this->get_parameter("normal_min_z", normal_min_z_);
    this->get_parameter("normal_max_z", normal_max_z_);

    this->get_parameter("auto_start_servo", auto_start_servo_);
    this->get_parameter("auto_start_check_dt_s", auto_start_check_dt_s_);
    this->get_parameter("auto_start_timeout_s", auto_start_timeout_s_);
    this->get_parameter("servo_start_service", servo_start_service_);
    this->get_parameter("auto_start_pid", auto_start_pid_);
    this->get_parameter("pid_enable_service", pid_enable_service_);
    this->get_parameter("auto_enable_spray", auto_enable_spray_);
    this->get_parameter("spray_enabled_topic", spray_enabled_topic_);
    this->get_parameter("required_joints", required_joints_);

    this->get_parameter("log_every_n", log_every_n_);
    if (log_every_n_ < 1) {
      log_every_n_ = 1;
    }
    this->get_parameter("max_path_history", max_path_history_);
    if (max_path_history_ < 1) {
      max_path_history_ = 1;
    }
    this->get_parameter("pose_debug_csv_path", pose_debug_csv_path_);
    this->get_parameter("horizon_debug_csv_path", horizon_debug_csv_path_);
    this->get_parameter("horizon_debug_frame", horizon_debug_frame_);
  }

  void perception_update()
  {
    if (!closest_grid_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for %s", closest_srv_.c_str());
      return;
    }
    if (!global_normal_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for %s", normal_srv_.c_str());
      return;
    }

    if (!closest_pending_ && !normal_pending_ &&
        !closest_response_ && !normal_response_)
    {
      auto closest_req = std::make_shared<futuraps_perception::srv::GetClosestGrid::Request>();
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

      auto normal_req = std::make_shared<futuraps_perception::srv::GetGlobalNormal::Request>();
      normal_req->frame_id = base_frame_;
      normal_req->min_x = normal_min_x_;
      normal_req->max_x = normal_max_x_;
      normal_req->min_y = normal_min_y_;
      normal_req->max_y = normal_max_y_;
      normal_req->min_z = normal_min_z_;
      normal_req->max_z = normal_max_z_;

      closest_future_ = closest_grid_client_->async_send_request(closest_req).future.share();
      normal_future_  = global_normal_client_->async_send_request(normal_req).future.share();
      closest_pending_ = true;
      normal_pending_  = true;
      perception_start_time_ = now();
      return;
    }

    if ((closest_pending_ || normal_pending_) &&
        (now() - perception_start_time_).seconds() > perception_timeout_s_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Perception timeout (closest/normal)");
      closest_pending_ = false;
      normal_pending_  = false;
      closest_response_.reset();
      normal_response_.reset();
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

    if (closest_pending_ || normal_pending_) return;
    if (!closest_response_ || !normal_response_) return;

    auto closest_res = closest_response_;
    auto normal_res = normal_response_;
    closest_response_.reset();
    normal_response_.reset();
    const rclcpp::Time cloud_stamp(closest_res->cloud_stamp);
    const std::string closest_result_frame =
      closest_res->cloud_frame_id.empty() ? closest_grid_frame_ : closest_res->cloud_frame_id;

    if (closest_res->x.empty() || closest_res->y.empty() || closest_res->z.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ClosestGrid empty arrays");
      have_measurement_ = false;
      return;
    }
    if (!(closest_res->x.size() == closest_res->y.size() &&
          closest_res->y.size() == closest_res->z.size()))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "ClosestGrid size mismatch: x=%zu y=%zu z=%zu",
        closest_res->x.size(), closest_res->y.size(), closest_res->z.size());
      have_measurement_ = false;
      return;
    }

    std::vector<HorizonPoint> closest_horizon_points = extractHorizonPoints(*closest_res);
    if (closest_horizon_points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "ClosestGrid has no usable horizon points");
      have_measurement_ = false;
      return;
    }

    const size_t i = selectCurrentClosestIndex(*closest_res);
    if (!finite3(closest_res->x[i], closest_res->y[i], closest_res->z[i])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Closest point not finite");
      have_measurement_ = false;
      return;
    }

    HorizonPoint closest_point_in_base;
    if (!transformHorizonPoint(
          HorizonPoint{
            static_cast<double>(closest_res->x[i]),
            static_cast<double>(closest_res->y[i]),
            static_cast<double>(closest_res->z[i])},
          closest_result_frame,
          base_frame_,
          closest_point_in_base,
          cloud_stamp,
          true))
    {
      have_measurement_ = false;
      return;
    }

    tf2::Vector3 p_cp(
      closest_point_in_base.x,
      closest_point_in_base.y,
      closest_point_in_base.z);
    const double sign_y = p_cp.y() >= 0.0 ? 1.0 : -1.0;

    std::vector<HorizonPoint> closest_horizon_points_base;
    if (!transformHorizonPoints(
          closest_horizon_points,
          closest_result_frame,
          base_frame_,
          closest_horizon_points_base,
          cloud_stamp,
          true))
    {
      have_measurement_ = false;
      return;
    }

    std::vector<HorizonPoint> surface_horizon_points_planning;
    std::vector<HorizonPoint> target_horizon_points_planning;
    surface_horizon_points_planning.reserve(closest_horizon_points_base.size());
    target_horizon_points_planning.reserve(closest_horizon_points_base.size());

    for (const auto & point_base : closest_horizon_points_base) {
      HorizonPoint surface_point_planning;
      if (!transformHorizonPoint(
            point_base,
            base_frame_,
            planning_frame_,
            surface_point_planning,
            cloud_stamp,
            true))
      {
        have_measurement_ = false;
        return;
      }

      const double requested_y = point_base.y - sign_y * standoff_m_;
      const double target_y = applyPlatformYKeepout(requested_y, sign_y, false);
      HorizonPoint target_point_planning;
      if (!transformHorizonPoint(
            HorizonPoint{point_base.x, target_y, z_target_},
            base_frame_,
            planning_frame_,
            target_point_planning,
            cloud_stamp,
            true))
      {
        have_measurement_ = false;
        return;
      }

      surface_horizon_points_planning.push_back(surface_point_planning);
      target_horizon_points_planning.push_back(target_point_planning);
    }

    if (target_horizon_points_planning.size() < 3) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ClosestGrid has fewer than 3 usable transformed horizon points");
      have_measurement_ = false;
      return;
    }

    perception_surface_horizon_points_ = surface_horizon_points_planning;
    perception_horizon_points_ = target_horizon_points_planning;
    horizon_update_pending_ = true;

    if (!finite3(normal_res->nx, normal_res->ny, normal_res->nz)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Normal not finite: [%f %f %f]", normal_res->nx, normal_res->ny, normal_res->nz);
      have_measurement_ = false;
      return;
    }

    tf2::Vector3 n_meas(normal_res->nx, normal_res->ny, normal_res->nz);
    if (n_meas.length2() < 1e-10) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Normal too small");
      have_measurement_ = false;
      return;
    }
    n_meas.normalize();

    const double dt = perception_dt_s_;
    const double alpha = clamp(dt / (filter_tau_s_ + dt), 0.0, 1.0);

    p_cp_filt_ = (1.0 - alpha) * p_cp_filt_ + alpha * p_cp;

    if (n_meas.dot(n_filt_) < 0.0) {
      n_meas = -n_meas;
    }

    n_filt_ = (1.0 - alpha) * n_filt_ + alpha * n_meas;
    if (n_filt_.length2() > 1e-10) n_filt_.normalize();

    have_measurement_ = true;
  }

  void open_pose_debug_csv()
  {
    if (pose_debug_csv_path_.empty()) {
      return;
    }

    pose_debug_file_.open(pose_debug_csv_path_, std::ios::out | std::ios::trunc);
    if (!pose_debug_file_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Could not open vertical servo pose debug CSV: %s",
        pose_debug_csv_path_.c_str());
      return;
    }

    pose_debug_file_ << std::setprecision(12);
    pose_debug_file_
      << "t,"
      << "actual_x,actual_y,actual_z,"
      << "desired_x,desired_y,desired_z,"
      << "err_x,err_y,err_z,err_norm,"
      << "actual_qx,actual_qy,actual_qz,actual_qw,"
      << "desired_qx,desired_qy,desired_qz,desired_qw,"
      << "orientation_error_rad\n";

    RCLCPP_INFO(
      get_logger(),
      "Writing vertical servo pose debug CSV to %s",
      pose_debug_csv_path_.c_str());
  }

  void write_pose_debug_sample(
    const tf2::Vector3 & p_cur,
    const tf2::Quaternion & q_cur,
    const tf2::Vector3 & p_tgt,
    const tf2::Quaternion & q_des,
    const tf2::Vector3 & e_pos)
  {
    if (!pose_debug_file_.is_open()) {
      return;
    }

    tf2::Quaternion q_err = q_des * q_cur.inverse();
    q_err.normalize();
    if (q_err.w() < 0.0) {
      q_err = tf2::Quaternion(-q_err.x(), -q_err.y(), -q_err.z(), -q_err.w());
    }
    const double orientation_error =
      2.0 * std::acos(clamp(static_cast<double>(q_err.w()), -1.0, 1.0));

    pose_debug_file_
      << now().seconds() << ","
      << p_cur.x() << "," << p_cur.y() << "," << p_cur.z() << ","
      << p_tgt.x() << "," << p_tgt.y() << "," << p_tgt.z() << ","
      << e_pos.x() << "," << e_pos.y() << "," << e_pos.z() << ","
      << e_pos.length() << ","
      << q_cur.x() << "," << q_cur.y() << "," << q_cur.z() << "," << q_cur.w() << ","
      << q_des.x() << "," << q_des.y() << "," << q_des.z() << "," << q_des.w() << ","
      << orientation_error << "\n";

    pose_debug_file_.flush();
  }

  void open_horizon_debug_csv()
  {
    if (horizon_debug_csv_path_.empty()) {
      return;
    }

    horizon_debug_file_.open(horizon_debug_csv_path_, std::ios::out | std::ios::trunc);
    if (!horizon_debug_file_.is_open()) {
      RCLCPP_WARN(
        get_logger(),
        "Could not open vertical servo horizon debug CSV: %s",
        horizon_debug_csv_path_.c_str());
      return;
    }

    horizon_debug_file_ << std::setprecision(12);
    horizon_debug_file_
      << "t,point_type,point_index,s,"
      << "local_x,local_y,local_z,"
      << "debug_x,debug_y,debug_z\n";

    RCLCPP_INFO(
      get_logger(),
      "Writing vertical servo horizon debug CSV to %s using debug frame '%s'",
      horizon_debug_csv_path_.c_str(),
      horizon_debug_frame_.c_str());
  }

  bool shouldRebuildHorizonPath()
  {
    if (!horizon_update_pending_) {
      return false;
    }
    if (!have_horizon_path_) {
      return true;
    }
    if (last_horizon_rebuild_time_.nanoseconds() == 0) {
      return true;
    }
    return (now() - last_horizon_rebuild_time_).seconds() >= path_rebuild_period_s_;
  }

  geometry_msgs::msg::PoseStamped makePathPose(
    const HorizonPoint & point,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = point.z;
    pose.pose.orientation.w = 1.0;
    return pose;
  }

  void publishControllerPerceptionPaths(
    const std::vector<HorizonPoint> & surface_points_planning,
    const std::vector<HorizonPoint> & raw_surface_waypoints_planning,
    const std::vector<HorizonPoint> & raw_offset_waypoints_planning,
    const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Path surface_path;
    surface_path.header.stamp = stamp;
    surface_path.header.frame_id = planning_frame_;

    for (const auto & point_planning : surface_points_planning) {
      surface_path.poses.push_back(makePathPose(point_planning, planning_frame_, stamp));
    }

    nav_msgs::msg::Path raw_surface_path;
    raw_surface_path.header.stamp = stamp;
    raw_surface_path.header.frame_id = planning_frame_;
    for (const auto & point_planning : raw_surface_waypoints_planning) {
      raw_surface_path.poses.push_back(makePathPose(point_planning, planning_frame_, stamp));
    }

    nav_msgs::msg::Path raw_offset_path;
    raw_offset_path.header.stamp = stamp;
    raw_offset_path.header.frame_id = planning_frame_;
    for (const auto & point_planning : raw_offset_waypoints_planning) {
      raw_offset_path.poses.push_back(makePathPose(point_planning, planning_frame_, stamp));
    }

    nav_msgs::msg::Path offset_path;
    offset_path.header.stamp = stamp;
    offset_path.header.frame_id = planning_frame_;
    const std::size_t sample_count = std::max<std::size_t>(80, horizon_s_.size() * 16);
    const double s_start = horizon_s_.front();
    const double s_end = horizon_s_.back();
    for (std::size_t i = 0; i < sample_count; ++i) {
      const double t =
        sample_count > 1 ? static_cast<double>(i) / static_cast<double>(sample_count - 1) : 0.0;
      const double s = s_start + t * (s_end - s_start);
      offset_path.poses.push_back(
        makePathPose(
          HorizonPoint{
            horizon_x_spline_(s),
            horizon_y_spline_(s),
            horizon_z_spline_(s)},
          planning_frame_,
          stamp));
    }

    perception_surface_path_pub_->publish(surface_path);
    perception_offset_path_pub_->publish(offset_path);
    perception_raw_surface_waypoints_pub_->publish(raw_surface_path);
    perception_raw_offset_waypoints_pub_->publish(raw_offset_path);
  }

  bool rebuildHorizonPath()
  {
    std::vector<double> s_values;
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> z_values;
    std::vector<HorizonPoint> raw_surface_waypoints_planning;
    std::vector<HorizonPoint> raw_offset_waypoints_planning;
    s_values.reserve(perception_horizon_points_.size());
    x_values.reserve(perception_horizon_points_.size());
    y_values.reserve(perception_horizon_points_.size());
    z_values.reserve(perception_horizon_points_.size());
    raw_surface_waypoints_planning.reserve(perception_horizon_points_.size());
    raw_offset_waypoints_planning.reserve(perception_horizon_points_.size());

    if (perception_horizon_points_.size() < 3) {
      have_horizon_path_ = false;
      return false;
    }

    const double horizon = std::max(path_horizon_m_, path_waypoint_spacing_m_ * 2.0);
    bool have_start = false;
    double x_start = 0.0;
    double previous_s = -1.0;
    for (std::size_t point_index = 0; point_index < perception_horizon_points_.size(); ++point_index) {
      const auto & point = perception_horizon_points_[point_index];
      if (!have_start) {
        x_start = point.x;
        have_start = true;
      }

      const double s = std::abs(point.x - x_start);
      if (s > horizon) {
        continue;
      }
      if (s <= previous_s + 1e-4) {
        continue;
      }

      s_values.push_back(s);
      x_values.push_back(point.x);
      y_values.push_back(point.y);
      z_values.push_back(point.z);
      if (point_index < perception_surface_horizon_points_.size()) {
        raw_surface_waypoints_planning.push_back(
          perception_surface_horizon_points_[point_index]);
      }
      raw_offset_waypoints_planning.push_back(point);
      previous_s = s;
    }

    if (s_values.size() < 3) {
      have_horizon_path_ = false;
      return false;
    }

    horizon_x_spline_.set_points(s_values, x_values, tk::spline::cspline_hermite);
    horizon_x_spline_.make_monotonic();
    horizon_y_spline_.set_points(s_values, y_values, tk::spline::cspline_hermite);
    horizon_y_spline_.make_monotonic();
    horizon_z_spline_.set_points(s_values, z_values, tk::spline::linear);
    horizon_s_ = s_values;
    horizon_x_ = x_values;
    horizon_y_ = y_values;
    horizon_z_ = z_values;
    have_horizon_path_ = true;
    horizon_update_pending_ = false;
    last_horizon_rebuild_time_ = now();
    publishControllerPerceptionPaths(
      perception_surface_horizon_points_,
      raw_surface_waypoints_planning,
      raw_offset_waypoints_planning,
      last_horizon_rebuild_time_);

    if (horizon_debug_file_.is_open()) {
      bool debug_frame_is_planning = horizon_debug_frame_ == planning_frame_;
      bool have_debug_transform = false;
      geometry_msgs::msg::TransformStamped debug_transform;
      if (!debug_frame_is_planning) {
        try {
          debug_transform = tf_buffer_->lookupTransform(
            horizon_debug_frame_, planning_frame_, tf2::TimePointZero);
          have_debug_transform = true;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Horizon debug TF %s <- %s unavailable: %s",
            horizon_debug_frame_.c_str(),
            planning_frame_.c_str(),
            ex.what());
        }
      }

      auto write_horizon_debug_point =
        [&](
          const char * point_type,
          std::size_t point_index,
          double s,
          double planning_x,
          double planning_y,
          double planning_z)
      {
        double local_x = std::numeric_limits<double>::quiet_NaN();
        double local_y = std::numeric_limits<double>::quiet_NaN();
        double local_z = std::numeric_limits<double>::quiet_NaN();
        double debug_x = planning_x;
        double debug_y = planning_y;
        double debug_z = planning_z;

        HorizonPoint local_point;
        if (transformHorizonPoint(
              HorizonPoint{planning_x, planning_y, planning_z},
              planning_frame_,
              base_frame_,
              local_point))
        {
          local_x = local_point.x;
          local_y = local_point.y;
          local_z = local_point.z;
        }

        if (debug_frame_is_planning) {
          debug_x = planning_x;
          debug_y = planning_y;
          debug_z = planning_z;
        } else if (have_debug_transform) {
          geometry_msgs::msg::PointStamped planning_point;
          geometry_msgs::msg::PointStamped debug_point;
          planning_point.header.stamp = last_horizon_rebuild_time_;
          planning_point.header.frame_id = planning_frame_;
          planning_point.point.x = planning_x;
          planning_point.point.y = planning_y;
          planning_point.point.z = planning_z;
          tf2::doTransform(planning_point, debug_point, debug_transform);
          debug_x = debug_point.point.x;
          debug_y = debug_point.point.y;
          debug_z = debug_point.point.z;
        }

        horizon_debug_file_
          << last_horizon_rebuild_time_.seconds() << ","
          << point_type << ","
          << point_index << ","
          << s << ","
          << local_x << ","
          << local_y << ","
          << local_z << ","
          << debug_x << ","
          << debug_y << ","
          << debug_z << "\n";
      };

      for (std::size_t i = 0; i < horizon_s_.size(); ++i) {
        if (i < raw_surface_waypoints_planning.size()) {
          write_horizon_debug_point(
            "surface_waypoint",
            i,
            horizon_s_[i],
            raw_surface_waypoints_planning[i].x,
            raw_surface_waypoints_planning[i].y,
            raw_surface_waypoints_planning[i].z);
        }
        if (i < raw_offset_waypoints_planning.size()) {
          write_horizon_debug_point(
            "offset_waypoint",
            i,
            horizon_s_[i],
            raw_offset_waypoints_planning[i].x,
            raw_offset_waypoints_planning[i].y,
            raw_offset_waypoints_planning[i].z);
        }
        write_horizon_debug_point(
          "waypoint", i, horizon_s_[i], horizon_x_[i], horizon_y_[i], horizon_z_[i]);
      }

      const std::size_t sample_count = std::max<std::size_t>(80, horizon_s_.size() * 16);
      const double s_start = horizon_s_.front();
      const double s_end = horizon_s_.back();
      for (std::size_t i = 0; i < sample_count; ++i) {
        const double t =
          sample_count > 1 ? static_cast<double>(i) / static_cast<double>(sample_count - 1) : 0.0;
        const double s = s_start + t * (s_end - s_start);
        write_horizon_debug_point(
          "spline",
          i,
          s,
          horizon_x_spline_(s),
          horizon_y_spline_(s),
          horizon_z_spline_(s));
      }

      horizon_debug_file_.flush();
    }

    return true;
  }

  bool getHorizonTarget(
    const tf2::Vector3 & p_cur,
    double & target_x,
    double & target_y,
    double & target_z,
    tf2::Vector3 * path_tangent_base = nullptr)
  {
    if (!path_tracking_enabled_) {
      return false;
    }

    if (shouldRebuildHorizonPath() && !rebuildHorizonPath()) {
      return false;
    }
    if (!have_horizon_path_ || horizon_s_.empty()) {
      return false;
    }

    HorizonPoint current_in_planning;
    if (!transformHorizonPoint(
          HorizonPoint{p_cur.x(), p_cur.y(), p_cur.z()},
          base_frame_,
          planning_frame_,
          current_in_planning))
    {
      return false;
    }

    const double s_current = estimateCurrentSOnHorizon(current_in_planning.x);
    const double s_query =
      clamp(s_current + path_follow_lookahead_m_, horizon_s_.front(), horizon_s_.back());

    if (path_tangent_base) {
      const double ds =
        std::max(0.01, std::min(0.10, path_waypoint_spacing_m_ * 0.5));
      const double s0 = clamp(s_query - ds, horizon_s_.front(), horizon_s_.back());
      const double s1 = clamp(s_query + ds, horizon_s_.front(), horizon_s_.back());
      HorizonPoint p0_base;
      HorizonPoint p1_base;
      if (s1 > s0 &&
          transformHorizonPoint(
            HorizonPoint{
              horizon_x_spline_(s0),
              horizon_y_spline_(s0),
              horizon_z_spline_(s0)},
            planning_frame_,
            base_frame_,
            p0_base) &&
          transformHorizonPoint(
            HorizonPoint{
              horizon_x_spline_(s1),
              horizon_y_spline_(s1),
              horizon_z_spline_(s1)},
            planning_frame_,
            base_frame_,
            p1_base))
      {
        tf2::Vector3 tangent(
          p1_base.x - p0_base.x,
          p1_base.y - p0_base.y,
          p1_base.z - p0_base.z);
        if (tangent.length2() > 1e-12) {
          tangent.normalize();
          *path_tangent_base = tangent;
        }
      }
    }

    HorizonPoint target_in_base;
    if (!transformHorizonPoint(
          HorizonPoint{
            horizon_x_spline_(s_query),
            horizon_y_spline_(s_query),
            horizon_z_spline_(s_query)},
          planning_frame_,
          base_frame_,
          target_in_base))
    {
      return false;
    }

    target_x = target_in_base.x;
    target_y = target_in_base.y;
    target_z = target_in_base.z;
    return true;
  }

  double estimateCurrentSOnHorizon(double current_x_planning) const
  {
    if (horizon_s_.empty()) {
      return 0.0;
    }
    if (horizon_s_.size() == 1) {
      return horizon_s_.front();
    }

    const double s_min = horizon_s_.front();
    const double s_max = horizon_s_.back();
    const double x_min = horizon_x_spline_(s_min);
    const double x_max = horizon_x_spline_(s_max);
    const bool increasing = x_max >= x_min;

    if (increasing) {
      if (current_x_planning <= x_min) {
        return s_min;
      }
      if (current_x_planning >= x_max) {
        return s_max;
      }
    } else {
      if (current_x_planning >= x_min) {
        return s_min;
      }
      if (current_x_planning <= x_max) {
        return s_max;
      }
    }

    double lo = s_min;
    double hi = s_max;
    for (int i = 0; i < 40; ++i) {
      const double mid = 0.5 * (lo + hi);
      const double x_mid = horizon_x_spline_(mid);
      if ((increasing && x_mid < current_x_planning) ||
          (!increasing && x_mid > current_x_planning))
      {
        lo = mid;
      } else {
        hi = mid;
      }
    }

    return 0.5 * (lo + hi);
  }

  void publish_horizon_target_pose(
    const tf2::Vector3 & p_tgt,
    const tf2::Quaternion & q_des)
  {
    geometry_msgs::msg::PoseStamped target_ps;
    target_ps.header.stamp = now();
    target_ps.header.frame_id = base_frame_;
    target_ps.pose.position.x = p_tgt.x();
    target_ps.pose.position.y = p_tgt.y();
    target_ps.pose.position.z = p_tgt.z();
    target_ps.pose.orientation = tf2::toMsg(q_des);
    horizon_target_pose_pub_->publish(target_ps);
  }

  void publish_logging_msgs(
    const tf2::Vector3 & p_cur,
    const tf2::Quaternion & q_cur,
    const tf2::Vector3 & p_tgt,
    const tf2::Quaternion & q_des,
    const tf2::Vector3 & e_pos)
  {
    geometry_msgs::msg::PoseStamped actual_ps;
    actual_ps.header.stamp = now();
    actual_ps.header.frame_id = base_frame_;
    actual_ps.pose.position.x = p_cur.x();
    actual_ps.pose.position.y = p_cur.y();
    actual_ps.pose.position.z = p_cur.z();
    actual_ps.pose.orientation = tf2::toMsg(q_cur);

    geometry_msgs::msg::PoseStamped desired_ps;
    desired_ps.header.stamp = now();
    desired_ps.header.frame_id = base_frame_;
    desired_ps.pose.position.x = p_tgt.x();
    desired_ps.pose.position.y = p_tgt.y();
    desired_ps.pose.position.z = p_tgt.z();
    desired_ps.pose.orientation = tf2::toMsg(q_des);

    geometry_msgs::msg::Vector3Stamped err_msg;
    err_msg.header.stamp = now();
    err_msg.header.frame_id = base_frame_;
    err_msg.vector.x = e_pos.x();
    err_msg.vector.y = e_pos.y();
    err_msg.vector.z = e_pos.z();

    actual_pose_pub_->publish(actual_ps);
    desired_pose_pub_->publish(desired_ps);
    pos_error_pub_->publish(err_msg);

    actual_path_msg_.header.stamp = now();
    desired_path_msg_.header.stamp = now();
    actual_path_msg_.poses.push_back(actual_ps);
    desired_path_msg_.poses.push_back(desired_ps);

    while (static_cast<int>(actual_path_msg_.poses.size()) > max_path_history_) {
      actual_path_msg_.poses.erase(actual_path_msg_.poses.begin());
    }
    while (static_cast<int>(desired_path_msg_.poses.size()) > max_path_history_) {
      desired_path_msg_.poses.erase(desired_path_msg_.poses.begin());
    }

    actual_path_pub_->publish(actual_path_msg_);
    desired_path_pub_->publish(desired_path_msg_);

    write_pose_debug_sample(p_cur, q_cur, p_tgt, q_des, e_pos);
  }

  void control_update()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = base_frame_;

    if (!servo_started_) {
      publish_spray_enabled(false);
      twist_pub_->publish(cmd);
      return;
    }

    try_start_platform_pid();

    if (!have_measurement_) {
      publish_spray_enabled(false);
      twist_pub_->publish(cmd);
      return;
    }

    geometry_msgs::msg::TransformStamped T;
    try {
      T = tf_buffer_->lookupTransform(base_frame_, ee_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
      publish_spray_enabled(false);
      twist_pub_->publish(cmd);
      return;
    }

    tf2::Vector3 p_cur(
      T.transform.translation.x,
      T.transform.translation.y,
      T.transform.translation.z);

    tf2::Quaternion q_cur;
    tf2::fromMsg(T.transform.rotation, q_cur);
    q_cur.normalize();

    const double y_cp = p_cp_filt_.y();
    const double sign_y = (y_cp >= 0.0) ? 1.0 : -1.0;
    const double y_cp_lookahead =
      y_cp + sign_y * platform_speed_mps_ * lookahead_time_s_;
    const double requested_target_y = y_cp_lookahead - sign_y * standoff_m_;
    double target_x = p_cur.x();
    double target_y = applyPlatformYKeepout(requested_target_y, sign_y, true);
    double target_z = z_target_;
    bool using_horizon_target = false;
    bool have_path_tangent = false;
    tf2::Vector3 path_tangent_base(0, 0, 1);

    if (path_tracking_enabled_) {
      double path_target_x = target_x;
      double path_target_y = target_y;
      double path_target_z = target_z;
      tf2::Vector3 horizon_tangent_base(0, 0, 1);
      tf2::Vector3 * horizon_tangent_ptr =
        usePathTangentOrientation() ? &horizon_tangent_base : nullptr;
      if (getHorizonTarget(
            p_cur,
            path_target_x,
            path_target_y,
            path_target_z,
            horizon_tangent_ptr))
      {
        target_x = path_target_x;
        target_y = applyPlatformYKeepout(path_target_y, sign_y, true);
        target_z = path_target_z;
        if (horizon_tangent_ptr != nullptr) {
          path_tangent_base = horizon_tangent_base;
          have_path_tangent = horizon_tangent_base.length2() > 1e-12;
        }
        using_horizon_target = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Path tracking enabled but horizon path is unavailable; using point target");
      }
    }

    tf2::Vector3 p_tgt;
    p_tgt.setX(p_cur.x()); // Let platform move in x, only servo in y (and maybe z)
    target_y = applyCanopyGapGuard(target_y, sign_y);
    p_tgt.setY(target_y);
    p_tgt.setZ(target_z);

    tf2::Vector3 e_pos = p_tgt - p_cur;

    tf2::Vector3 forward = n_filt_;
    forward.normalize();
    if (point_tool_into_surface_) {
      forward = -forward;
    }

    tf2::Vector3 to_surface = (p_cp_filt_ - p_cur);
    if (to_surface.length2() > 1e-10) {
      if (forward.dot(to_surface) < 0.0) {
        forward = -forward;
      }
    }

    if (orientation_clamp_enabled_) {
      const tf2::Vector3 nominal_forward(0.0, sign_y, 0.0);
      double unclamped_angle = 0.0;
      const tf2::Vector3 clamped_forward = clampDirectionToCone(
        forward,
        nominal_forward,
        deg_to_rad(orientation_clamp_max_deviation_deg_),
        &unclamped_angle);

      if (unclamped_angle > deg_to_rad(orientation_clamp_max_deviation_deg_)) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Orientation clamp active: forward angle %.1f deg limited to %.1f deg",
          unclamped_angle * 180.0 / 3.14159265358979323846,
          orientation_clamp_max_deviation_deg_);
      }

      forward = clamped_forward;
    }

    if (useStaticYawOrientation()) {
      forward = keepOnlyVerticalTilt(forward, sign_y);
    } else if (usePathTangentOrientation() && have_path_tangent) {
      forward = removeForwardComponentAlongTangent(forward, path_tangent_base);
    }

    tf2::Quaternion q_des = quatAlignToolAxisToForward(
      forward, tf2::Vector3(0, 0, 1), tool_axis_forward_);

    tf2::Vector3 axis_tool(1, 0, 0);
    if (tool_axis_forward_ == 1) axis_tool = tf2::Vector3(0, 1, 0);
    if (tool_axis_forward_ == 2) axis_tool = tf2::Vector3(0, 0, 1);

    tf2::Quaternion q_roll;
    q_roll.setRotation(axis_tool, tool_roll_orientation_);
    q_roll.normalize();

    q_des = q_des * q_roll;
    q_des.normalize();

    if (using_horizon_target) {
      publish_horizon_target_pose(p_tgt, q_des);
    }

    tf2::Quaternion q_err = q_des * q_cur.inverse();
    q_err.normalize();
    if (q_err.w() < 0.0) {
      q_err = tf2::Quaternion(-q_err.x(), -q_err.y(), -q_err.z(), -q_err.w());
    }

    double angle = 2.0 * std::acos(clamp((double)q_err.w(), -1.0, 1.0));
    tf2::Vector3 axis(q_err.x(), q_err.y(), q_err.z());
    if (axis.length2() < 1e-12 || !std::isfinite(angle)) {
      axis = tf2::Vector3(0,0,0);
      angle = 0.0;
    } else {
      axis.normalize();
    }

    tf2::Vector3 v_cmd = kp_pos_ * e_pos;
    tf2::Vector3 w_cmd = kp_rot_ * angle * axis;

    const double v_norm = v_cmd.length();
    if (std::isfinite(v_norm) && v_norm > v_max_ && v_norm > 1e-12) {
      v_cmd *= (v_max_ / v_norm);
    }
    const double w_norm = w_cmd.length();
    if (std::isfinite(w_norm) && w_norm > w_max_ && w_norm > 1e-12) {
      w_cmd *= (w_max_ / w_norm);
    }

    cmd.twist.linear.x  = v_cmd.x();
    cmd.twist.linear.y  = v_cmd.y();
    cmd.twist.linear.z  = v_cmd.z();
    cmd.twist.angular.x = w_cmd.x();
    cmd.twist.angular.y = w_cmd.y();
    cmd.twist.angular.z = w_cmd.z();

    if (!finite_twist(cmd)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "NaN twist blocked");
      cmd.twist.linear.x = cmd.twist.linear.y = cmd.twist.linear.z = 0.0;
      cmd.twist.angular.x = cmd.twist.angular.y = cmd.twist.angular.z = 0.0;
      have_measurement_ = false;
    }

    publish_spray_enabled(have_measurement_);
    twist_pub_->publish(cmd);

    log_counter_++;
    if ((log_counter_ % static_cast<std::size_t>(log_every_n_)) == 0U) {
      publish_logging_msgs(p_cur, q_cur, p_tgt, q_des, e_pos);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "y_cp=%.3f y_pred=%.3f y_tgt=%.3f standoff=%.3f | p_cur=[%.2f %.2f %.2f] p_tgt=[%.2f %.2f %.2f] v=[%.3f %.3f %.3f]",
      y_cp, y_cp_lookahead, p_tgt.y(), standoff_m_,
      p_cur.x(), p_cur.y(), p_cur.z(),
      p_tgt.x(), p_tgt.y(), p_tgt.z(),
      cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.linear.z
    );
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VerticalSprayServo>());
  rclcpp::shutdown();
  return 0;
}
