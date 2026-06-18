#include "futuraps_task_planner/platform_moving.hpp"

#include <chrono>
#include <cmath>
#include <functional>

using namespace std::chrono_literals;

namespace futuraps_task_planner
{

PlatformMoving::PlatformMoving(rclcpp::Node * node)
: node_(node)
{
  enable_pid_client_ =
    node_->create_client<std_srvs::srv::SetBool>("/pid_controller/enable");
  cmd_vel_pub_ =
    node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/diff_drive_controller/cmd_vel", 10);
  cmd_vel_unstamped_pub_ =
    node_->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  motion_timer_ = node_->create_wall_timer(
    100ms,
    std::bind(&PlatformMoving::motionTimerCallback, this)
  );
}

void PlatformMoving::configure(const PlatformMovingConfig& config)
{
  config_ = config;
}

bool PlatformMoving::startMovePlatformDistance(double dist_in_m)
{
  if (is_robot_moving_ || move_start_pending_) {
    RCLCPP_WARN(node_->get_logger(), "Robot is already moving or waiting for PID enable");
    return false;
  }

  if (dist_in_m <= 0.0) {
    RCLCPP_WARN(node_->get_logger(), "Distance must be > 0");
    return false;
  }

  if (!setPlatformStartPosition()) {
    RCLCPP_WARN(node_->get_logger(), "Could not set platform start position");
    return false;
  }

  target_distance_ = dist_in_m;
  motion_finished_ = false;
  move_start_pending_ = config_.use_pid;

  if (config_.use_pid) {
    if (!startMovingForward()) {
      move_start_pending_ = false;
      target_distance_ = 0.0;
      RCLCPP_WARN(node_->get_logger(), "Could not request PID enable for platform move");
      return false;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Requested platform move of %.3f m; waiting for PID enable confirmation",
      target_distance_);
  } else {
    is_robot_moving_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "Tracking manual platform move of %.3f m with PID disabled",
      target_distance_);
  }
  return true;
}

bool PlatformMoving::startMovingForward()
{
  brake_hold_active_ = false;
  return requestPidEnabled(true, PidRequestPurpose::START_MOVE);
}

bool PlatformMoving::stopMovingForward()
{
  if (config_.use_pid) {
    if (!requestPidEnabled(false, PidRequestPurpose::STOP_MOVE)) {
      return false;
    }
  }

  is_robot_moving_ = false;
  move_start_pending_ = false;
  target_distance_ = 0.0;
  if (config_.use_pid) {
    startBrakeHold();
  }
  return true;
}

void PlatformMoving::startBrakeHold()
{
  brake_hold_active_ = true;
  brake_hold_until_ =
    node_->now() + rclcpp::Duration::from_seconds(brake_hold_duration_s_);
  publishZeroCommand();
}

void PlatformMoving::publishZeroCommand()
{
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = node_->now();
  stamped.header.frame_id = config_.base_frame;
  cmd_vel_pub_->publish(stamped);

  geometry_msgs::msg::Twist unstamped;
  cmd_vel_unstamped_pub_->publish(unstamped);
}

void PlatformMoving::publishBrakeHoldIfActive()
{
  if (!brake_hold_active_) {
    return;
  }

  if (node_->now() <= brake_hold_until_) {
    publishZeroCommand();
    return;
  }

  brake_hold_active_ = false;
}

bool PlatformMoving::requestPidEnabled(bool enabled, PidRequestPurpose purpose)
{
  if (pid_request_pending_) {
    if (pid_request_value_ == enabled && pid_request_purpose_ == purpose) {
      return true;
    }

    RCLCPP_WARN(
      node_->get_logger(),
      "PID enable request already pending; cannot send a conflicting request");
    return false;
  }

  if (!enable_pid_client_->service_is_ready()) {
    RCLCPP_WARN(node_->get_logger(), "PID enable service not ready");
    return false;
  }

  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = enabled;
  pid_enable_future_ = enable_pid_client_->async_send_request(req).future.share();
  pid_request_pending_ = true;
  pid_request_value_ = enabled;
  pid_request_purpose_ = purpose;
  pid_request_sent_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    enabled ? "Sent request to enable PID controller"
            : "Sent request to disable PID controller");

  return true;
}

void PlatformMoving::pollPidEnableRequest()
{
  if (!pid_request_pending_) {
    return;
  }

  if (pid_enable_future_.valid() &&
    pid_enable_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    const auto response = pid_enable_future_.get();
    if (response->success) {
      RCLCPP_INFO(
        node_->get_logger(),
        "PID controller %s confirmed: %s",
        pid_request_value_ ? "enable" : "disable",
        response->message.c_str());
      handlePidEnableSuccess();
    } else {
      handlePidEnableFailure(response->message);
    }
    return;
  }

  const double age_s = (node_->now() - pid_request_sent_time_).seconds();
  if (age_s < pid_request_timeout_s_) {
    return;
  }

  const bool enabled = pid_request_value_;
  const auto purpose = pid_request_purpose_;
  pid_request_pending_ = false;
  pid_request_purpose_ = PidRequestPurpose::NONE;

  RCLCPP_WARN(
    node_->get_logger(),
    "Timed out waiting for PID controller %s response; retrying",
    enabled ? "enable" : "disable");
  (void)requestPidEnabled(enabled, purpose);
}

void PlatformMoving::handlePidEnableSuccess()
{
  const auto purpose = pid_request_purpose_;
  pid_request_pending_ = false;
  pid_request_purpose_ = PidRequestPurpose::NONE;

  if (purpose == PidRequestPurpose::START_MOVE) {
    is_robot_moving_ = true;
    move_start_pending_ = false;
    RCLCPP_INFO(node_->get_logger(), "Started platform move of %.3f m", target_distance_);
  } else if (purpose == PidRequestPurpose::STOP_MOVE) {
    RCLCPP_INFO(node_->get_logger(), "PID controller disabled");
  }
}

void PlatformMoving::handlePidEnableFailure(const std::string& reason)
{
  const auto purpose = pid_request_purpose_;
  pid_request_pending_ = false;
  pid_request_purpose_ = PidRequestPurpose::NONE;

  RCLCPP_WARN(
    node_->get_logger(),
    "PID controller request failed: %s",
    reason.c_str());

  if (purpose == PidRequestPurpose::START_MOVE) {
    is_robot_moving_ = false;
    move_start_pending_ = false;
    target_distance_ = 0.0;
    motion_finished_ = false;
    publishZeroCommand();
  }
}

bool PlatformMoving::setPlatformStartPosition()
{
  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = tf_buffer_->lookupTransform(config_.map_frame, config_.base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TF lookup failed for platform motion frame '%s' -> '%s': %s",
      config_.map_frame.c_str(), config_.base_frame.c_str(), ex.what());
    return false;
  }

  start_position_ = tf.transform.translation;
  return true;
}

bool PlatformMoving::robotHasMovedDist(double target_dist)
{
  geometry_msgs::msg::TransformStamped tf;

  try {
    tf = tf_buffer_->lookupTransform(config_.map_frame, config_.base_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "TF lookup failed for platform motion frame '%s' -> '%s': %s",
      config_.map_frame.c_str(), config_.base_frame.c_str(), ex.what());
    return false;
  }

  const auto current = tf.transform.translation;

  const double dx = current.x - start_position_.x;
  const double dy = current.y - start_position_.y;
  const double dz = current.z - start_position_.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "Moved distance: %.3f / %.3f", dist, target_dist);

  return dist >= target_dist;
}

void PlatformMoving::motionTimerCallback()
{
  if (config_.use_pid) {
    pollPidEnableRequest();
    publishBrakeHoldIfActive();
  }

  if (!is_robot_moving_) {
    return;
  }

  if (robotHasMovedDist(target_distance_)) {
     if (!stopMovingForward()) {
      RCLCPP_WARN(node_->get_logger(), "Could not disable robot from moving forward");
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        config_.use_pid ? "Target distance reached, robot stopped"
                        : "Target distance reached during manual platform move");
      motion_finished_ = true;
    }
  }
}

bool PlatformMoving::motionFinished() const {
  return motion_finished_;
}

}  // namespace futuraps_task_planner
