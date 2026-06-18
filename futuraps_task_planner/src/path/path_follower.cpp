#include "futuraps_task_planner/path/path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <rmw/qos_profiles.h>

namespace
{

tf2::Vector3 computeOrientationError(
  const geometry_msgs::msg::Quaternion & current,
  const geometry_msgs::msg::Quaternion & target)
{
  tf2::Quaternion q_current, q_target;
  tf2::fromMsg(current, q_current);
  tf2::fromMsg(target, q_target);

  q_current.normalize();
  q_target.normalize();

  tf2::Quaternion q_err = q_target * q_current.inverse();
  q_err.normalize();

  if (q_err.w() < 0.0) {
    q_err = tf2::Quaternion(-q_err.x(), -q_err.y(), -q_err.z(), -q_err.w());
  }

  const double angle = q_err.getAngle();
  tf2::Vector3 axis(q_err.x(), q_err.y(), q_err.z());

  if (axis.length2() < 1e-12 || std::abs(angle) < 1e-12) {
    return tf2::Vector3(0.0, 0.0, 0.0);
  }

  axis.normalize();
  return axis * angle;
}

double speedProfile(double s, double stroke_length, double v_min, double v_max)
{
  if (stroke_length < 1e-9) {
    return v_min;
  }

  const double r = std::clamp(s / stroke_length, 0.0, 1.0);

  return v_min + (v_max - v_min) * std::sin(M_PI * r);
}

} // namespace

namespace futuraps_task_planner
{

PathFollower::PathFollower(rclcpp::Node * node)
: node_(node)
{
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));

  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/servo_node/delta_twist_cmds", qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PathFollower::configure(const PathFollowerConfig & config)
{
  pos_gain_ = config.pos_gain;
  ori_gain_ = config.ori_gain;
  max_linear_speed_ = config.max_linear_speed;
  max_angular_speed_ = config.max_angular_speed;
  pos_tolerance_ = config.pos_tolerance;
  ori_tolerance_ = config.ori_tolerance;
  lookahead_points_ = std::max(0, config.lookahead_points);
  v_min_ = config.v_min;
  v_max_ = config.v_max;
  path_frame_ = config.path_frame;
  command_frame_ = config.command_frame;
  ee_frame_ = config.ee_frame;
}

bool PathFollower::startPathFollowing(const SprayPath& spray_path)
{
  if (spray_path.waypoints.empty()) {
    return false;
  }

  active_path_ = spray_path;
  current_index_ = 0;
  active_ = true;
  finished_ = false;

  if (active_path_.waypoints.size() >= 2) {
    resetSpeedDebugLog();
  }

  return true;
}

void PathFollower::update()
{
  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "PathFollower update: active=%s finished=%s index=%zu",
    active_ ? "true" : "false",
    finished_ ? "true" : "false",
    current_index_);

  if (!active_ || finished_) {
    return;
  }

  geometry_msgs::msg::Pose current_pose = getCurrentPose();

  if (current_index_ >= active_path_.waypoints.size()) {
    stop();
    finished_ = true;
    return;
  }

  // ------------------------------------------------------------
  // 1. Advance progress using segment projection
  // ------------------------------------------------------------
  Eigen::Vector3d p_current(
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);

  const size_t last_index = active_path_.waypoints.size() - 1;

  while (current_index_ + 1 < active_path_.waypoints.size()) {
    const auto & wp0 = active_path_.waypoints[current_index_];
    const auto & wp1 = active_path_.waypoints[current_index_ + 1];
    const auto wp0_cmd = transformPoseToCommandFrame(wp0.pose);
    const auto wp1_cmd = transformPoseToCommandFrame(wp1.pose);

    Eigen::Vector3d p0(
      wp0_cmd.position.x,
      wp0_cmd.position.y,
      wp0_cmd.position.z);

    Eigen::Vector3d p1(
      wp1_cmd.position.x,
      wp1_cmd.position.y,
      wp1_cmd.position.z);

    Eigen::Vector3d segment = p1 - p0;
    const double segment_length = segment.norm();

    if (segment_length < 1e-6) {
      current_index_++;
      continue;
    }

    Eigen::Vector3d t_seg = segment / segment_length;

    const double s_local = (p_current - p0).dot(t_seg);
    const double dist_to_next = (p1 - p_current).norm();

    const bool passed_next_wp = s_local > segment_length;
    const bool close_to_next_wp = dist_to_next < pos_tolerance_;

    if (passed_next_wp || close_to_next_wp) {
      current_index_++;
    } else {
      break;
    }
  }

  if (current_index_ + 1 >= active_path_.waypoints.size()) {
    stop();
    finished_ = true;
    return;
  }

  // ------------------------------------------------------------
  // 2. Use lookahead target for velocity command
  // ------------------------------------------------------------
  const size_t turnaround_index = last_index / 2;

  size_t target_index = std::min(
    current_index_ + lookahead_points_,
    last_index);

  const auto & target_wp = active_path_.waypoints[target_index];
  const auto target = transformPoseToCommandFrame(target_wp.pose);

  const double ex = target.position.x - current_pose.position.x;
  const double ey = target.position.y - current_pose.position.y;
  const double ez = target.position.z - current_pose.position.z;

  const double err_norm = std::sqrt(ex * ex + ey * ey + ez * ez);

  tf2::Vector3 e_ori = computeOrientationError(
    current_pose.orientation,
    target.orientation);

  const double ori_err_norm = e_ori.length();

  // ------------------------------------------------------------
  // 3. Speed profile: slow at stroke ends, fast in middle
  // ------------------------------------------------------------
  const auto & wp0 = active_path_.waypoints[current_index_];
  const auto & wp1 = active_path_.waypoints[current_index_ + 1];
  const auto wp0_cmd = transformPoseToCommandFrame(wp0.pose);
  const auto wp1_cmd = transformPoseToCommandFrame(wp1.pose);

  Eigen::Vector3d p0(
    wp0_cmd.position.x,
    wp0_cmd.position.y,
    wp0_cmd.position.z);

  Eigen::Vector3d p1(
    wp1_cmd.position.x,
    wp1_cmd.position.y,
    wp1_cmd.position.z);

  Eigen::Vector3d segment = p1 - p0;
  const double segment_length = segment.norm();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  double s_local = 0.0;
  if (segment_length > 1e-6) {
    t = segment / segment_length;
    s_local = std::clamp((p_current - p0).dot(t), 0.0, segment_length);
  }

  const double s_current = wp0.s + s_local;
  const double s_target = active_path_.waypoints[target_index].s;
  const double stroke_length = active_path_.waypoints[turnaround_index].s;

  double s_stroke = s_current;
  if (s_current > stroke_length) {
    s_stroke = s_current - stroke_length;
  }

  const double v_path = speedProfile(s_stroke, stroke_length, v_min_, v_max_);

  // ------------------------------------------------------------
  // 4. Feedforward + feedback linear velocity
  // ------------------------------------------------------------
  const double vx_ff = v_path * t.x();
  const double vy_ff = v_path * t.y();
  const double vz_ff = v_path * t.z();

  const double vx_fb = pos_gain_ * ex;
  const double vy_fb = pos_gain_ * ey;
  const double vz_fb = pos_gain_ * ez;

  double vx = vx_ff + vx_fb;
  double vy = vy_ff + vy_fb;
  double vz = vz_ff + vz_fb;

  // ------------------------------------------------------------
  // 5. Orientation feedback
  // ------------------------------------------------------------
  double wx = ori_gain_ * e_ori.x();
  double wy = ori_gain_ * e_ori.y();
  double wz = ori_gain_ * e_ori.z();

  // ------------------------------------------------------------
  // 6. Saturate linear velocity
  // ------------------------------------------------------------
  const double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

  if (v_norm > max_linear_speed_) {
    const double scale = max_linear_speed_ / v_norm;
    vx *= scale;
    vy *= scale;
    vz *= scale;
  }

  // ------------------------------------------------------------
  // 7. Saturate angular velocity
  // ------------------------------------------------------------
  const double w_norm = std::sqrt(wx * wx + wy * wy + wz * wz);

  if (w_norm > max_angular_speed_) {
    const double scale = max_angular_speed_ / w_norm;
    wx *= scale;
    wy *= scale;
    wz *= scale;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "idx=%zu target=%zu pos_err=%.3f ori_err=%.3f v_path=%.3f "
    "ff=(%.3f, %.3f, %.3f) fb=(%.3f, %.3f, %.3f) "
    "cmd=(%.3f, %.3f, %.3f) ang=(%.3f, %.3f, %.3f)",
    current_index_, target_index, err_norm, ori_err_norm, v_path,
    vx_ff, vy_ff, vz_ff,
    vx_fb, vy_fb, vz_fb,
    vx, vy, vz,
    wx, wy, wz);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = command_frame_;

  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.linear.z = vz;

  cmd.twist.angular.x = wx;
  cmd.twist.angular.y = wy;
  cmd.twist.angular.z = wz;

  logSpeedDebug(
    current_pose,
    current_index_,
    target_index,
    s_target,
    s_stroke,
    v_path,
    err_norm,
    vx,
    vy,
    vz);

  twist_pub_->publish(cmd);
}

void PathFollower::updateMoveToPose()
{
  if (!active_ || finished_) {
    return;
  }

  geometry_msgs::msg::Pose current_pose = getCurrentPose();

  if (active_path_.waypoints.empty()) {
    stop();
    finished_ = true;
    return;
  }

  const auto target = transformPoseToCommandFrame(active_path_.waypoints.front().pose);

  const double ex = target.position.x - current_pose.position.x;
  const double ey = target.position.y - current_pose.position.y;
  const double ez = target.position.z - current_pose.position.z;

  const double err_norm = std::sqrt(ex * ex + ey * ey + ez * ez);

  tf2::Vector3 e_ori = computeOrientationError(
    current_pose.orientation,
    target.orientation);

  const double ori_err_norm = e_ori.length();

  if (err_norm < pos_tolerance_ && ori_err_norm < ori_tolerance_) {
    stop();
    finished_ = true;
    return;
  }

  double vx = pos_gain_ * ex;
  double vy = pos_gain_ * ey;
  double vz = pos_gain_ * ez;

  double wx = ori_gain_ * e_ori.x();
  double wy = ori_gain_ * e_ori.y();
  double wz = ori_gain_ * e_ori.z();

  const double v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
  if (v_norm > max_linear_speed_) {
    const double scale = max_linear_speed_ / v_norm;
    vx *= scale;
    vy *= scale;
    vz *= scale;
  }

  const double w_norm = std::sqrt(wx * wx + wy * wy + wz * wz);
  if (w_norm > max_angular_speed_) {
    const double scale = max_angular_speed_ / w_norm;
    wx *= scale;
    wy *= scale;
    wz *= scale;
  }

  RCLCPP_INFO_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 1000,
    "MoveToPose: pos_err=%.3f ori_err=%.3f lin_cmd=(%.3f, %.3f, %.3f) ang_cmd=(%.3f, %.3f, %.3f)",
    err_norm, ori_err_norm, vx, vy, vz, wx, wy, wz);

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = node_->now();
  cmd.header.frame_id = command_frame_;

  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.linear.z = vz;

  cmd.twist.angular.x = wx;
  cmd.twist.angular.y = wy;
  cmd.twist.angular.z = wz;

  twist_pub_->publish(cmd);
}

geometry_msgs::msg::Pose PathFollower::getCurrentPose() const
{
  geometry_msgs::msg::Pose pose;

  try {
    auto tf = tf_buffer_->lookupTransform(
      command_frame_,
      ee_frame_,
      tf2::TimePointZero);

    pose.position.x = tf.transform.translation.x;
    pose.position.y = tf.transform.translation.y;
    pose.position.z = tf.transform.translation.z;
    pose.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PathFollower TF lookup failed: %s",
      ex.what());
  }

  return pose;
}

geometry_msgs::msg::Pose PathFollower::transformPoseToCommandFrame(
  const geometry_msgs::msg::Pose & pose_in_path_frame) const
{
  const std::string source_frame =
    active_path_.frame_id.empty() ? path_frame_ : active_path_.frame_id;

  if (source_frame == command_frame_) {
    return pose_in_path_frame;
  }

  geometry_msgs::msg::PoseStamped in;
  geometry_msgs::msg::PoseStamped out;
  in.header.stamp = node_->now();
  in.header.frame_id = source_frame;
  in.pose = pose_in_path_frame;

  try {
    const auto tf =
      tf_buffer_->lookupTransform(command_frame_, source_frame, tf2::TimePointZero);
    tf2::doTransform(in, out, tf);
    return out.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "PathFollower pose transform %s <- %s failed: %s",
      command_frame_.c_str(), source_frame.c_str(), ex.what());
  }

  return pose_in_path_frame;
}

void PathFollower::stop()
{
  geometry_msgs::msg::TwistStamped stop_cmd;
  stop_cmd.header.stamp = node_->now();
  stop_cmd.header.frame_id = command_frame_;

  twist_pub_->publish(stop_cmd);

  active_ = false;
}

bool PathFollower::isFinished() const
{
  return finished_;
}

void PathFollower::resetSpeedDebugLog()
{
  speed_debug_file_.close();
  speed_debug_file_.open("/tmp/path_follower_speed_debug.csv", std::ios::out);

  if (!speed_debug_file_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Could not open /tmp/path_follower_speed_debug.csv for writing");
    return;
  }

  speed_debug_file_
    << "t,current_index,target_index,s_target,s_stroke,"
    << "v_path,cmd_speed,actual_speed,pos_err,"
    << "cmd_vx,cmd_vy,cmd_vz,x,y,z\n";
  speed_debug_file_.flush();

  speed_debug_have_last_pose_ = false;
}

void PathFollower::logSpeedDebug(
  const geometry_msgs::msg::Pose& current_pose,
  size_t current_index,
  size_t target_index,
  double s_target,
  double s_stroke,
  double v_path,
  double err_norm,
  double vx,
  double vy,
  double vz)
{
  if (!speed_debug_file_) {
    return;
  }

  const rclcpp::Time now = node_->now();
  double actual_speed = 0.0;

  if (speed_debug_have_last_pose_) {
    const double dt = (now - speed_debug_last_time_).seconds();

    if (dt > 1e-6) {
      const double dx =
        current_pose.position.x - speed_debug_last_pose_.position.x;
      const double dy =
        current_pose.position.y - speed_debug_last_pose_.position.y;
      const double dz =
        current_pose.position.z - speed_debug_last_pose_.position.z;

      actual_speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
    }
  }

  const double cmd_speed = std::sqrt(vx * vx + vy * vy + vz * vz);

  speed_debug_file_
    << std::setprecision(12)
    << now.seconds() << ","
    << current_index << ","
    << target_index << ","
    << s_target << ","
    << s_stroke << ","
    << v_path << ","
    << cmd_speed << ","
    << actual_speed << ","
    << err_norm << ","
    << vx << ","
    << vy << ","
    << vz << ","
    << current_pose.position.x << ","
    << current_pose.position.y << ","
    << current_pose.position.z << "\n";

  speed_debug_file_.flush();

  speed_debug_last_pose_ = current_pose;
  speed_debug_last_time_ = now;
  speed_debug_have_last_pose_ = true;
}

} // namespace futuraps_task_planner
