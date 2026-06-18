#include "futuraps_task_planner/horizontal_path_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace futuraps_task_planner
{

namespace
{

constexpr double kEps = 1e-12;
constexpr double kMaxInwardAngleDeg = 25.0;

double clamp_value(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

bool finite3_local(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

tf2::Vector3 safe_normalized(const tf2::Vector3 & v, const tf2::Vector3 & fallback)
{
  if (v.length2() < kEps) {
    tf2::Vector3 out = fallback;
    out.normalize();
    return out;
  }

  tf2::Vector3 out = v;
  out.normalize();
  return out;
}

double angle_between(const tf2::Vector3 & a_in, const tf2::Vector3 & b_in)
{
  tf2::Vector3 a = safe_normalized(a_in, tf2::Vector3(0, 1, 0));
  tf2::Vector3 b = safe_normalized(b_in, tf2::Vector3(0, 1, 0));

  const double d = clamp_value(a.dot(b), -1.0, 1.0);
  return std::acos(d);
}

tf2::Vector3 rotate_towards_limited(
  const tf2::Vector3 & reference_in,
  const tf2::Vector3 & desired_in,
  double max_angle_rad)
{
  tf2::Vector3 reference = safe_normalized(reference_in, tf2::Vector3(0, 1, 0));
  tf2::Vector3 desired = safe_normalized(desired_in, reference);

  const double ang = angle_between(reference, desired);
  if (ang <= max_angle_rad) {
    return desired;
  }

  tf2::Vector3 axis = reference.cross(desired);
  if (axis.length2() < kEps) {
    return reference;
  }
  axis.normalize();

  tf2::Quaternion q_limit(axis, max_angle_rad);
  tf2::Vector3 limited = tf2::quatRotate(q_limit, reference);
  return safe_normalized(limited, reference);
}

tf2::Vector3 compute_nominal_inward_direction(
  const geometry_msgs::msg::Pose & current_pose,
  const PerceptionResult & perception)
{
  double y_sum = 0.0;
  int y_count = 0;

  for (size_t i = 0; i < perception.found.size(); ++i) {
    if (!perception.found[i]) {
      continue;
    }
    if (i >= perception.y.size()) {
      continue;
    }

    const double y = perception.y[i];
    if (!std::isfinite(y)) {
      continue;
    }

    y_sum += y;
    ++y_count;
  }

  const double mean_y = (y_count > 0) ?
    (y_sum / static_cast<double>(y_count)) :
    (current_pose.position.y + 1.0);

  const double sign = (mean_y >= current_pose.position.y) ? 1.0 : -1.0;
  return tf2::Vector3(0.0, sign, 0.0);
}

tf2::Vector3 clamp_forward_to_robot_side(
  const tf2::Vector3 & desired_forward_in,
  const tf2::Vector3 & nominal_inward_in,
  double max_angle_rad)
{
  tf2::Vector3 nominal_inward = safe_normalized(nominal_inward_in, tf2::Vector3(0, 1, 0));
  tf2::Vector3 desired_forward = safe_normalized(desired_forward_in, nominal_inward);

  if (desired_forward.dot(nominal_inward) < 0.0) {
    desired_forward = nominal_inward;
  }

  return rotate_towards_limited(nominal_inward, desired_forward, max_angle_rad);
}

}  // namespace

bool HorizontalPathBuilder::finite3(double a, double b, double c)
{
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

tf2::Quaternion HorizontalPathBuilder::quatAlignToolAxisToForward(
  const tf2::Vector3 & forward_in,
  const tf2::Vector3 & up_hint_in,
  int tool_axis)
{
  tf2::Vector3 forward = forward_in;
  if (forward.length2() < kEps) {
    forward = tf2::Vector3(1, 0, 0);
  }
  forward.normalize();

  tf2::Vector3 up_hint = up_hint_in;
  if (up_hint.length2() < kEps) {
    up_hint = tf2::Vector3(0, 0, 1);
  }
  up_hint.normalize();

  if (std::fabs(forward.dot(up_hint)) > 0.95) {
    up_hint = tf2::Vector3(1, 0, 0);
  }

  tf2::Vector3 b = up_hint.cross(forward);
  if (b.length2() < kEps) {
    b = tf2::Vector3(0, 1, 0);
  }
  b.normalize();

  tf2::Vector3 a = forward.cross(b);
  a.normalize();

  tf2::Vector3 X, Y, Z;
  if (tool_axis == 0) {
    X = forward;
    Y = b;
    Z = a;
  } else if (tool_axis == 1) {
    Y = forward;
    Z = b;
    X = a;
  } else {
    Z = forward;
    X = b;
    Y = a;
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

std::vector<geometry_msgs::msg::Pose> HorizontalPathBuilder::buildWaypoints(
  const geometry_msgs::msg::Pose & current_pose,
  const PerceptionResult & perception,
  const HorizontalPathConfig & config) const
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (!perception.valid) {
    return waypoints;
  }

  if (perception.x.empty() || perception.y.empty() ||
      perception.z.empty() || perception.found.empty()) {
    return waypoints;
  }

  if (perception.normals.empty() || perception.normal_valid.empty()) {
    return waypoints;
  }

  if (perception.x.size() != perception.y.size() ||
      perception.x.size() != perception.z.size() ||
      perception.x.size() != perception.found.size()) {
    return waypoints;
  }

  if (perception.normals.size() != perception.normal_valid.size()) {
    return waypoints;
  }

  const double x_current = current_pose.position.x;
  const tf2::Vector3 nominal_inward =
    compute_nominal_inward_direction(current_pose, perception);
  const double max_angle_rad = kMaxInwardAngleDeg * M_PI / 180.0;

  double previous_y = std::numeric_limits<double>::quiet_NaN();
  double previous_z = std::numeric_limits<double>::quiet_NaN();
  bool have_previous = false;

  for (int i = 0; i < config.rows; ++i) {
    bool found_in_row = false;
    double closest_y = 0.0;
    double closest_z = 0.0;
    double best_metric = std::numeric_limits<double>::infinity();

    if (i >= static_cast<int>(perception.normals.size()) ||
        i >= static_cast<int>(perception.normal_valid.size())) {
      continue;
    }

    if (!perception.normal_valid[i]) {
      continue;
    }

    tf2::Vector3 forward = perception.normals[static_cast<size_t>(i)];
    if (forward.length2() < kEps) {
      continue;
    }
    forward.normalize();

    if (config.point_tool_into_surface) {
      forward = -forward;
    }

    forward = clamp_forward_to_robot_side(forward, nominal_inward, max_angle_rad);

    for (int j = 0; j < config.cols; ++j) {
      const int idx = i * config.cols + j;
      if (idx >= static_cast<int>(perception.found.size())) {
        continue;
      }

      if (!perception.found[static_cast<size_t>(idx)]) {
        continue;
      }

      const double x = perception.x[static_cast<size_t>(idx)];
      const double y = perception.y[static_cast<size_t>(idx)];
      const double z = perception.z[static_cast<size_t>(idx)];

      if (!finite3_local(x, y, z)) {
        continue;
      }

      const double reference_y =
        have_previous ? previous_y : current_pose.position.y;
      const double reference_z =
        have_previous ? previous_z : current_pose.position.z;

      const double metric =
        std::abs(y - reference_y) +
        0.35 * std::abs(z - reference_z) +
        0.05 * std::abs(y);

      if (metric < best_metric) {
        best_metric = metric;
        closest_y = y;
        closest_z = z;
        found_in_row = true;
      }
    }

    if (!found_in_row) {
      continue;
    }

    const double row_center_z =
      config.z0 + (static_cast<double>(i) + 0.5) * config.cell_size;

    const double blended_z =
      (1.0 - config.row_z_blend) * row_center_z +
      config.row_z_blend * closest_z;

    geometry_msgs::msg::Pose pose;
    pose.position.x = x_current;
    pose.position.y = closest_y - config.standoff_m;
    pose.position.z = std::min(blended_z, config.max_z);

    if (std::abs(pose.position.y) > config.max_abs_y) {
      continue;
    }

    if (have_previous) {
      if (std::abs(pose.position.y - previous_y) > config.max_row_delta_y) {
        continue;
      }
      if (std::abs(pose.position.z - previous_z) > config.max_row_delta_z) {
        continue;
      }
    }

    tf2::Quaternion q_des =
      quatAlignToolAxisToForward(forward, tf2::Vector3(0, 0, 1), config.tool_axis_forward);
    q_des.normalize();

    tf2::Vector3 axis_tool(1, 0, 0);
    if (config.tool_axis_forward == 1) {
      axis_tool = tf2::Vector3(0, 1, 0);
    } else if (config.tool_axis_forward == 2) {
      axis_tool = tf2::Vector3(0, 0, 1);
    }

    tf2::Quaternion q_roll;
    q_roll.setRotation(axis_tool, config.tool_roll_orientation);
    q_roll.normalize();

    q_des = q_des * q_roll;
    q_des.normalize();

    pose.orientation.x = q_des.x();
    pose.orientation.y = q_des.y();
    pose.orientation.z = q_des.z();
    pose.orientation.w = q_des.w();

    waypoints.push_back(pose);
    previous_y = pose.position.y;
    previous_z = pose.position.z;
    have_previous = true;
  }

  return waypoints;
}

}  // namespace futuraps_task_planner