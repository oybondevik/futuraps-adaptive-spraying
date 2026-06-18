#include <cmath>
#include <limits>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <Eigen/Core>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "futuraps_task_planner/path/path_builder.hpp"

namespace
{

constexpr double kEps = 1e-12;

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

tf2::Quaternion quatAlignToolAxisToForward(
  const tf2::Vector3 & forward_in,
  const tf2::Vector3 & up_hint_in,
  int tool_axis)
{
  tf2::Vector3 forward = safe_normalized(forward_in, tf2::Vector3(1, 0, 0));
  tf2::Vector3 up_hint = safe_normalized(up_hint_in, tf2::Vector3(0, 0, 1));

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

double clamp_value(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
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

tf2::Vector3 clamp_forward_to_nominal(
  const tf2::Vector3 & desired_forward_in,
  const tf2::Vector3 & nominal_forward_in,
  double max_angle_rad)
{
  tf2::Vector3 nominal = safe_normalized(nominal_forward_in, tf2::Vector3(0, 1, 0));
  tf2::Vector3 desired = safe_normalized(desired_forward_in, nominal);

  // Prevent flipping to wrong side entirely
  if (desired.dot(nominal) < 0.0) {
    desired = nominal;
  }

  return rotate_towards_limited(nominal, desired, max_angle_rad);
}

static Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point & p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

double distanceBetween(
  const geometry_msgs::msg::Point& a,
  const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<double> computeArcLengths(
  const std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  std::vector<double> s;
  s.reserve(waypoints.size());

  s.push_back(0.0);

  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double ds = distanceBetween(
      waypoints[i - 1].position,
      waypoints[i].position);

    s.push_back(s.back() + ds);
  }

  return s;
}

geometry_msgs::msg::Quaternion interpolateOrientation(
  const geometry_msgs::msg::Quaternion& q0_msg,
  const geometry_msgs::msg::Quaternion& q1_msg,
  double u)
{
  tf2::Quaternion q0(
    q0_msg.x,
    q0_msg.y,
    q0_msg.z,
    q0_msg.w);

  tf2::Quaternion q1(
    q1_msg.x,
    q1_msg.y,
    q1_msg.z,
    q1_msg.w);

  q0.normalize();
  q1.normalize();

  tf2::Quaternion q = q0.slerp(q1, u);
  q.normalize();

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();

  return out;
}

std::vector<geometry_msgs::msg::Pose> sampleSplinePath(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  double sample_spacing)
{
  if (waypoints.size() < 2) {
    return waypoints;
  }

  // Remove consecutive duplicate / collapsed waypoints before spline fitting.
  std::vector<geometry_msgs::msg::Pose> clean;
  clean.reserve(waypoints.size());
  clean.push_back(waypoints.front());

  for (size_t i = 1; i < waypoints.size(); ++i) {
    if (distanceBetween(clean.back().position, waypoints[i].position) > 1e-6) {
      clean.push_back(waypoints[i]);
    }
  }

  if (clean.size() < 2) {
    return clean;
  }

  const auto s = computeArcLengths(clean);
  const double length = s.back();

  if (length < 1e-9) {
    return clean;
  }

  std::vector<double> ys;
  std::vector<double> zs;

  ys.reserve(clean.size());
  zs.reserve(clean.size());

  for (const auto& wp : clean) {
    ys.push_back(wp.position.y);
    zs.push_back(wp.position.z);
  }

  tk::spline spline_y;
  tk::spline spline_z;

  spline_y.set_points(s, ys);
  spline_z.set_points(s, zs);

  const int num_samples =
    std::max(2, static_cast<int>(std::ceil(length / sample_spacing)) + 1);

  std::vector<geometry_msgs::msg::Pose> sampled;
  sampled.reserve(num_samples);

  for (int k = 0; k < num_samples; ++k) {
    const double alpha =
      static_cast<double>(k) / static_cast<double>(num_samples - 1);

    const double s_query = alpha * length;

    geometry_msgs::msg::Pose pose;

    pose.position.x = 0.0;
    pose.position.y = spline_y(s_query);
    pose.position.z = spline_z(s_query);

    auto upper = std::upper_bound(s.begin(), s.end(), s_query);
    size_t idx1 = std::distance(s.begin(), upper);

    if (idx1 == 0) {
      pose.orientation = clean.front().orientation;
    } else if (idx1 >= s.size()) {
      pose.orientation = clean.back().orientation;
    } else {
      const size_t idx0 = idx1 - 1;
      const double segment_length = s[idx1] - s[idx0];

      double u = 0.0;
      if (segment_length > 1e-9) {
        u = (s_query - s[idx0]) / segment_length;
      }

      pose.orientation = interpolateOrientation(
        clean[idx0].orientation,
        clean[idx1].orientation,
        u);
    }

    sampled.push_back(pose);
  }

  return sampled;
}

std::vector<geometry_msgs::msg::Pose> makeBackAndForthPath(
  const std::vector<geometry_msgs::msg::Pose>& forward_path)
{
  std::vector<geometry_msgs::msg::Pose> result = forward_path;

  if (forward_path.size() < 2) {
    return result;
  }

  for (int i = static_cast<int>(forward_path.size()) - 2; i >= 0; --i) {
    result.push_back(forward_path[i]);
  }

  return result;
}

void savePosesToCsv(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  const std::string& filename)
{
  std::ofstream file(filename);

  file << "i,x,y,z,qx,qy,qz,qw\n";

  for (size_t i = 0; i < poses.size(); ++i) {
    const auto& p = poses[i].position;
    const auto& q = poses[i].orientation;

    file << i << ","
         << p.x << "," << p.y << "," << p.z << ","
         << q.x << "," << q.y << "," << q.z << "," << q.w
         << "\n";
  }
}

void ensureDirectoryExists(const std::string& directory)
{
  mkdir(directory.c_str(), 0775);
}

std::string segmentCsvPath(
  const size_t segment_index,
  const std::string& name)
{
  std::ostringstream oss;
  oss << "/tmp/horizontal_path_segments/segment_"
      << std::setw(3) << std::setfill('0') << segment_index
      << "_" << name << ".csv";
  return oss.str();
}

void saveSegmentPosesToCsv(
  const size_t segment_index,
  const std::string& name,
  const std::vector<geometry_msgs::msg::Pose>& poses)
{
  ensureDirectoryExists("/tmp/horizontal_path_segments");
  savePosesToCsv(poses, segmentCsvPath(segment_index, name));
}

void saveSegmentMetadata(
  const size_t segment_index,
  const futuraps_task_planner::HorizontalPathConfig& config,
  const size_t raw_count,
  const size_t final_count)
{
  ensureDirectoryExists("/tmp/horizontal_path_segments");

  const std::string filename = segmentCsvPath(segment_index, "metadata");
  std::ofstream file(filename);

  file << "key,value\n";
  file << "segment," << segment_index << "\n";
  file << "sample_spacing," << config.sample_spacing << "\n";
  file << "cell_size," << config.cell_size << "\n";
  file << "rows," << config.rows << "\n";
  file << "cols," << config.cols << "\n";
  file << "z0," << config.z0 << "\n";
  file << "standoff_m," << config.standoff_m << "\n";
  file << "orientation_mode," << config.orientation_mode << "\n";
  file << "use_perception_yaw," << (config.use_perception_yaw ? 1 : 0) << "\n";
  file << "local_y_smoothing_enabled,"
       << (config.local_y_smoothing_enabled ? 1 : 0) << "\n";
  file << "local_y_smoothing_radius,"
       << config.local_y_smoothing_radius << "\n";
  file << "prune_low_z_outward_waypoints_enabled,"
       << (config.prune_low_z_outward_waypoints_enabled ? 1 : 0) << "\n";
  file << "raw_waypoint_count," << raw_count << "\n";
  file << "final_spline_count," << final_count << "\n";
}

geometry_msgs::msg::Quaternion makeToolOrientationFromForward(
  const tf2::Vector3& forward)
{
  tf2::Quaternion q = quatAlignToolAxisToForward(
    forward,
    tf2::Vector3(0, 0, 1),
    2);

  tf2::Quaternion q_roll_fix;
  q_roll_fix.setRPY(0.0, 0.0, M_PI);

  q = q * q_roll_fix;
  q.normalize();

  geometry_msgs::msg::Quaternion orientation;
  orientation.x = q.x();
  orientation.y = q.y();
  orientation.z = q.z();
  orientation.w = q.w();

  return orientation;
}

geometry_msgs::msg::Quaternion makeToolOrientationFromForwardAndTangent(
  const tf2::Vector3& forward,
  const tf2::Vector3& tangent)
{
  tf2::Vector3 forward_unit = safe_normalized(forward, tf2::Vector3(0, 1, 0));
  tf2::Vector3 tangent_unit = safe_normalized(tangent, tf2::Vector3(0, 0, 1));

  tf2::Vector3 adjusted_forward =
    forward_unit - tangent_unit * forward_unit.dot(tangent_unit);

  if (adjusted_forward.length2() < kEps) {
    adjusted_forward = forward_unit;
  }

  adjusted_forward.normalize();

  return makeToolOrientationFromForward(adjusted_forward);
}

tf2::Vector3 toolForwardFromOrientation(
  const geometry_msgs::msg::Quaternion& orientation)
{
  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);
  q.normalize();

  return safe_normalized(
    tf2::quatRotate(q, tf2::Vector3(0, 0, 1)),
    tf2::Vector3(0, 1, 0));
}

tf2::Vector3 pathTangentAt(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  size_t i)
{
  if (poses.size() < 2) {
    return tf2::Vector3(0, 0, 1);
  }

  const auto point_to_tf = [](const geometry_msgs::msg::Point& p) {
    return tf2::Vector3(p.x, p.y, p.z);
  };

  const auto tangent_or_zero = [](
    const tf2::Vector3& a,
    const tf2::Vector3& b) {
    return b - a;
  };

  tf2::Vector3 t;
  if (i == 0) {
    t = tangent_or_zero(
      point_to_tf(poses[0].position),
      point_to_tf(poses[1].position));
  } else if (i + 1 < poses.size()) {
    t = tangent_or_zero(
      point_to_tf(poses[i - 1].position),
      point_to_tf(poses[i + 1].position));

    if (t.length2() < kEps) {
      t = tangent_or_zero(
        point_to_tf(poses[i].position),
        point_to_tf(poses[i + 1].position));
    }

    if (t.length2() < kEps) {
      t = tangent_or_zero(
        point_to_tf(poses[i - 1].position),
        point_to_tf(poses[i].position));
    }
  } else {
    t = tangent_or_zero(
      point_to_tf(poses[i - 1].position),
      point_to_tf(poses[i].position));
  }

  return safe_normalized(t, tf2::Vector3(0, 0, 1));
}

void applyPathTangentOrientations(
  std::vector<geometry_msgs::msg::Pose>& poses)
{
  for (size_t i = 0; i < poses.size(); ++i) {
    const tf2::Vector3 forward =
      toolForwardFromOrientation(poses[i].orientation);
    const tf2::Vector3 tangent =
      pathTangentAt(poses, i);

    poses[i].orientation =
      makeToolOrientationFromForwardAndTangent(forward, tangent);
  }
}

tf2::Vector3 predefinedTiltForward(
  int spray_side,
  double tilt_deg,
  bool moving_up)
{
  const double tilt_rad = tilt_deg * M_PI / 180.0;

  tf2::Vector3 nominal_forward(
    0.0,
    spray_side > 0 ? 1.0 : -1.0,
    0.0);

  const double z_sign = moving_up ? 1.0 : -1.0;

  tf2::Vector3 forward =
    nominal_forward + tf2::Vector3(0.0, 0.0, z_sign * std::tan(tilt_rad));

  forward.normalize();
  return forward;
}

tf2::Vector3 predefinedTiltForwardWithYaw(
  int spray_side,
  double tilt_deg,
  bool moving_up,
  const tf2::Vector3& yaw_forward)
{
  tf2::Vector3 horizontal_forward(
    yaw_forward.x(),
    yaw_forward.y(),
    0.0);

  if (horizontal_forward.length2() < kEps) {
    horizontal_forward = tf2::Vector3(
      0.0,
      spray_side > 0 ? 1.0 : -1.0,
      0.0);
  }

  horizontal_forward.normalize();

  const double tilt_rad = tilt_deg * M_PI / 180.0;
  const double z_sign = moving_up ? 1.0 : -1.0;

  tf2::Vector3 forward =
    horizontal_forward + tf2::Vector3(0.0, 0.0, z_sign * std::tan(tilt_rad));

  forward.normalize();
  return forward;
}

bool isMovingUpAt(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  size_t i)
{
  if (poses.size() == 1) {
    return true;
  }

  if (i == 0) {
    return poses[i + 1].position.z >= poses[i].position.z;
  }

  if (i == poses.size() - 1) {
    return poses[i].position.z >= poses[i - 1].position.z;
  }

  return poses[i + 1].position.z >= poses[i - 1].position.z;
}

void applyPredefinedTiltOrientations(
  std::vector<geometry_msgs::msg::Pose>& poses,
  int spray_side,
  double tilt_up_deg,
  double tilt_down_deg,
  bool use_existing_yaw)
{
  if (poses.empty()) {
    return;
  }

  for (size_t i = 0; i < poses.size(); ++i) {
    const bool moving_up = isMovingUpAt(poses, i);
    const double tilt_deg = moving_up ? tilt_up_deg : tilt_down_deg;

    const tf2::Vector3 forward =
      use_existing_yaw
        ? predefinedTiltForwardWithYaw(
            spray_side,
            tilt_deg,
            moving_up,
            toolForwardFromOrientation(poses[i].orientation))
        : predefinedTiltForward(spray_side, tilt_deg, moving_up);

    poses[i].orientation = makeToolOrientationFromForward(forward);
  }
}

tf2::Vector3 nearestReferenceForwardByZ(
  const std::vector<geometry_msgs::msg::Pose>& reference_poses,
  double z)
{
  if (reference_poses.empty()) {
    return tf2::Vector3(0, 1, 0);
  }

  size_t best_index = 0;
  double best_dz = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < reference_poses.size(); ++i) {
    const double dz = std::abs(reference_poses[i].position.z - z);
    if (dz < best_dz) {
      best_dz = dz;
      best_index = i;
    }
  }

  return toolForwardFromOrientation(reference_poses[best_index].orientation);
}

void applyPredefinedTiltOrientationsFromReferenceYaw(
  std::vector<geometry_msgs::msg::Pose>& poses,
  const std::vector<geometry_msgs::msg::Pose>& reference_poses,
  int spray_side,
  double tilt_up_deg,
  double tilt_down_deg)
{
  if (poses.empty()) {
    return;
  }

  for (size_t i = 0; i < poses.size(); ++i) {
    const bool moving_up = isMovingUpAt(poses, i);
    const double tilt_deg = moving_up ? tilt_up_deg : tilt_down_deg;
    const tf2::Vector3 yaw_forward =
      nearestReferenceForwardByZ(reference_poses, poses[i].position.z);

    const tf2::Vector3 forward =
      predefinedTiltForwardWithYaw(
        spray_side,
        tilt_deg,
        moving_up,
        yaw_forward);

    poses[i].orientation = makeToolOrientationFromForward(forward);
  }
}

double smoothstep01(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

std::size_t findSharpestYTransition(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  double min_sharpness)
{
  if (poses.size() < 5) {
    return 0;
  }

  std::size_t best_index = 0;
  double best_score = 0.0;

  for (std::size_t i = 1; i + 1 < poses.size(); ++i) {
    const double dy_prev =
      poses[i].position.y - poses[i - 1].position.y;

    const double dy_next =
      poses[i + 1].position.y - poses[i].position.y;

    const double score = std::abs(dy_next - dy_prev);

    if (score > best_score) {
      best_score = score;
      best_index = i;
    }
  }

  if (best_score < min_sharpness) {
    return 0;
  }

  return best_index;
}

std::vector<geometry_msgs::msg::Pose> smoothLocalYTransition(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  std::size_t center_index,
  int radius)
{
  if (poses.size() < 5 || radius < 2) {
    return poses;
  }

  if (center_index == 0 || center_index >= poses.size() - 1) {
    return poses;
  }

  const std::size_t r = static_cast<std::size_t>(radius);

  const std::size_t start =
    center_index > r ? center_index - r : 0;

  const std::size_t end =
    std::min(center_index + r, poses.size() - 1);

  if (end <= start + 2) {
    return poses;
  }

  std::vector<geometry_msgs::msg::Pose> smoothed = poses;

  const double y_start = poses[start].position.y;
  const double y_end = poses[end].position.y;

  for (std::size_t i = start; i <= end; ++i) {
    const double t =
      static_cast<double>(i - start) /
      static_cast<double>(end - start);

    const double s = smoothstep01(t);

    smoothed[i].position.y =
      y_start + s * (y_end - y_start);
  }

  return smoothed;
}

std::vector<geometry_msgs::msg::Pose> smoothSharpestLocalYTransition(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  double min_sharpness,
  int radius,
  const rclcpp::Logger& logger)
{
  const std::size_t sharp_index =
    findSharpestYTransition(poses, min_sharpness);

  if (sharp_index == 0) {
    RCLCPP_INFO(
      logger,
      "Local y smoothing: no sharp transition found. min_sharpness=%.4f",
      min_sharpness);

    return poses;
  }

  RCLCPP_INFO(
    logger,
    "Local y smoothing: smoothing around index=%zu y=%.3f z=%.3f radius=%d",
    sharp_index,
    poses[sharp_index].position.y,
    poses[sharp_index].position.z,
    radius);

  return smoothLocalYTransition(
    poses,
    sharp_index,
    radius);
}

bool isLowZOutwardWaypoint(
  const geometry_msgs::msg::Pose& pose,
  double max_z,
  double min_abs_y)
{
  return pose.position.z <= max_z &&
         std::abs(pose.position.y) >= min_abs_y;
}

std::vector<geometry_msgs::msg::Pose> pruneLowZOutwardEndpointWaypoints(
  const std::vector<geometry_msgs::msg::Pose>& poses,
  double max_z,
  double min_abs_y,
  int min_remaining,
  const rclcpp::Logger& logger)
{
  if (poses.empty()) {
    return poses;
  }

  const std::size_t min_keep =
    static_cast<std::size_t>(std::max(2, min_remaining));

  std::size_t start = 0;
  std::size_t end = poses.size();

  while (end - start > min_keep &&
         isLowZOutwardWaypoint(poses[start], max_z, min_abs_y)) {
    ++start;
  }

  while (end - start > min_keep &&
         isLowZOutwardWaypoint(poses[end - 1], max_z, min_abs_y)) {
    --end;
  }

  if (start == 0 && end == poses.size()) {
    RCLCPP_INFO(
      logger,
      "Low-z outward pruning: no endpoint waypoints removed. max_z=%.3f min_abs_y=%.3f",
      max_z,
      min_abs_y);

    return poses;
  }

  RCLCPP_INFO(
    logger,
    "Low-z outward pruning: removed start=%zu end=%zu max_z=%.3f min_abs_y=%.3f",
    start,
    poses.size() - end,
    max_z,
    min_abs_y);

  return std::vector<geometry_msgs::msg::Pose>(
    poses.begin() + static_cast<std::ptrdiff_t>(start),
    poses.begin() + static_cast<std::ptrdiff_t>(end));
}

}  // namespace

namespace futuraps_task_planner
{

void PathBuilder::configure(const HorizontalPathConfig& config)
{
  config_ = config;
  max_inward_angle_deg_ = config.max_inward_angle_deg;
  sample_spacing_ = config.sample_spacing;
}

void PathBuilder::setReachabilityFilter(std::shared_ptr<ReachabilityFilter> filter)
{
  reachability_filter_ = filter;
}

SprayPath PathBuilder::buildPath(const PerceptionResult& perception) const
{
  const bool use_predefined_tilt =
    config_.orientation_mode == "predefined_tilt";
  const bool use_perception_yaw =
    use_predefined_tilt && config_.use_perception_yaw;
  const bool use_path_tangent_orientation =
    config_.orientation_mode == "perception_normals_path_tangent" ||
    config_.orientation_mode == "path_tangent";
  const size_t segment_index = debug_segment_index_++;

  auto raw_waypoints = extractWaypoints(perception);
  savePosesToCsv(raw_waypoints, "/tmp/raw_waypoints.csv");
  saveSegmentPosesToCsv(segment_index, "raw_waypoints", raw_waypoints);

  auto initial_spline_waypoints = sampleSplinePath(raw_waypoints, sample_spacing_);

  if (use_predefined_tilt) {
    applyPredefinedTiltOrientations(
      initial_spline_waypoints,
      spray_side_,
      config_.predefined_tilt_up_deg,
      config_.predefined_tilt_down_deg,
      use_perception_yaw);
  } else if (use_path_tangent_orientation) {
    applyPathTangentOrientations(initial_spline_waypoints);
  }

  savePosesToCsv(initial_spline_waypoints, "/tmp/initial_spline.csv");
  saveSegmentPosesToCsv(segment_index, "initial_spline", initial_spline_waypoints);

  auto reachable_waypoints =
    reachability_filter_
      ? reachability_filter_->filterReachableSegment(initial_spline_waypoints)
      : initial_spline_waypoints;

  savePosesToCsv(reachable_waypoints, "/tmp/filtered_waypoints.csv");
  saveSegmentPosesToCsv(segment_index, "filtered_waypoints", reachable_waypoints);

  auto pruned_reachable_waypoints = reachable_waypoints;

  if (config_.prune_low_z_outward_waypoints_enabled) {
    pruned_reachable_waypoints =
      pruneLowZOutwardEndpointWaypoints(
        reachable_waypoints,
        config_.prune_low_z_outward_max_z,
        config_.prune_low_z_outward_min_abs_y,
        config_.prune_low_z_outward_min_remaining,
        logger_);
  }

  savePosesToCsv(
    pruned_reachable_waypoints,
    "/tmp/filtered_waypoints_after_low_z_pruning.csv");
  saveSegmentPosesToCsv(
    segment_index,
    "filtered_waypoints_after_low_z_pruning",
    pruned_reachable_waypoints);
  savePosesToCsv(
    pruned_reachable_waypoints,
    "/tmp/filtered_waypoints_before_smoothing.csv");
  saveSegmentPosesToCsv(
    segment_index,
    "filtered_waypoints_before_smoothing",
    pruned_reachable_waypoints);

  auto smoothed_reachable_waypoints = pruned_reachable_waypoints;

  if (config_.local_y_smoothing_enabled) {
    smoothed_reachable_waypoints =
      smoothSharpestLocalYTransition(
        pruned_reachable_waypoints,
        config_.local_y_smoothing_min_sharpness,
        config_.local_y_smoothing_radius,
        logger_);

    if (use_predefined_tilt && use_perception_yaw) {
      applyPredefinedTiltOrientationsFromReferenceYaw(
        smoothed_reachable_waypoints,
        initial_spline_waypoints,
        spray_side_,
        config_.predefined_tilt_up_deg,
        config_.predefined_tilt_down_deg);
    } else if (use_predefined_tilt) {
      applyPredefinedTiltOrientations(
        smoothed_reachable_waypoints,
        spray_side_,
        config_.predefined_tilt_up_deg,
        config_.predefined_tilt_down_deg,
        false);
    } else if (use_path_tangent_orientation) {
      applyPathTangentOrientations(smoothed_reachable_waypoints);
    }
  }

  savePosesToCsv(
    smoothed_reachable_waypoints,
    "/tmp/filtered_waypoints_after_local_smoothing.csv");
  saveSegmentPosesToCsv(
    segment_index,
    "filtered_waypoints_after_local_smoothing",
    smoothed_reachable_waypoints);

  auto final_spline_waypoints =
    sampleSplinePath(smoothed_reachable_waypoints, sample_spacing_);

  if (use_predefined_tilt && use_perception_yaw) {
    applyPredefinedTiltOrientationsFromReferenceYaw(
      final_spline_waypoints,
      initial_spline_waypoints,
      spray_side_,
      config_.predefined_tilt_up_deg,
      config_.predefined_tilt_down_deg);
  } else if (use_predefined_tilt) {
    applyPredefinedTiltOrientations(
      final_spline_waypoints,
      spray_side_,
      config_.predefined_tilt_up_deg,
      config_.predefined_tilt_down_deg,
      false);
  } else if (use_path_tangent_orientation) {
    applyPathTangentOrientations(final_spline_waypoints);
  }

  savePosesToCsv(final_spline_waypoints, "/tmp/final_spline.csv");
  saveSegmentPosesToCsv(segment_index, "final_spline", final_spline_waypoints);
  saveSegmentMetadata(
    segment_index,
    config_,
    raw_waypoints.size(),
    final_spline_waypoints.size());

  auto mirrored_waypoints = makeBackAndForthPath(final_spline_waypoints);

  if (use_predefined_tilt) {
    applyPredefinedTiltOrientations(
      mirrored_waypoints,
      spray_side_,
      config_.predefined_tilt_up_deg,
      config_.predefined_tilt_down_deg,
      use_perception_yaw);
  } else if (use_path_tangent_orientation) {
    applyPathTangentOrientations(mirrored_waypoints);
  }

  auto path = makeSprayPath(mirrored_waypoints);

  return path;
}

std::vector<geometry_msgs::msg::Pose> PathBuilder::extractWaypoints(
  const PerceptionResult& perception) const
{
  const bool use_predefined_tilt =
    config_.orientation_mode == "predefined_tilt";
  const bool use_perception_yaw =
    use_predefined_tilt && config_.use_perception_yaw;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (!perception.valid) {
    RCLCPP_WARN(logger_, "extractWaypoints: perception result is not valid");
    return waypoints;
  }

  if (perception.x.empty() || perception.y.empty() ||
      perception.z.empty() || perception.found.empty()) {
    RCLCPP_WARN(logger_, "extractWaypoints: perception arrays are empty");
    return waypoints;
  }

  if (!use_predefined_tilt &&
      (perception.normals.empty() || perception.normal_valid.empty())) {
    RCLCPP_WARN(logger_, "extractWaypoints: normals arrays are empty");
    return waypoints;
  }

  if (use_perception_yaw &&
      (perception.normals.empty() || perception.normal_valid.empty())) {
    RCLCPP_WARN(
      logger_,
      "extractWaypoints: use_perception_yaw enabled but normals arrays are empty; "
      "falling back to predefined yaw");
  }

  if (perception.x.size() != perception.y.size() ||
      perception.x.size() != perception.z.size() ||
      perception.x.size() != perception.found.size()) {
    RCLCPP_WARN(
      logger_,
      "extractWaypoints: inconsistent perception array sizes "
      "(x=%zu y=%zu z=%zu found=%zu)",
      perception.x.size(),
      perception.y.size(),
      perception.z.size(),
      perception.found.size());
    return waypoints;
  }

  if (!use_predefined_tilt && perception.normals.size() != perception.normal_valid.size()) {
    RCLCPP_WARN(
      logger_,
      "extractWaypoints: normals and normal_valid size mismatch "
      "(normals=%zu normal_valid=%zu)",
      perception.normals.size(),
      perception.normal_valid.size());
    return waypoints;
  }

  if (use_perception_yaw && perception.normals.size() != perception.normal_valid.size()) {
    RCLCPP_WARN(
      logger_,
      "extractWaypoints: use_perception_yaw enabled but normals and normal_valid size mismatch "
      "(normals=%zu normal_valid=%zu); falling back to predefined yaw",
      perception.normals.size(),
      perception.normal_valid.size());
  }

  const bool normals_available =
    !perception.normals.empty() &&
    perception.normals.size() == perception.normal_valid.size();

  const int num_normals =
    (use_predefined_tilt && !use_perception_yaw) || !normals_available
      ? 1
      : static_cast<int>(perception.normals.size());

  bool has_last_valid_forward = false;
  tf2::Vector3 last_valid_forward(
    0.0,
    spray_side_ > 0 ? 1.0 : -1.0,
    0.0);

  for (int i = 0; i < config_.rows; i++) {
    bool found_in_row = false;

    double closest_y = 0.0;
    double closest_z = 0.0;
    double best_metric = std::numeric_limits<double>::infinity();

    const int normal_idx = std::clamp(
      static_cast<int>(
        std::floor(
          static_cast<double>(i) /
          static_cast<double>(config_.rows) *
          static_cast<double>(num_normals))),
      0,
      num_normals - 1);

    tf2::Vector3 nominal_forward(
      0.0,
      spray_side_ > 0 ? 1.0 : -1.0,
      0.0);

    tf2::Vector3 forward;
    bool using_fallback_normal = false;
    bool using_predefined_yaw_fallback = false;

    if (use_predefined_tilt) {
      tf2::Vector3 yaw_forward = nominal_forward;

      if (use_perception_yaw && normals_available) {
        if (!perception.normal_valid[static_cast<size_t>(normal_idx)] ||
            perception.normals[static_cast<size_t>(normal_idx)].length2() < kEps) {
          if (has_last_valid_forward) {
            yaw_forward = last_valid_forward;
            using_fallback_normal = true;
          } else {
            using_predefined_yaw_fallback = true;
          }
        } else {
          yaw_forward = perception.normals[static_cast<size_t>(normal_idx)];
          yaw_forward.normalize();
          yaw_forward = -yaw_forward;

          const double max_angle_rad =
            max_inward_angle_deg_ * M_PI / 180.0;

          yaw_forward = clamp_forward_to_nominal(
            yaw_forward,
            nominal_forward,
            max_angle_rad);

          last_valid_forward = yaw_forward;
          has_last_valid_forward = true;
        }
      } else if (use_perception_yaw) {
        using_predefined_yaw_fallback = true;
      }

      // Raw extracted path is upward; later passes recompute up/down tilt from
      // the actual path direction.
      forward =
        use_perception_yaw
          ? predefinedTiltForwardWithYaw(
              spray_side_,
              config_.predefined_tilt_up_deg,
              true,
              yaw_forward)
          : predefinedTiltForward(
              spray_side_,
              config_.predefined_tilt_up_deg,
              true);
    } else {
      if (!perception.normal_valid[static_cast<size_t>(normal_idx)] ||
          perception.normals[static_cast<size_t>(normal_idx)].length2() < kEps) {

        if (!has_last_valid_forward) {
          RCLCPP_WARN(
            logger_,
            "Skipping row %d: normal %d invalid and no previous normal available",
            i,
            normal_idx);
          continue;
        }

        forward = last_valid_forward;
        using_fallback_normal = true;

      } else {
        forward = perception.normals[static_cast<size_t>(normal_idx)];
        forward.normalize();
        forward = -forward;

        const double max_angle_rad =
          max_inward_angle_deg_ * M_PI / 180.0;

        forward = clamp_forward_to_nominal(
          forward,
          nominal_forward,
          max_angle_rad);

        last_valid_forward = forward;
        has_last_valid_forward = true;
      }
    }

    if (using_fallback_normal) {
      RCLCPP_WARN(
        logger_,
        "Row %d: using previous valid normal instead of normal %d",
        i,
        normal_idx);
    }

    if (using_predefined_yaw_fallback) {
      RCLCPP_WARN(
        logger_,
        "Row %d: perception yaw unavailable; using predefined yaw",
        i);
    }

    for (int j = 0; j < config_.cols; j++) {
      const int idx = i * config_.cols + j;

      if (idx >= static_cast<int>(perception.found.size())) {
        RCLCPP_WARN(
          logger_,
          "Row %d col %d: index %d outside found array",
          i,
          j,
          idx);
        continue;
      }

      if (!perception.found[static_cast<size_t>(idx)]) {
        continue;
      }

      const double x = perception.x[static_cast<size_t>(idx)];
      const double y = perception.y[static_cast<size_t>(idx)];
      const double z = perception.z[static_cast<size_t>(idx)];

      if (!std::isfinite(x) ||
          !std::isfinite(y) ||
          !std::isfinite(z)) {
        RCLCPP_WARN(
          logger_,
          "Row %d col %d: non-finite point values "
          "(x=%.3f y=%.3f z=%.3f)",
          i,
          j,
          x,
          y,
          z);
        continue;
      }

      const double metric = std::abs(y);

      if (metric < best_metric) {
        best_metric = metric;
        closest_y = y;
        closest_z = z;
        found_in_row = true;
      }
    }

    if (!found_in_row) {
      RCLCPP_WARN(
        logger_,
        "Skipping row %d: no valid point found in row",
        i);
      continue;
    }

    const double row_center_z =
      config_.z0 + config_.cell_size * (static_cast<double>(i) + 0.5);

    geometry_msgs::msg::Point closest_point;
    closest_point.x = 0.0;
    closest_point.y = closest_y - config_.standoff_m;
    closest_point.z = closest_z; // row_center_z;

    geometry_msgs::msg::Quaternion orientation =
      makeToolOrientationFromForward(forward);

    geometry_msgs::msg::Pose pose;
    pose.position = closest_point;
    pose.orientation = orientation;

    waypoints.push_back(pose);

    RCLCPP_INFO(
      logger_,
      "Added waypoint row=%d normal=%d y=%.3f z=%.3f fallback_normal=%s",
      i,
      normal_idx,
      closest_point.y,
      closest_point.z,
      (using_fallback_normal || using_predefined_yaw_fallback) ? "true" : "false");
  }

  if (config_.add_top_waypoint &&
      perception.top_point_found &&
      !waypoints.empty())
  {
    const double desired_top_z =
      perception.top_z + config_.top_waypoint_margin;

    const double current_top_z =
      waypoints.back().position.z;

    const double dz_top =
      desired_top_z - current_top_z;

    if (dz_top > config_.min_top_extension &&
        dz_top < config_.max_top_extension &&
        desired_top_z < config_.max_z)
    {
      geometry_msgs::msg::Pose top_pose = waypoints.back();

      const double previous_tool_y = waypoints.back().position.y;
      double top_tool_y = perception.top_y - config_.standoff_m;

      // Do not let the top extension move closer to the robot/platform than
      // the previous waypoint.
      if (std::abs(top_tool_y) < std::abs(previous_tool_y)) {
        top_tool_y = previous_tool_y;
      }

      top_pose.position.x = 0.0;
      top_pose.position.y = top_tool_y;
      top_pose.position.z = desired_top_z;

      waypoints.push_back(top_pose);

      RCLCPP_INFO(
        logger_,
        "Added top canopy waypoint: previous_z=%.3f top_z=%.3f waypoint_z=%.3f y=%.3f count=%u",
        current_top_z,
        perception.top_z,
        top_pose.position.z,
        top_pose.position.y,
        perception.top_count);
    } else {
      RCLCPP_INFO(
        logger_,
        "Top canopy point not added: current_top_z=%.3f desired_top_z=%.3f dz=%.3f",
        current_top_z,
        desired_top_z,
        dz_top);
    }
  }

  RCLCPP_INFO(
    logger_,
    "extractWaypoints: created %zu waypoints from %d rows using %d normals",
    waypoints.size(),
    config_.rows,
    num_normals);

  return waypoints;
}

SprayPath PathBuilder::makeSprayPath(
  const std::vector<geometry_msgs::msg::Pose>& waypoints) const
{
  SprayPath sprayPath;

  if (waypoints.empty()) {
    return sprayPath;
  }

  for (size_t i = 0; i < waypoints.size(); i++) {
    SprayWaypoint sprayWaypoint;

    sprayWaypoint.pose = waypoints[i];

    if (waypoints.size() == 1) {
      sprayWaypoint.tangent = Eigen::Vector3d::Zero();
    } else {
      Eigen::Vector3d t;

      if (i == 0) {
        t = pointToEigen(waypoints[1].position) -
            pointToEigen(waypoints[0].position);
      } else if (i < waypoints.size() - 1) {
        t = pointToEigen(waypoints[i + 1].position) -
            pointToEigen(waypoints[i - 1].position);
      } else {
        t = pointToEigen(waypoints[i].position) -
            pointToEigen(waypoints[i - 1].position);
      }

      sprayWaypoint.tangent =
        t.norm() > 1e-9 ? t.normalized() : Eigen::Vector3d::Zero();
    }

    if (i == 0) {
      sprayWaypoint.s = 0.0;
    } else {
      const double dx = waypoints[i].position.x - waypoints[i - 1].position.x;
      const double dy = waypoints[i].position.y - waypoints[i - 1].position.y;
      const double dz = waypoints[i].position.z - waypoints[i - 1].position.z;

      const double ds = std::sqrt(dx * dx + dy * dy + dz * dz);
      sprayWaypoint.s = sprayPath.waypoints[i - 1].s + ds;
    }

    sprayPath.waypoints.push_back(sprayWaypoint);
  }

  return sprayPath;
}
} // namespace futuraps_task_planner
