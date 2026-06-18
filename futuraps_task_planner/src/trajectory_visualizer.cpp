#include "futuraps_task_planner/trajectory_visualizer.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace futuraps_task_planner
{

TrajectoryVisualizer::TrajectoryVisualizer(rclcpp::Node * node)
: node_(node)
{
  goal_marker_pub_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/spray/goal_markers", 10);

  desired_path_pub_ =
    node_->create_publisher<nav_msgs::msg::Path>(
      "/spray/desired_path", 10);

  actual_path_pub_ =
    node_->create_publisher<nav_msgs::msg::Path>(
      "/spray/actual_path", 10);

  desired_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/spray/ee_desired_pose", 10);

  actual_pose_pub_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/spray/ee_actual_pose", 10);
}

void TrajectoryVisualizer::configure(
  const std::string & fixed_frame,
  const std::string & ee_frame)
{
  fixed_frame_ = fixed_frame;
  ee_frame_ = ee_frame;

  desired_path_msg_.header.frame_id = fixed_frame_;
  actual_path_msg_.header.frame_id = fixed_frame_;
}

void TrajectoryVisualizer::reset()
{
  desired_path_msg_.poses.clear();
  actual_path_msg_.poses.clear();

  desired_path_msg_.header.stamp = node_->now();
  actual_path_msg_.header.stamp = node_->now();

  desired_path_pub_->publish(desired_path_msg_);
  actual_path_pub_->publish(actual_path_msg_);

  visualization_msgs::msg::MarkerArray delete_array;
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = fixed_frame_;
  delete_marker.header.stamp = node_->now();
  delete_marker.ns = "goal_points";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  delete_array.markers.push_back(delete_marker);
  goal_marker_pub_->publish(delete_array);
}

geometry_msgs::msg::PoseStamped
TrajectoryVisualizer::makePoseStamped(const geometry_msgs::msg::Pose & pose) const
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = fixed_frame_;
  ps.header.stamp = node_->now();
  ps.pose = pose;
  return ps;
}

void TrajectoryVisualizer::publishGoalPoints(
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = fixed_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "goal_points";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = waypoints[i];
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 0.1;
    marker.color.g = 0.8;
    marker.color.b = 0.1;
    marker_array.markers.push_back(marker);
  }

  goal_marker_pub_->publish(marker_array);
}

void TrajectoryVisualizer::publishDesiredPath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  desired_path_msg_.header.frame_id = fixed_frame_;
  desired_path_msg_.header.stamp = node_->now();
  desired_path_msg_.poses.clear();

  for (const auto & pose : waypoints) {
    desired_path_msg_.poses.push_back(makePoseStamped(pose));
  }

  desired_path_pub_->publish(desired_path_msg_);
}

void TrajectoryVisualizer::publishDesiredPose(
  const geometry_msgs::msg::Pose & pose)
{
  auto ps = makePoseStamped(pose);
  desired_pose_pub_->publish(ps);
}

void TrajectoryVisualizer::appendActualPose(
  const geometry_msgs::msg::Pose & pose)
{
  auto ps = makePoseStamped(pose);

  actual_pose_pub_->publish(ps);

  actual_path_msg_.header.frame_id = fixed_frame_;
  actual_path_msg_.header.stamp = node_->now();
  actual_path_msg_.poses.push_back(ps);

  actual_path_pub_->publish(actual_path_msg_);
}

bool TrajectoryVisualizer::appendActualPoseFromTF(
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(
      fixed_frame_, ee_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Actual EE TF lookup failed: %s", ex.what());
    return false;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = fixed_frame_;
  ps.header.stamp = node_->now();
  ps.pose.position.x = tf.transform.translation.x;
  ps.pose.position.y = tf.transform.translation.y;
  ps.pose.position.z = tf.transform.translation.z;
  ps.pose.orientation = tf.transform.rotation;

  actual_path_msg_.header.frame_id = fixed_frame_;
  actual_path_msg_.header.stamp = node_->now();
  actual_path_msg_.poses.push_back(ps);

  actual_path_pub_->publish(actual_path_msg_);
  actual_pose_pub_->publish(ps);

  return true;
}

}  // namespace futuraps_task_planner